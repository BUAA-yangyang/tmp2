"""Fail-closed, reversible DWA speed profiling for entry transit."""

import math


class EntrySpeedLimitError(RuntimeError):
    """The entry speed profile could not be applied or restored exactly."""


class EntrySpeedLimiter:
    """Temporarily apply a validated State_RL command envelope.

    The caller supplies ROS message factories so the state machine can be
    unit-tested without importing ROS.  Calling the service with an empty
    request returns the authoritative live DWA configuration; parameter-server
    values are deliberately not trusted as a snapshot.
    """

    REQUIRED_PARAMETERS = (
        "max_vel_x",
        "max_vel_y",
        "max_vel_trans",
        "max_vel_theta",
        "min_vel_x",
        "min_vel_trans",
        "min_vel_theta",
        "sim_time",
    )

    def __init__(
            self,
            call_service,
            request_factory,
            double_parameter_factory,
            limits,
            tolerance=1e-6,
            restore_retries=3,
            restore_retry_backoff_s=0.5,
            sleep=None):
        self._call_service = call_service
        self._request_factory = request_factory
        self._double_parameter_factory = double_parameter_factory
        self._limits = {
            name: float(
                limits.get(name, limits["min_vel_trans"])
                if name == "min_vel_x" else limits[name]
            )
            for name in self.REQUIRED_PARAMETERS
        }
        self._tolerance = float(tolerance)
        self._restore_retries = max(1, int(restore_retries))
        self._restore_retry_backoff_s = max(0.0, float(restore_retry_backoff_s))
        if sleep is None:
            import time as _time
            sleep = _time.sleep
        self._sleep = sleep
        self._snapshot = None
        self._active = False
        self._validate_limits()

    @property
    def active(self):
        return self._active

    @property
    def limits(self):
        return dict(self._limits)

    @staticmethod
    def _config_values(response):
        config = getattr(response, "config", None)
        if config is None:
            raise EntrySpeedLimitError(
                "dynamic_reconfigure response has no config"
            )
        values = {}
        for parameter in getattr(config, "doubles", ()):
            values[str(parameter.name)] = float(parameter.value)
        return values

    def _request(self, values=None):
        request = self._request_factory()
        if values:
            request.config.doubles = [
                self._double_parameter_factory(name=name, value=value)
                for name, value in values.items()
            ]
        return request

    def _validate_limits(self):
        if not math.isfinite(self._tolerance) or self._tolerance < 0.0:
            raise EntrySpeedLimitError(
                "verification tolerance must be finite and nonnegative"
            )
        for name, value in self._limits.items():
            if not math.isfinite(value) or value < 0.0:
                raise EntrySpeedLimitError(
                    "%s limit must be finite and nonnegative" % name
                )
        for name in ("max_vel_x", "max_vel_trans", "max_vel_theta"):
            if self._limits[name] <= 0.0:
                raise EntrySpeedLimitError(
                    "%s limit must be positive" % name
                )
        if self._limits["sim_time"] <= 0.0:
            raise EntrySpeedLimitError("sim_time must be positive")
        if self._limits["min_vel_trans"] > self._limits["max_vel_trans"]:
            raise EntrySpeedLimitError(
                "min_vel_trans must not exceed max_vel_trans"
            )
        if self._limits["min_vel_x"] > self._limits["max_vel_x"]:
            raise EntrySpeedLimitError(
                "min_vel_x must not exceed max_vel_x"
            )
        if self._limits["min_vel_theta"] > self._limits["max_vel_theta"]:
            raise EntrySpeedLimitError(
                "min_vel_theta must not exceed max_vel_theta"
            )

    def _required_values(self, response, operation):
        values = self._config_values(response)
        # Older test doubles and older DWA releases may omit min_vel_x.  Their
        # translational minimum is the authoritative compatible fallback.
        if "min_vel_x" not in values and "min_vel_trans" in values:
            values["min_vel_x"] = values["min_vel_trans"]
        missing = [
            name for name in self.REQUIRED_PARAMETERS if name not in values
        ]
        if missing:
            raise EntrySpeedLimitError(
                "%s response is missing %s"
                % (operation, ", ".join(sorted(missing)))
            )
        selected = {name: values[name] for name in self.REQUIRED_PARAMETERS}
        if not all(math.isfinite(value) for value in selected.values()):
            raise EntrySpeedLimitError(
                "%s response contains a non-finite speed value" % operation
            )
        return selected

    def _verify(self, response, expected, operation):
        values = self._required_values(response, operation)
        mismatches = [
            "%s expected %.9g got %.9g"
            % (name, expected[name], values[name])
            for name in self.REQUIRED_PARAMETERS
            if abs(values[name] - expected[name]) > self._tolerance
        ]
        if mismatches:
            raise EntrySpeedLimitError(
                "%s verification failed: %s"
                % (operation, "; ".join(mismatches))
            )

    def apply(self):
        if self._active:
            raise EntrySpeedLimitError("entry speed profile is already active")
        try:
            current_response = self._call_service(self._request())
            snapshot = self._required_values(
                current_response, "live configuration query"
            )
        except EntrySpeedLimitError:
            raise
        except Exception as error:
            raise EntrySpeedLimitError(
                "live configuration query failed: %s" % error
            )

        # Maxima are safety ceilings and may never be raised here. Positive
        # minima keep DWA out of State_RL's empirically near-stationary command
        # region while traversing the already validated entrance corridor.
        maximum_names = (
            "max_vel_x", "max_vel_y", "max_vel_trans", "max_vel_theta"
        )
        increases = [
            "%s %.9g -> %.9g"
            % (name, snapshot[name], self._limits[name])
            for name in maximum_names
            if self._limits[name] > snapshot[name] + self._tolerance
        ]
        if increases:
            raise EntrySpeedLimitError(
                "entry profile must not increase live speed bounds: %s"
                % "; ".join(increases)
            )

        self._snapshot = snapshot
        try:
            response = self._call_service(self._request(self._limits))
            self._verify(response, self._limits, "entry profile apply")
            self._active = True
        except Exception as apply_error:
            rollback_error = None
            try:
                response = self._call_service(self._request(snapshot))
                self._verify(response, snapshot, "entry profile rollback")
            except Exception as error:
                rollback_error = error
            self._snapshot = None
            self._active = False
            if rollback_error is not None:
                raise EntrySpeedLimitError(
                    "entry profile apply failed: %s; rollback failed: %s"
                    % (apply_error, rollback_error)
                )
            raise EntrySpeedLimitError(
                "entry profile apply failed and was rolled back: %s"
                % apply_error
            )

    def restore(self):
        """Restore the pre-entry DWA profile, retrying transient failures.

        The reconfigure ServiceProxy can go stale while the entry profile is
        active (observed: ``[Errno 104] Connection reset by peer``).  The old
        code failed closed on the first error, which left the 0.05 m/s entry
        crawl profile ACTIVE for the whole return leg -- the robot reached the
        building door and then could not make progress.  Restore is now
        retried with backoff and is idempotent; it still fails closed once the
        retry budget is exhausted so a caller can stop the robot.
        """
        if not self._active:
            return
        snapshot = dict(self._snapshot)
        last_error = None
        for attempt in range(self._restore_retries):
            try:
                response = self._call_service(self._request(snapshot))
                self._verify(response, snapshot, "entry profile restore")
                self._snapshot = None
                self._active = False
                return
            except Exception as error:
                last_error = error
                if attempt + 1 < self._restore_retries:
                    self._sleep(
                        self._restore_retry_backoff_s * (2 ** attempt))
        raise EntrySpeedLimitError(
            "entry profile restore failed after %d attempts: %s"
            % (self._restore_retries, last_error)
        )

    def verify_active_profile(self, expected):
        """Fail closed unless the live DWA profile equals ``expected``.

        Used as a precondition before any return leg so the robot can never
        drive home on the entry crawl profile.
        """
        try:
            response = self._call_service(self._request())
            live = self._required_values(
                response, "entry profile verification")
        except EntrySpeedLimitError:
            raise
        except Exception as error:
            raise EntrySpeedLimitError(
                "entry profile verification failed: %s" % error
            )
        mismatched = [
            "%s live=%.9g expected=%.9g" % (name, live[name], expected[name])
            for name in self.REQUIRED_PARAMETERS
            if abs(live[name] - float(expected[name])) > self._tolerance
        ]
        if mismatched:
            raise EntrySpeedLimitError(
                "entry profile is not restored: %s" % "; ".join(mismatched)
            )
        return True
