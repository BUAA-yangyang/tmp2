#!/usr/bin/env python3
"""Fail-closed helpers for the DEV-ONLY indoor-start runner."""

import argparse
import json
import os
import re
import subprocess
import sys
import time


STARTUP_SENTINEL = "Simulation startup command completed."
PAUSE_PATTERN = re.compile(r"^pause:\s*(?:True|true|1)\s*$", re.MULTILINE)


class RunnerFailure(RuntimeError):
    pass


class PauseStability:
    """Require consecutive observations after all startup prerequisites."""

    def __init__(self, required=3):
        if required < 1:
            raise ValueError("required stable observations must be positive")
        self.required = int(required)
        self.consecutive = 0

    def observe(
            self, startup_complete, simulation_alive, controller_alive,
            services_ready, pause_call_succeeded, physics_output):
        ready = (
            startup_complete
            and simulation_alive
            and controller_alive
            and services_ready
            and pause_call_succeeded
            and physics_is_paused(physics_output)
        )
        self.consecutive = self.consecutive + 1 if ready else 0
        return self.consecutive >= self.required


def physics_is_paused(output):
    return bool(PAUSE_PATTERN.search(output or ""))


def read_pid(path):
    try:
        value = open(path, encoding="utf-8").read().strip()
        pid = int(value)
    except (OSError, TypeError, ValueError) as error:
        raise RunnerFailure(
            "controller pid file is unavailable or invalid: %s: %s"
            % (path, error)
        )
    if pid <= 0:
        raise RunnerFailure("controller pid must be positive: %d" % pid)
    return pid


def process_alive(pid):
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def startup_log_complete(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as stream:
            return STARTUP_SENTINEL in stream.read()
    except OSError:
        return False


def run_command(arguments):
    try:
        return subprocess.run(
            arguments,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=5.0,
        )
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode("utf-8", errors="replace")
        return subprocess.CompletedProcess(arguments, 124, stdout=output)


def wait_for_ready(arguments):
    deadline = time.monotonic() + arguments.timeout
    stability = PauseStability(arguments.stable_observations)
    last = {}
    while time.monotonic() < deadline:
        simulation_alive = process_alive(arguments.simulation_pid)
        startup_complete = startup_log_complete(arguments.simulation_log)
        try:
            controller_pid = read_pid(arguments.controller_pid_file)
            controller_alive = process_alive(controller_pid)
        except RunnerFailure:
            controller_pid = None
            controller_alive = False

        services = run_command(["rosservice", "list"])
        service_names = set(services.stdout.splitlines())
        services_ready = (
            services.returncode == 0
            and "/gazebo/pause_physics" in service_names
            and "/gazebo/get_physics_properties" in service_names
        )
        pause = None
        physics = None
        if startup_complete and services_ready:
            pause = run_command(
                ["rosservice", "call", "/gazebo/pause_physics"]
            )
            physics = run_command(
                ["rosservice", "call", "/gazebo/get_physics_properties"]
            )
        pause_succeeded = pause is not None and pause.returncode == 0
        physics_output = "" if physics is None else physics.stdout
        last = {
            "startup_complete": startup_complete,
            "simulation_pid": arguments.simulation_pid,
            "simulation_alive": simulation_alive,
            "controller_pid": controller_pid,
            "controller_alive": controller_alive,
            "services_ready": services_ready,
            "pause_call_succeeded": pause_succeeded,
            "physics_call_succeeded": (
                physics is not None and physics.returncode == 0
            ),
            "physics_paused": physics_is_paused(physics_output),
            "stable_observations": stability.consecutive,
        }
        if stability.observe(
            startup_complete,
            simulation_alive,
            controller_alive,
            services_ready,
            pause_succeeded,
            physics_output,
        ):
            last["stable_observations"] = stability.consecutive
            last["ready"] = True
            print(json.dumps(last, indent=2, sort_keys=True))
            return 0
        if not simulation_alive:
            raise RunnerFailure(
                "simulation process exited before the readiness handshake: "
                + json.dumps(last, sort_keys=True)
            )
        time.sleep(arguments.poll_interval)
    raise RunnerFailure(
        "timed out waiting for completed startup and stable paused physics: "
        + json.dumps(last, sort_keys=True)
    )


def top_level_success(path):
    try:
        with open(path, encoding="utf-8") as stream:
            document = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise RunnerFailure("acceptance JSON is unreadable: %s" % error)
    if not isinstance(document, dict):
        raise RunnerFailure("acceptance JSON root must be an object")
    success = document.get("success")
    if type(success) is not bool:
        raise RunnerFailure(
            "acceptance JSON top-level success must be a boolean"
        )
    return success


def check_result(arguments):
    if not top_level_success(arguments.result_json):
        print("indoor-start acceptance reported top-level failure",
              file=sys.stderr)
        return 1
    return 0


def parse_arguments():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    ready = subparsers.add_parser("wait-ready")
    ready.add_argument("--simulation-log", required=True)
    ready.add_argument("--simulation-pid", type=int, required=True)
    ready.add_argument("--controller-pid-file", required=True)
    ready.add_argument("--timeout", type=float, default=300.0)
    ready.add_argument("--poll-interval", type=float, default=0.10)
    ready.add_argument("--stable-observations", type=int, default=3)
    ready.set_defaults(function=wait_for_ready)

    result = subparsers.add_parser("check-result")
    result.add_argument("result_json")
    result.set_defaults(function=check_result)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    try:
        return arguments.function(arguments)
    except RunnerFailure as error:
        print("indoor-start runner failed closed: %s" % error,
              file=sys.stderr)
        return 74


if __name__ == "__main__":
    raise SystemExit(main())
