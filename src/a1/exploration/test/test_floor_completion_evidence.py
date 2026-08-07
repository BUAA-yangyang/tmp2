#!/usr/bin/env python3
"""A floor must not be declared explored on two seconds of evidence.

Pins the mf72 floor-2 measurement: ENTERED_FLOOR at sim 735.40,
EXPLORATION_DONE at 737.30 -- 1.9 s, three distinct map contents, 8.7 % of the
ROI observed, 0/4 room transactions, and the three danger sources on that floor
20.5 / 27.0 / 29.1 m from anywhere the robot ever stood. That was the entire
14-point recognition score for the run.
"""
import unittest

from a1_exploration.frontier import NoFrontierEvidence


class MinimumEvidenceDurationTest(unittest.TestCase):
    def test_distinct_contents_alone_do_not_finish_a_floor(self):
        """The mf72 floor-2 shape: three distinct contents inside two seconds."""
        evidence = NoFrontierEvidence(3, 10.0, 10.0)
        result = None
        for index, now in enumerate((735.50, 736.10, 736.30, 737.20)):
            result = evidence.observe("content-%d" % index, now)
            self.assertFalse(
                result["complete"],
                "floor completed after %.2f s on %d distinct contents"
                % (now - 735.50, result["count"]),
            )
        self.assertGreaterEqual(result["count"], 3)
        self.assertIn("two seconds of evidence", result["reason"])

    def test_same_evidence_completes_once_it_has_lasted(self):
        evidence = NoFrontierEvidence(3, 10.0, 10.0)
        for index, now in enumerate((735.50, 736.10, 736.30, 737.20)):
            evidence.observe("content-%d" % index, now)
        result = evidence.observe("content-later", 745.55)
        self.assertTrue(result["complete"])
        self.assertGreaterEqual(result["elapsed"], 10.0)

    def test_a_changing_fingerprint_cannot_deadlock_completion(self):
        """Why this is a minimum *duration* and not an AND with the dwell.

        ``version`` is a content fingerprint and a live LiDAR map differs from
        frame to frame even with the robot parked, so every observation can be
        distinct. Requiring the stable dwell as well would reset stable_since
        on every frame and the floor would never complete at all -- a hang is
        worse than the bug being fixed. A monotone timer cannot do that.
        """
        evidence = NoFrontierEvidence(3, 10.0, 10.0)
        now = 100.0
        completed_at = None
        for index in range(400):          # 40 s of never-repeating content
            now += 0.1
            result = evidence.observe("unique-%d" % index, now)
            self.assertEqual(result["stable_for"], 0.0)
            if result["complete"]:
                completed_at = now
                break
        self.assertIsNotNone(
            completed_at, "completion deadlocked on an ever-changing map"
        )
        self.assertAlmostEqual(completed_at - 100.1, 10.0, delta=0.15)

    def test_progress_restarts_the_evidence_clock(self):
        evidence = NoFrontierEvidence(3, 10.0, 10.0)
        for index in range(5):
            evidence.observe("content-%d" % index, 200.0 + index)
        evidence.reset()                   # a target was dispatched
        result = evidence.observe("content-fresh", 260.0)
        self.assertFalse(result["complete"])
        self.assertEqual(result["elapsed"], 0.0)

    def test_zero_minimum_duration_preserves_the_old_behaviour(self):
        evidence = NoFrontierEvidence(3, 10.0, 0.0)
        for index, now in enumerate((735.50, 736.10, 736.30)):
            result = evidence.observe("content-%d" % index, now)
        self.assertTrue(result["complete"])

    def test_minimum_duration_must_be_non_negative(self):
        with self.assertRaises(ValueError):
            NoFrontierEvidence(3, 10.0, -1.0)


if __name__ == "__main__":
    unittest.main()
