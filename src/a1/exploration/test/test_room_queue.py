"""Rooms are a persistent work queue, not a window that slides with travel.

run12 opened with a 21.68 m corridor frontier. The old gate kept only doorways
inside [maximum_corridor_progress - backtrack, robot + lookahead], and that mark
is a monotonic high-water mark pushed by whatever the robot last chased, so one
long goal evicted every room behind it and the corridor had to be re-walked.
"""

import unittest

from a1_exploration.frontier import room_queue_order


STATION_10_LEFT = (10, 1)
STATION_10_RIGHT = (10, -1)
STATION_20_LEFT = (20, 1)
STATION_30_LEFT = (30, 1)


def candidates():
    return [
        (STATION_20_LEFT, 20.0, 1.5),
        (STATION_10_LEFT, 10.0, 1.5),
        (STATION_10_RIGHT, 10.0, -1.5),
        (STATION_30_LEFT, 30.0, 1.5),
    ]


class RoomQueueOrderTest(unittest.TestCase):
    def test_serves_rooms_outward_from_the_entrance(self):
        self.assertEqual(
            room_queue_order(candidates(), set()),
            [
                STATION_10_LEFT,
                STATION_10_RIGHT,
                STATION_20_LEFT,
                STATION_30_LEFT,
            ],
        )

    def test_left_precedes_right_at_one_door_station(self):
        order = room_queue_order(candidates(), set())
        self.assertLess(
            order.index(STATION_10_LEFT), order.index(STATION_10_RIGHT)
        )

    def test_only_proven_coverage_removes_a_branch(self):
        order = room_queue_order(candidates(), {STATION_10_LEFT})
        self.assertNotIn(STATION_10_LEFT, order)
        self.assertEqual(order[0], STATION_10_RIGHT)

    def test_travel_never_evicts_a_room_behind_the_robot(self):
        # The regression itself: the robot has run to longitudinal 21 m, past
        # every station-10 doorway. Under the old window both were dropped;
        # they must still be queued, and still be served first.
        order = room_queue_order(candidates(), set())
        self.assertEqual(order[0], STATION_10_LEFT)
        self.assertIn(STATION_10_RIGHT, order)

    def test_a_room_far_ahead_is_queued_but_not_served_first(self):
        order = room_queue_order(candidates(), set())
        self.assertIn(STATION_30_LEFT, order)
        self.assertEqual(order[-1], STATION_30_LEFT)

    def test_empty_queue_when_every_branch_is_complete(self):
        complete = {
            STATION_10_LEFT,
            STATION_10_RIGHT,
            STATION_20_LEFT,
            STATION_30_LEFT,
        }
        self.assertEqual(room_queue_order(candidates(), complete), [])

    def test_no_candidates_is_an_empty_queue(self):
        self.assertEqual(room_queue_order([], set()), [])


if __name__ == "__main__":
    unittest.main()
