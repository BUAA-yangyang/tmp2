#!/usr/bin/env python3
"""A floor may not be closed by a room the transaction gave up on.

mf13 declared "completed 4/4 distinct rooms" on floor 1 while its own log
carried "room transaction could not bound the room; leaving". The transaction
set last_room_transaction_proven=False and the caller never read it, so an
abandoned room closed the floor. These tests pin the rule that replaced it.
"""
import unittest

from a1_exploration.frontier import room_completion_state


class RoomCompletionStateTest(unittest.TestCase):
    TARGET = 4
    ATTEMPTS = 2

    def state(self, completed, unproven):
        return room_completion_state(
            set(completed), dict(unproven), self.TARGET, self.ATTEMPTS
        )

    def test_quota_of_proven_rooms_closes_the_floor(self):
        complete, revivable = self.state([1, 2, 3, 4], {})
        self.assertTrue(complete)
        self.assertEqual(revivable, [])

    def test_an_unproven_room_does_not_close_the_floor(self):
        # The mf13 case: four rooms covered, one of them abandoned on budget.
        complete, revivable = self.state([1, 2, 3, 4], {3: 1})
        self.assertFalse(complete)
        self.assertEqual(revivable, [3])

    def test_unproven_room_counts_once_its_revisits_are_spent(self):
        complete, revivable = self.state([1, 2, 3, 4], {3: self.ATTEMPTS})
        self.assertTrue(complete)
        self.assertEqual(revivable, [])

    def test_unproven_room_outside_the_quota_is_not_revivable(self):
        # A branch that was never completed cannot be revived through this path.
        complete, revivable = self.state([1, 2, 3, 4], {9: 0})
        self.assertTrue(complete)
        self.assertEqual(revivable, [])

    def test_short_of_quota_is_never_complete(self):
        for unproven in ({}, {2: 0}, {2: 5}):
            complete, _revivable = self.state([1, 2, 3], unproven)
            self.assertFalse(complete)

    def test_revivable_order_is_deterministic(self):
        _complete, revivable = self.state([1, 2, 3, 4], {4: 0, 2: 1, 3: 0})
        self.assertEqual(revivable, [2, 3, 4])

    def test_disabled_quota_never_completes(self):
        for target in (0, None, -1):
            complete, revivable = room_completion_state(
                {1, 2, 3, 4}, {}, target, self.ATTEMPTS
            )
            self.assertFalse(complete)
            self.assertEqual(revivable, [])


if __name__ == "__main__":
    unittest.main()
