#!/usr/bin/env python3
import unittest

from a1_building_behavior.public_scene import (
    PublicSceneError,
    resolve_entry_door,
)


def public_document(initial_open=True):
    return {
        "schema": "team_scene_info_v1",
        "allowed_interfaces": {"services": ["/set_door_state"]},
        "public_scene": {
            "door_ids": [
                {
                    "floor_index": 0,
                    "id": "published-main-id",
                    "initial_open": initial_open,
                    "kind": "main_entrance",
                },
                {
                    "floor_index": 0,
                    "id": "elevator-floor-0",
                    "initial_open": True,
                    "kind": "elevator",
                },
            ]
        },
        "referee_only": {"forbidden_files": ["layout_metadata.json"]},
    }


class PublicSceneTest(unittest.TestCase):
    def test_resolves_main_entrance_from_public_door_list(self):
        self.assertEqual(
            resolve_entry_door(public_document(), 0),
            "published-main-id",
        )

    def test_explicit_id_is_still_validated_as_main_entrance(self):
        self.assertEqual(
            resolve_entry_door(
                public_document(), 0, requested_id="published-main-id"
            ),
            "published-main-id",
        )
        with self.assertRaises(PublicSceneError):
            resolve_entry_door(
                public_document(), 0, requested_id="elevator-floor-0"
            )

    def test_fails_closed_without_public_authorization(self):
        document = public_document()
        document["allowed_interfaces"]["services"] = []
        with self.assertRaises(PublicSceneError):
            resolve_entry_door(document, 0)

    def test_fails_closed_if_public_initial_state_is_not_open(self):
        with self.assertRaises(PublicSceneError):
            resolve_entry_door(public_document(initial_open=False), 0)

    def test_does_not_consume_referee_only_fields(self):
        document = public_document()
        document["referee_only"] = {
            "layout_metadata": {
                "door_ids": [{"id": "forbidden", "kind": "main_entrance"}]
            }
        }
        self.assertEqual(
            resolve_entry_door(document, 0),
            "published-main-id",
        )


if __name__ == "__main__":
    unittest.main()
