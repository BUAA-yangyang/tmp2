"""Strict parser for the competition-public team_scene_info document."""

import json


class PublicSceneError(ValueError):
    """Raised when public scene information cannot safely authorize a behavior."""


def load_public_scene(path):
    try:
        with open(path, "r", encoding="utf-8") as stream:
            document = json.load(stream)
    except (OSError, ValueError) as error:
        raise PublicSceneError(
            "cannot read public team scene information: %s" % error
        )
    if not isinstance(document, dict):
        raise PublicSceneError("team scene information must be a JSON object")
    return document


def resolve_entry_door(document, floor_id, requested_id=""):
    """Resolve a public main-entrance door without consulting private metadata."""
    if not isinstance(document, dict):
        raise PublicSceneError("team scene information must be a JSON object")
    if document.get("schema") != "team_scene_info_v1":
        raise PublicSceneError("unsupported or missing team_scene_info schema")

    allowed = document.get("allowed_interfaces")
    services = allowed.get("services") if isinstance(allowed, dict) else None
    if not isinstance(services, list) or "/set_door_state" not in services:
        raise PublicSceneError(
            "public scene does not allow /set_door_state"
        )

    public_scene = document.get("public_scene")
    doors = (
        public_scene.get("door_ids")
        if isinstance(public_scene, dict) else None
    )
    if not isinstance(doors, list):
        raise PublicSceneError("public_scene.door_ids must be a list")

    candidates = []
    for door in doors:
        if not isinstance(door, dict):
            continue
        identifier = door.get("id")
        door_floor = door.get("floor_index")
        kind = door.get("kind")
        if not isinstance(identifier, str) or not identifier:
            continue
        if requested_id:
            matches = identifier == requested_id
        else:
            matches = kind == "main_entrance" and door_floor == int(floor_id)
        if matches:
            candidates.append(door)

    if len(candidates) != 1:
        description = (
            "id %r" % requested_id
            if requested_id else "main_entrance on floor %d" % int(floor_id)
        )
        raise PublicSceneError(
            "expected exactly one public door for %s, found %d"
            % (description, len(candidates))
        )

    door = candidates[0]
    if door.get("floor_index") != int(floor_id):
        raise PublicSceneError(
            "public door %s belongs to floor %r, not %d"
            % (door["id"], door.get("floor_index"), int(floor_id))
        )
    if door.get("kind") != "main_entrance":
        raise PublicSceneError(
            "entry behavior only accepts a public main_entrance door"
        )
    if door.get("initial_open") is not True:
        raise PublicSceneError(
            "public main entrance is not declared initial_open=true"
        )
    return door["id"]
