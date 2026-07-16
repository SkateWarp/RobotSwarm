#!/usr/bin/env python3
"""Helpers for stable, numeric TurtleBot namespace ordering."""

import re
from typing import Iterable, List, Tuple


_ROBOT_ID_PATTERN = re.compile(r"^(?P<prefix>[A-Za-z][A-Za-z0-9_-]*?)(?P<index>\d+)$")


def robot_id_sort_key(robot_id: str) -> Tuple[str, int, str]:
    """Sort ``tb3_2`` before ``tb3_10`` while remaining deterministic."""
    value = str(robot_id).strip()
    match = _ROBOT_ID_PATTERN.match(value)
    if match is None:
        return value, -1, value
    return match.group("prefix"), int(match.group("index")), value


def sort_robot_ids(robot_ids: Iterable[str]) -> List[str]:
    """Return unique, non-empty robot IDs in stable numeric order."""
    normalized = {str(robot_id).strip() for robot_id in robot_ids if str(robot_id).strip()}
    return sorted(normalized, key=robot_id_sort_key)


def validate_robot_ids(robot_ids: Iterable[str]) -> List[str]:
    """Validate caller-supplied TurtleBot runtime namespaces."""
    normalized = [str(robot_id).strip() for robot_id in robot_ids]
    if any(not robot_id for robot_id in normalized):
        raise ValueError("robot IDs must not be empty")
    if len(set(normalized)) != len(normalized):
        raise ValueError("robot IDs must be unique")
    invalid = [robot_id for robot_id in normalized if not re.match(r"^tb3_\d+$", robot_id)]
    if invalid:
        raise ValueError(
            "robot IDs must use the tb3_<number> namespace format: {}".format(
                ", ".join(invalid)
            )
        )
    return sort_robot_ids(normalized)
