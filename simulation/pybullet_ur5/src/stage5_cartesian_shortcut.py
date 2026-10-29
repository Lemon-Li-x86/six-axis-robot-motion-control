"""Post-process production C APF waypoints with production C geometry only.

This is NOT another APF implementation. It removes only intermediate
Cartesian points proven redundant by robot_apf_segment_blocked(), retaining
the last waypoint of every explicitly specified task leg. The reduced
path is NOT executable until the caller repeats the real C IK/FK/rate
checks and actual PyBullet full-link collision preflight.

No PyBullet side effects, UART command or new robot-model constants.
"""
from __future__ import annotations

import ctypes
import math
from typing import Sequence

from stage5_static_obstacle_probe import AABB, Point


class InvalidShortcut(ValueError):
    pass


def shortcut_c_apf_legs(
    c,
    start: Point,
    points: Sequence[Point],
    leg_end_counts: Sequence[int],
    obstacles: Sequence[AABB],
    clearance_mm: float,
) -> list[Point]:
    """Greedy, leg-preserving visibility shortcuts using authoritative C geometry.

    leg_end_counts are cumulative exclusive point counts, e.g. [12, 26].
    Returns newly selected C Point objects; never writes to caller buffers.
    """
    if (not points or not leg_end_counts or not obstacles or
            not math.isfinite(clearance_mm) or clearance_mm < 0):
        raise InvalidShortcut("invalid C APF shortcut input")
    if (leg_end_counts[-1] != len(points) or
            any(type(end) is not int or end < 1
                for end in leg_end_counts) or
            any(a >= b for a, b in zip((0, *leg_end_counts[:-1]),
                                       leg_end_counts))):
        raise InvalidShortcut("leg boundaries must be strictly increasing")
    c.robot_apf_segment_blocked.restype = ctypes.c_int
    c.robot_apf_segment_blocked.argtypes = [
        ctypes.POINTER(Point), ctypes.POINTER(Point),
        ctypes.POINTER(AABB), ctypes.c_size_t,
        ctypes.c_double, ctypes.POINTER(ctypes.c_uint),
    ]
    boxes=(AABB * len(obstacles))(*obstacles)
    selected=[]
    previous=start
    cursor=0
    for end in leg_end_counts:
        while cursor < end:
            chosen=None
            for idx in range(end-1, cursor-1, -1):
                blocked=ctypes.c_uint()
                status=c.robot_apf_segment_blocked(
                    ctypes.byref(previous), ctypes.byref(points[idx]),
                    boxes, len(obstacles), clearance_mm,
                    ctypes.byref(blocked))
                if status:
                    raise InvalidShortcut(
                        f"C visibility check rejected candidate: status={status}")
                if not blocked.value:
                    chosen=idx
                    break
            if chosen is None:
                raise InvalidShortcut(
                    "even adjacent C APF points are no longer visible")
            previous=points[chosen]
            selected.append(previous)
            cursor=chosen+1
        # The last point of a task leg is always preserved. Never skip
        # a pick/place or explicit intermediate avoidance boundary.
    return selected
