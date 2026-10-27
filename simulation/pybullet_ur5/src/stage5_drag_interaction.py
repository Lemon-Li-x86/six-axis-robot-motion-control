"""Mouse intent transport and physical drag for the live UART simulation.

Only DIRECT owns dynamics. The GUI sends a point constraint, never joint angles,
firmware setpoints or fabricated feedback. Display and input are lossy mailboxes.
"""
from __future__ import annotations

import math
import json
import time

DRAG_LEASE_S = 0.4
DRAG_FORCE_N = 2000.0  # Virtual mouse fixture; not a firmware or hardware limit.
MAX_DRAG_DISTANCE_M = 0.35


class DragMailbox:
    def __init__(self, context):
        # monotonic timestamp, active, link, local pivot xyz, world target xyz
        self.values = context.Array("d", 9, lock=False)
        self.lock = context.Lock()
        self.dropped = context.Value("L", 0, lock=False)

    def publish(self, active, link, pivot, target, now=None):
        values = (time.monotonic() if now is None else now, float(active), link,
                  *pivot, *target)
        if (len(values) != 9 or not all(math.isfinite(v) for v in values)
                or link < -1 or type(link) is not int):
            raise ValueError("Invalid virtual mouse drag")
        if not self.lock.acquire(False):
            self.dropped.value += 1
            return False
        try:
            self.values[:] = values
        finally:
            self.lock.release()
        return True

    def read(self, now=None):
        if not self.lock.acquire(False):
            return None
        try:
            values = tuple(self.values)
        finally:
            self.lock.release()
        age = (time.monotonic() if now is None else now) - values[0]
        active = bool(values[1]) and 0 <= age <= DRAG_LEASE_S
        return active, int(values[2]), values[3:6], values[6:9]


def screen_ray(camera, x, y):
    """Unproject GUI pixels using the actual OpenGL view/projection matrices."""
    import numpy as np
    width, height, view, projection = camera[:4]
    inverse = np.linalg.inv(np.array(projection).reshape(4, 4, order="F") @
                            np.array(view).reshape(4, 4, order="F"))
    def unproject(depth):
        point = inverse @ np.array((2*x/width-1, 1-2*y/height, depth, 1.0))
        return tuple(point[:3] / point[3])
    return unproject(-1), unproject(1)


def plane_target(start, end, point, normal):
    direction = tuple(b-a for a, b in zip(start, end))
    denominator = sum(a*b for a, b in zip(direction, normal))
    if abs(denominator) < 1e-9:
        return point
    fraction = sum((p-a)*n for p, a, n in zip(point, start, normal)) / denominator
    hit = tuple(a + fraction*d for a, d in zip(start, direction))
    delta = tuple(h-p for h, p in zip(hit, point))
    length = math.sqrt(sum(v*v for v in delta))
    scale = min(1.0, MAX_DRAG_DISTANCE_M / max(length, 1e-9))
    return tuple(p+v*scale for p, v in zip(point, delta))


class MouseDrag:
    def __init__(self, mailbox, audit_path=None):
        self.mailbox = mailbox
        self.audit_path = audit_path
        self.active = False
        self.link = -1
        self.pivot = (0., 0., 0.)
        self.target = self.pivot
        self.anchor = self.pivot
        self.normal = (0., 0., 1.)
        self.release_after = None

    def poll(self, bullet, arm):
        events = bullet.getMouseEvents()
        now = time.monotonic()
        if self.release_after is not None and now >= self.release_after:
            self.active = False
            self.release_after = None
        released = False
        triggered = False
        hit_debug = None
        for kind, x, y, button, state in events:
            if kind == 2 and button == 0 and state & bullet.KEY_WAS_RELEASED:
                released = True
            if kind == 2 and button == 0 and state & bullet.KEY_WAS_TRIGGERED:
                triggered = True
                camera = bullet.getDebugVisualizerCamera()
                start, end = screen_ray(camera, x, y)
                hit = bullet.rayTest(start, end)[0]
                hit_debug = dict(camera_size=camera[:2], start=start, end=end, hit=hit)
                if hit[0] == arm and hit[1] >= 0:
                    self.link, self.anchor = hit[1], tuple(hit[3])
                    pose = bullet.getLinkState(arm, self.link, computeForwardKinematics=True)
                    inverse = bullet.invertTransform(pose[0], pose[1])
                    self.pivot = bullet.multiplyTransforms(*inverse, self.anchor, (0,0,0,1))[0]
                    self.normal = tuple(camera[5])
                    self.target = self.anchor
                    self.active = True
                    self.release_after = None
            elif kind == 1 and self.active:
                start, end = screen_ray(bullet.getDebugVisualizerCamera(), x, y)
                self.target = plane_target(start, end, self.anchor, self.normal)
        if released:
            if triggered and self.active:
                # Bullet can coalesce press+release (state bits 2|4) into one
                # input batch. Give that real mouse stroke a bounded physical
                # impulse instead of discarding it before the worker can read.
                self.release_after = now + 0.08
            else:
                self.active = False
        if events and self.audit_path is not None:
            with self.audit_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(dict(events=events, active=self.active, link=self.link,
                    target=self.target, pick=hit_debug)) + "\n")
        # Heartbeat: a paused/crashed GUI cannot leave a physical grab attached.
        self.mailbox.publish(self.active, self.link, self.pivot, self.target)


class PhysicalDrag:
    def __init__(self, mailbox):
        self.mailbox = mailbox
        self.constraint = None
        self.grab = None
        self.started = 0
        self.released = 0
        self.active = False
        self.last_read = 0.0

    def apply(self, bullet, arm):
        intent = self.mailbox.read()
        if intent is None:
            if time.monotonic()-self.last_read <= DRAG_LEASE_S:
                return
            intent = (False, -1, (0.,0.,0.), (0.,0.,0.))
        else:
            self.last_read = time.monotonic()
        active, link, pivot, target = intent
        if link < 0 or link >= bullet.getNumJoints(arm):
            active = False
        grab = (link, pivot)
        if self.constraint is not None and (not active or grab != self.grab):
            bullet.removeConstraint(self.constraint)
            self.constraint = None
            self.released += 1
        if active:
            if self.constraint is None:
                self.constraint = bullet.createConstraint(arm, link, -1, -1,
                    bullet.JOINT_POINT2POINT, (0,0,0), pivot, target)
                self.grab = grab
                self.started += 1
            bullet.changeConstraint(self.constraint, target, maxForce=DRAG_FORCE_N)
        self.active = active

