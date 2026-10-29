# Stage5/6: FreeRTOS feedback / reference tick ordering (Issue #22)

## Fault evidence, not a timeout workaround

The original real Cortex-M4 FreeRTOS → QEMU UART → PyBullet blocked route
reported `ROBOT_STATUS_ERROR_NOT_READY (-7)` after a previously accepted
joint request. The firmware had `planner_faults=1`, `controller_faults=0`,
and `feedback_stale_count=0`. Increasing the QEMU wall-time limit,
shortening a 26-segment path to four, and modifying the collision fixture
did not remove the intermittent abort. The failure is recorded in
[Issue #22](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/issues/22).

## Concurrency defect and ownership

The original Trajectory Task did:

```c
TickType_t now = xTaskGetTickCount(); // sampled first
motor_state_get_feedback(&observation); // mutex read follows
motor_state_feedback_is_fresh(&observation, now, max_age);
```

ProtocolRX is the feedback's sole writer. If it commits a newer
`accepted_tick` between the first two operations, unsigned 32-bit
`now - accepted_tick` underflows. The computed age is almost 2^32
ticks even though the feedback has just been accepted; the task
incorrectly cancels the motion as NOT_READY.

The production `motor_state_get_fresh_feedback()` gateway now obtains the
mutex-protected feedback snapshot first, then calls `xTaskGetTickCount()`
and validates the configured timeout. It returns a **complete validated
observation or no output**, without inventing a second feedback cache,
timer, set of defaults, or UART codec. Both Trajectory Task and Control
Task use the gateway. Control additionally samples a *post-queue* tick
for its reference freshness, because a reference written by Trajectory
between its original period-start tick and its queue receive creates
the same underflow risk. The original task tick is retained for
PID `dt`, preserving task scheduling semantics.

A deterministic host harness builds the **real** `application/motor_state.c`
against minimal FreeRTOS mocks, reproduces the newer-feedback stamp
than the old time sample, validates actual stale rejection, output
atomicity, and legitimate tick rollover. Native firmware and ARM QEMU
builds still use the actual FreeRTOS headers, not the mocks.

## Physical regression evidence

[Stage5 blocked feedback regression #40880542267](https://github.com/Lemon-Li-x86/six-axis-robot-motion-control/actions/runs/40880542267)
successfully executed the scanned static-box fixture without modifying
the PID, feedback timeout (3000 ms), or 10 mm full-link minimum
clearance:

- Real C APF raw route **28** Cartesian points, compressed to **4**
  after repeated real C IK and PyBullet full-link safety gates.
- The direct Cartesian segment was blocked by the actual PyBullet AABB.
- **4/4** segments settled, **947** physical collision scans,
  `planner_faults=0`, `controller_faults=0`,
  `feedback_stale_count=0`, `last_fault=0`.
- Joint segment ending errors: 0.07°, 0.05°, 0.06°, 0.06°.
- No continuous-time swept-volume, self-collision, dynamic obstacle,
  native Windows GUI, or real hardware safety certification is implied.

The CI above predates the additional Control post-reference tick fix.
The final branch commit must pass **its own** test suite before merge.
The four-segment pass is strong evidence that this race matters, but
hardware/QEMU clock-policy changes and longer mission loads need separate
Stage6 stress/exception tests. Do not remove the nonzero fault fail-fast
check on the simulator or weaken RTOS feedback freshness.
