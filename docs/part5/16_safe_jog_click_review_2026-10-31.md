# Windows SAFE JOG click review — 2026-10-31

Base: PR #33 head `bfbb224009cccb2bf83131d68102cc964207bd92`.
PR must remain Draft; this review does not close the intermittent GUI issue.

## Evidence

- Uploaded `stage5-j6-manual-evidence.zip`: independent acceptance PASS,
  four initial segments, one J6 +0.5 degree jog and one completed C/UART
  recovery; final HOLD error 0.02 degree, zero firmware faults.
- Local `build/stage5-manual-20261031-codex-retry`: four initial segments,
  no recorded jog or recovery, then GUI disconnect at `addUserDebugText`.
  Last native health probe reported `hung=0`; UART continued until disconnect.
  This does not establish the cause of the reported frozen window.
- Actual mouse reproduction on unchanged base (`build/stage5-click-diagnosis`):
  J1 +2 degrees followed by J6 +0.5 degree, real C/UART recoveries and FINISH
  all succeeded. Independent acceptance PASS. No scripted CI jog injection.
- Native OpenGL2 window did not show debug status text. Motion can complete
  without any persistent visible completion feedback.

## Changes

Expose WAIT/READY/BUSY/REFUSED/RETURN COMPLETE and action counts in the Windows
caption, with a bounded 50 ms native message and rate-limited updates. Keep
the scene status text for renderers that support it. Caption updates are
presentation only and cannot authorize motion or a PASS.

Record a click receipt before selection queries and collision/C preview,
and log input poll activity every five seconds. A subsequent failure can
distinguish no received click from a call blocked after receipt. Correct the
HOLD instruction to describe the selected 0.5–2 degree step.

No firmware, collision clearance, joint limits, controller or UART changes.

## Verification

- Guarded jog tests: 15 passed, including receipt retained on native preflight
  failure and no completion indication before real recovery.
- Visual/control bridge tests: 36 passed.
- Live GUI tests: 24 passed.
- Acceptance verifier tests: 5 passed.
- Final actual Windows mouse run (`build/stage5-caption-j6-verified`): select
  J6 and 0.5 degree, click +, observe BUSY then RETURN COMPLETE in caption,
  click FINISH, orderly exit. Independent acceptance PASS, applied=completed=1,
  final HOLD error 0.02 degree and all firmware fault counters zero.

An initial restricted launch could not connect to QMP. The same launch with
local process networking available succeeded. This startup failure is separate
from the reported post-click behavior. Socket-based regression tests also
require local networking.

The earlier intermittent freeze remains unproven and unresolved; do not describe
the caption/diagnostic fix or one successful native run as a deadlock fix.
