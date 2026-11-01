"""Isolate the real C PID/limiter + native PyBullet from QEMU/UART.

Reuses the user's first accepted firmware APF waypoint as a test fixture.
No firmware or PID parameters are overridden and no environments are changed.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT / "simulation" / "pybullet_ur5" / "src"
sys.path.insert(0, str(SIM))
from stage4_dynamic_closed_loop import (  # noqa: E402
    START_RAW, URDF, IntJoints, joint_indices, angles_raw, load_bridge,
)
from stage5_avoidance_uart_gate import load_scene  # noqa: E402

# Test-only fixture taken directly from the first ARM APF accepted joint proposal.
FIRST_ARM_APF_TARGET_RAW = (3030, -4652, 6131, 1831, 525, -809)
SETTLE_ERROR_DEG = 0.15  # Existing Stage5 acceptance criterion, not C config.
REQUIRED_SETTLE_FRAMES = 15


def run(library: Path, output_dir: Path, frames: int) -> dict:
    import pybullet as p

    output_dir.mkdir(parents=True, exist_ok=True)
    bridge = load_bridge(library)
    sim_handle = bridge.stage4_sim_create(
        IntJoints(*START_RAW), IntJoints(*FIRST_ARM_APF_TARGET_RAW))
    if not sim_handle:
        raise RuntimeError("Production C bridge rejected the Stage5 test fixture")

    client = -1
    result = {
        "passed": False,
        "scope": "production_C_PID_limiter_native_PyBullet_no_QEMU",
        "target_raw": list(FIRST_ARM_APF_TARGET_RAW),
        "frames": 0,
        "settled_frames": 0,
        "min_error_deg": None,
        "last_feedback_raw": None,
        "last_command_raw": None,
        "controller": "unmodified_stage4_sim_bridge",
    }
    try:
        client = p.connect(p.DIRECT)
        if client < 0:
            raise RuntimeError("PyBullet DIRECT not available")
        p.setGravity(0, 0, -9.81, physicsClientId=client)
        p.setTimeStep(0.001, physicsClientId=client)
        arm = p.loadURDF(str(URDF), useFixedBase=True, physicsClientId=client)
        joints = joint_indices(p, arm)
        for index, value in zip(joints, START_RAW):
            p.resetJointState(
                arm, index, math.radians(value / 100),
                physicsClientId=client,
            )
        efforts = [
            float(p.getJointInfo(arm, j, physicsClientId=client)[10])
            for j in joints
        ]
        p.setJointMotorControlArray(
            arm, joints, p.POSITION_CONTROL,
            targetPositions=[math.radians(v / 100) for v in START_RAW],
            forces=efforts, physicsClientId=client,
        )
        for _ in range(100):
            p.stepSimulation(physicsClientId=client)

        # Same blocked scene's static box as the embedded APF UART test.
        scene, _ = load_scene("blocked")
        center = scene["obstacle"]["center_offset_ee_mm"]
        ee = p.getLinkState(
            arm, p.getNumJoints(arm, physicsClientId=client) - 1,
            computeForwardKinematics=True, physicsClientId=client,
        )[4]
        shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=scene["obstacle"]["half_extents_m"],
            physicsClientId=client,
        )
        p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=shape,
            basePosition=[ee[i] + center[i] / 1000 for i in range(3)],
            physicsClientId=client,
        )

        best = math.inf
        consecutive = 0
        for index in range(frames):
            observed = angles_raw(p, arm, joints)
            commands = IntJoints()
            trajectory = IntJoints()
            status = bridge.stage4_sim_step(
                sim_handle, IntJoints(*observed),
                index * 0.01, 0.01, commands, trajectory,
            )
            if status != 0:
                raise RuntimeError(
                    f"Production C stage4_sim_step returned {status} "
                    f"at frame {index}"
                )
            p.setJointMotorControlArray(
                arm, joints, p.POSITION_CONTROL,
                targetPositions=[
                    math.radians(commands[j] / 100) for j in range(6)
                ],
                forces=efforts, physicsClientId=client,
            )
            for _ in range(10):
                p.stepSimulation(physicsClientId=client)
            feedback = angles_raw(p, arm, joints)
            error = max(
                abs(feedback[j] - FIRST_ARM_APF_TARGET_RAW[j]) / 100
                for j in range(6)
            )
            best = min(best, error)
            consecutive = consecutive + 1 if error <= SETTLE_ERROR_DEG else 0
            result.update(
                frames=index + 1,
                settled_frames=consecutive,
                min_error_deg=best,
                last_max_error_deg=error,
                last_feedback_raw=list(feedback),
                last_command_raw=[commands[j] for j in range(6)],
                last_reference_raw=[trajectory[j] for j in range(6)],
            )
            if (index + 1) % 100 == 0:
                print(
                    f"[Stage5 host Bullet] frame={index + 1} "
                    f"error={error:.3f}deg best={best:.3f}deg "
                    f"settled={consecutive}/15", flush=True,
                )
            if consecutive >= REQUIRED_SETTLE_FRAMES:
                result["passed"] = True
                break
    except Exception as exc:
        result["error"] = repr(exc)
        raise
    finally:
        if client >= 0:
            p.disconnect(physicsClientId=client)
        bridge.stage4_sim_destroy(sim_handle)
        (output_dir / "result.json").write_text(
            json.dumps(result, indent=2), encoding="utf-8",
        )
    print(json.dumps(result, indent=2))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--frames", type=int, default=800)
    args = parser.parse_args()
    if not args.library.is_file():
        parser.error(f"Missing C bridge: {args.library}")
    if not 20 <= args.frames <= 3000:
        parser.error("--frames must be between 20 and 3000")
    result = run(args.library, args.output_dir, args.frames)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
