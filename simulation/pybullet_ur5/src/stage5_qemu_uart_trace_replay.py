"""录制并回放 QEMU/FreeRTOS 固件控制下的 PyBullet 实测六轴运动。

录制：真实 QEMU + UART + PyBullet DIRECT 闭环产生 joint_trace.csv。
回放：GUI 逐帧显示 *已记录的实际关节位置*，不重新计算轨迹或 PID。

这两步故意分离：GUI 绘制不能拖慢 UART 反馈，破坏 FreeRTOS 控制时序。
回放不是第二次动态闭环仿真，也不证明 GUI 模式下实时控制通过。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

from stage4_dynamic_closed_loop import JOINT_NAMES, ROOT, URDF, joint_indices
from stage5_qemu_uart_closed_loop import PHYSICS_DT_S
from stage5_qemu_uart_visual_demo import DemoOptions, run


MIN_VISIBLE_RANGE_DEG = 2.0  # 仅用于演示的实测运动幅度检查。


def load_trace(path: Path) -> list[tuple[float, int, tuple[float, ...]]]:
    """读取连续实测姿态。时间单位 s；六轴单位 deg；拒绝损坏记录。"""
    if not path.is_file():
        raise FileNotFoundError(f"Measured trace not found: {path}")
    expected = {"simulated_time_s", "segment"}
    expected.update(f"actual_{name}_deg" for name in JOINT_NAMES)
    frames: list[tuple[float, int, tuple[float, ...]]] = []
    previous_time = -math.inf

    with path.open("r", encoding="utf-8-sig", newline="") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None or not expected.issubset(reader.fieldnames):
            raise ValueError("Measured trace is missing required CSV columns")
        for line_no, row in enumerate(reader, start=2):
            try:
                stamp = float(row["simulated_time_s"])
                segment = int(row["segment"])
                angles = tuple(float(row[f"actual_{name}_deg"]) for name in JOINT_NAMES)
            except (ValueError, TypeError, KeyError) as exc:
                raise ValueError(f"Invalid CSV row {line_no}") from exc
            if (not math.isfinite(stamp) or stamp <= previous_time or
                    segment < 0 or not all(math.isfinite(x) for x in angles)):
                raise ValueError(f"Non-monotonic time or invalid angles at row {line_no}")
            previous_time = stamp
            frames.append((stamp, segment, angles))
    if len(frames) < 2:
        raise ValueError("Measured trace has fewer than two frames")
    return frames


def measured_ranges(frames: list[tuple[float, int, tuple[float, ...]]]) -> tuple[float, ...]:
    """逐关节统计实测运动幅度，不用目标姿态冒充运动证据。"""
    return tuple(
        max(f[2][joint] for f in frames) - min(f[2][joint] for f in frames)
        for joint in range(len(JOINT_NAMES))
    )


def record_closed_loop(elf: Path, output_dir: Path, scenario: str,
                       qemu_bin: str, timeout_s: float) -> None:
    """复用已有完整闭环测试；不触碰固件、PID、串口协议。"""
    print("[CAPTURE] QEMU + FreeRTOS + UART + PyBullet DIRECT...")
    options = DemoOptions(
        scenario=scenario, headless=True, frame_delay_s=0.0,
        timeout_s=timeout_s, hold_window=False,
    )
    result = run(elf, output_dir, qemu_bin, options)
    if not result["passed"]:
        raise RuntimeError("Closed-loop capture FAILED; see result.json")
    print(f"[CAPTURE] PASS, segments={result['segments_settled']}/"
          f"{result['segments_requested']}, frames={result['target_frames']}")


def replay(frames: list[tuple[float, int, tuple[float, ...]]],
           speed: float, repeats: int, hold: bool) -> None:
    """按 CSV 的仿真时间和实测关节角刷新 GUI；不模拟第二个控制器。"""
    import pybullet as p

    client = p.connect(p.GUI)
    if client < 0:
        raise RuntimeError("PyBullet GUI failed to open")
    try:
        p.setRealTimeSimulation(0)
        p.setGravity(0, 0, 0)  # 回放阶段不再施加动力学扰动。
        p.setTimeStep(PHYSICS_DT_S)
        arm = p.loadURDF(str(URDF), useFixedBase=True)
        joints = joint_indices(p, arm)
        p.setJointMotorControlArray(
            arm, joints, p.VELOCITY_CONTROL, forces=[0.0] * len(joints)
        )
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5, cameraYaw=50, cameraPitch=-27,
            cameraTargetPosition=[0.0, 0.0, 0.35],
        )
        for cycle in range(repeats):
            last_tool = None
            text_id = -1
            previous_timestamp = frames[0][0]
            for i, (timestamp, segment, angles) in enumerate(frames):
                frame_started = time.monotonic()
                for joint, angle_deg in zip(joints, angles):
                    p.resetJointState(arm, joint, math.radians(angle_deg))
                # 显式触发 GUI 物理服务刷新；展示位置由上面的实测角决定。
                p.stepSimulation()
                if i % 5 == 0:
                    tool = p.getLinkState(
                        arm, p.getNumJoints(arm) - 1,
                        computeForwardKinematics=True,
                    )[4]
                    if last_tool is not None:
                        p.addUserDebugLine(
                            last_tool, tool, [0.1, 0.7, 1.0],
                            lineWidth=2, lifeTime=120,
                        )
                    last_tool = tool
                if i % 20 == 0:
                    text_id = p.addUserDebugText(
                        f"QEMU measured | segment {segment}/3 | "
                        f"repeat {cycle + 1}/{repeats}",
                        [-0.7, -0.7, 0.9],
                        textColorRGB=[0.1, 0.9, 0.2],
                        replaceItemUniqueId=text_id,
                    )
                wait = max(0.0, (timestamp - previous_timestamp) / speed)
                time.sleep(max(0.0, wait - (time.monotonic() - frame_started)))
                previous_timestamp = timestamp
            if cycle + 1 < repeats:
                time.sleep(1)
        if hold:
            try:
                input("[GUI] 回放完成，按 Enter 关闭窗口...")
            except EOFError:
                pass
    finally:
        p.disconnect(client)


def main() -> int:
    """命令行入口：先录制再回放，或只回放以前的合格录制。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--elf", type=Path,
        default=ROOT / "build/stage5-local-uart/firmware/freertos_demo/freertos_demo.elf",
    )
    parser.add_argument("--qemu", default="qemu-system-arm")
    parser.add_argument("--scenario", choices=("six_axis", "joint_sweep"),
                        default="six_axis")
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "build/stage5-visual-six-axis")
    parser.add_argument("--replay-only", action="store_true",
                        help="Replay existing verified result.json and joint_trace.csv")
    parser.add_argument("--verify-only", action="store_true",
                        help="Verify recorded frames without opening GUI")
    parser.add_argument("--speed", type=float, default=0.35,
                        help="Playback speed relative to recorded physics time")
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--timeout-s", type=float, default=210.0)
    parser.add_argument("--no-hold", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0.05 <= args.speed <= 4.0:
        parser.error("--speed must be between 0.05 and 4.0")
    if not 1 <= args.repeats <= 20:
        parser.error("--repeats must be between 1 and 20")

    if not args.replay_only:
        record_closed_loop(args.elf, args.output_dir, args.scenario,
                           args.qemu, args.timeout_s)
    report_path = args.output_dir / "result.json"
    if not report_path.is_file():
        raise FileNotFoundError(report_path)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if (report.get("passed") is not True or
            report.get("segments_settled") != report.get("segments_requested")):
        raise RuntimeError("Refusing to replay an unverified or incomplete run")

    frames = load_trace(args.output_dir / "joint_trace.csv")
    ranges = measured_ranges(frames)
    print("[MEASURED] six-joint motion ranges (deg): "
          + ", ".join(f"J{i + 1}={value:.2f}" for i, value in enumerate(ranges)))
    if max(ranges) < MIN_VISIBLE_RANGE_DEG:
        raise RuntimeError("Robot barely moved in recorded physics (<2 deg)")
    if args.verify_only:
        print(f"[VERIFIED] {len(frames)} measured samples; no GUI opened")
        return 0
    print(f"[GUI] Playing {len(frames)} measured frames, "
          f"{args.speed:.2f}x, {args.repeats} repeats")
    replay(frames, args.speed, args.repeats, not args.no_hold)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
