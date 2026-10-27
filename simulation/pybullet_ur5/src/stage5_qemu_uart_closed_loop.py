"""Stage5 QEMU UART <-> real FreeRTOS PID <-> PyBullet closed loop.

This is an automated, SOFTWARE-ONLY integration exercise. It executes the
ARM firmware in QEMU and uses real serial bytes over QEMU's UART0 TCP chardev.
Unlike the Stage4 native-C host bridge, trajectory and PID are computed
INSIDE FreeRTOS, not in Python. Python owns only simulated motor physics.

Output is a virtual serial target frame, not a physical actuator command.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import subprocess
import time
from pathlib import Path

import qemu_uart_receiver as wire
from stage4_dynamic_closed_loop import (
    ROOT, URDF, JOINT_NAMES, START_RAW, joint_indices, angles_raw,
)

GOAL_RAW = (3000, -4400, 6000, 2000, 500, -1000)
MAX_WALL_DURATION_S = 50.0
MAX_TARGET_FRAMES = 130
PHYSICS_DT_S = 0.001
FEEDBACK_PERIOD_S = 0.01
MIN_JOINT_MOVEMENT_DEG = 0.20
MIN_ACCEPTED_COMMAND_CHANGE_DEG = 0.20
TIMEOUT_CONNECT_S = 15.0


def frame(command: int, payload: bytes) -> bytes:
    if len(payload) > 64:
        raise ValueError("Frame payload too long")
    body = bytes((command, len(payload))) + payload
    return wire.FRAME_HEADER + body + bytes((sum(body) & 0xFF,))


class FrameStream:
    """Transport framing only. Command semantics remain in the C firmware."""
    def __init__(self):
        self.buf = bytearray()
        self.bad_checksum = 0

    def consume(self, incoming: bytes):
        self.buf.extend(incoming)
        frames = []
        while True:
            start = self.buf.find(wire.FRAME_HEADER)
            if start < 0:
                # Keep a trailing candidate 0xAA between TCP recv chunks.
                self.buf[:] = self.buf[-1:] if self.buf.endswith(b"\xaa") else b""
                return frames
            if start:
                del self.buf[:start]
            if len(self.buf) < 4:
                return frames
            length = self.buf[3]
            if length > 64:
                del self.buf[0]
                continue
            size = 5 + length
            if len(self.buf) < size:
                return frames
            candidate = bytes(self.buf[:size])
            if candidate[-1] != (sum(candidate[2:-1]) & 0xFF):
                self.bad_checksum += 1
                del self.buf[0]
                continue
            frames.append((candidate[2], candidate[4:-1]))
            del self.buf[:size]


def vacant_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def join_qemu_serial(port: int, process: subprocess.Popen) -> socket.socket:
    deadline = time.monotonic() + TIMEOUT_CONNECT_S
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"QEMU quit at startup (exit {process.returncode})")
        try:
            sock = socket.create_connection(("127.0.0.1", port), timeout=0.5)
            sock.setblocking(False)
            return sock
        except OSError:
            time.sleep(0.1)
    raise RuntimeError("QEMU UART TCP socket did not appear")


def run(elf: Path, output_dir: Path, qemu_bin: str) -> dict:
    import pybullet as p

    if not elf.is_file():
        raise FileNotFoundError(elf)
    if len(GOAL_RAW) != len(JOINT_NAMES):
        raise ValueError("Inconsistent simulated robot joint count")

    port = vacant_tcp_port()
    qemu = subprocess.Popen([
        qemu_bin, "-M", "mps2-an386", "-kernel", str(elf.resolve()),
        "-display", "none", "-monitor", "none",
        "-serial", f"tcp:127.0.0.1:{port},server,nowait",
        # No -icount: use QEMU's wall-clock virtual timer backend.
        # An instruction-counted emulation can advance <1 simulated ms
        # per 10ms Python feedback cycle and starve the periodic planner.
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    client = -1
    try:
        with join_qemu_serial(port, qemu) as sock:
            client = p.connect(p.DIRECT)
            if client < 0:
                raise RuntimeError("PyBullet DIRECT failed")
            p.setGravity(0, 0, -9.81)
            p.setTimeStep(PHYSICS_DT_S)
            arm = p.loadURDF(str(URDF), useFixedBase=True)
            joints = joint_indices(p, arm)
            for index, raw in zip(joints, START_RAW):
                p.resetJointState(arm, index, math.radians(raw / 100))
            starting_radians = [math.radians(v / 100) for v in START_RAW]
            p.setJointMotorControlArray(
                arm, joints, p.POSITION_CONTROL,
                targetPositions=starting_radians, forces=[150.0] * len(joints))
            for _ in range(100):
                p.stepSimulation()

            incoming = FrameStream()
            started = time.monotonic()
            received_targets = 0
            received_feedback_acks = 0
            move_ack_status = None
            move_request_sent = False
            max_target_change_deg = 0.0
            first_sim_joint2_deg = angles_raw(p, arm, joints)[1] / 100.0
            output = []
            tx_frames = 0
            seq = 41
            diag_items = tuple(wire.SIM_MOTION_DIAGNOSTIC_SELECTORS.items())
            diag_index = 0
            diag_pending = None
            diag_sent_at = 0.0
            diagnostics = {}

            # QEMU can run its virtual Cortex-M4 MUCH slower than host wall
            # time. Each firmware ProtocolTX frame represents a real
            # scheduled CONTROL PERIOD on the emulated RTOS. Physics and
            # feedback advance once per such frame, never at a separately
            # guessed host 100Hz that floods the virtual UART with interrupts.
            while time.monotonic() - started < MAX_WALL_DURATION_S:
                if qemu.poll() is not None:
                    raise RuntimeError("QEMU exited while moving")

                try:
                    data = sock.recv(8192)
                except BlockingIOError:
                    data = None
                if data == b"":
                    raise RuntimeError("QEMU UART disconnected")
                if data:
                    for command, payload in incoming.consume(data):
                        if (command == wire.CMD_SET_JOINT_TARGETS
                                and len(payload) == wire.JOINT_PAYLOAD_LEN):
                            values = struct.unpack(
                                "<" + str(wire.JOINT_COUNT) + "h", payload)
                            received_targets += 1
                            if move_ack_status == 0:
                                p.setJointMotorControlArray(
                                    arm, joints, p.POSITION_CONTROL,
                                    targetPositions=[
                                        math.radians(v / wire.JOINT_ANGLE_SCALE)
                                        for v in values
                                    ],
                                    forces=[150.0] * len(joints))
                                max_target_change_deg = max(
                                    max_target_change_deg,
                                    abs(values[1] / wire.JOINT_ANGLE_SCALE
                                        - first_sim_joint2_deg),
                                )
                            for _ in range(round(FEEDBACK_PERIOD_S / PHYSICS_DT_S)):
                                p.stepSimulation()
                            feedback = angles_raw(p, arm, joints)
                            sock.sendall(frame(
                                wire.CMD_JOINT_STATE,
                                struct.pack("<" + str(wire.JOINT_COUNT) + "h",
                                            *feedback)))
                            tx_frames += 1
                            if received_targets % 10 == 0:
                                output.append({
                                    "elapsed_wall_s": round(time.monotonic()-started, 3),
                                    "simulated_physics_s": round(
                                        received_targets*FEEDBACK_PERIOD_S,3),
                                    "joint2_actual_deg": feedback[1]/100.0,
                                    "received_targets": received_targets,
                                    "feedback_acks": received_feedback_acks,
                                    "move_ack": move_ack_status,
                                })
                        elif (command == wire.CMD_JOINT_STATE_ACK
                              and len(payload) == wire.JOINT_PAYLOAD_LEN):
                            received_feedback_acks += 1
                            if (not move_request_sent and
                                    received_feedback_acks >= 3):
                                body = bytes((seq,)) + struct.pack(
                                    "<" + str(wire.JOINT_COUNT) + "h",
                                    *GOAL_RAW)
                                sock.sendall(
                                    frame(wire.CMD_REQUEST_JOINT_MOVE,body))
                                move_request_sent = True
                        elif (command == wire.CMD_JOINT_MOVE_ACK
                              and len(payload) == wire.MOVE_ACK_PAYLOAD_LEN):
                            if payload[0] == seq:
                                move_ack_status = struct.unpack(
                                    "<b",payload[1:])[0]
                        elif (command == wire.CMD_DIAGNOSTICS_RESPONSE
                              and len(payload) == 4 and diag_pending is not None):
                            diagnostics[diag_pending] = struct.unpack(
                                "<I",payload)[0]
                            diag_pending = None

                now = time.monotonic()
                if (move_ack_status is not None and received_targets >= 25
                        and diag_pending is None and diag_index < len(diag_items)):
                    diag_name, diag_selector = diag_items[diag_index]
                    sock.sendall(frame(
                        wire.CMD_GET_DIAGNOSTICS,bytes((diag_selector,))))
                    diag_index += 1
                    diag_pending = diag_name
                    diag_sent_at = now
                elif diag_pending is not None and now - diag_sent_at > 2.0:
                    diagnostics[diag_pending] = None
                    diag_pending = None

                # Finish on a measured number of QEMU control ticks, after
                # the serial diagnostics have been returned, not wall time.
                if (received_targets >= MAX_TARGET_FRAMES and
                        diag_index == len(diag_items) and
                        diag_pending is None):
                    break
                if not data:
                    time.sleep(0.001)

            movement = abs(
                angles_raw(p, arm, joints)[1] / 100.0 - first_sim_joint2_deg)
            accepted = move_ack_status == 0
            debug_pass = (
                (diagnostics.get("requests_accepted") or 0) >= 1 and
                (diagnostics.get("trajectory_samples") or 0) > 0 and
                (diagnostics.get("controller_steps") or 0) > 0 and
                diagnostics.get("controller_faults") == 0 and
                diagnostics.get("planner_faults") == 0
            )
            passed = (accepted and debug_pass and received_feedback_acks >= 5
                      and received_targets >= 10
                      and movement >= MIN_JOINT_MOVEMENT_DEG
                      and max_target_change_deg >= MIN_ACCEPTED_COMMAND_CHANGE_DEG)
            report = {
                "scope": "real_ARM_QEMU_UART_FreeRTOS_C_PID_PyBullet_feedback",
                "qemu_machine": "mps2-an386",
                "motor_commands": "virtual_serial_only",
                "command_output_compile_gate": "ON only for QEMU test",
                "elapsed_wall_s": round(time.monotonic()-started,3),
                "simulated_physics_s": round(tx_frames*FEEDBACK_PERIOD_S,3),
                "feedback_pacing": "one measured feedback per QEMU ProtocolTX control-period frame",
                "feedback_frames_sent": tx_frames,
                "feedback_acks_received": received_feedback_acks,
                "target_frames_received": received_targets,
                "move_request_sent": move_request_sent,
                "move_ack_status": move_ack_status,
                "measured_joint2_movement_deg": movement,
                "max_commanded_joint2_change_deg": max_target_change_deg,
                "bad_checksum_frames": incoming.bad_checksum,
                "motion_diagnostics": diagnostics,
                "motion_diagnostics_passed": debug_pass,
                "passed": passed,
                "not_tested": [
                    "real industrial robot hardware",
                    "full-link collision safety",
                    "long-term worst-case timing",
                    "actuator emergency stop",
                ],
            }
    finally:
        if client >= 0:
            p.disconnect(client)
        qemu.terminate()
        try:
            _, stderr = qemu.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            qemu.kill()
            _, stderr = qemu.communicate()
        if qemu.returncode not in (None,0,-15) and stderr:
            print("QEMU stderr:",stderr.decode(errors="replace")[-2500:])

    output_dir.mkdir(parents=True,exist_ok=True)
    (output_dir/"stage5_uart_pybullet_metrics.json").write_text(
        json.dumps(report,indent=2,ensure_ascii=False),encoding="utf-8")
    with (output_dir/"stage5_uart_pybullet_trace.json").open("w",encoding="utf-8") as file:
        json.dump(output,file,indent=2)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf",required=True,type=Path)
    parser.add_argument("--qemu",default="qemu-system-arm")
    parser.add_argument("--output-dir",type=Path,
                        default=ROOT/"build/stage5-uart")
    args = parser.parse_args()
    outcome = run(args.elf,args.output_dir,args.qemu)
    print(json.dumps(outcome,indent=2,ensure_ascii=False))
    return 0 if outcome["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
