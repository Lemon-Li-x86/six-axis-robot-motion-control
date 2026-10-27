#!/usr/bin/env python3
"""Prevent the legacy Python QEMU UART receiver's wire constants from
silently diverging from the firmware C protocol and robot type contracts.

Stdlib-only contract check; not an end-to-end UART or timing test.
C protocol.h and robot_types.h remain AUTHORITATIVE. The Python receiver
is an explicitly audited mirror pending a shared wire-schema generator.
"""
from __future__ import annotations

import importlib.util
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware/freertos_demo"
RECEIVER = ROOT / "simulation/pybullet_ur5/src/qemu_uart_receiver.py"


def number_macro(text: str, name: str) -> float:
    match = re.search(
        r"(?m)^\s*#define\s+" + re.escape(name)
        + r"\s+(0x[0-9a-fA-F]+|[0-9]+(?:\.[0-9]+)?)[uUlLfF]*\s*$",
        text,
    )
    if match is None:
        raise AssertionError(f"Missing or changed firmware macro: {name}")
    value = match.group(1)
    return float(int(value, 16)) if value.lower().startswith("0x") else float(value)


def main() -> None:
    protocol = (FIRMWARE / "communications/protocol.h").read_text(encoding="utf-8")
    types = (FIRMWARE / "common/robot_types.h").read_text(encoding="utf-8")
    spec = importlib.util.spec_from_file_location("qemu_uart_receiver", RECEIVER)
    if spec is None or spec.loader is None:
        raise AssertionError(f"Unable to load UART receiver: {RECEIVER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    header = bytes((
        int(number_macro(protocol, "PROTOCOL_HEADER_0")),
        int(number_macro(protocol, "PROTOCOL_HEADER_1")),
    ))
    command = int(number_macro(protocol, "CMD_SET_JOINT_TARGETS"))
    joints = int(number_macro(types, "ROBOT_JOINT_COUNT"))
    degree_per_raw = number_macro(types, "ROBOT_JOINT_ANGLE_UNIT_DEG")
    if not 0.0 < degree_per_raw <= 1.0:
        raise AssertionError("Invalid firmware degree/raw conversion")
    if module.FRAME_HEADER != header:
        raise AssertionError("Python UART frame header drift")
    if module.CMD_SET_JOINT_TARGETS != command:
        raise AssertionError("Python UART command ID drift")
    for c_macro, py_attr in (
        ("CMD_REQUEST_JOINT_MOVE", "CMD_REQUEST_JOINT_MOVE"),
        ("CMD_JOINT_STATE", "CMD_JOINT_STATE"),
        ("CMD_JOINT_STATE_ACK", "CMD_JOINT_STATE_ACK"),
        ("CMD_JOINT_MOVE_ACK", "CMD_JOINT_MOVE_ACK"),
    ):
        if getattr(module, py_attr) != int(number_macro(protocol, c_macro)):
            raise AssertionError(f"UART command mirror drift: {c_macro}")
    if module.MOVE_REQUEST_PAYLOAD_LEN != joints * 2 + 1:
        raise AssertionError("Move request payload drift")
    if module.MOVE_ACK_PAYLOAD_LEN != 2:
        raise AssertionError("Move admission ACK payload drift")
    if not re.search(
        r"#define\\s+PROTOCOL_MOVE_REQUEST_PAYLOAD_LEN\\s+"
        r"\\(1U\\s*\\+\\s*PROTOCOL_JOINT_PAYLOAD_LEN\\)", protocol
    ):
        raise AssertionError("C move request wire schema changed")
    if not re.search(
        r"#define\\s+PROTOCOL_MOVE_ACK_PAYLOAD_LEN\\s+2U", protocol
    ):
        raise AssertionError("C move ACK wire schema changed")

    if module.JOINT_COUNT != joints:
        raise AssertionError("Python UART joint count drift")
    if module.JOINT_PAYLOAD_LEN != joints * 2:
        raise AssertionError("Python UART payload length drift")
    if abs((1.0 / module.JOINT_ANGLE_SCALE) - degree_per_raw) > 1e-7:
        raise AssertionError("Python UART centidegree scale drift")
    if not re.search(
        r"#define\s+PROTOCOL_JOINT_PAYLOAD_LEN\s*\\\s*"
        r"\(\s*ROBOT_JOINT_COUNT\s*\*\s*2U\s*\)",
        protocol,
    ):
        raise AssertionError("Firmware joint payload format changed: review Python unpacking")
    print("PASS: legacy Python UART constants match authoritative C wire protocol.")


if __name__ == "__main__":
    main()
