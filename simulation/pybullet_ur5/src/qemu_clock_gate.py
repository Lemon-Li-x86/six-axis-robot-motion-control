"""Pause QEMU's virtual clock while one PyBullet UART physics batch executes.

Stage5-only test harness transport. This is not a motor-control clock,
a PID change, or an alternative UART frame protocol. QMP stop/cont freezes
guest FreeRTOS time during expensive host physics/rendering calls.
"""
from __future__ import annotations

import json
import socket
import time
from contextlib import contextmanager


class QemuClockGate:
    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.reader = sock.makefile("rb")
        self.sequence = 0
        self.pauses = 0

    @classmethod
    def connect(cls, port: int, process, timeout_s: float = 5.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("QEMU exited before QMP clock-gate connection")
            try:
                connection = socket.create_connection(("127.0.0.1", port), timeout=0.5)
                connection.settimeout(2.0)
                gate = cls(connection)
                try:
                    greeting = gate._read_message()
                    if "QMP" not in greeting:
                        raise RuntimeError("QMP greeting missing")
                    gate.command("qmp_capabilities")
                    if gate.command("query-status").get("status") != "running":
                        raise RuntimeError("QEMU not running before physics startup")
                    return gate
                except BaseException:
                    gate.close()
                    raise
            except (ConnectionRefusedError, TimeoutError, OSError):
                time.sleep(0.025)
        raise TimeoutError("QMP clock-gate connection timed out")

    def _read_message(self) -> dict:
        line = self.reader.readline()
        if not line:
            raise ConnectionError("QMP connection closed")
        result = json.loads(line)
        if not isinstance(result, dict):
            raise RuntimeError("Invalid QMP response")
        return result

    def command(self, name: str) -> dict:
        self.sequence += 1
        command_id = self.sequence
        self.sock.sendall((json.dumps({"execute": name, "id": command_id}) +
                           "\r\n").encode("utf-8"))
        while True:
            message = self._read_message()
            if "event" in message:
                continue  # QMP STOP/RESUME events can precede command replies.
            if message.get("id") != command_id:
                raise RuntimeError(f"Unexpected QMP response for {name}: {message}")
            if "error" in message:
                raise RuntimeError(f"QMP {name} failed: {message['error']}")
            if "return" not in message or not isinstance(message["return"], dict):
                raise RuntimeError(f"Malformed QMP {name} reply")
            return message["return"]

    @contextmanager
    def paused_for_physics(self, has_targets: bool):
        if not has_targets:
            yield
            return
        self.command("stop")
        # A successful stop reply is not a substitute for observed guest state.
        if self.command("query-status").get("status") != "paused":
            raise RuntimeError("QEMU failed to pause before PyBullet physics")
        self.pauses += 1
        try:
            yield
        finally:
            self.command("cont")
            if self.command("query-status").get("status") != "running":
                raise RuntimeError("QEMU failed to resume after PyBullet physics")

    def close(self):
        try:
            self.reader.close()
        finally:
            self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
