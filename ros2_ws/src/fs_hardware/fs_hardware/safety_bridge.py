"""The safety PLC's state on ROS, read from its diagnostic PDO; the software heartbeat back.

The safety PLC (docs/safety_concept.md) acts on its own safe inputs: e-stop circuits, safety
scanners, encoders, inclinometer. It also sends a non-safety diagnostic PDO so software knows what it
did, and it reads a heartbeat and the requested mode (travel / planting) from software as
non-safety inputs, used only to choose field sets and to log. Nothing software sends can switch a
safety function off.

  PLC -> software   COB-ID 0x180 + plc_node (TPDO1), 3 bytes, every 20 ms
      byte 0  bit 0 e-stop, 1 protective stop, 2 warning, 3 tool zone, 4 tilt, 5 STO drives, 6 STO tool,
              7 heartbeat ok
      byte 1  active field set (0 = standstill)
      byte 2  alive counter
  software -> PLC   COB-ID 0x200 + plc_node (RPDO1), 2 bytes, every 50 ms
      byte 0  requested mode: 0 travel, 1 planting, 2 maintenance
      byte 1  heartbeat counter
"""
from __future__ import annotations

import socket
import struct

BITS = ("estop", "protective_stop", "warning", "tool_zone", "tilt", "sto_drive", "sto_tool", "heartbeat_ok")
MODES = {"travel": 0, "planting": 1, "maintenance": 2}


def decode(data: bytes) -> dict:
    b0 = data[0] if data else 0
    out = {k: bool(b0 >> i & 1) for i, k in enumerate(BITS)}
    out["field_set"] = data[1] if len(data) > 1 else 0
    out["alive"] = data[2] if len(data) > 2 else 0
    return out


def encode(state: dict, field_set: int = 0, alive: int = 0) -> bytes:
    """The PLC side (tests, and the mock bus)."""
    b0 = sum(1 << i for i, k in enumerate(BITS) if state.get(k))
    return bytes([b0, field_set & 0xFF, alive & 0xFF])


class CanSocket:
    FMT = "=IB3x8s"

    def __init__(self, ifname: str):
        self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((ifname,))
        self.s.setblocking(False)

    def send(self, cob: int, data: bytes):
        self.s.send(struct.pack(self.FMT, cob, len(data), data.ljust(8, b"\0")))

    def recv(self):
        try:
            frame = self.s.recv(16)
        except BlockingIOError:
            return None
        cob, n, data = struct.unpack(self.FMT, frame)
        return cob & 0x7FF, data[:n]
