"""TerraMind USART3 protocol v1 reference codec and minimal serial monitor.

The codec uses only Python's standard library. The command-line monitor needs
pyserial: python -m pip install pyserial
"""

from __future__ import annotations

import argparse
import dataclasses
import struct
import time

MAX_PAYLOAD = 256
CONTROL_TYPE = 0x01
STATUS_TYPE = 0x81


def crc16(data: bytes) -> int:
    """CRC-16/XMODEM: poly 0x1021, init 0, no reflection or xorout."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def tlv(tag: int, value: bytes) -> bytes:
    if len(value) > 255:
        raise ValueError("TLV value too long")
    return bytes((tag, len(value))) + value


@dataclasses.dataclass
class Control:
    enable: bool = False
    stop: bool = False
    linear_mps: float = 0.0
    angular_radps: float = 0.0
    left_on: bool = False
    left_rpm: float = 0.0
    right_on: bool = False
    right_rpm: float = 0.0
    mowing_on: bool = False
    mowing_percent: float = 0.0
    lift_on: bool = False
    lift_height_mm: float = 0.0
    spray_on: bool = False
    spray_percent: float = 0.0  # Logical 0..100; firmware maps nonzero to 70..100% PWM.


def build_control(command: Control, seq: int) -> bytes:
    payload = b"".join((
        tlv(0x01, bytes((int(command.enable) | (int(command.stop) << 1),))),
        tlv(0x10, struct.pack("<ff", command.linear_mps, command.angular_radps)),
        tlv(0x20, struct.pack("<Bf", int(command.left_on), command.left_rpm)),
        tlv(0x21, struct.pack("<Bf", int(command.right_on), command.right_rpm)),
        tlv(0x30, struct.pack("<Bf", int(command.mowing_on), command.mowing_percent)),
        tlv(0x40, struct.pack("<Bf", int(command.lift_on), command.lift_height_mm)),
        tlv(0x50, struct.pack("<Bf", int(command.spray_on), command.spray_percent)),
    ))
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = struct.pack("<BBHH", 1, CONTROL_TYPE, seq & 0xFFFF, len(payload)) + payload
    return b"\xFC\xFB" + body + struct.pack("<H", crc16(body)) + b"\xFD\xFE"


def decode_status(payload: bytes) -> dict:
    blocks = {}
    pos = 0
    while pos < len(payload):
        if len(payload) - pos < 2:
            raise ValueError("truncated TLV header")
        tag, size = payload[pos:pos + 2]
        pos += 2
        if len(payload) - pos < size:
            raise ValueError("truncated TLV value")
        if tag in blocks:
            raise ValueError("duplicate TLV")
        blocks[tag] = payload[pos:pos + size]
        pos += size

    formats = {
        0x01: "<IHBBHHHH",
        0x10: "<ffffff",
        0x20: "<Bfff",
        0x21: "<Bfff",
        0x30: "<Bf",
        0x40: "<BffB",
        0x50: "<BffB",
    }
    if not formats.keys() <= blocks.keys():
        raise ValueError("missing required status block")
    values = {}
    for tag, fmt in formats.items():
        if len(blocks[tag]) != struct.calcsize(fmt):
            raise ValueError(f"bad length for block 0x{tag:02X}")
        values[tag] = struct.unpack(fmt, blocks[tag])

    system = values[0x01]
    chassis = values[0x10]
    return {
        "uptime_ms": system[0], "last_command_seq": system[1],
        "result": system[2], "mode": system[3], "faults": system[4],
        "capabilities": system[5], "command_age_ms": system[6],
        "rx_error_count": system[7],
        "linear_mps": chassis[0], "angular_radps": chassis[1],
        "left_target_rpm": chassis[2], "right_target_rpm": chassis[3],
        "left_actual_rpm": chassis[4], "right_actual_rpm": chassis[5],
        "left_seeder": values[0x20], "right_seeder": values[0x21],
        "mowing": values[0x30], "lift": values[0x40], "spray": values[0x50],
        "unknown_blocks": {tag: value for tag, value in blocks.items() if tag not in formats},
    }


class StatusParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes) -> list[tuple[int, dict]]:
        self.buffer.extend(data)
        output = []
        while True:
            start = self.buffer.find(b"\xFC\xFB")
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b"\xFC" else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 8:
                break
            version, kind, seq, length = struct.unpack_from("<BBHH", self.buffer, 2)
            if version != 1 or kind != STATUS_TYPE or length > MAX_PAYLOAD:
                self.bad_frames += 1
                del self.buffer[0]
                continue
            total = 12 + length
            if len(self.buffer) < total:
                break
            frame = self.buffer[:total]
            if frame[-2:] != b"\xFD\xFE" or crc16(frame[2:8 + length]) != struct.unpack_from("<H", frame, 8 + length)[0]:
                self.bad_frames += 1
                del self.buffer[0]
                continue
            try:
                status = decode_status(frame[8:8 + length])
            except ValueError:
                self.bad_frames += 1
            else:
                output.append((seq, status))
            del self.buffer[:total]
        return output


def main() -> None:
    parser = argparse.ArgumentParser(description="Monitor TerraMind USART3 status")
    parser.add_argument("port", help="COM port, for example COM5")
    args = parser.parse_args()
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("Install pyserial: python -m pip install pyserial") from exc

    monitor = StatusParser()
    seq = 0
    command = Control()  # Disarmed heartbeat; never starts an actuator.
    with serial.Serial(args.port, 115200, timeout=0.01) as link:
        next_send = 0.0
        try:
            while True:
                now = time.monotonic()
                if now >= next_send:
                    link.write(build_control(command, seq))
                    seq = (seq + 1) & 0xFFFF
                    next_send = now + 0.05
                for status_seq, status in monitor.feed(link.read(256)):
                    print(status_seq, status)
        except KeyboardInterrupt:
            link.write(build_control(Control(stop=True), seq))


if __name__ == "__main__":
    main()
