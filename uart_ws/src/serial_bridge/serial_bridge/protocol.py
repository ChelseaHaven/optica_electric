"""Wire protocol shared with optic_electric2; no ROS or serial dependency."""
import math
import struct

MAX_PAYLOAD = 128
VELOCITY = WHEEL_SPEED = 0x0101
IMU_RAW = 0x0102


def crc(data, initial, polynomial):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value


def pack(command, payload=b'', sequence=0):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('payload too long')
    header = struct.pack('<BHB', 0xa5, len(payload), sequence & 255)
    header += bytes([crc(header, 0xff, 0x8c)])
    frame = header + struct.pack('<H', command) + payload
    return frame + struct.pack('<H', crc(frame, 0xffff, 0x8408))


def pack_velocity(vx, vy, wz, sequence=0):
    if not all(math.isfinite(v) and abs(v) <= 3.402823466e38 for v in (vx, vy, wz)):
        raise ValueError('velocity must be finite float32')
    return pack(VELOCITY, struct.pack('<3f', vx, vy, wz), sequence)


def decode_telemetry(command, payload):
    count = {WHEEL_SPEED: 4, IMU_RAW: 9}.get(command)
    if count is None or len(payload) != count * 4:
        return None
    values = struct.unpack('<' + 'f' * count, payload)
    return values if all(math.isfinite(v) for v in values) else None


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.last_time = None
        self.frames = self.crc_errors = self.length_errors = 0

    def reset(self):
        self.buffer.clear()
        self.last_time = None

    def feed(self, data, now):
        if self.last_time is not None and now - self.last_time > 0.1:
            self.buffer.clear()
        self.last_time = now
        packets = []
        # Input is bounded by transport.read(); consume bytewise to bound memory.
        for byte in data:
            self.buffer.append(byte)
            while self.buffer:
                if self.buffer[0] != 0xa5:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < 5:
                    break
                if crc(self.buffer[:4], 0xff, 0x8c) != self.buffer[4]:
                    self.crc_errors += 1
                    del self.buffer[0]
                    continue
                length = int.from_bytes(self.buffer[1:3], 'little')
                if length > MAX_PAYLOAD:
                    self.length_errors += 1
                    del self.buffer[0]
                    continue
                total = length + 9
                if len(self.buffer) < total:
                    break
                frame = self.buffer[:total]
                if crc(frame[:-2], 0xffff, 0x8408) != int.from_bytes(frame[-2:], 'little'):
                    self.crc_errors += 1
                    del self.buffer[0]
                    continue
                packets.append((int.from_bytes(frame[5:7], 'little'), bytes(frame[7:-2]), frame[3]))
                del self.buffer[:total]
                self.frames += 1
        return packets
