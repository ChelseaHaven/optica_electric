import math
import struct

import pytest
from serial_bridge.protocol import Parser, pack, pack_velocity, decode_telemetry


def test_stream_and_crc():
    frame = pack(0x0101, struct.pack('<4f', 1, -2, 3, -4), 255)
    p = Parser()
    assert p.feed(b'noise' + frame[:7], 0) == []
    assert p.feed(frame[7:] + frame, .01) == [(0x0101, frame[7:-2], 255)] * 2
    bad = bytearray(frame)
    bad[10] ^= 1
    assert p.feed(bad + frame, .02) == [(0x0101, frame[7:-2], 255)]
    assert p.crc_errors > 0
    assert len(p.buffer) < 137


def test_timeout_and_invalid_values():
    frame = pack_velocity(.2, -.3, .4)
    p = Parser()
    assert not p.feed(frame[:10], 0)
    assert not p.feed(frame[10:], .2)
    assert len(p.feed(frame, .21)) == 1
    with pytest.raises(ValueError):
        pack_velocity(math.nan, 0, 0)
    assert decode_telemetry(0x0102, struct.pack('<9f', *([math.inf] * 9))) is None
    assert decode_telemetry(0x0101, b'bad') is None


def test_firmware_vectors():
    import json
    from pathlib import Path
    vectors = json.loads(Path(__file__).with_name('firmware_vectors.json').read_text())
    for v in vectors:
        assert pack(v['command'], bytes.fromhex(v['payload']), v['sequence']).hex() == v['frame']
