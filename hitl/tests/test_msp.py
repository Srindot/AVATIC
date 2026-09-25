"""MSP codec tests. Byte layouts written by hand from the MagisV2 source
(firmware/magisv2/src/main/io/serial_msp.cpp)."""
import os
import struct
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))                               # avatic_hitl
sys.path.insert(0, os.path.join(HERE, '..', '..', 'outerloop_controller'))  # avatic_drone.types
from avatic_hitl import msp  # noqa: E402


def reply(cmd, payload):
    return msp.encode(cmd, payload, header=b'$M>')


def test_request_framing_and_checksum():
    assert msp.encode(msp.MSP_ATTITUDE) == b'$M<\x00\x6c\x6c'   # size 0 ^ cmd 108
    rc = msp.encode_set_raw_rc([1500] * 8)
    assert rc[:5] == b'$M<\x10\xc8'                            # 16 bytes, cmd 200
    assert rc[5:21] == struct.pack('<8H', *[1500] * 8)
    check = 16 ^ 200
    for b in rc[5:21]:
        check ^= b
    assert rc[21] == check and len(rc) == 22


def test_set_command_is_two_bytes():
    frame = msp.encode_set_command(msp.COMMAND_LAND)
    assert frame[3] == 2 and frame[4] == 217 and frame[5:7] == b'\x02\x00'


def test_parser_handles_split_garbage_and_corruption():
    good = reply(msp.MSP_ALTITUDE, struct.pack('<ih', 123, -4))
    bad = bytearray(reply(msp.MSP_ATTITUDE, b'\x01\x02\x03\x04\x05\x06'))
    bad[-1] ^= 0xFF  # wrong checksum
    stream = b'noise$$M' + bytes(bad) + good + b'$M<\x00\x65\x65'  # a request is ignored
    parser = msp.Parser()
    frames = []
    for i in range(0, len(stream), 3):  # 3-byte chunks
        frames += parser.feed(stream[i:i + 3])
    assert [f.cmd for f in frames] == [msp.MSP_ALTITUDE]
    assert msp.decode_altitude(frames[0].payload) == msp.Altitude(123, -4)
    assert parser.errors >= 1


def test_error_frames_are_flagged():
    frames = msp.Parser().feed(msp.encode(200, b'', header=b'$M!'))
    assert frames[0].is_error and frames[0].cmd == 200


def test_decode_status_flags():
    flags = 1 << msp.FLAG_ARMED | 1 << msp.FLAG_LOW_BATTERY
    payload = struct.pack('<HHHIB', flags, 0, 0b11, 0b1001, 0)
    status = msp.decode_status(payload)
    assert status.armed and not status.ok_to_arm and status.flag(msp.FLAG_LOW_BATTERY)
    assert status.mode_flags == 0b1001
    with pytest.raises(ValueError):
        msp.decode_status(payload[:5])


def test_decode_attitude_analog():
    att = msp.decode_attitude(struct.pack('<hhh', -52, 31, 270))
    assert (att.roll_decideg, att.pitch_decideg, att.heading_deg) == (-52, 31, 270)
    ana = msp.decode_analog(struct.pack('<HHHHBB', 3912, 150, 80, 520, 87, 0))
    assert ana.battery_mv == 3912 and ana.state_of_charge_pct == 87


def test_rc_channel_mapping_matches_simulator():
    from avatic_hitl.hardware import _to_channels
    from avatic_drone.types import Command
    assert _to_channels(Command(), arm=False) == [1500, 1500, 1000, 1500, 2000, 1000, 1000, 1000]
    assert _to_channels(Command(roll=1, pitch=-1, yaw_rate=0.2, throttle=1,
                                altitude_hold=True), arm=True) == \
        [2000, 1000, 2000, 1600, 2000, 1000, 1500, 1500]
    assert _to_channels(Command(roll=5, throttle=-2), arm=True)[:3] == [2000, 1500, 1000]
