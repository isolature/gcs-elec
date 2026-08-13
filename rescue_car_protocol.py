#!/usr/bin/env python3
"""Draft v0.1 RescueCar wire protocol used by the production client."""

from dataclasses import dataclass
import struct


MAGIC = b"RC"
PROTOCOL_VERSION = 1
SCHEMA_HASH = 0x0B0ECAF4
HEADER_SIZE = 32
CRC_SIZE = 4
MAX_DECODED_SIZE = 4096

MSG_HELLO = 0x0001
MSG_HELLO_ACK = 0x0002
MSG_ARM_COMMAND = 0x0010
MSG_CHASSIS_SETPOINT = 0x0011
MSG_GRIPPER_COMMAND = 0x0012
MSG_SAFE_STOP = 0x0013
MSG_HEARTBEAT = 0x0014
MSG_COMMAND_RESULT = 0x0080
MSG_ROBOT_STATE = 0x0081
MSG_SAFETY_STATUS = 0x0082
MSG_HEARTBEAT_ACK = 0x0083

ARM_TARGET_DISARMED = 1
ARM_TARGET_ARMED = 2
GRIPPER_TARGET_OPEN = 1
GRIPPER_TARGET_CLOSED = 2
SAFE_STOP_REASON_USER_REQUEST = 1
UPPER_CONTROL_STATE_ACTIVE = 3

COMMAND_RESULT_ACCEPTED = 1
COMMAND_RESULT_REJECTED = 2
COMMAND_RESULT_COMPLETED = 3

SAFETY_STATE_DISARMED = 1
SAFETY_STATE_ARMED = 2
SAFETY_STATE_SAFE_STOP = 3
SAFETY_STATE_FAULT = 4

COORDINATE_FRAME_BASE_LINK = 1

GRIPPER_STATE_OPEN = 1
GRIPPER_STATE_CLOSED = 2
GRIPPER_STATE_OPENING = 3
GRIPPER_STATE_CLOSING = 4
GRIPPER_STATE_UNKNOWN = 5
GRIPPER_STATE_FAULT = 6

EXPECTED_FIRMWARE_VERSION = 0x00010000
EXPECTED_CONFIG_HASH = 0
CAPABILITY_WHEEL_SPEED = 1 << 0


class ProtocolError(ValueError):
    """A malformed or incompatible protocol frame was received."""


@dataclass(frozen=True)
class Frame:
    msg_type: int
    session_id: int
    seq: int
    source_timestamp_us: int
    payload: bytes


@dataclass(frozen=True)
class HelloAck:
    protocol_version: int
    firmware_version: int
    boot_id: int
    config_hash: int
    capabilities: int


@dataclass(frozen=True)
class CommandResult:
    command_id: int
    status: int
    reason: int


@dataclass(frozen=True)
class SafetyStatus:
    link_ready: bool
    armed: bool
    safety_state: int
    stop_reason: int
    fault_bits: int
    motion_age_ms: int
    battery_mv: int
    watchdog_triggered: bool
    validity_flags: int


@dataclass(frozen=True)
class RobotState:
    state_seq: int
    timestamp_us: int
    coordinate_frame: int
    last_command_seq: int
    linear_mm_s: int
    angular_mrad_s: int
    delta_x_mm: int
    delta_y_mm: int
    delta_yaw_mrad: int
    yaw_mrad: int
    yaw_rate_mrad_s: int
    gripper_state: int
    validity_flags: int
    gripper_target_percent: int
    left_rpm_x10: int
    right_rpm_x10: int


def crc32c(data):
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def cobs_encode(data):
    output = bytearray([0])
    code_index = 0
    code = 1
    for value in data:
        if value == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(value)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data):
    output = bytearray()
    offset = 0
    while offset < len(data):
        code = data[offset]
        if code == 0:
            raise ProtocolError("zero byte inside COBS frame")
        offset += 1
        end = offset + code - 1
        if end > len(data):
            raise ProtocolError("truncated COBS block")
        output.extend(data[offset:end])
        offset = end
        if code != 0xFF and offset < len(data):
            output.append(0)
    return bytes(output)


def encode_varint(value):
    if not 0 <= value <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("varint is out of range")
    output = bytearray()
    while value > 0x7F:
        output.append((value & 0x7F) | 0x80)
        value >>= 7
    output.append(value)
    return bytes(output)


def decode_varint(data, offset):
    value = 0
    for shift in range(0, 64, 7):
        if offset >= len(data):
            raise ProtocolError("truncated Protobuf varint")
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, offset
    raise ProtocolError("Protobuf varint is too long")


def encode_sint32(value):
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError("sint32 value must be an integer")
    if not -(1 << 31) <= value < (1 << 31):
        raise ValueError("sint32 value is out of range")
    return encode_varint(((value << 1) ^ (value >> 31)) & 0xFFFFFFFF)


def decode_sint32(value):
    value &= 0xFFFFFFFF
    return (value >> 1) ^ -(value & 1)


def build_wire_frame(msg_type, payload, session_id, seq, timestamp_us):
    if not 1 <= session_id <= 0xFFFFFFFF:
        raise ValueError("session_id must be non-zero uint32")
    if len(payload) > MAX_DECODED_SIZE - HEADER_SIZE - CRC_SIZE:
        raise ValueError("payload is too large")
    header = struct.pack(
        "<2sBBHHIIIQHH",
        MAGIC,
        PROTOCOL_VERSION,
        0,
        msg_type,
        HEADER_SIZE,
        SCHEMA_HASH,
        session_id,
        seq & 0xFFFFFFFF,
        timestamp_us & 0xFFFFFFFFFFFFFFFF,
        len(payload),
        0,
    )
    decoded = header + payload
    decoded += struct.pack("<I", crc32c(decoded))
    return cobs_encode(decoded) + b"\x00"


def parse_wire_frame(wire):
    if not wire or wire[-1] != 0:
        raise ProtocolError("frame delimiter is missing")
    decoded = cobs_decode(wire[:-1])
    if len(decoded) < HEADER_SIZE + CRC_SIZE:
        raise ProtocolError("frame is too short")
    values = struct.unpack_from("<2sBBHHIIIQHH", decoded)
    magic, version, flags, msg_type, header_size = values[:5]
    schema_hash, session_id, seq, timestamp_us = values[5:9]
    payload_length, reserved = values[9:]
    if magic != MAGIC or version != PROTOCOL_VERSION:
        raise ProtocolError("frame magic or protocol version does not match")
    if flags != 0 or header_size != HEADER_SIZE or reserved != 0:
        raise ProtocolError("unsupported fixed header")
    if schema_hash != SCHEMA_HASH:
        raise ProtocolError("schema hash does not match")
    if len(decoded) != HEADER_SIZE + payload_length + CRC_SIZE:
        raise ProtocolError("frame payload length does not match")
    stored_crc = struct.unpack_from("<I", decoded, len(decoded) - 4)[0]
    if stored_crc != crc32c(decoded[:-4]):
        raise ProtocolError("frame CRC32C does not match")
    return Frame(
        msg_type,
        session_id,
        seq,
        timestamp_us,
        decoded[HEADER_SIZE:-CRC_SIZE],
    )


def _varint_fields(payload):
    fields = {}
    offset = 0
    while offset < len(payload):
        key, offset = decode_varint(payload, offset)
        field_number = key >> 3
        wire_type = key & 7
        if field_number == 0 or wire_type != 0:
            raise ProtocolError("unsupported Protobuf field")
        fields[field_number], offset = decode_varint(payload, offset)
    return fields


def _mixed_fields(payload, fixed64=(), fixed32=()):
    fields = {}
    offset = 0
    while offset < len(payload):
        key, offset = decode_varint(payload, offset)
        field_number = key >> 3
        wire_type = key & 7
        if field_number == 0:
            raise ProtocolError("Protobuf field number is zero")
        if wire_type == 0:
            fields[field_number], offset = decode_varint(payload, offset)
        elif wire_type == 1 and field_number in fixed64:
            if offset + 8 > len(payload):
                raise ProtocolError("truncated fixed64 field")
            fields[field_number] = struct.unpack_from("<Q", payload, offset)[0]
            offset += 8
        elif wire_type == 5 and field_number in fixed32:
            if offset + 4 > len(payload):
                raise ProtocolError("truncated fixed32 field")
            fields[field_number] = struct.unpack_from("<I", payload, offset)[0]
            offset += 4
        else:
            raise ProtocolError("unsupported Protobuf wire type")
    return fields


def build_hello_payload():
    return b""


def build_arm_payload(armed, command_id):
    target = ARM_TARGET_ARMED if armed else ARM_TARGET_DISARMED
    return b"\x08" + encode_varint(target) + b"\x10" + encode_varint(command_id)


def build_chassis_payload(linear_mm_s, angular_mrad_s, ttl_ms):
    if not 1 <= ttl_ms <= 1000:
        raise ValueError("ttl_ms must be in 1..1000")
    payload = bytearray()
    if linear_mm_s:
        payload += b"\x08" + encode_sint32(linear_mm_s)
    if angular_mrad_s:
        payload += b"\x10" + encode_sint32(angular_mrad_s)
    payload += b"\x18" + encode_varint(ttl_ms)
    return bytes(payload)


def build_gripper_payload(opened, command_id):
    target = GRIPPER_TARGET_OPEN if opened else GRIPPER_TARGET_CLOSED
    return b"\x08" + encode_varint(target) + b"\x10" + encode_varint(command_id)


def build_safe_stop_payload(command_id):
    return (
        b"\x08"
        + encode_varint(SAFE_STOP_REASON_USER_REQUEST)
        + b"\x10"
        + encode_varint(command_id)
    )


def build_heartbeat_payload(heartbeat_seq):
    return (
        b"\x08"
        + encode_varint(heartbeat_seq)
        + b"\x10"
        + encode_varint(UPPER_CONTROL_STATE_ACTIVE)
    )


def parse_hello_ack(frame):
    if frame.msg_type != MSG_HELLO_ACK:
        raise ProtocolError("frame is not HELLO_ACK")
    fields = _mixed_fields(frame.payload, fixed32=(2, 3, 4, 5))
    if any(number not in fields for number in range(1, 6)):
        raise ProtocolError("HELLO_ACK is missing a required field")
    ack = HelloAck(fields[1], fields[2], fields[3], fields[4], fields[5])
    if ack.protocol_version != PROTOCOL_VERSION:
        raise ProtocolError("HELLO_ACK protocol version is incompatible")
    if ack.firmware_version != EXPECTED_FIRMWARE_VERSION:
        raise ProtocolError("HELLO_ACK firmware version is incompatible")
    if ack.boot_id == 0 or ack.config_hash != EXPECTED_CONFIG_HASH:
        raise ProtocolError("HELLO_ACK boot or configuration is incompatible")
    if not ack.capabilities & CAPABILITY_WHEEL_SPEED:
        raise ProtocolError("wheel-speed capability is missing")
    return ack


def parse_command_result(frame):
    if frame.msg_type != MSG_COMMAND_RESULT:
        raise ProtocolError("frame is not COMMAND_RESULT")
    fields = _varint_fields(frame.payload)
    return CommandResult(fields.get(1, 0), fields.get(2, 0), fields.get(3, 0))


def parse_heartbeat_ack(frame):
    if frame.msg_type != MSG_HEARTBEAT_ACK:
        raise ProtocolError("frame is not HEARTBEAT_ACK")
    fields = _mixed_fields(frame.payload, fixed64=(2,))
    if fields.get(1, 0) == 0 or 2 not in fields:
        raise ProtocolError("HEARTBEAT_ACK is missing a required field")
    return fields[1], fields[2]


def parse_safety_status(frame):
    if frame.msg_type != MSG_SAFETY_STATUS:
        raise ProtocolError("frame is not SAFETY_STATUS")
    fields = _mixed_fields(frame.payload, fixed32=(5, 9))
    return SafetyStatus(
        bool(fields.get(1, 0)), bool(fields.get(2, 0)), fields.get(3, 0),
        fields.get(4, 0), fields.get(5, 0), fields.get(6, 0),
        fields.get(7, 0), bool(fields.get(8, 0)), fields.get(9, 0),
    )


def parse_robot_state(frame):
    if frame.msg_type != MSG_ROBOT_STATE:
        raise ProtocolError("frame is not ROBOT_STATE")
    fields = _mixed_fields(frame.payload, fixed64=(2,), fixed32=(13,))
    signed = lambda number: decode_sint32(fields.get(number, 0))
    return RobotState(
        fields.get(1, 0), fields.get(2, 0), fields.get(3, 0),
        fields.get(4, 0), signed(5), signed(6), signed(7), signed(8),
        signed(9), signed(10), signed(11), fields.get(12, 0),
        fields.get(13, 0), fields.get(14, 0), signed(15), signed(16),
    )


def self_check():
    if struct.calcsize("<2sBBHHIIIQHH") != HEADER_SIZE:
        raise RuntimeError("protocol header size self-check failed")
    if crc32c(b"123456789") != 0xE3069283:
        raise RuntimeError("CRC32C self-check failed")
    wire = build_wire_frame(MSG_HELLO, b"", 0x12345678, 1, 0)
    if parse_wire_frame(wire) != Frame(MSG_HELLO, 0x12345678, 1, 0, b""):
        raise RuntimeError("wire frame round-trip self-check failed")
