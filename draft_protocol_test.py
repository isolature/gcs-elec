#!/usr/bin/env python3
"""Generate and optionally send a Draft v0.1 RescueCar protocol test frame."""

import argparse
import glob
import os
import struct
import time


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

COMMAND_RESULT_STATUS_ACCEPTED = 1
COMMAND_RESULT_STATUS_REJECTED = 2

COMMAND_RESULT_REASON_NAMES = {
    1: "NONE",
    2: "NOT_READY",
    3: "NOT_ARMED",
    4: "FAULT",
    5: "NOT_CONFIGURED",
    6: "INVALID_ARGUMENT",
}

SAFETY_STATE_NAMES = {
    1: "DISARMED",
    2: "ARMED",
    3: "SAFE_STOP",
    4: "FAULT",
}

SAFETY_STOP_REASON_NAMES = {
    1: "NONE",
    2: "USER_STOP",
    3: "COMMAND_TIMEOUT",
    4: "HEARTBEAT_LOST",
    5: "UPPER_RESTARTED",
    6: "LOWER_RESTARTED",
    7: "DRIVER_FAULT",
    8: "UNDERVOLTAGE",
    9: "INVALID_COMMAND",
    10: "PHYSICAL_ESTOP",
}

GRIPPER_STATE_NAMES = {
    1: "OPEN",
    2: "CLOSED",
    3: "OPENING",
    4: "CLOSING",
    5: "UNKNOWN",
    6: "FAULT",
}

ARM_TARGET_DISARMED = 1
ARM_TARGET_ARMED = 2
SAFE_STOP_REASON_USER_REQUEST = 1
UPPER_CONTROL_STATE_ACTIVE = 3
GRIPPER_TARGET_OPEN = 1
GRIPPER_TARGET_CLOSED = 2

DEFAULT_SESSION_ID = 0x12345678
DEFAULT_SEQ = 1
DEFAULT_TIMESTAMP_US = 0
EXPECTED_FIRMWARE_VERSION = 0x00010000
EXPECTED_CONFIG_HASH = 0x00000000
CAPABILITY_WHEEL_SPEED = 1 << 0


def crc32c(data):
    """Return standard reflected CRC32C (Castagnoli)."""
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x82F63B78
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


def cobs_encode(data):
    """COBS-encode bytes without appending the 0x00 frame delimiter."""
    output = bytearray()
    code_index = 0
    code = 1
    output.append(0)

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
    """Decode COBS bytes that do not include the trailing delimiter."""
    output = bytearray()
    read_index = 0

    while read_index < len(data):
        code = data[read_index]
        if code == 0:
            raise ValueError("zero byte inside COBS data")
        read_index += 1

        block_end = read_index + code - 1
        if block_end > len(data):
            raise ValueError("truncated COBS block")
        output.extend(data[read_index:block_end])
        read_index = block_end

        if code != 0xFF and read_index < len(data):
            output.append(0)

    return bytes(output)


def encode_varint(value):
    if not 0 <= value <= 0xFFFFFFFFFFFFFFFF:
        raise ValueError("varint value is out of range")

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
            raise ValueError("truncated Protobuf varint")
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, offset
    raise ValueError("Protobuf varint is too long")


def decode_sint32(value):
    value &= 0xFFFFFFFF
    return (value >> 1) ^ -(value & 1)


def encode_sint32(value):
    if not -(1 << 31) <= value < (1 << 31):
        raise ValueError("sint32 value is out of range")
    return encode_varint(((value << 1) ^ (value >> 31)) & 0xFFFFFFFF)


def build_command_payload(
    message_name,
    command_id,
    linear_velocity_mm_s,
    angular_velocity_mrad_s,
    ttl_ms,
):
    """Build the small Draft v0.1 Protobuf payloads without a runtime dependency."""
    if message_name == "hello":
        # Proto3 omits both zero-valued Hello fields, so the payload is empty.
        return MSG_HELLO, b""

    if message_name == "chassis":
        if not 1 <= ttl_ms <= 1000:
            raise ValueError("ttl_ms must be in 1..1000")
        payload = bytearray()
        if linear_velocity_mm_s != 0:
            payload += b"\x08" + encode_sint32(linear_velocity_mm_s)
        if angular_velocity_mrad_s != 0:
            payload += b"\x10" + encode_sint32(angular_velocity_mrad_s)
        payload += b"\x18" + encode_varint(ttl_ms)
        return MSG_CHASSIS_SETPOINT, bytes(payload)

    if message_name == "arm":
        target = ARM_TARGET_ARMED
        msg_type = MSG_ARM_COMMAND
    elif message_name == "disarm":
        target = ARM_TARGET_DISARMED
        msg_type = MSG_ARM_COMMAND
    elif message_name == "safe-stop":
        target = SAFE_STOP_REASON_USER_REQUEST
        msg_type = MSG_SAFE_STOP
    else:
        raise ValueError(f"unsupported test message: {message_name}")

    # Field 1: enum (wire type 0). Field 2: command_id (wire type 0).
    payload = b"\x08" + encode_varint(target)
    payload += b"\x10" + encode_varint(command_id)
    return msg_type, payload


def build_decoded_frame(
    payload=b"",
    *,
    msg_type=MSG_HELLO,
    flags=0,
    schema_hash=SCHEMA_HASH,
    session_id=DEFAULT_SESSION_ID,
    seq=DEFAULT_SEQ,
    source_timestamp_us=DEFAULT_TIMESTAMP_US,
):
    """Build header + payload + little-endian CRC32C."""
    if len(payload) > MAX_DECODED_SIZE - HEADER_SIZE - CRC_SIZE:
        raise ValueError("payload is too large for one protocol frame")

    header = struct.pack(
        "<2sBBHHIIIQHH",
        MAGIC,
        PROTOCOL_VERSION,
        flags,
        msg_type,
        HEADER_SIZE,
        schema_hash,
        session_id,
        seq,
        source_timestamp_us,
        len(payload),
        0,
    )
    content = header + payload
    return content + struct.pack("<I", crc32c(content))


def build_wire_frame(**kwargs):
    """Build COBS(header + payload + CRC32C) + 0x00."""
    decoded = build_decoded_frame(**kwargs)
    return decoded, cobs_encode(decoded) + b"\x00"


def find_stm32():
    candidates = (
        glob.glob("/dev/serial/by-id/*STM*")
        + glob.glob("/dev/serial/by-id/*STMicroelectronics*")
    )
    if candidates:
        return candidates[0]
    if os.path.exists("/dev/ttyACM0"):
        return "/dev/ttyACM0"
    raise FileNotFoundError(
        "No STM32 serial device found. Check USB and run: ls /dev/ttyACM*"
    )


def parse_int(value):
    return int(value, 0)


def hex_bytes(data):
    return " ".join(f"{value:02X}" for value in data)


def parse_wire_reply(wire):
    """Validate and describe one zero-delimited Draft v0.1 reply frame."""
    if not wire or wire[-1] != 0:
        raise ValueError("reply is missing the 0x00 frame delimiter")

    decoded = cobs_decode(wire[:-1])
    if len(decoded) < HEADER_SIZE + CRC_SIZE:
        raise ValueError("reply is shorter than the fixed header and CRC")

    (
        magic,
        version,
        flags,
        msg_type,
        header_size,
        schema_hash,
        session_id,
        seq,
        source_timestamp_us,
        payload_length,
        reserved,
    ) = struct.unpack_from("<2sBBHHIIIQHH", decoded, 0)

    expected_length = HEADER_SIZE + payload_length + CRC_SIZE
    if len(decoded) != expected_length:
        raise ValueError(
            f"reply length mismatch: got {len(decoded)}, expected {expected_length}"
        )
    if magic != MAGIC:
        raise ValueError(f"bad reply magic: {magic!r}")
    if version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported reply version: {version}")
    if header_size != HEADER_SIZE:
        raise ValueError(f"bad reply header size: {header_size}")

    stored_crc = struct.unpack_from("<I", decoded, len(decoded) - CRC_SIZE)[0]
    calculated_crc = crc32c(decoded[:-CRC_SIZE])
    if stored_crc != calculated_crc:
        raise ValueError(
            f"bad reply CRC32C: stored 0x{stored_crc:08X}, "
            f"calculated 0x{calculated_crc:08X}"
        )

    return {
        "decoded": decoded,
        "flags": flags,
        "msg_type": msg_type,
        "schema_hash": schema_hash,
        "session_id": session_id,
        "seq": seq,
        "source_timestamp_us": source_timestamp_us,
        "payload": decoded[HEADER_SIZE:-CRC_SIZE],
        "reserved": reserved,
        "stored_crc": stored_crc,
    }


def print_hello_ack(wire, expected_session_id):
    reply = parse_wire_reply(wire)
    print("reply received")
    print(f"reply wire     : {hex_bytes(wire)}")
    print(f"reply decoded  : {hex_bytes(reply['decoded'])}")
    print(f"reply msg type : 0x{reply['msg_type']:04X}")
    print(f"reply schema   : 0x{reply['schema_hash']:08X}")
    print(f"reply session  : 0x{reply['session_id']:08X}")
    print(f"reply seq      : {reply['seq']}")
    print(f"reply CRC32C   : 0x{reply['stored_crc']:08X}")

    fields = validate_hello_ack(reply, expected_session_id)

    print(f"firmware       : {format_firmware_version(fields['firmware_version'])}")
    print(f"boot ID        : 0x{fields['boot_id']:08X}")
    print(f"config hash    : 0x{fields['config_hash']:08X}")
    print(f"capabilities   : 0x{fields['capabilities']:08X}")
    print("HELLO_ACK compatible: ARM may proceed")


def validate_hello_ack(reply, expected_session_id):
    if reply["msg_type"] != MSG_HELLO_ACK:
        raise ValueError("reply is not HELLO_ACK")
    if reply["schema_hash"] != SCHEMA_HASH:
        raise ValueError("HELLO_ACK schema hash does not match")
    if reply["session_id"] != expected_session_id:
        raise ValueError("HELLO_ACK session ID does not match")
    fields = decode_hello_ack(reply["payload"])
    if fields["protocol_version"] != PROTOCOL_VERSION:
        raise ValueError("HELLO_ACK payload protocol version does not match")
    if fields["firmware_version"] != EXPECTED_FIRMWARE_VERSION:
        raise ValueError(
            "HELLO_ACK firmware version is incompatible: "
            f"0x{fields['firmware_version']:08X}"
        )
    if fields["boot_id"] == 0:
        raise ValueError("HELLO_ACK boot ID must be non-zero")
    if fields["config_hash"] != EXPECTED_CONFIG_HASH:
        raise ValueError(
            "HELLO_ACK config hash is incompatible: "
            f"0x{fields['config_hash']:08X}"
        )
    if not fields["capabilities"] & CAPABILITY_WHEEL_SPEED:
        raise ValueError("HELLO_ACK does not advertise wheel-speed feedback")
    return fields


def decode_hello_ack(payload):
    fields = {}
    offset = 0
    while offset < len(payload):
        key, offset = decode_varint(payload, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 1 and wire_type == 0:
            fields[field_number], offset = decode_varint(payload, offset)
        elif field_number in (2, 3, 4, 5) and wire_type == 5:
            if offset + 4 > len(payload):
                raise ValueError("truncated HELLO_ACK fixed32 field")
            fields[field_number] = struct.unpack_from("<I", payload, offset)[0]
            offset += 4
        else:
            raise ValueError(
                f"unsupported HELLO_ACK field {field_number} wire type {wire_type}"
            )

    missing = [field for field in range(1, 6) if field not in fields]
    if missing:
        raise ValueError(f"HELLO_ACK missing fields: {missing}")
    return {
        "protocol_version": fields[1],
        "firmware_version": fields[2],
        "boot_id": fields[3],
        "config_hash": fields[4],
        "capabilities": fields[5],
    }


def format_firmware_version(version):
    return f"{version >> 16}.{(version >> 8) & 0xFF}.{version & 0xFF}"


def build_hello_ack_payload(
    *,
    protocol_version=PROTOCOL_VERSION,
    firmware_version=EXPECTED_FIRMWARE_VERSION,
    boot_id=1,
    config_hash=EXPECTED_CONFIG_HASH,
    capabilities=CAPABILITY_WHEEL_SPEED,
):
    return (
        b"\x08"
        + encode_varint(protocol_version)
        + b"\x15"
        + struct.pack("<I", firmware_version)
        + b"\x1D"
        + struct.pack("<I", boot_id)
        + b"\x25"
        + struct.pack("<I", config_hash)
        + b"\x2D"
        + struct.pack("<I", capabilities)
    )


def print_command_result(wire, expected_session_id, expected_command_id):
    reply = parse_wire_reply(wire)
    print("reply received")
    print(f"reply wire     : {hex_bytes(wire)}")
    print(f"reply decoded  : {hex_bytes(reply['decoded'])}")
    print(f"reply msg type : 0x{reply['msg_type']:04X}")
    print(f"reply schema   : 0x{reply['schema_hash']:08X}")
    print(f"reply session  : 0x{reply['session_id']:08X}")
    print(f"reply seq      : {reply['seq']}")
    print(f"reply CRC32C   : 0x{reply['stored_crc']:08X}")

    if reply["msg_type"] != MSG_COMMAND_RESULT:
        raise ValueError("reply is not COMMAND_RESULT")
    if reply["schema_hash"] != SCHEMA_HASH:
        raise ValueError("COMMAND_RESULT schema hash does not match")
    if reply["session_id"] != expected_session_id:
        raise ValueError("COMMAND_RESULT session ID does not match")

    fields = {}
    offset = 0
    payload = reply["payload"]
    while offset < len(payload):
        key, offset = decode_varint(payload, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 0 or wire_type != 0:
            raise ValueError("unsupported COMMAND_RESULT Protobuf field")
        fields[field_number], offset = decode_varint(payload, offset)

    command_id = fields.get(1, 0)
    status = fields.get(2, 0)
    reason = fields.get(3, 0)
    status_name = {
        COMMAND_RESULT_STATUS_ACCEPTED: "ACCEPTED",
        COMMAND_RESULT_STATUS_REJECTED: "REJECTED",
        3: "COMPLETED",
    }.get(status, f"UNKNOWN({status})")
    reason_name = COMMAND_RESULT_REASON_NAMES.get(reason, f"UNKNOWN({reason})")
    print(f"command id     : {command_id}")
    print(f"command status : {status_name}")
    print(f"command reason : {reason_name}")

    if command_id != expected_command_id:
        raise ValueError("COMMAND_RESULT command_id does not match")
    if status not in (
        COMMAND_RESULT_STATUS_ACCEPTED,
        COMMAND_RESULT_STATUS_REJECTED,
    ):
        raise ValueError("COMMAND_RESULT status is invalid")
    if reason not in COMMAND_RESULT_REASON_NAMES:
        raise ValueError("COMMAND_RESULT reason is invalid")
    print("COMMAND_RESULT valid: command ID, status, reason and CRC all match")
    return status, reason


def build_heartbeat_payload(heartbeat_seq):
    if not 1 <= heartbeat_seq <= 0xFFFFFFFF:
        raise ValueError("heartbeat_seq must be in 1..0xFFFFFFFF")
    return (
        b"\x08"
        + encode_varint(heartbeat_seq)
        + b"\x10"
        + encode_varint(UPPER_CONTROL_STATE_ACTIVE)
    )


def build_gripper_payload(target_name, command_id):
    if command_id == 0:
        raise ValueError("command_id must be non-zero")
    target = (
        GRIPPER_TARGET_OPEN
        if target_name == "open"
        else GRIPPER_TARGET_CLOSED
    )
    return b"\x08" + encode_varint(target) + b"\x10" + encode_varint(command_id)


def decode_heartbeat_ack(payload):
    fields = {}
    offset = 0
    while offset < len(payload):
        key, offset = decode_varint(payload, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 1 and wire_type == 0:
            fields[1], offset = decode_varint(payload, offset)
        elif field_number == 2 and wire_type == 1:
            if offset + 8 > len(payload):
                raise ValueError("truncated HEARTBEAT_ACK timestamp")
            fields[2] = struct.unpack_from("<Q", payload, offset)[0]
            offset += 8
        else:
            raise ValueError("unsupported HEARTBEAT_ACK Protobuf field")

    heartbeat_seq = fields.get(1, 0)
    if heartbeat_seq == 0 or 2 not in fields:
        raise ValueError("HEARTBEAT_ACK is missing a required field")
    return heartbeat_seq, fields[2]


def print_heartbeat_ack(wire, expected_session_id):
    reply = parse_wire_reply(wire)
    if reply["msg_type"] != MSG_HEARTBEAT_ACK:
        raise ValueError("reply is not HEARTBEAT_ACK")
    if reply["schema_hash"] != SCHEMA_HASH:
        raise ValueError("HEARTBEAT_ACK schema hash does not match")
    if reply["session_id"] != expected_session_id:
        raise ValueError("HEARTBEAT_ACK session ID does not match")

    heartbeat_seq, lower_timestamp_us = decode_heartbeat_ack(reply["payload"])
    print(
        "HEARTBEAT_ACK "
        f"reply_seq={reply['seq']} heartbeat_seq={heartbeat_seq} "
        f"lower_t_us={lower_timestamp_us}"
    )
    return heartbeat_seq


def decode_safety_status(payload):
    fields = {}
    offset = 0
    while offset < len(payload):
        key, offset = decode_varint(payload, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 0:
            raise ValueError("SAFETY_STATUS has field number zero")

        if wire_type == 0:
            fields[field_number], offset = decode_varint(payload, offset)
        elif wire_type == 5:
            if offset + 4 > len(payload):
                raise ValueError("truncated SAFETY_STATUS fixed32 field")
            fields[field_number] = struct.unpack_from("<I", payload, offset)[0]
            offset += 4
        else:
            raise ValueError("unsupported SAFETY_STATUS Protobuf wire type")

    return {
        "link_ready": bool(fields.get(1, 0)),
        "armed": bool(fields.get(2, 0)),
        "safety_state": fields.get(3, 0),
        "stop_reason": fields.get(4, 0),
        "fault_bits": fields.get(5, 0),
        "motion_age_ms": fields.get(6, 0),
        "battery_mv": fields.get(7, 0),
        "watchdog_triggered": bool(fields.get(8, 0)),
        "validity_flags": fields.get(9, 0),
    }


def print_safety_status(wire, expected_session_id):
    reply = parse_wire_reply(wire)
    if reply["msg_type"] != MSG_SAFETY_STATUS:
        raise ValueError("reply is not SAFETY_STATUS")
    if reply["schema_hash"] != SCHEMA_HASH:
        raise ValueError("SAFETY_STATUS schema hash does not match")
    if reply["session_id"] != expected_session_id:
        raise ValueError("SAFETY_STATUS session ID does not match")

    status = decode_safety_status(reply["payload"])
    state_name = SAFETY_STATE_NAMES.get(
        status["safety_state"], f"UNKNOWN({status['safety_state']})"
    )
    reason_name = SAFETY_STOP_REASON_NAMES.get(
        status["stop_reason"], f"UNKNOWN({status['stop_reason']})"
    )
    print(
        "SAFETY_STATUS  "
        f"seq={reply['seq']} "
        f"link_ready={int(status['link_ready'])} "
        f"armed={int(status['armed'])} "
        f"state={state_name} "
        f"reason={reason_name} "
        f"age_ms={status['motion_age_ms']} "
        f"validity=0x{status['validity_flags']:08X}"
    )
    return status


def decode_robot_state(payload):
    fields = {}
    offset = 0
    while offset < len(payload):
        key, offset = decode_varint(payload, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 0:
            raise ValueError("ROBOT_STATE has field number zero")

        if wire_type == 0:
            fields[field_number], offset = decode_varint(payload, offset)
        elif wire_type == 1:
            if offset + 8 > len(payload):
                raise ValueError("truncated ROBOT_STATE fixed64 field")
            fields[field_number] = struct.unpack_from("<Q", payload, offset)[0]
            offset += 8
        elif wire_type == 5:
            if offset + 4 > len(payload):
                raise ValueError("truncated ROBOT_STATE fixed32 field")
            fields[field_number] = struct.unpack_from("<I", payload, offset)[0]
            offset += 4
        else:
            raise ValueError("unsupported ROBOT_STATE Protobuf wire type")

    return {
        "state_seq": fields.get(1, 0),
        "timestamp_us": fields.get(2, 0),
        "coordinate_frame": fields.get(3, 0),
        "last_command_seq": fields.get(4, 0),
        "linear_mm_s": decode_sint32(fields.get(5, 0)),
        "angular_mrad_s": decode_sint32(fields.get(6, 0)),
        "delta_x_mm": decode_sint32(fields.get(7, 0)),
        "delta_y_mm": decode_sint32(fields.get(8, 0)),
        "delta_yaw_mrad": decode_sint32(fields.get(9, 0)),
        "yaw_mrad": decode_sint32(fields.get(10, 0)),
        "yaw_rate_mrad_s": decode_sint32(fields.get(11, 0)),
        "gripper_state": fields.get(12, 0),
        "validity_flags": fields.get(13, 0),
        "gripper_target_percent": fields.get(14, 0),
        "left_rpm_x10": decode_sint32(fields.get(15, 0)),
        "right_rpm_x10": decode_sint32(fields.get(16, 0)),
    }


def print_robot_state(wire, expected_session_id):
    reply = parse_wire_reply(wire)
    if reply["msg_type"] != MSG_ROBOT_STATE:
        raise ValueError("reply is not ROBOT_STATE")
    if reply["schema_hash"] != SCHEMA_HASH:
        raise ValueError("ROBOT_STATE schema hash does not match")
    if reply["session_id"] != expected_session_id:
        raise ValueError("ROBOT_STATE session ID does not match")

    state = decode_robot_state(reply["payload"])
    gripper_name = GRIPPER_STATE_NAMES.get(
        state["gripper_state"], f"UNKNOWN({state['gripper_state']})"
    )
    frame_name = "BASE_LINK" if state["coordinate_frame"] == 1 else (
        f"UNKNOWN({state['coordinate_frame']})"
    )
    print(
        "ROBOT_STATE   "
        f"seq={reply['seq']} "
        f"state_seq={state['state_seq']} "
        f"t_us={state['timestamp_us']} "
        f"frame={frame_name} "
        f"last_cmd={state['last_command_seq']} "
        f"linear={state['linear_mm_s']}mm/s "
        f"angular={state['angular_mrad_s']}mrad/s "
        f"left_rpm_x10={state['left_rpm_x10']} "
        f"right_rpm_x10={state['right_rpm_x10']} "
        f"gripper={gripper_name} "
        f"target={state['gripper_target_percent']}% "
        f"validity=0x{state['validity_flags']:08X}"
    )
    return state


def read_next_wire_frame(stm32, buffered, deadline):
    while time.monotonic() < deadline:
        delimiter = buffered.find(0)
        if delimiter >= 0:
            wire = bytes(buffered[: delimiter + 1])
            del buffered[: delimiter + 1]
            if len(wire) > 1:
                return wire
            continue

        chunk = stm32.read(256)
        if chunk:
            buffered.extend(chunk)
    return b""


def wait_for_reply(stm32, buffered, expected_msg_type, session_id, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        wire = read_next_wire_frame(stm32, buffered, deadline)
        if not wire:
            break
        reply = parse_wire_reply(wire)
        if reply["msg_type"] == MSG_SAFETY_STATUS:
            print_safety_status(wire, session_id)
            continue
        if reply["msg_type"] == MSG_ROBOT_STATE:
            print_robot_state(wire, session_id)
            continue
        if reply["msg_type"] == expected_msg_type:
            return wire
        print(f"ignored unexpected reply type 0x{reply['msg_type']:04X}")
    return b""


def listen_for_telemetry(stm32, buffered, session_id, duration_s):
    deadline = time.monotonic() + duration_s
    safety_count = 0
    robot_count = 0
    while time.monotonic() < deadline:
        wire = read_next_wire_frame(stm32, buffered, deadline)
        if not wire:
            break
        reply = parse_wire_reply(wire)
        if reply["msg_type"] == MSG_SAFETY_STATUS:
            print_safety_status(wire, session_id)
            safety_count += 1
        elif reply["msg_type"] == MSG_ROBOT_STATE:
            print_robot_state(wire, session_id)
            robot_count += 1
    print(
        f"received {safety_count} SAFETY_STATUS and "
        f"{robot_count} ROBOT_STATE frame(s) in {duration_s:g} s"
    )


def run_watchdog_test(serial_module, port, args):
    """Establish a session, feed the watchdog, then prove loss is detected."""
    receive_buffer = bytearray()
    frame_seq = args.seq
    heartbeat_seq = 1
    heartbeat_acks = 0
    duration_s = args.duration_s if args.duration_s > 0.0 else 2.0
    period_s = 1.0 / args.rate_hz
    final_status = None

    def send_frame(stm32, msg_type, payload):
        nonlocal frame_seq
        _, wire = build_wire_frame(
            payload=payload,
            msg_type=msg_type,
            session_id=args.session_id,
            seq=frame_seq,
            source_timestamp_us=args.timestamp_us,
        )
        frame_seq = (frame_seq + 1) & 0xFFFFFFFF
        stm32.write(wire)
        stm32.flush()

    print("Draft v0.1 HEARTBEAT WATCHDOG TEST")
    print(f"schema hash    : 0x{SCHEMA_HASH:08X}")
    print(f"heartbeat rate : {args.rate_hz:g} Hz")
    print(f"feed duration  : {duration_s:g} s")
    print("loss timeout   : 300 ms")

    with serial_module.Serial(port=port, baudrate=115200, timeout=0.02) as stm32:
        stm32.reset_input_buffer()

        send_frame(stm32, MSG_HELLO, b"")
        wire = wait_for_reply(
            stm32, receive_buffer, MSG_HELLO_ACK, args.session_id, 1.0
        )
        if not wire:
            raise SystemExit("watchdog test failed: no HELLO_ACK")
        print_hello_ack(wire, args.session_id)

        _, arm_payload = build_command_payload(
            "arm", args.command_id, 0, 0, args.ttl_ms
        )
        send_frame(stm32, MSG_ARM_COMMAND, arm_payload)
        wire = wait_for_reply(
            stm32, receive_buffer, MSG_COMMAND_RESULT, args.session_id, 1.0
        )
        if not wire:
            raise SystemExit("watchdog test failed: no ARM COMMAND_RESULT")
        print_command_result(wire, args.session_id, args.command_id)

        start_time = time.monotonic()
        stop_time = start_time + duration_s
        next_send_time = start_time
        while next_send_time < stop_time:
            delay_s = next_send_time - time.monotonic()
            if delay_s > 0.0:
                time.sleep(delay_s)

            send_frame(
                stm32,
                MSG_HEARTBEAT,
                build_heartbeat_payload(heartbeat_seq),
            )
            heartbeat_seq += 1

            # If a motion target was requested, refresh it beside the heartbeat.
            if args.linear_mm_s != 0 or args.angular_mrad_s != 0:
                _, chassis_payload = build_command_payload(
                    "chassis",
                    args.command_id,
                    args.linear_mm_s,
                    args.angular_mrad_s,
                    args.ttl_ms,
                )
                send_frame(stm32, MSG_CHASSIS_SETPOINT, chassis_payload)

            next_send_time += period_s
            read_deadline = min(next_send_time, stop_time)
            while time.monotonic() < read_deadline:
                wire = read_next_wire_frame(stm32, receive_buffer, read_deadline)
                if not wire:
                    break
                reply = parse_wire_reply(wire)
                if reply["msg_type"] == MSG_HEARTBEAT_ACK:
                    print_heartbeat_ack(wire, args.session_id)
                    heartbeat_acks += 1
                elif reply["msg_type"] == MSG_SAFETY_STATUS:
                    print_safety_status(wire, args.session_id)
                elif reply["msg_type"] == MSG_ROBOT_STATE:
                    print_robot_state(wire, args.session_id)

        print("heartbeat transmission stopped; waiting for HEARTBEAT_LOST...")
        loss_deadline = time.monotonic() + 0.8
        while time.monotonic() < loss_deadline:
            wire = read_next_wire_frame(stm32, receive_buffer, loss_deadline)
            if not wire:
                break
            reply = parse_wire_reply(wire)
            if reply["msg_type"] == MSG_HEARTBEAT_ACK:
                print_heartbeat_ack(wire, args.session_id)
                heartbeat_acks += 1
            elif reply["msg_type"] == MSG_SAFETY_STATUS:
                status = print_safety_status(wire, args.session_id)
                if (
                    status["safety_state"] == 3
                    and status["stop_reason"] == 4
                    and status["watchdog_triggered"]
                ):
                    final_status = status
                    break
            elif reply["msg_type"] == MSG_ROBOT_STATE:
                print_robot_state(wire, args.session_id)

    print(f"received {heartbeat_acks} HEARTBEAT_ACK frame(s)")
    if final_status is None:
        raise SystemExit(
            "watchdog test failed: no SAFE_STOP / HEARTBEAT_LOST status received"
        )
    print(
        "WATCHDOG TEST PASSED: heartbeat loss caused SAFE_STOP, "
        "HEARTBEAT_LOST and watchdog_triggered=1"
    )


def run_gripper_test(serial_module, port, args):
    """Establish control, command one gripper target, then disarm cleanly."""
    receive_buffer = bytearray()
    frame_seq = args.seq
    heartbeat_seq = 1
    duration_s = args.duration_s if args.duration_s > 0.0 else 1.5
    period_s = 1.0 / args.rate_hz
    expected_percent = 100 if args.gripper_target == "open" else 0
    target_seen = False

    def send_frame(stm32, msg_type, payload):
        nonlocal frame_seq
        _, wire = build_wire_frame(
            payload=payload,
            msg_type=msg_type,
            session_id=args.session_id,
            seq=frame_seq,
            source_timestamp_us=args.timestamp_us,
        )
        frame_seq = (frame_seq + 1) & 0xFFFFFFFF
        stm32.write(wire)
        stm32.flush()

    def wait_for_gripper_result(stm32, timeout_s):
        nonlocal heartbeat_seq, target_seen
        deadline = time.monotonic() + timeout_s
        next_heartbeat = time.monotonic() + period_s

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                send_frame(
                    stm32,
                    MSG_HEARTBEAT,
                    build_heartbeat_payload(heartbeat_seq),
                )
                heartbeat_seq += 1
                next_heartbeat += period_s

            wire = read_next_wire_frame(
                stm32, receive_buffer, min(deadline, next_heartbeat)
            )
            if not wire:
                continue

            reply = parse_wire_reply(wire)
            if reply["msg_type"] == MSG_COMMAND_RESULT:
                return wire
            if reply["msg_type"] == MSG_HEARTBEAT_ACK:
                print_heartbeat_ack(wire, args.session_id)
            elif reply["msg_type"] == MSG_SAFETY_STATUS:
                print_safety_status(wire, args.session_id)
            elif reply["msg_type"] == MSG_ROBOT_STATE:
                state = print_robot_state(wire, args.session_id)
                if state["gripper_target_percent"] == expected_percent:
                    target_seen = True
        return b""

    print("Draft v0.1 GRIPPER TEST")
    print(f"schema hash    : 0x{SCHEMA_HASH:08X}")
    print(f"target         : {args.gripper_target.upper()}")
    print(f"expected target: {expected_percent}%")

    with serial_module.Serial(port=port, baudrate=115200, timeout=0.02) as stm32:
        stm32.reset_input_buffer()

        send_frame(stm32, MSG_HELLO, b"")
        wire = wait_for_reply(
            stm32, receive_buffer, MSG_HELLO_ACK, args.session_id, 1.0
        )
        if not wire:
            raise SystemExit("gripper test failed: no HELLO_ACK")
        print_hello_ack(wire, args.session_id)

        _, arm_payload = build_command_payload(
            "arm", args.command_id, 0, 0, args.ttl_ms
        )
        send_frame(stm32, MSG_ARM_COMMAND, arm_payload)
        wire = wait_for_reply(
            stm32, receive_buffer, MSG_COMMAND_RESULT, args.session_id, 1.0
        )
        if not wire:
            raise SystemExit("gripper test failed: no ARM COMMAND_RESULT")
        status, _ = print_command_result(
            wire, args.session_id, args.command_id
        )
        if status != COMMAND_RESULT_STATUS_ACCEPTED:
            raise SystemExit("gripper test failed: ARM was rejected")

        # Feed the newly armed watchdog before sending the actuator command.
        send_frame(
            stm32,
            MSG_HEARTBEAT,
            build_heartbeat_payload(heartbeat_seq),
        )
        heartbeat_seq += 1
        send_frame(
            stm32,
            MSG_GRIPPER_COMMAND,
            build_gripper_payload(args.gripper_target, args.command_id + 1),
        )
        wire = wait_for_gripper_result(stm32, 1.0)
        if not wire:
            raise SystemExit("gripper test failed: no GRIPPER COMMAND_RESULT")
        status, _ = print_command_result(
            wire, args.session_id, args.command_id + 1
        )
        if status != COMMAND_RESULT_STATUS_ACCEPTED:
            raise SystemExit("gripper test failed: GRIPPER_COMMAND was rejected")

        start_time = time.monotonic()
        stop_time = start_time + duration_s
        next_send_time = start_time
        while next_send_time < stop_time:
            delay_s = next_send_time - time.monotonic()
            if delay_s > 0.0:
                time.sleep(delay_s)
            send_frame(
                stm32,
                MSG_HEARTBEAT,
                build_heartbeat_payload(heartbeat_seq),
            )
            heartbeat_seq += 1
            next_send_time += period_s

            read_deadline = min(next_send_time, stop_time)
            while time.monotonic() < read_deadline:
                wire = read_next_wire_frame(stm32, receive_buffer, read_deadline)
                if not wire:
                    break
                reply = parse_wire_reply(wire)
                if reply["msg_type"] == MSG_HEARTBEAT_ACK:
                    print_heartbeat_ack(wire, args.session_id)
                elif reply["msg_type"] == MSG_SAFETY_STATUS:
                    print_safety_status(wire, args.session_id)
                elif reply["msg_type"] == MSG_ROBOT_STATE:
                    state = print_robot_state(wire, args.session_id)
                    if state["gripper_target_percent"] == expected_percent:
                        target_seen = True

        # End the test deliberately so watchdog loss is not mistaken for a fault.
        _, disarm_payload = build_command_payload(
            "disarm", args.command_id + 2, 0, 0, args.ttl_ms
        )
        send_frame(stm32, MSG_ARM_COMMAND, disarm_payload)
        wire = wait_for_reply(
            stm32, receive_buffer, MSG_COMMAND_RESULT, args.session_id, 1.0
        )
        if not wire:
            raise SystemExit("gripper test failed: no DISARM COMMAND_RESULT")
        print_command_result(wire, args.session_id, args.command_id + 2)

    if not target_seen:
        raise SystemExit(
            "gripper test failed: ROBOT_STATE did not report the target percent"
        )
    print(
        "GRIPPER TEST PASSED: command accepted, target reported, "
        "and the test ended DISARMED"
    )


def self_check():
    if struct.calcsize("<2sBBHHIIIQHH") != HEADER_SIZE:
        raise RuntimeError("Draft header is not 32 bytes")
    if crc32c(b"123456789") != 0xE3069283:
        raise RuntimeError("CRC32C self-check failed")
    if cobs_encode(bytes.fromhex("11 00 22")) != bytes.fromhex(
        "02 11 02 22"
    ):
        raise RuntimeError("COBS self-check failed")

    samples = (
        b"",
        b"\x00",
        bytes.fromhex("11 00 22"),
        bytes(range(256)),
        b"\x55" * 254,
        b"\x55" * 255 + b"\x00",
    )
    for sample in samples:
        if cobs_decode(cobs_encode(sample)) != sample:
            raise RuntimeError("COBS round-trip self-check failed")

    hello_ack = decode_hello_ack(build_hello_ack_payload(boot_id=0x12345678))
    if hello_ack != {
        "protocol_version": PROTOCOL_VERSION,
        "firmware_version": EXPECTED_FIRMWARE_VERSION,
        "boot_id": 0x12345678,
        "config_hash": EXPECTED_CONFIG_HASH,
        "capabilities": CAPABILITY_WHEEL_SPEED,
    }:
        raise RuntimeError("HELLO_ACK payload self-check failed")

    for incompatible_payload in (
        build_hello_ack_payload(protocol_version=PROTOCOL_VERSION + 1),
        build_hello_ack_payload(firmware_version=0x00020000),
        build_hello_ack_payload(boot_id=0),
        build_hello_ack_payload(config_hash=1),
        build_hello_ack_payload(capabilities=0),
    ):
        _, incompatible_wire = build_wire_frame(
            msg_type=MSG_HELLO_ACK,
            payload=incompatible_payload,
        )
        try:
            validate_hello_ack(
                parse_wire_reply(incompatible_wire), DEFAULT_SESSION_ID
            )
        except ValueError:
            pass
        else:
            raise RuntimeError("incompatible HELLO_ACK was accepted")


def main():
    parser = argparse.ArgumentParser(
        description="Build Draft v0.1 RescueCar protocol test frames."
    )
    parser.add_argument(
        "message",
        nargs="?",
        choices=(
            "hello",
            "arm",
            "disarm",
            "chassis",
            "safe-stop",
            "watchdog-test",
            "gripper-test",
        ),
        default="hello",
        help="message to build; default: hello",
    )
    parser.add_argument(
        "--send",
        nargs="?",
        const="auto",
        metavar="PORT",
        help="send once; omit PORT to auto-detect the STM32",
    )
    parser.add_argument("--session-id", type=parse_int, default=DEFAULT_SESSION_ID)
    parser.add_argument("--seq", type=parse_int, default=DEFAULT_SEQ)
    parser.add_argument("--command-id", type=parse_int, default=1)
    parser.add_argument("--linear-mm-s", type=int, default=0)
    parser.add_argument("--angular-mrad-s", type=int, default=0)
    parser.add_argument("--ttl-ms", type=int, default=200)
    parser.add_argument(
        "--gripper-target",
        choices=("open", "closed"),
        default="open",
        help="target used by gripper-test; default: open",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help=(
            "repeat a chassis command, feed watchdog-test heartbeats, or "
            "observe gripper-test for this many seconds"
        ),
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=10.0,
        help="repeat rate used with --duration-s; default: 10 Hz",
    )
    parser.add_argument(
        "--timestamp-us", type=parse_int, default=DEFAULT_TIMESTAMP_US
    )
    parser.add_argument(
        "--listen-s",
        type=float,
        default=0.0,
        help="after the reply, print safety and robot state for this many seconds",
    )
    parser.add_argument(
        "--bad-crc",
        action="store_true",
        help="corrupt one CRC byte to test STM32 rejection",
    )
    args = parser.parse_args()

    if args.duration_s < 0.0:
        parser.error("--duration-s must be non-negative")
    if args.listen_s < 0.0:
        parser.error("--listen-s must be non-negative")
    if args.listen_s > 0.0 and args.send is None:
        parser.error("--listen-s requires --send [PORT]")
    if args.duration_s > 0.0 and args.message not in (
        "chassis",
        "watchdog-test",
        "gripper-test",
    ):
        parser.error(
            "--duration-s is only supported for chassis, watchdog-test "
            "and gripper-test"
        )
    if args.duration_s > 0.0 and args.send is None:
        parser.error("--duration-s requires --send [PORT]")
    if args.message == "watchdog-test" and args.send is None:
        parser.error("watchdog-test requires --send [PORT]")
    if args.message == "gripper-test" and args.send is None:
        parser.error("gripper-test requires --send [PORT]")
    if not 1.0 <= args.rate_hz <= 100.0:
        parser.error("--rate-hz must be in 1..100")
    if args.duration_s > 0.0 and args.bad_crc:
        parser.error("--bad-crc cannot be combined with --duration-s")
    if args.message == "watchdog-test" and args.bad_crc:
        parser.error("--bad-crc cannot be combined with watchdog-test")
    if args.message == "gripper-test" and args.bad_crc:
        parser.error("--bad-crc cannot be combined with gripper-test")
    if args.duration_s > 0.0 and args.ttl_ms <= (1000.0 / args.rate_hz):
        parser.error("--ttl-ms must be longer than one repeat interval")
    if args.message in ("watchdog-test", "gripper-test") and (
        1000.0 / args.rate_hz
    ) >= 300.0:
        parser.error("heartbeat interval must be below 300 ms")
    if args.message == "gripper-test" and not 1 <= args.command_id <= 0xFFFFFFFD:
        parser.error("gripper-test command ID must be in 1..0xFFFFFFFD")

    self_check()
    if args.message in ("watchdog-test", "gripper-test"):
        try:
            import serial
        except ImportError as exc:
            raise SystemExit(
                "pyserial is required for --send: pip install pyserial"
            ) from exc

        port = find_stm32() if args.send == "auto" else args.send
        if args.message == "watchdog-test":
            run_watchdog_test(serial, port, args)
        else:
            run_gripper_test(serial, port, args)
        return

    msg_type, payload = build_command_payload(
        args.message,
        args.command_id,
        args.linear_mm_s,
        args.angular_mrad_s,
        args.ttl_ms,
    )
    decoded, wire = build_wire_frame(
        payload=payload,
        msg_type=msg_type,
        session_id=args.session_id,
        seq=args.seq,
        source_timestamp_us=args.timestamp_us,
    )

    if args.bad_crc:
        corrupted = bytearray(decoded)
        corrupted[-1] ^= 0x01
        decoded = bytes(corrupted)
        wire = cobs_encode(decoded) + b"\x00"

    stored_crc = struct.unpack_from("<I", decoded, len(decoded) - CRC_SIZE)[0]
    print(f"Draft v0.1 {args.message.upper()}")
    print(f"schema hash    : 0x{SCHEMA_HASH:08X}")
    print(f"payload        : {hex_bytes(payload) if payload else '(empty)'}")
    print(f"decoded length : {len(decoded)} bytes")
    print(f"stored CRC32C  : 0x{stored_crc:08X}")
    print(f"decoded frame  : {hex_bytes(decoded)}")
    print(f"wire length    : {len(wire)} bytes")
    print(f"wire frame     : {hex_bytes(wire)}")

    if args.send is None:
        print("not sent; use --send [PORT] to transmit once")
        return

    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required for --send: pip install pyserial") from exc

    port = find_stm32() if args.send == "auto" else args.send
    if args.duration_s > 0.0:
        period_s = 1.0 / args.rate_hz
        start_time = time.monotonic()
        stop_time = start_time + args.duration_s
        next_send_time = start_time
        sent_frames = 0
        written = 0

        with serial.Serial(port=port, baudrate=115200, timeout=0.2) as stm32:
            while next_send_time < stop_time:
                delay_s = next_send_time - time.monotonic()
                if delay_s > 0.0:
                    time.sleep(delay_s)

                _, repeated_wire = build_wire_frame(
                    payload=payload,
                    msg_type=msg_type,
                    session_id=args.session_id,
                    seq=(args.seq + sent_frames) & 0xFFFFFFFF,
                    source_timestamp_us=args.timestamp_us,
                )
                written += stm32.write(repeated_wire)
                sent_frames += 1
                next_send_time = start_time + sent_frames * period_s
            stm32.flush()

        print(
            f"sent {sent_frames} frames ({written} bytes) to {port} "
            f"at {args.rate_hz:g} Hz for {args.duration_s:g} s"
        )
        print(
            f"expected STM32 result: motion continues during the stream, "
            f"then stops within {args.ttl_ms} ms"
        )
        return

    reply_wire = b""
    receive_buffer = bytearray()
    expects_reply = args.message in (
        "hello",
        "arm",
        "disarm",
        "safe-stop",
    ) and not args.bad_crc
    reply_timeout = 1.0 if expects_reply else 0.2
    with serial.Serial(port=port, baudrate=115200, timeout=0.05) as stm32:
        stm32.reset_input_buffer()
        written = stm32.write(wire)
        stm32.flush()
        if expects_reply:
            expected_type = (
                MSG_HELLO_ACK if args.message == "hello" else MSG_COMMAND_RESULT
            )
            reply_wire = wait_for_reply(
                stm32,
                receive_buffer,
                expected_type,
                args.session_id,
                reply_timeout,
            )

        if args.listen_s > 0.0:
            listen_for_telemetry(
                stm32,
                receive_buffer,
                args.session_id,
                args.listen_s,
            )

    print(f"sent {written} bytes to {port}")
    if args.bad_crc:
        print("expected STM32 result: crc_errors increases by 1")
    elif args.message == "hello":
        if not reply_wire:
            raise SystemExit("no HELLO_ACK received within 1.0 s")
        try:
            print_hello_ack(reply_wire, args.session_id)
        except ValueError as exc:
            raise SystemExit(f"invalid HELLO_ACK: {exc}") from exc
    elif args.message in ("arm", "disarm", "safe-stop"):
        if not reply_wire:
            raise SystemExit("no COMMAND_RESULT received within 1.0 s")
        try:
            print_command_result(
                reply_wire,
                args.session_id,
                args.command_id,
            )
        except ValueError as exc:
            raise SystemExit(f"invalid COMMAND_RESULT: {exc}") from exc
    elif args.message == "chassis" and (
        args.linear_mm_s != 0 or args.angular_mrad_s != 0
    ):
        print("expected STM32 result: commands_dispatched increases by 1")
    else:
        print("expected STM32 result: commands_dispatched increases by 1")


if __name__ == "__main__":
    main()
