#!/usr/bin/env python3
"""Generate and optionally send a Draft v0.1 RescueCar protocol test frame."""

import argparse
import glob
import os
import struct
import time


MAGIC = b"RC"
PROTOCOL_VERSION = 1
SCHEMA_HASH = 0x53D3AA6F
HEADER_SIZE = 32
CRC_SIZE = 4
MAX_DECODED_SIZE = 4096

MSG_HELLO = 0x0001
MSG_HELLO_ACK = 0x0002
MSG_ARM_COMMAND = 0x0010
MSG_CHASSIS_SETPOINT = 0x0011
MSG_SAFE_STOP = 0x0013

ARM_TARGET_DISARMED = 1
ARM_TARGET_ARMED = 2
SAFE_STOP_REASON_USER_REQUEST = 1

DEFAULT_SESSION_ID = 0x12345678
DEFAULT_SEQ = 1
DEFAULT_TIMESTAMP_US = 0


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

    if reply["msg_type"] != MSG_HELLO_ACK:
        raise ValueError("reply is not HELLO_ACK")
    if reply["schema_hash"] != SCHEMA_HASH:
        raise ValueError("HELLO_ACK schema hash does not match")
    if reply["session_id"] != expected_session_id:
        raise ValueError("HELLO_ACK session ID does not match")
    if reply["payload"]:
        raise ValueError("minimal HELLO_ACK payload should be empty")
    print("HELLO_ACK valid: protocol, schema, session and CRC all match")


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


def main():
    parser = argparse.ArgumentParser(
        description="Build Draft v0.1 RescueCar protocol test frames."
    )
    parser.add_argument(
        "message",
        nargs="?",
        choices=("hello", "arm", "disarm", "chassis", "safe-stop"),
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
        "--duration-s",
        type=float,
        default=0.0,
        help="repeat a chassis command for this many seconds; default: send once",
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
        "--bad-crc",
        action="store_true",
        help="corrupt one CRC byte to test STM32 rejection",
    )
    args = parser.parse_args()

    if args.duration_s < 0.0:
        parser.error("--duration-s must be non-negative")
    if args.duration_s > 0.0 and args.message != "chassis":
        parser.error("--duration-s is only supported for chassis messages")
    if args.duration_s > 0.0 and args.send is None:
        parser.error("--duration-s requires --send [PORT]")
    if not 1.0 <= args.rate_hz <= 100.0:
        parser.error("--rate-hz must be in 1..100")
    if args.duration_s > 0.0 and args.bad_crc:
        parser.error("--bad-crc cannot be combined with --duration-s")
    if args.duration_s > 0.0 and args.ttl_ms <= (1000.0 / args.rate_hz):
        parser.error("--ttl-ms must be longer than one repeat interval")

    self_check()
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
    reply_timeout = 1.0 if args.message == "hello" and not args.bad_crc else 0.2
    with serial.Serial(port=port, baudrate=115200, timeout=reply_timeout) as stm32:
        written = stm32.write(wire)
        stm32.flush()
        if args.message == "hello" and not args.bad_crc:
            reply_wire = stm32.read_until(b"\x00")

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
    elif args.message == "chassis" and (
        args.linear_mm_s != 0 or args.angular_mrad_s != 0
    ):
        print("expected STM32 result: commands_dispatched increases by 1")
    else:
        print("expected STM32 result: commands_dispatched increases by 1")


if __name__ == "__main__":
    main()
