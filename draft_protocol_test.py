#!/usr/bin/env python3
"""Generate and optionally send a Draft v0.1 RescueCar protocol test frame."""

import argparse
import glob
import os
import struct


MAGIC = b"RC"
PROTOCOL_VERSION = 1
HEADER_SIZE = 32
CRC_SIZE = 4
MAX_DECODED_SIZE = 4096

MSG_HELLO = 0x0001

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


def build_decoded_frame(
    payload=b"",
    *,
    msg_type=MSG_HELLO,
    flags=0,
    schema_hash=0,
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
        description="Build a Draft v0.1 empty-payload HELLO test frame."
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
    parser.add_argument(
        "--timestamp-us", type=parse_int, default=DEFAULT_TIMESTAMP_US
    )
    parser.add_argument(
        "--bad-crc",
        action="store_true",
        help="corrupt one CRC byte to test STM32 rejection",
    )
    args = parser.parse_args()

    self_check()
    decoded, wire = build_wire_frame(
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
    print("Draft v0.1 HELLO, empty payload")
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
    with serial.Serial(port=port, baudrate=115200, timeout=0.2) as stm32:
        written = stm32.write(wire)
        stm32.flush()

    print(f"sent {written} bytes to {port}")
    if args.bad_crc:
        print("expected STM32 result: crc_errors increases by 1")
    else:
        print("expected STM32 result: valid_frames increases by 1")


if __name__ == "__main__":
    main()
