from __future__ import annotations

import argparse
import binascii
import time
from pathlib import Path

import serial


def main() -> int:
    parser = argparse.ArgumentParser(description="Debug the board UART setup channel.")
    parser.add_argument("--port", default="COM13", help="Windows serial port, for example COM13")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument(
        "--command",
        default="START",
        help="Command to send. Use START for a non-image handshake test.",
    )
    parser.add_argument(
        "--dump-seconds",
        type=float,
        default=3.0,
        help="How long to print raw incoming data after sending the command.",
    )
    parser.add_argument(
        "--save-snapshot",
        default="debug_snapshot.pgm",
        help="When command is SNAPSHOT, save the returned image payload to this file.",
    )
    args = parser.parse_args()

    with serial.Serial(args.port, args.baudrate, timeout=args.timeout) as ser:
        print(f"opened {ser.name} at {ser.baudrate}")
        ser.reset_input_buffer()
        payload = args.command.rstrip("\r\n").encode("utf-8") + b"\n"
        print(f">>> {payload!r}")
        ser.write(payload)
        ser.flush()

        if args.command.strip().upper() == "SNAPSHOT":
            return read_snapshot(ser, Path(args.save_snapshot), args.timeout)

        deadline = time.monotonic() + args.dump_seconds
        got_any = False
        while time.monotonic() < deadline:
            chunk = ser.read(64)
            if not chunk:
                continue
            got_any = True
            hex_text = binascii.hexlify(chunk, sep=" ").decode("ascii")
            printable = chunk.decode("utf-8", errors="replace")
            print(f"<<< raw={chunk!r}")
            print(f"<<< hex={hex_text}")
            print(f"<<< text={printable!r}")

        if not got_any:
            print("no response before timeout")
            return 1
        return 0


def read_snapshot(ser: serial.Serial, output_path: Path, timeout: float) -> int:
    deadline = time.monotonic() + timeout
    marker = b"SNAPSHOT "
    buffer = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(64)
        if not chunk:
            continue
        buffer.extend(chunk)
        marker_pos = buffer.find(marker)
        if marker_pos < 0:
            if len(buffer) > 8192:
                del buffer[:-len(marker)]
            continue

        line_end = buffer.find(b"\n", marker_pos)
        if line_end < 0:
            continue

        raw_header = bytes(buffer[marker_pos:line_end])
        print(f"<<< header raw={raw_header!r}")
        text = raw_header.decode("utf-8", errors="ignore").strip()

        parts = text.split()
        if len(parts) != 6:
            print(f"bad header: {text}")
            return 1

        preview_width = int(parts[1])
        preview_height = int(parts[2])
        logical_width = int(parts[3])
        logical_height = int(parts[4])
        payload_size = int(parts[5])
        print(
            "snapshot header ok: "
            f"preview={preview_width}x{preview_height}, "
            f"logical={logical_width}x{logical_height}, "
            f"payload={payload_size} bytes"
        )

        payload_start = line_end + 1
        payload = bytes(buffer[payload_start:])
        if len(payload) < payload_size:
            tail, remaining = read_exact_partial(ser, payload_size - len(payload), timeout)
            payload += tail
            if remaining > 0:
                print(f"warning: snapshot payload timeout, {remaining} bytes remaining")
                payload += b"\x00" * remaining
        elif len(payload) > payload_size:
            payload = payload[:payload_size]
        output_path.write_bytes(payload)
        print(f"saved {len(payload)} bytes to {output_path}")
        if payload.startswith(b"P5\n"):
            print("payload looks like a valid PGM image")
        elif payload.startswith(b"P6\n"):
            print("payload looks like a valid PPM image")
        else:
            print("payload does not start with PGM/PPM magic P5/P6")
        return 0

    print("no SNAPSHOT header before timeout")
    return 1


def read_exact_partial(ser: serial.Serial, size: int, timeout: float) -> tuple[bytes, int]:
    deadline = time.monotonic() + timeout
    chunks: list[bytes] = []
    remaining = size
    while remaining > 0:
        if time.monotonic() >= deadline:
            return b"".join(chunks), remaining
        chunk = ser.read(remaining)
        if not chunk:
            continue
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks), 0


if __name__ == "__main__":
    raise SystemExit(main())
