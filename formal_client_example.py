#!/usr/bin/env python3
"""Wheels-off integration example for the unified formal control chain."""

import argparse
import time

from rescue_control import CompetitionLowerLink, ControlCore, SafeStopReason


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        required=True,
        help="exact /dev/serial/by-id/... path for the STM32",
    )
    args = parser.parse_args()

    core = ControlCore(CompetitionLowerLink(port=args.port))
    try:
        core.connect()
        lease = core.acquire_lease("formal-example", duration_s=3.0)
        core.arm(lease)
        core.set_chassis(lease, 150, 0, 300)
        time.sleep(2.0)
        core.stop(lease)
    finally:
        core.shutdown(SafeStopReason.SHUTDOWN)


if __name__ == "__main__":
    main()
