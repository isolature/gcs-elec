#!/bin/sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$script_dir"

if [ "$#" -eq 0 ]; then
    printf '%s\n' \
        "usage: ./run_teleop.sh --port /dev/serial/by-id/<exact-device-name>" \
        >&2
    exit 2
fi

exec python3 -m rescue_control teleop "$@"
