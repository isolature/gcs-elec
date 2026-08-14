#!/usr/bin/env python3
"""Repeatable Fake/WSL latency benchmark for the teleop control chain.

Measures the wall-clock latency from a ControlCore motion/stop call to the
moment the corresponding frame is written to the (fake) serial wire, through
the complete deployed chain:

    ControlCore -> CompetitionLowerLink -> RescueCarClient -> rescue_car_protocol -> FakeStm32Serial

Two configurations are compared with identical code and identical velocity
setpoints (linear 200 mm/s, angular 0 mrad/s), so the only variable is the
runtime client's idle I/O slice:

  baseline  io_slice_s=0.02  -- the hardcoded behaviour merged in PR #1
                               (main@adaae84): the worker thread blocks up to
                               20 ms in an idle serial read before it checks
                               the request queue.
  fast      io_slice_s=0.005 -- the new CompetitionLinkConfig default.

This deliberately separates *input latency* (time domain, measured here) from
*speed settings* (magnitude domain, held constant): the reported improvement
cannot come from raising velocities.

Traffic shape mirrors real teleop: bursts of ~30 Hz motion-key repeats, one
stop (key release / Space), then a short operator pause.  Because the 20 ms
baseline slice drains received frames at only one frame per worker cycle, it
is marginal under sustained command pressure and can non-deterministically
trip the client's heartbeat-ack watchdog (fail-closed disconnect).  The
benchmark therefore counts ``heartbeat_disconnects`` per mode and transparently
reconnects to keep sampling; latency statistics only cover delivered commands.

Usage:
    python3 benchmark_teleop_latency.py                 # compare, human table
    python3 benchmark_teleop_latency.py --json          # single NDJSON record
    python3 benchmark_teleop_latency.py --samples 500
    python3 benchmark_teleop_latency.py --require-improvement 0.2
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import rescue_car_protocol as protocol
from test_rescue_car_client import FakeStm32Serial

from rescue_control import (
    CompetitionLinkConfig,
    CompetitionLowerLink,
    ControlCore,
    CoreConfig,
    SafeStopReason,
)

BENCH_LINEAR_MM_S = 200
BENCH_ANGULAR_MRAD_S = 0
BENCH_TTL_MS = 200
BASELINE_IO_SLICE_S = 0.02
FAST_IO_SLICE_S = 0.005
WARMUP_BURSTS = 4
BURST_REPEATS = 5
BURST_REPEAT_INTERVAL_S = 0.033
BURST_PAUSE_S = 0.15
POST_ARM_SETTLE_S = 0.8
MAX_RECONNECTS = 6


class TimedFakeSerial(FakeStm32Serial):
    """Records monotonic write times per chassis setpoint value."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.motion_writes: list[tuple[float, int, int]] = []

    def write(self, data):
        now = time.monotonic()
        written = super().write(data)
        frame = self.received[-1]
        if frame.msg_type == protocol.MSG_CHASSIS_SETPOINT:
            fields = protocol._varint_fields(frame.payload)
            linear = protocol.decode_sint32(fields.get(1, 0))
            angular = protocol.decode_sint32(fields.get(2, 0))
            self.motion_writes.append((now, linear, angular))
        return written


def _first_motion_write_after(serial, t0, linear, angular):
    for stamp, lin, ang in serial.motion_writes:
        if stamp >= t0 and lin == linear and ang == angular:
            return stamp
    raise AssertionError(
        f"no ({linear},{angular}) chassis write observed after t0"
    )


class _Session:
    """One armed benchmark session over a fresh fake link."""

    def __init__(self, io_slice_s: float) -> None:
        self.io_slice_s = io_slice_s
        self.serial: TimedFakeSerial | None = None
        holder: dict[str, TimedFakeSerial] = {}

        def factory(**kwargs):
            serial = TimedFakeSerial(**kwargs)
            holder["serial"] = serial
            return serial

        self.link = CompetitionLowerLink(
            port="/dev/serial/by-id/usb-STM32-bench",
            serial_factory=factory,
            reconnect=False,
            config=CompetitionLinkConfig(io_slice_s=io_slice_s),
        )
        self.core = ControlCore(
            self.link, config=CoreConfig(max_lease_duration_s=60.0)
        )
        self.core.connect()
        self.token = self.core.acquire_lease("latency-bench", duration_s=60.0)
        arm_started = time.monotonic()
        self.core.arm(self.token)
        self.arm_to_confirmed_ms = (time.monotonic() - arm_started) * 1_000.0
        self.serial = holder["serial"]
        time.sleep(POST_ARM_SETTLE_S)

    def close(self) -> None:
        try:
            self.core.shutdown(SafeStopReason.SHUTDOWN)
        except Exception:
            pass


def _run_burst(session: _Session, record: bool, motion_ms, stop_ms) -> None:
    core, token, serial = session.core, session.token, session.serial
    for _ in range(BURST_REPEATS):
        t0 = time.monotonic()
        core.set_chassis(token, BENCH_LINEAR_MM_S, BENCH_ANGULAR_MRAD_S, BENCH_TTL_MS)
        t_motion = _first_motion_write_after(
            serial, t0, BENCH_LINEAR_MM_S, BENCH_ANGULAR_MRAD_S
        )
        if record:
            motion_ms.append((t_motion - t0) * 1_000.0)
        time.sleep(BURST_REPEAT_INTERVAL_S)
    ts0 = time.monotonic()
    core.stop(token)
    t_zero = _first_motion_write_after(serial, ts0, 0, 0)
    if record:
        stop_ms.append((t_zero - ts0) * 1_000.0)
    time.sleep(BURST_PAUSE_S)


def measure(io_slice_s: float, samples: int) -> dict[str, object]:
    motion_ms: list[float] = []
    stop_ms: list[float] = []
    disconnects = 0
    disconnect_reasons: list[str] = []
    arm_ms: list[float] = []
    session = _Session(io_slice_s)
    arm_ms.append(session.arm_to_confirmed_ms)
    max_bursts = WARMUP_BURSTS + samples // BURST_REPEATS + MAX_RECONNECTS
    try:
        burst_index = 0
        while len(motion_ms) < samples and burst_index < max_bursts:
            record = burst_index >= WARMUP_BURSTS
            burst_index += 1
            try:
                _run_burst(session, record, motion_ms, stop_ms)
            except Exception as exc:
                # Baseline-slice sessions can trip the client heartbeat-ack
                # watchdog under sustained traffic (fail-closed).  Count it,
                # reconnect and keep sampling delivered-command latency.
                disconnects += 1
                disconnect_reasons.append(f"{type(exc).__name__}: {exc}")
                if disconnects > MAX_RECONNECTS:
                    raise
                session.close()
                session = _Session(io_slice_s)
                arm_ms.append(session.arm_to_confirmed_ms)
    finally:
        session.close()

    if len(motion_ms) < samples:
        raise RuntimeError(
            f"could not collect {samples} motion samples at io_slice_s="
            f"{io_slice_s} within {max_bursts} bursts "
            f"({disconnects} disconnects)"
        )

    def stats(values: list[float]) -> dict[str, float]:
        ordered = sorted(values)
        count = len(ordered)
        return {
            "mean_ms": round(statistics.fmean(ordered), 3),
            "p50_ms": round(ordered[count // 2], 3),
            "p95_ms": round(ordered[min(count - 1, int(count * 0.95))], 3),
            "max_ms": round(ordered[-1], 3),
            "min_ms": round(ordered[0], 3),
        }

    return {
        "io_slice_s": io_slice_s,
        "samples": len(motion_ms),
        "motion_latency": stats(motion_ms),
        "stop_latency": stats(stop_ms),
        "arm_to_confirmed_ms": round(arm_ms[0], 3),
        "heartbeat_disconnects": disconnects,
        "disconnect_reasons": disconnect_reasons,
    }


def _improvement(baseline: float, fast: float) -> float:
    return (baseline - fast) / baseline if baseline > 0 else 0.0


def run_compare(samples: int, required: float) -> tuple[dict[str, object], bool]:
    baseline = measure(BASELINE_IO_SLICE_S, samples)
    fast = measure(FAST_IO_SLICE_S, samples)
    motion_p50_gain = _improvement(
        baseline["motion_latency"]["p50_ms"],
        fast["motion_latency"]["p50_ms"],
    )
    motion_mean_gain = _improvement(
        baseline["motion_latency"]["mean_ms"],
        fast["motion_latency"]["mean_ms"],
    )
    stop_p50_gain = _improvement(
        baseline["stop_latency"]["p50_ms"],
        fast["stop_latency"]["p50_ms"],
    )
    passed = motion_p50_gain >= required and motion_mean_gain >= required
    report = {
        "schema_version": 1,
        "chain": (
            "ControlCore -> CompetitionLowerLink -> RescueCarClient -> "
            "rescue_car_protocol -> FakeStm32Serial"
        ),
        "speed_settings": {
            "linear_mm_s": BENCH_LINEAR_MM_S,
            "angular_mrad_s": BENCH_ANGULAR_MRAD_S,
            "ttl_ms": BENCH_TTL_MS,
            "identical_in_both_modes": True,
        },
        "baseline": baseline,
        "fast": fast,
        "improvement": {
            "motion_p50": round(motion_p50_gain, 4),
            "motion_mean": round(motion_mean_gain, 4),
            "stop_p50": round(stop_p50_gain, 4),
        },
        "gate": {
            "required_min": required,
            "metric": "motion latency p50 and mean",
            "passed": passed,
        },
    }
    return report, passed


def _print_report(report: dict[str, object]) -> None:
    baseline = report["baseline"]
    fast = report["fast"]
    speed = report["speed_settings"]
    print(
        "chain: "
        "ControlCore -> CompetitionLowerLink -> RescueCarClient -> "
        "rescue_car_protocol -> FakeStm32Serial"
    )
    print(
        "speed settings identical in both modes: "
        f"linear={speed['linear_mm_s']} mm/s, "
        f"angular={speed['angular_mrad_s']} mrad/s, ttl={speed['ttl_ms']} ms"
    )
    print()
    print(
        f"{'mode':<10}{'io_slice_s':<12}{'motion p50':<14}"
        f"{'motion mean':<14}{'motion p95':<14}{'stop p50':<12}"
        f"{'arm ms':<10}{'hb drop':<10}"
    )

    def row(name: str, data: dict[str, object]) -> None:
        motion = data["motion_latency"]
        stop = data["stop_latency"]
        print(
            f"{name:<10}{data['io_slice_s']:<12}{motion['p50_ms']:<14}"
            f"{motion['mean_ms']:<14}{motion['p95_ms']:<14}"
            f"{stop['p50_ms']:<12}{data['arm_to_confirmed_ms']:<10}"
            f"{data['heartbeat_disconnects']:<10}"
        )

    row("baseline", baseline)
    row("fast", fast)
    improvement = report["improvement"]
    gate = report["gate"]
    print()
    print(
        f"motion latency improvement: p50 {improvement['motion_p50']:.1%}, "
        f"mean {improvement['motion_mean']:.1%}; "
        f"stop p50 {improvement['stop_p50']:.1%}"
    )
    print(
        f"gate (>= {gate['required_min']:.0%} on motion p50 and mean): "
        f"{'PASSED' if gate['passed'] else 'FAILED'}"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--mode",
        choices=("compare", "baseline", "fast"),
        default="compare",
        help="compare runs both configurations (default)",
    )
    parser.add_argument("--samples", type=int, default=300)
    parser.add_argument(
        "--require-improvement",
        type=float,
        default=0.2,
        help="minimum motion-latency improvement for exit code 0 (compare mode)",
    )
    parser.add_argument("--json", action="store_true", help="emit NDJSON report")
    args = parser.parse_args(argv)

    if args.samples < 50:
        parser.error("--samples must be at least 50 for stable percentiles")

    if args.mode in ("baseline", "fast"):
        io_slice = (
            BASELINE_IO_SLICE_S if args.mode == "baseline" else FAST_IO_SLICE_S
        )
        report = measure(io_slice, args.samples)
        report["schema_version"] = 1
        if args.json:
            print(json.dumps(report, ensure_ascii=False, sort_keys=True))
        else:
            print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
        return 0

    report, passed = run_compare(args.samples, args.require_improvement)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    else:
        _print_report(report)
    return 0 if passed else 3


if __name__ == "__main__":
    raise SystemExit(main())
