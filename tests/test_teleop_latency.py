"""Automated before/after latency gate for the teleop control chain.

Runs the Fake/WSL benchmark in-process and asserts the acceptance target:
motion key-to-wire latency must improve by at least 20 % (p50 and mean) with
identical velocity setpoints, proving the gain comes from input latency, not
from raising speed settings.  Intermittent WSL Fake-link heartbeat watchdog
disconnects are tolerated: the benchmark reconnects, counts them per mode and
only delivered-command latency enters the statistics (see FORMAL_CLIENT.md).
"""

from __future__ import annotations

import unittest

import benchmark_teleop_latency as bench


class TeleopLatencyBenchmarkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.report, cls.passed = bench.run_compare(
            samples=100, required=0.2
        )

    def test_motion_latency_improves_at_least_20_percent(self) -> None:
        self.assertTrue(self.passed, self.report["improvement"])
        self.assertGreaterEqual(
            self.report["improvement"]["motion_p50"], 0.2
        )
        self.assertGreaterEqual(
            self.report["improvement"]["motion_mean"], 0.2
        )

    def test_speed_settings_are_identical_in_both_modes(self) -> None:
        speed = self.report["speed_settings"]
        self.assertTrue(speed["identical_in_both_modes"])
        self.assertEqual(speed["linear_mm_s"], 200)
        self.assertEqual(speed["angular_mrad_s"], 0)
        self.assertEqual(
            self.report["baseline"]["io_slice_s"],
            bench.BASELINE_IO_SLICE_S,
        )
        self.assertEqual(
            self.report["fast"]["io_slice_s"],
            bench.FAST_IO_SLICE_S,
        )

    def test_report_counts_fake_link_disconnects_per_mode(self) -> None:
        # Both modes may show the pre-existing intermittent heartbeat-ack
        # watchdog trip under synthetic Fake traffic; it must be counted and
        # must never be silent.
        for mode in ("baseline", "fast"):
            with self.subTest(mode=mode):
                self.assertIn(
                    "heartbeat_disconnects", self.report[mode]
                )
                self.assertLessEqual(
                    self.report[mode]["heartbeat_disconnects"], 6
                )


if __name__ == "__main__":
    unittest.main()
