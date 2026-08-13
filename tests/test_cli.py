from __future__ import annotations

import io
import json
import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

from rescue_control import (
    ControlCore,
    CoreErrorCode,
    DeliveryState,
    FakeLowerLink,
    GripperTarget,
    ManualClock,
    SafeStopReason,
)
from rescue_control.cli import (
    BUILTIN_SCENARIOS,
    OutputWriter,
    ScenarioRunner,
    apply_control_key,
    main,
    run_builtin_scenarios,
    run_interactive_session,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


def parse_ndjson(stream: io.StringIO) -> list[dict[str, object]]:
    return [json.loads(line) for line in stream.getvalue().splitlines()]


class ScriptedCliTests(unittest.TestCase):
    def make_runner(self, name: str = "test"):
        clock = ManualClock()
        output = io.StringIO()
        writer = OutputWriter(output, json_mode=True, clock=clock)
        return ScenarioRunner(name, writer, clock=clock), output

    def test_all_builtin_scenarios_are_isolated_and_pass(self) -> None:
        output = io.StringIO()
        code = main(["scenario", "all", "--json"], stdout=output)
        records = parse_ndjson(output)
        self.assertEqual(code, 0)
        checks = [r for r in records if r["event"] == "scenario_check"]
        self.assertEqual(len(checks), len(BUILTIN_SCENARIOS))
        self.assertTrue(all(r["passed"] is True for r in checks))
        for check in checks:
            self.assertEqual(check["expected_exit"], check["actual_exit"])
            self.assertEqual(
                check["expected_cleanup_ok"], check["actual_cleanup_ok"]
            )
            self.assertEqual(
                check["expected_stop_confirmed"],
                check["actual_stop_confirmed"],
            )
            self.assertEqual(check["expected_cause"], check["actual_cause"])
        self.assertEqual(records[-1]["event"], "aggregate_summary")
        self.assertTrue(records[-1]["passed"])
        self.assertEqual([r["seq"] for r in records], list(range(1, len(records) + 1)))

    def test_script_time_uses_manual_clock(self) -> None:
        runner, output = self.make_runner()
        result = runner.run("connect\nadvance 125\nstatus\n")
        self.assertEqual(result.exit_code, 0)
        records = parse_ndjson(output)
        status = next(r for r in records if r["action"] == "status")
        self.assertEqual(status["time_ms"], 125.0)

    def test_script_asserts_timeout_delivery_and_emits_backend_fields(self) -> None:
        cases = (
            ("timeout-before-effect", DeliveryState.NOT_SENT, True),
            ("timeout-after-effect", DeliveryState.MAY_HAVE_APPLIED, False),
        )
        for fault, delivery, retryable in cases:
            with self.subTest(fault=fault):
                runner, output = self.make_runner()
                script = f"""
                connect
                lease acquire pilot 500 as p
                arm p
                fault next set_chassis {fault}
                expect error BACKEND_TIMEOUT delivery {delivery.value} press p w
                expect mode SAFE_STOPPED
                expect lease none
                expect safety BACKEND_FAILURE
                expect stop-confirmed true
                expect backend safe_stop count 1
                """
                result = runner.run(script)
                self.assertEqual(result.exit_code, 0)
                expected = next(
                    record
                    for record in parse_ndjson(output)
                    if record["outcome"] == "EXPECTED"
                )
                self.assertEqual(
                    expected["error_code"], CoreErrorCode.BACKEND_TIMEOUT.value
                )
                self.assertEqual(expected["operation"], "set_chassis")
                self.assertEqual(expected["delivery"], delivery.value)
                self.assertIs(expected["retryable"], retryable)

    def test_wrong_timeout_delivery_assertion_fails(self) -> None:
        runner, _ = self.make_runner()
        result = runner.run(
            """
            connect
            lease acquire pilot 500 as p
            arm p
            fault next set_chassis timeout-after-effect
            expect error BACKEND_TIMEOUT delivery NOT_SENT press p w
            """
        )
        self.assertEqual(result.exit_code, 3)

    def test_real_keyboard_interrupt_is_exit_130_and_is_cleaned(self) -> None:
        runner, output = self.make_runner("real-keyboard-interrupt")

        def interrupt(words, *, line_number=0):
            raise KeyboardInterrupt()

        runner.core.connect()
        runner.execute = interrupt
        result = runner.run("status\n")
        self.assertEqual(result.exit_code, 130)
        self.assertIs(result.cause, SafeStopReason.KEYBOARD_INTERRUPT)
        self.assertTrue(result.cleanup.ok)
        self.assertTrue(result.cleanup.stop_confirmed)
        records = parse_ndjson(output)
        self.assertEqual(records[-1]["exit_code"], 130)
        self.assertEqual(records[-1]["cause"], "KEYBOARD_INTERRUPT")

    def test_builtin_check_cannot_false_green_on_expected_70(self) -> None:
        script, expected_exit = BUILTIN_SCENARIOS["unhandled-exception"]
        script = script.replace(
            "terminate exception",
            "fault next safe_stop timeout-before-effect\n"
            "        terminate exception",
        )
        output = io.StringIO()
        clock = ManualClock()
        writer = OutputWriter(output, json_mode=True, clock=clock)
        with patch.dict(
            BUILTIN_SCENARIOS,
            {"unhandled-exception": (script, expected_exit)},
        ):
            code = run_builtin_scenarios(
                "unhandled-exception", writer=writer
            )
        self.assertEqual(code, 3)
        check = next(
            record
            for record in parse_ndjson(output)
            if record["event"] == "scenario_check"
        )
        self.assertEqual(check["expected_exit"], 70)
        self.assertEqual(check["actual_exit"], 70)
        self.assertTrue(check["expected_cleanup_ok"])
        self.assertFalse(check["actual_cleanup_ok"])
        self.assertFalse(check["passed"])

    def test_invalid_syntax_and_assertion_have_stable_exit_codes(self) -> None:
        syntax, _ = self.make_runner("syntax")
        self.assertEqual(syntax.run("unknown command\n").exit_code, 2)
        assertion, _ = self.make_runner("assertion")
        self.assertEqual(assertion.run("connect\nexpect mode ARMED\n").exit_code, 3)

    def test_unknown_alias_fault_result_and_non_finite_numbers_are_syntax_errors(self) -> None:
        scripts = (
            "connect\narm missing\n",
            "connect\nfault next nope disconnect\n",
            "connect\nresult next get_health SENT\n",
            "connect\nlease acquire p nan as p\n",
            "connect\nadvance -1\n",
        )
        for script in scripts:
            with self.subTest(script=script):
                runner, _ = self.make_runner()
                self.assertEqual(runner.run(script).exit_code, 2)

    def test_every_key_and_case_routes_through_core(self) -> None:
        for key, expected in (
            ("w", (200, 0)), ("W", (200, 0)),
            ("s", (-200, 0)), ("S", (-200, 0)),
            ("a", (0, 800)), ("A", (0, 800)),
            ("d", (0, -800)), ("D", (0, -800)),
        ):
            with self.subTest(key=key):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                token = core.acquire_lease("pilot")
                core.arm(token)
                apply_control_key(core, token, key)
                self.assertEqual(
                    (core.snapshot().commanded_linear_velocity_mm_s, core.snapshot().commanded_angular_velocity_mrad_s),
                    expected,
                )
        for key in ("x", "X", "space", " "):
            with self.subTest(key=key):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                token = core.acquire_lease("pilot")
                core.arm(token)
                core.set_chassis(token, 100, 0, 200)
                apply_control_key(core, token, key)
                self.assertEqual(core.snapshot().commanded_linear_velocity_mm_s, 0)
        for key, expected in (
            ("o", GripperTarget.OPEN),
            ("O", GripperTarget.OPEN),
            ("c", GripperTarget.CLOSED),
            ("C", GripperTarget.CLOSED),
        ):
            with self.subTest(key=key):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                token = core.acquire_lease("pilot")
                core.arm(token)
                apply_control_key(core, token, key)
                self.assertIs(
                    core.snapshot().commanded_gripper_target, expected
                )

    def test_unknown_holder_key_safe_stops_but_stale_token_cannot_dos_holder(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = ControlCore(fake, clock=clock)
        core.connect()
        old = core.acquire_lease("old")
        core.release_lease(old)
        current = core.acquire_lease("current")
        core.arm(current)
        deadline = core.snapshot().lease.deadline_s
        with self.assertRaises(Exception) as stale:
            apply_control_key(core, old, "?")
        self.assertEqual(stale.exception.code, CoreErrorCode.INVALID_LEASE)
        self.assertEqual(core.snapshot().lease.deadline_s, deadline)
        self.assertEqual(fake.count("safe_stop"), 0)
        with self.assertRaises(Exception) as invalid:
            apply_control_key(core, current, "z")
        self.assertEqual(invalid.exception.code, CoreErrorCode.INVALID_INPUT)
        self.assertEqual(fake.count("safe_stop"), 1)

    def test_ndjson_is_parseable_and_contains_required_fields(self) -> None:
        output = io.StringIO()
        code = main(["scenario", "normal", "--json"], stdout=output)
        self.assertEqual(code, 0)
        required = {
            "schema_version", "seq", "time_ms", "event", "action",
            "outcome", "error_code",
        }
        for record in parse_ndjson(output):
            self.assertTrue(required.issubset(record))

    def test_cli_output_is_deterministic(self) -> None:
        first = io.StringIO()
        second = io.StringIO()
        self.assertEqual(main(["scenario", "all", "--json"], stdout=first), 0)
        self.assertEqual(main(["scenario", "all", "--json"], stdout=second), 0)
        self.assertEqual(first.getvalue(), second.getvalue())


class EventSource:
    def __init__(self, events: list[object], *, exit_error: Exception | None = None):
        self.events = list(events)
        self.entered = False
        self.exited = False
        self.exit_count = 0
        self.exit_error = exit_error

    def __enter__(self):
        self.entered = True
        return self

    def read(self, timeout_s: float):
        if not self.events:
            return "q"
        event = self.events.pop(0)
        if isinstance(event, BaseException):
            raise event
        return event

    def __exit__(self, exc_type, exc, traceback):
        self.exited = True
        self.exit_count += 1
        if self.exit_error:
            raise self.exit_error


class AdvancingEventSource(EventSource):
    def __init__(self, clock: ManualClock, events: list[tuple[float, object]]):
        super().__init__([])
        self.clock = clock
        self.timed_events = list(events)

    def read(self, timeout_s: float):
        if not self.timed_events:
            return "q"
        advance_s, event = self.timed_events.pop(0)
        self.clock.advance(advance_s)
        if isinstance(event, BaseException):
            raise event
        return event


class PermanentlyFailingWriter(OutputWriter):
    def __init__(self, stream, *, clock):
        super().__init__(stream, json_mode=True, clock=clock)
        self.failed = False

    def emit(self, event, action, outcome, **kwargs):
        if event == "interactive_start":
            self.failed = True
        if self.failed:
            raise BrokenPipeError("synthetic writer failure")
        return super().emit(event, action, outcome, **kwargs)


class FailOnceWriter(OutputWriter):
    def __init__(self, stream, *, clock, fail_event: str):
        super().__init__(stream, json_mode=True, clock=clock)
        self.fail_event = fail_event
        self.failed = False

    def emit(self, event, action, outcome, **kwargs):
        if event == self.fail_event and not self.failed:
            self.failed = True
            raise BrokenPipeError("synthetic writer failure")
        return super().emit(event, action, outcome, **kwargs)


class ShutdownFailingCore(ControlCore):
    def shutdown(self, reason=SafeStopReason.SHUTDOWN):
        raise RuntimeError("synthetic cleanup failure")


class InteractiveCleanupTests(unittest.TestCase):
    def run_events(self, events: list[object], *, exit_error=None):
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = ControlCore(fake, clock=clock)
        output = io.StringIO()
        source = EventSource(events, exit_error=exit_error)
        writer = OutputWriter(output, json_mode=True, clock=clock)
        code = run_interactive_session(core, source, writer)
        return code, source, core, fake, parse_ndjson(output)

    def test_normal_eof_ctrl_c_and_unhandled_all_restore_and_clean(self) -> None:
        cases = (
            (["l", "r", "w", "q"], 0, "SHUTDOWN"),
            (["l", "r", "w", ""], 0, "EOF"),
            (["l", "r", "w", KeyboardInterrupt()], 130, "KEYBOARD_INTERRUPT"),
            (["l", "r", "w", RuntimeError("boom")], 70, "UNHANDLED_EXCEPTION"),
        )
        for events, expected_code, reason in cases:
            with self.subTest(reason=reason):
                code, source, core, fake, records = self.run_events(events)
                self.assertEqual(code, expected_code)
                self.assertTrue(source.entered)
                self.assertTrue(source.exited)
                self.assertEqual(source.exit_count, 1)
                self.assertEqual(core.snapshot().mode.value, "CLOSED")
                self.assertIsNone(core.snapshot().lease)
                operations = [r.operation for r in fake.history]
                self.assertEqual(operations[-2:], ["safe_stop", "close"])
                self.assertEqual(records[-1]["event"], "summary")

    def test_all_interactive_controls_case_and_help_are_observable(self) -> None:
        code, source, _, fake, records = self.run_events(
            ["L", "R", "O", "c", "W", "X", "U", "V", "?", "Q"]
        )
        self.assertEqual(code, 0)
        self.assertEqual(source.exit_count, 1)
        actions = [record["action"] for record in records]
        for action in (
            "LEASE", "ARM", "O", "C", "W", "X", "DISARM", "RELEASE",
            "STATUS_HELP",
        ):
            self.assertIn(action, actions)
        self.assertEqual(fake.count("set_gripper"), 2)
        help_records = [
            record
            for record in records
            if record["action"] == "STATUS_HELP"
        ]
        self.assertEqual(len(help_records), 2)
        for record in help_records:
            self.assertFalse(record["reliable_key_up"])
            self.assertEqual(record["lease_timeout_ms"], 500)
            self.assertEqual(
                set(record["keys"]),
                {
                    "L", "R", "U", "W/S", "A/D", "Space/X",
                    "O/C", "E", "V", "?", "Q",
                },
            )
            self.assertIn("cannot reliably detect physical key-up", record["message"])
            self.assertIn("Space/X", record["message"])

    def test_explicit_e_safe_stop_is_observable_in_both_cases(self) -> None:
        for key in ("E", "e"):
            with self.subTest(key=key):
                code, source, _, fake, records = self.run_events(
                    ["L", "R", "W", key, "?", "Q"]
                )
                self.assertEqual(code, 0)
                self.assertEqual(source.exit_count, 1)
                safe_stop = next(
                    record
                    for record in records
                    if record["action"] == "SAFE_STOP"
                )
                self.assertEqual(safe_stop["safety_reason"], "USER_REQUEST")
                self.assertGreaterEqual(fake.count("safe_stop"), 2)

    def test_lowercase_disarm_and_release_are_observable(self) -> None:
        code, source, _, _, records = self.run_events(
            ["l", "r", "u", "v", "q"]
        )
        self.assertEqual(code, 0)
        self.assertEqual(source.exit_count, 1)
        actions = [record["action"] for record in records]
        self.assertIn("DISARM", actions)
        self.assertIn("RELEASE", actions)

    def test_late_motion_key_after_deadline_cannot_reactivate(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = ControlCore(fake, clock=clock)
        output = io.StringIO()
        source = AdvancingEventSource(
            clock,
            [(0, "L"), (0, "R"), (0, "W"), (0.5, "W"), (0, "Q")],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        code = run_interactive_session(core, source, writer)
        self.assertEqual(code, 0)
        self.assertEqual(source.exit_count, 1)
        self.assertEqual(fake.count("set_chassis"), 1)
        motion_records = [
            record
            for record in parse_ndjson(output)
            if record["event"] == "key" and record["action"] == "W"
        ]
        self.assertEqual(
            [record["outcome"] for record in motion_records],
            ["COMPLETED", "REJECTED"],
        )
        self.assertEqual(
            motion_records[-1]["error_code"],
            CoreErrorCode.INVALID_LEASE.value,
        )
        self.assertEqual(
            motion_records[-1]["safety_reason"], "COMMAND_TIMEOUT"
        )

    def test_writer_and_shutdown_failures_restore_terminal_exactly_once(self) -> None:
        for writer_kind in ("permanent", "cleanup", "summary"):
            with self.subTest(writer_kind=writer_kind):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                output = io.StringIO()
                source = EventSource(["Q"])
                if writer_kind == "permanent":
                    writer = PermanentlyFailingWriter(output, clock=clock)
                else:
                    writer = FailOnceWriter(
                        output, clock=clock, fail_event=writer_kind
                    )
                code = run_interactive_session(core, source, writer)
                self.assertEqual(code, 70)
                self.assertEqual(source.exit_count, 1)

        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = ShutdownFailingCore(fake, clock=clock)
        output = io.StringIO()
        source = EventSource(["Q"])
        writer = OutputWriter(output, json_mode=True, clock=clock)
        code = run_interactive_session(core, source, writer)
        self.assertEqual(code, 70)
        self.assertEqual(source.exit_count, 1)
        cleanup = next(
            record
            for record in parse_ndjson(output)
            if record["event"] == "cleanup"
        )
        self.assertEqual(cleanup["error_code"], "CLEANUP_FAILED")

    def test_human_startup_help_explains_deadman_and_key_release(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = ControlCore(fake, clock=clock)
        output = io.StringIO()
        source = EventSource(["Q"])
        writer = OutputWriter(output, json_mode=False, clock=clock)
        self.assertEqual(
            run_interactive_session(core, source, writer), 0
        )
        self.assertIn("cannot reliably detect physical key-up", output.getvalue())
        self.assertIn("Space/X", output.getvalue())

    def test_terminal_restore_failure_is_reported_as_runtime_failure(self) -> None:
        code, source, _, _, records = self.run_events(["q"], exit_error=RuntimeError("restore"))
        self.assertEqual(code, 70)
        self.assertTrue(source.exited)
        self.assertEqual(source.exit_count, 1)
        failures = [r for r in records if r.get("error_code") == "TERMINAL_RESTORE_FAILED"]
        self.assertEqual(len(failures), 1)


class ImportIsolationTests(unittest.TestCase):
    def test_fresh_import_does_not_import_serial_or_legacy_hardware_scripts(self) -> None:
        code = (
            "import builtins, sys\n"
            "real_import = builtins.__import__\n"
            "def guard(name, *args, **kwargs):\n"
            "    if name == 'serial' or name.startswith('serial.'):\n"
            "        raise AssertionError('serial import attempted')\n"
            "    return real_import(name, *args, **kwargs)\n"
            "builtins.__import__ = guard\n"
            "import rescue_control\n"
            "assert 'serial' not in sys.modules\n"
            "assert 'elec' not in sys.modules\n"
            "assert 'car_control' not in sys.modules\n"
        )
        completed = subprocess.run(
            [sys.executable, "-I", "-c", code],
            cwd=REPO_ROOT,
            env={"PYTHONPATH": str(REPO_ROOT)},
            text=True,
            capture_output=True,
        )
        if completed.returncode != 0 and "No module named 'rescue_control'" in completed.stderr:
            code = "import sys; sys.path.insert(0, %r); " % str(REPO_ROOT) + code
            completed = subprocess.run(
                [sys.executable, "-I", "-c", code],
                cwd=REPO_ROOT,
                text=True,
                capture_output=True,
            )
        self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == "__main__":
    unittest.main()
