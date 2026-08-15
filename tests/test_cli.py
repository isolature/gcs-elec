from __future__ import annotations

import io
import json
import subprocess
import sys
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from unittest.mock import patch

from rescue_control import (
    ControlCore,
    CoreConfig,
    CoreErrorCode,
    DeliveryState,
    FakeLowerLink,
    FaultKind,
    GripperTarget,
    ManualClock,
    SafeStopReason,
)
from rescue_control.cli import (
    BUILTIN_SCENARIOS,
    INTERACTIVE_LEASE_S,
    INTERACTIVE_MAX_LEASE_S,
    MOTION_TTL_MS,
    OutputWriter,
    ScenarioRunner,
    apply_control_key,
    build_parser,
    main,
    run_builtin_scenarios,
    run_interactive_session,
)


REPO_ROOT = Path(__file__).resolve().parents[1]


def make_interactive_core(fake: FakeLowerLink, clock: ManualClock) -> ControlCore:
    """Interactive sessions use a 30 s control lease; the core must allow it."""
    return ControlCore(
        fake,
        clock=clock,
        config=CoreConfig(max_lease_duration_s=INTERACTIVE_MAX_LEASE_S),
    )


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
        if callable(event):
            return event()
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
        if callable(event):
            return event()
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
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = EventSource(events, exit_error=exit_error)
        writer = OutputWriter(output, json_mode=True, clock=clock)
        code = run_interactive_session(core, source, writer)
        return code, source, core, fake, parse_ndjson(output)

    def test_normal_eof_ctrl_c_and_unhandled_all_restore_and_clean(self) -> None:
        cases = (
            (["r", "w", "q"], 0, "SHUTDOWN"),
            (["r", "w", ""], 0, "EOF"),
            (["r", "w", KeyboardInterrupt()], 130, "KEYBOARD_INTERRUPT"),
            (["r", "w", RuntimeError("boom")], 70, "UNHANDLED_EXCEPTION"),
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
                self.assertEqual(records[-1]["safety_reason"], reason)

    def test_all_interactive_controls_case_and_help_are_observable(self) -> None:
        code, source, _, fake, records = self.run_events(
            [
                "R", "O", "c", "W", " ", "s", " ", "A", " ", "d", " ",
                "U", "r", "H", "?", "Q",
            ]
        )
        self.assertEqual(code, 0)
        self.assertEqual(source.exit_count, 1)
        actions = [record["action"] for record in records]
        for action in (
            "ARM", "O", "C", "W", "SPACE", "S", "A", "D", "DISARM",
            "STATUS_HELP",
        ):
            self.assertIn(action, actions)
        self.assertEqual(fake.count("set_gripper"), 2)
        help_records = [
            record
            for record in records
            if record["action"] == "STATUS_HELP"
        ]
        self.assertEqual(len(help_records), 3)
        for record in help_records:
            self.assertFalse(record["reliable_key_up"])
            self.assertEqual(
                record["lease_timeout_ms"], round(INTERACTIVE_LEASE_S * 1_000)
            )
            self.assertEqual(record["motion_release_ms"], 150)
            self.assertEqual(record["motion_ttl_ms"], MOTION_TTL_MS)
            self.assertEqual(
                set(record["keys"]),
                {
                    "R", "U", "W/S/A/D", "Space", "O/C", "P/V", "E",
                    "H/?", "Q",
                },
            )
            self.assertIn("cannot reliably detect physical key-up", record["message"])
            self.assertIn("Space", record["message"])

    def test_explicit_e_safe_stop_is_observable_in_both_cases(self) -> None:
        for key in ("E", "e"):
            with self.subTest(key=key):
                code, source, _, fake, records = self.run_events(
                    ["R", "W", key, "?", "Q"]
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

    def test_lowercase_disarm_and_rearm_are_observable(self) -> None:
        code, source, _, _, records = self.run_events(
            ["r", "u", "r", "q"]
        )
        self.assertEqual(code, 0)
        self.assertEqual(source.exit_count, 1)
        actions = [record["action"] for record in records]
        self.assertIn("DISARM", actions)
        self.assertEqual(actions.count("ARM"), 2)

    def test_late_motion_key_after_deadline_cannot_reactivate(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = AdvancingEventSource(
            clock,
            [(0, "R"), (0, "W"), (INTERACTIVE_LEASE_S, "W"), (0, "Q")],
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
            CoreErrorCode.INVALID_INPUT.value,
        )
        self.assertEqual(
            motion_records[-1]["safety_reason"], "COMMAND_TIMEOUT"
        )

    def test_motion_repeat_refreshes_lease_without_a_management_key(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        # Realistic 30 Hz key repeats: gaps stay below the 150 ms release
        # threshold, so no automatic zero setpoint may interleave, and the
        # lease must survive far beyond the old 500 ms timeout.
        repeats = [(0.03, "W") for _ in range(20)]
        source = AdvancingEventSource(
            clock,
            [(0, "R"), (0, "W"), *repeats, (0, "Q")],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 0)
        self.assertEqual(fake.count("set_chassis"), 21)
        self.assertEqual(fake.count("arm"), 1)
        records = parse_ndjson(output)
        self.assertEqual(
            [r for r in records if r["action"] == "RELEASE_STOP"], []
        )
        timeout_events = [
            event
            for event in core.events
            if event.event == "safe_stop"
            and dict(event.details).get("reason") == "COMMAND_TIMEOUT"
        ]
        self.assertEqual(timeout_events, [])

    def test_r_after_timeout_uses_new_authority_without_replaying_motion(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = AdvancingEventSource(
            clock,
            [(0, "R"), (0, "W"), (INTERACTIVE_LEASE_S, None), (0, "R"), (0, "Q")],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 0)
        self.assertEqual(fake.count("set_chassis"), 1)
        self.assertEqual(fake.count("arm"), 2)
        arm_records = [
            record
            for record in parse_ndjson(output)
            if record["event"] == "key" and record["action"] == "ARM"
        ]
        self.assertEqual(len(arm_records), 2)
        self.assertGreater(
            arm_records[1]["lease_generation"],
            arm_records[0]["lease_generation"],
        )

    def test_single_r_holds_control_across_operator_pauses(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        # Pauses of 5 s between inputs far exceed the old 500 ms lease; one
        # R press must keep control and ARM through the whole session.
        source = AdvancingEventSource(
            clock,
            [
                (0, "R"), (0, "W"), (5.0, "W"), (5.0, " "),
                (5.0, "S"), (5.0, "o"), (0, "Q"),
            ],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 0)
        self.assertEqual(fake.count("arm"), 1)
        self.assertEqual(fake.count("safe_stop"), 1)  # final cleanup only
        records = parse_ndjson(output)
        key_records = [r for r in records if r["event"] == "key"]
        # DEDUPLICATED is the success outcome of Space right after the
        # automatic release already zeroed the setpoint.
        self.assertTrue(
            all(
                r["outcome"] in ("COMPLETED", "DEDUPLICATED")
                for r in key_records
            ),
            key_records,
        )
        generations = {
            r["lease_generation"]
            for r in key_records
            if r["lease_generation"] is not None
        }
        self.assertEqual(len(generations), 1)
        timeout_events = [
            event
            for event in core.events
            if event.event == "safe_stop"
            and dict(event.details).get("reason") == "COMMAND_TIMEOUT"
        ]
        self.assertEqual(timeout_events, [])

    def test_key_release_sends_zero_setpoint_and_keeps_control(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = AdvancingEventSource(
            clock,
            [(0, "R"), (0, "W"), (0.2, None), (0, "S"), (0, "Q")],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 0)
        chassis = [
            dict(record.parameters)
            for record in fake.history
            if record.operation == "set_chassis"
        ]
        self.assertEqual(
            [
                (p["linear_velocity_mm_s"], p["angular_velocity_mrad_s"])
                for p in chassis
            ],
            [(200, 0), (0, 0), (-200, 0)],
        )
        records = parse_ndjson(output)
        release = [
            r for r in records if r["action"] == "RELEASE_STOP"
        ]
        self.assertEqual([r["outcome"] for r in release], ["COMPLETED"])
        # Control and ARM survive a release; motion resumes without R.
        self.assertEqual(fake.count("arm"), 1)
        self.assertEqual(fake.count("stop"), 0)
        self.assertEqual(fake.count("safe_stop"), 1)  # final cleanup only
        resume = [
            r for r in records if r["event"] == "key" and r["action"] == "S"
        ]
        self.assertEqual([r["outcome"] for r in resume], ["COMPLETED"])

    def test_no_release_zero_when_already_stopped_by_space(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = AdvancingEventSource(
            clock,
            [(0, "R"), (0, "W"), (0, " "), (0.5, None), (0, "Q")],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 0)
        self.assertEqual(fake.count("set_chassis"), 1)
        self.assertEqual(fake.count("stop"), 1)
        self.assertEqual(
            [
                r
                for r in parse_ndjson(output)
                if r["action"] == "RELEASE_STOP"
            ],
            [],
        )

    def test_space_stop_keeps_control_and_motion_resumes_without_r(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = AdvancingEventSource(
            clock,
            [(0, "R"), (0, "W"), (0, " "), (0, "S"), (0, "Q")],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 0)
        self.assertEqual(fake.count("arm"), 1)
        self.assertEqual(fake.count("stop"), 1)
        records = parse_ndjson(output)
        space = next(
            r for r in records if r["event"] == "key" and r["action"] == "SPACE"
        )
        self.assertEqual(space["outcome"], "COMPLETED")
        self.assertEqual(space["core_mode"], "ARMED")
        self.assertIsNotNone(space["lease_generation"])
        resume = [
            r for r in records if r["event"] == "key" and r["action"] == "S"
        ]
        self.assertEqual([r["outcome"] for r in resume], ["COMPLETED"])

    def test_e_safe_stop_releases_control_and_r_recovers(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = AdvancingEventSource(
            clock,
            [(0, "R"), (0, "W"), (0, "E"), (0, "W"), (0, "R"), (0, "W"), (0, "Q")],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 0)
        records = parse_ndjson(output)
        safe_stop = next(
            r for r in records if r["action"] == "SAFE_STOP"
        )
        self.assertEqual(safe_stop["safety_reason"], "USER_REQUEST")
        self.assertIsNone(safe_stop["lease_generation"])
        motion = [
            r for r in records if r["event"] == "key" and r["action"] == "W"
        ]
        self.assertEqual(
            [r["outcome"] for r in motion],
            ["COMPLETED", "REJECTED", "COMPLETED"],
        )
        self.assertEqual(
            motion[1]["error_code"], CoreErrorCode.INVALID_INPUT.value
        )
        self.assertGreater(
            motion[2]["lease_generation"], motion[0]["lease_generation"]
        )

    def test_disarm_keeps_lease_but_motion_requires_rearm(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = AdvancingEventSource(
            clock,
            [(0, "R"), (0, "U"), (0, "W"), (0, "Q")],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 0)
        self.assertEqual(fake.count("disarm"), 1)
        self.assertEqual(fake.count("set_chassis"), 0)
        self.assertEqual(fake.count("safe_stop"), 1)  # final cleanup only
        motion = [
            r
            for r in parse_ndjson(output)
            if r["event"] == "key" and r["action"] == "W"
        ]
        self.assertEqual([r["outcome"] for r in motion], ["REJECTED"])
        self.assertEqual(
            motion[0]["error_code"], CoreErrorCode.INVALID_STATE.value
        )

    def test_input_silence_still_expires_into_command_timeout(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = AdvancingEventSource(
            clock,
            [(0, "R"), (0, "W"), (INTERACTIVE_LEASE_S, None), (0, "Q")],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 0)
        self.assertGreaterEqual(fake.count("safe_stop"), 1)
        timeout_events = [
            event
            for event in core.events
            if event.event == "lease_expired"
        ]
        self.assertEqual(len(timeout_events), 1)
        records = parse_ndjson(output)
        self.assertEqual(records[-1]["safety_reason"], "SHUTDOWN")
        cleanup = next(r for r in records if r["event"] == "cleanup")
        self.assertEqual(cleanup["outcome"], "OK")

    def test_release_stop_failure_is_fail_closed(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = AdvancingEventSource(
            clock,
            [
                (0, "R"),
                (0, "W"),
                (
                    0,
                    lambda: (
                        fake.fail_next(
                            "set_chassis", FaultKind.TIMEOUT_AFTER_EFFECT
                        ),
                        None,
                    )[1],
                ),
                (0.2, None),
                (0, "Q"),
            ],
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 0)
        records = parse_ndjson(output)
        release = [
            r for r in records if r["action"] == "RELEASE_STOP"
        ]
        self.assertEqual([r["outcome"] for r in release], ["REJECTED"])
        self.assertEqual(release[0]["error_code"], "BACKEND_TIMEOUT")
        self.assertEqual(release[0]["safety_reason"], "BACKEND_FAILURE")
        self.assertIsNone(core.snapshot().lease)
        self.assertGreaterEqual(fake.count("safe_stop"), 1)

    def test_link_disconnect_terminates_and_runs_cleanup(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = EventSource(
            ["R", "W", lambda: (fake.inject_disconnect(), None)[1]]
        )
        writer = OutputWriter(output, json_mode=True, clock=clock)
        self.assertEqual(run_interactive_session(core, source, writer), 70)
        self.assertEqual(source.exit_count, 1)
        self.assertIsNone(core.snapshot().lease)
        self.assertEqual(core.snapshot().mode.value, "CLOSED")
        self.assertEqual([record.operation for record in fake.history][-1], "close")
        summary = parse_ndjson(output)[-1]
        self.assertEqual(summary["exit_code"], 70)
        self.assertEqual(summary["safety_reason"], "LINK_DISCONNECTED")

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
        self.assertIn("Space", output.getvalue())
        self.assertNotIn("acquire/renew lease", output.getvalue())

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


class TeleopEntryTests(unittest.TestCase):
    def test_teleop_requires_an_exact_by_id_port(self) -> None:
        parser = build_parser()
        errors = io.StringIO()
        with redirect_stderr(errors):
            with self.assertRaises(SystemExit):
                parser.parse_args(["teleop"])
            with self.assertRaises(SystemExit):
                parser.parse_args(["teleop", "--port", "/dev/ttyACM0"])
        self.assertIn("required: --port", errors.getvalue())
        self.assertIn("/dev/ttyACM0 is not accepted", errors.getvalue())
        args = parser.parse_args(
            ["teleop", "--port", "/dev/serial/by-id/usb-STM32-test"]
        )
        self.assertEqual(args.command, "teleop")

    def test_teleop_builds_competition_link_and_reuses_keyboard_loop(self) -> None:
        fake = FakeLowerLink()
        source = EventSource(["Q"])
        output = io.StringIO()
        port = "/dev/serial/by-id/usb-STM32-test"
        with patch(
            "rescue_control.competition_lower_link.CompetitionLowerLink",
            return_value=fake,
        ) as link_factory, patch(
            "rescue_control.cli.TerminalInputSource", return_value=source
        ):
            code = main(
                ["teleop", "--port", port],
                stdout=output,
                stdin=io.StringIO(),
            )
        self.assertEqual(code, 0)
        link_factory.assert_called_once_with(port=port)
        self.assertEqual([record.operation for record in fake.history], [
            "connect", "get_health", "get_robot_state", "get_health",
            "safe_stop", "close"
        ])

    def test_run_script_no_args_fails_clearly_without_starting_python(self) -> None:
        script = REPO_ROOT / "run_teleop.sh"
        self.assertNotEqual(script.stat().st_mode & 0o111, 0)
        completed = subprocess.run(
            ["sh", str(script)],
            cwd=REPO_ROOT,
            text=True,
            capture_output=True,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("--port /dev/serial/by-id/", completed.stderr)


class CameraSessionIntegrationTests(unittest.TestCase):
    """P/V keys drive the camera bridge without touching driving safety."""

    def run_with_camera(
        self,
        events: list[object],
        app,
    ):
        import time as _time

        from rescue_control.capture import TeleopCameraSession

        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = EventSource(events)
        writer = OutputWriter(output, json_mode=True, clock=clock)
        session = TeleopCameraSession(lambda: app)

        def wait_busy():
            deadline = _time.monotonic() + 5.0
            while not session.busy and _time.monotonic() < deadline:
                _time.sleep(0.005)
            return "p"

        def release_then_video():
            app.gate.set()
            deadline = _time.monotonic() + 5.0
            while session.busy and _time.monotonic() < deadline:
                _time.sleep(0.005)
            return "v"

        resolved: list[object] = []
        for event in events:
            if event == "WAIT_BUSY":
                resolved.append(wait_busy)
            elif event == "RELEASE_THEN_VIDEO":
                resolved.append(release_then_video)
            else:
                resolved.append(event)
        source.events = resolved

        code = run_interactive_session(core, source, writer, camera=session)
        session.close(timeout_s=5.0)
        records = parse_ndjson(output)
        return code, fake, records, app, session

    def test_photo_without_camera_reports_skipped(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = EventSource(["p", "v", "q"])
        writer = OutputWriter(output, json_mode=True, clock=clock)
        code = run_interactive_session(core, source, writer)
        records = parse_ndjson(output)
        self.assertEqual(code, 0)
        photo_records = [
            record for record in records if record["action"] in ("PHOTO", "VIDEO")
        ]
        self.assertEqual(len(photo_records), 2)
        for record in photo_records:
            self.assertEqual(record["outcome"], "SKIPPED")
            self.assertEqual(record["error_code"], "CAMERA_NOT_ENABLED")
        self.assertEqual(fake.count("set_chassis"), 0)

    def test_photo_and_video_during_motion_are_non_blocking(self) -> None:
        app = CameraFakeApp()
        code, fake, records, app, session = self.run_with_camera(
            ["r", "w", "p", "WAIT_BUSY", "RELEASE_THEN_VIDEO", "q"], app
        )
        self.assertEqual(code, 0)

        def records_for(event: str, action: str | None = None):
            selected = [
                record for record in records if record["event"] == event
            ]
            if action is None:
                return selected
            return [
                record
                for record in selected
                if record.get("camera_action") == action
            ]

        # driving still works and is untouched by camera work
        self.assertEqual(fake.count("arm"), 1)
        self.assertEqual(fake.count("set_chassis"), 1)
        self.assertEqual(
            [record["action"] for record in records if record["event"] == "key"],
            ["ARM", "W", "PHOTO", "PHOTO", "VIDEO"],
        )
        key_outcomes = {}
        for record in records:
            if record["event"] == "key":
                key_outcomes.setdefault(record["action"], record["outcome"])
        self.assertEqual(key_outcomes["W"], "COMPLETED")
        self.assertEqual(key_outcomes["VIDEO"], "ACCEPTED")
        # first P accepted, second P reported BUSY while camera thread worked
        photo_keys = [
            record
            for record in records
            if record["event"] == "key" and record["action"] == "PHOTO"
        ]
        self.assertEqual(
            [record["outcome"] for record in photo_keys],
            ["ACCEPTED", "BUSY"],
        )
        self.assertEqual(photo_keys[1]["error_code"], "CAMERA_BUSY")

        # camera results flow back as capture events with session identity
        ready = records_for("capture")[0]
        self.assertEqual(ready["camera_event"], "READY")
        key_results = records_for("capture", "PHOTO") + records_for(
            "capture", "VIDEO_TOGGLE"
        )
        self.assertEqual(
            [record["outcome"] for record in key_results],
            ["COMPLETED", "COMPLETED"],
        )
        self.assertEqual(
            [record["message"] for record in key_results],
            ["photo_captured", "recording_started"],
        )
        closed = [
            record
            for record in records_for("capture")
            if record["camera_event"] == "CLOSED"
        ]
        self.assertEqual(len(closed), 1)
        self.assertEqual(closed[0]["outcome"], "OK")
        self.assertEqual(closed[0]["session_id"], "20260815T120000_fake0001")
        self.assertIn("inventory/transfer", closed[0]["message"])
        self.assertEqual(app.shutdown_calls, [("teleop_exit", True)])
        # cleanup still safe-stops through the formal chain
        self.assertEqual(
            [record.operation for record in fake.history][-2:],
            ["safe_stop", "close"],
        )

    def test_camera_init_failure_degrades_to_events(self) -> None:
        import time as _time

        from rescue_control.capture import (
            CaptureUnavailableError,
            TeleopCameraSession,
        )

        class FailingApp:
            def initialize(self):
                raise CaptureUnavailableError("no cameras on this host")

        app = FailingApp()
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = make_interactive_core(fake, clock)
        output = io.StringIO()
        source = EventSource(["p", "q"])
        session = TeleopCameraSession(lambda: app)  # type: ignore[arg-type]
        writer = OutputWriter(output, json_mode=True, clock=clock)
        session.start()

        # let the worker fail initialization before the loop starts
        deadline = _time.monotonic() + 5.0
        while not session.closed and _time.monotonic() < deadline:
            _time.sleep(0.005)
        self.assertTrue(session.closed)

        code = run_interactive_session(core, source, writer, camera=session)
        records = parse_ndjson(output)
        self.assertEqual(code, 0)
        unavailable = [
            record
            for record in records
            if record["event"] == "capture"
            and record["camera_event"] == "UNAVAILABLE"
        ]
        self.assertEqual(len(unavailable), 1)
        self.assertIn("no cameras", unavailable[0]["message"])
        photo = [
            record
            for record in records
            if record["event"] == "key" and record["action"] == "PHOTO"
        ]
        self.assertEqual(photo[0]["outcome"], "UNAVAILABLE")
        self.assertEqual(photo[0]["error_code"], "CAMERA_SESSION_CLOSED")
        self.assertEqual(
            [record.operation for record in fake.history][-2:],
            ["safe_stop", "close"],
        )


class CameraFakeApp:
    """Minimal camera app double used by CameraSessionIntegrationTests."""

    session_id = "20260815T120000_fake0001"
    session_dir = "/tmp/fake-session-dir"

    def __init__(self) -> None:
        import threading

        self.store = type(
            "Store",
            (),
            {"session_id": self.session_id, "session_dir": self.session_dir},
        )()
        self.keys: list[str] = []
        self.shutdown_calls: list[tuple[str, bool]] = []
        self.gate = threading.Event()

    def initialize(self) -> None:
        pass

    def handle_key(self, key: str, now: float | None = None) -> str:
        self.keys.append(key)
        self.gate.wait(5.0)
        return {
            "p": "photo_captured",
            "v": "recording_started",
        }.get(key, "ignored")

    def poll(self) -> None:
        pass

    def shutdown(self, reason: str, graceful: bool) -> list[str]:
        self.shutdown_calls.append((reason, graceful))
        return []


if __name__ == "__main__":
    unittest.main()
