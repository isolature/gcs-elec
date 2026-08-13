"""Fake-only command-line rehearsal for the upper control boundary."""

from __future__ import annotations

import argparse
import io
import json
import math
import shlex
import sys
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import TextIO

from .control_core import (
    CleanupResult,
    ControlCore,
    CoreBackendError,
    CoreError,
    CoreErrorCode,
    CoreMode,
    InvalidControlInput,
    LeaseToken,
)
from .fake_lower_link import FakeLowerLink, FaultKind, ManualClock
from .models import (
    CommandStatus,
    DeliveryState,
    FieldValidity,
    GripperTarget,
    SafeStopReason,
)


MOTION_TTL_MS = 200
INTERACTIVE_LEASE_S = 0.5
MOTION_KEYS = {
    "w": (200, 0, "forward"),
    "s": (-200, 0, "backward"),
    "a": (0, 800, "turn_left"),
    "d": (0, -800, "turn_right"),
}
INTERACTIVE_KEY_BINDINGS = {
    "L": "acquire or renew the 500 ms input lease",
    "R": "ARM",
    "U": "DISARM",
    "W/S": "forward/backward pulse",
    "A/D": "left/right turn pulse",
    "Space/X": "explicit key-release substitute: ordinary stop",
    "O/C": "open/close gripper",
    "E": "user-requested safety stop",
    "V": "release the input lease",
    "?": "show this help and current status",
    "Q": "quit with shutdown cleanup",
}

INTERACTIVE_HELP_MESSAGE = (
    "L acquire/renew lease; R ARM; U DISARM; W/S forward/backward; "
    "A/D turn left/right; Space/X explicit key-release substitute (ordinary stop); "
    "O/C gripper OPEN/CLOSED; E safety stop; V release lease; ? status/help; "
    "Q quit. This terminal cannot reliably detect physical key-up: repeat motion "
    "keys to renew the 500 ms deadman lease and press Space/X when releasing a "
    "key. If successful input renewal stops, lease expiry causes safety stop."
)


class ScenarioSyntaxError(RuntimeError):
    pass


class ScenarioAssertionError(RuntimeError):
    pass


class _SyntheticEOF(BaseException):
    pass


class _SyntheticKeyboardInterrupt(BaseException):
    pass


class _SyntheticUnhandled(RuntimeError):
    pass


@dataclass(frozen=True)
class SessionResult:
    name: str
    exit_code: int
    cleanup: CleanupResult
    cause: SafeStopReason
    error: str | None = None


class OutputWriter:
    """Stable human or NDJSON output; JSON mode never mixes plain text."""

    def __init__(
        self,
        stream: TextIO,
        *,
        json_mode: bool,
        clock,
    ) -> None:
        self._stream = stream
        self._json = json_mode
        self._clock = clock
        self._sequence = 0

    def emit(
        self,
        event: str,
        action: str,
        outcome: str,
        *,
        core: ControlCore | None = None,
        scenario: str | None = None,
        error_code: str | None = None,
        **details: object,
    ) -> None:
        self._sequence += 1
        snapshot = core.snapshot() if core is not None else None
        record: dict[str, object] = {
            "schema_version": 1,
            "seq": self._sequence,
            "time_ms": round(float(snapshot.time_s if snapshot else self._clock()) * 1_000, 3),
            "event": event,
            "action": action,
            "outcome": outcome,
            "error_code": error_code,
        }
        if scenario is not None:
            record["scenario"] = scenario
        if snapshot is not None:
            record.update(
                {
                    "core_mode": snapshot.mode.value,
                    "connected": snapshot.connected,
                    "armed": snapshot.armed,
                    "lease_owner": (
                        snapshot.lease.owner if snapshot.lease else None
                    ),
                    "lease_generation": (
                        snapshot.lease.generation if snapshot.lease else None
                    ),
                    "safety_reason": (
                        snapshot.last_safety_reason.value
                        if snapshot.last_safety_reason
                        else None
                    ),
                    "stop_attempted": snapshot.stop_attempted,
                    "stop_confirmed": snapshot.stop_confirmed,
                    "commanded_linear_mm_s": (
                        snapshot.commanded_linear_velocity_mm_s
                    ),
                    "commanded_angular_mrad_s": (
                        snapshot.commanded_angular_velocity_mrad_s
                    ),
                    "gripper_target": (
                        snapshot.commanded_gripper_target.value
                        if snapshot.commanded_gripper_target
                        else None
                    ),
                }
            )
        record.update({key: _normalize(value) for key, value in details.items()})
        if self._json:
            self._stream.write(
                json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n"
            )
        else:
            mode = f" mode={record.get('core_mode')}" if core else ""
            owner = (
                f" owner={record.get('lease_owner')}"
                if core and record.get("lease_owner")
                else ""
            )
            error = f" error={error_code}" if error_code else ""
            message = (
                f"\n  {record['message']}"
                if isinstance(record.get("message"), str)
                and record["message"]
                else ""
            )
            label = f"[{scenario}] " if scenario else ""
            self._stream.write(
                f"{label}{self._sequence:04d} "
                f"t={record['time_ms']}ms {event} {action} -> {outcome}"
                f"{mode}{owner}{error}{message}\n"
            )
        self._stream.flush()


def _normalize(value: object) -> object:
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, tuple):
        return [_normalize(item) for item in value]
    if isinstance(value, list):
        return [_normalize(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _normalize(item) for key, item in value.items()}
    return value


def _core_error_details(exc: CoreError) -> dict[str, object]:
    if not isinstance(exc, CoreBackendError):
        return {}
    return {
        "operation": exc.operation,
        "delivery": exc.delivery,
        "retryable": exc.retryable,
    }


def apply_control_key(
    core: ControlCore, token: LeaseToken, key: str, *, ttl_ms: int = MOTION_TTL_MS
):
    """Map one semantic key through ControlCore; never call LowerLink directly."""
    normalized = key.lower()
    if normalized in MOTION_KEYS:
        linear, angular, _ = MOTION_KEYS[normalized]
        return core.set_chassis(token, linear, angular, ttl_ms)
    if normalized in ("x", "space", " "):
        return core.stop(token)
    if normalized == "o":
        return core.set_gripper(token, GripperTarget.OPEN)
    if normalized == "c":
        return core.set_gripper(token, GripperTarget.CLOSED)

    # The raw invalid sentinel deliberately enters ControlCore before validation.
    # A valid, armed holder therefore causes the approved INVALID_COMMAND safe-stop;
    # stale/non-owner tokens are rejected before they can disturb the real holder.
    return core.set_chassis(token, object(), 0, ttl_ms)


class ScenarioRunner:
    """Line-oriented deterministic DSL around a FakeLowerLink session."""

    def __init__(
        self,
        name: str,
        writer: OutputWriter,
        *,
        clock: ManualClock | None = None,
    ) -> None:
        self.name = name
        self.clock = clock or ManualClock()
        self.fake = FakeLowerLink(self.clock)
        self.core = ControlCore(self.fake, clock=self.clock)
        self.writer = writer
        self.tokens: dict[str, LeaseToken] = {}

    def run(self, text: str) -> SessionResult:
        exit_code = 0
        cause = SafeStopReason.SHUTDOWN
        error_text: str | None = None
        try:
            for line_number, raw_line in enumerate(text.splitlines(), 1):
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                try:
                    words = shlex.split(line, comments=True)
                except ValueError as exc:
                    raise ScenarioSyntaxError(
                        f"line {line_number}: {exc}"
                    ) from exc
                if not words:
                    continue
                self.execute(words, line_number=line_number)
        except _SyntheticEOF:
            cause = SafeStopReason.EOF
            self.writer.emit(
                "termination",
                "EOF",
                "EXPECTED",
                core=self.core,
                scenario=self.name,
            )
        except (_SyntheticKeyboardInterrupt, KeyboardInterrupt):
            cause = SafeStopReason.KEYBOARD_INTERRUPT
            exit_code = 130
            self.writer.emit(
                "termination",
                "CTRL_C",
                "EXPECTED",
                core=self.core,
                scenario=self.name,
            )
        except ScenarioSyntaxError as exc:
            cause = SafeStopReason.UNHANDLED_EXCEPTION
            exit_code = 2
            error_text = str(exc)
            self.writer.emit(
                "script_error",
                "syntax",
                "ERROR",
                core=self.core,
                scenario=self.name,
                error_code="SCRIPT_SYNTAX",
                message=error_text,
            )
        except ScenarioAssertionError as exc:
            cause = SafeStopReason.UNHANDLED_EXCEPTION
            exit_code = 3
            error_text = str(exc)
            self.writer.emit(
                "assertion",
                "expect",
                "FAILED",
                core=self.core,
                scenario=self.name,
                error_code="ASSERTION_FAILED",
                message=error_text,
            )
        except (CoreError, _SyntheticUnhandled, Exception) as exc:
            cause = SafeStopReason.UNHANDLED_EXCEPTION
            exit_code = 70
            error_text = str(exc)
            is_core_error = isinstance(exc, CoreError)
            code = exc.code.value if is_core_error else "UNEXPECTED_RUNTIME_ERROR"
            self.writer.emit(
                "runtime_error",
                type(exc).__name__,
                "ERROR",
                core=self.core,
                scenario=self.name,
                error_code=code,
                message=error_text,
                **(_core_error_details(exc) if is_core_error else {}),
            )
        finally:
            cleanup = self.core.shutdown(cause)
            if (not cleanup.ok or not cleanup.stop_confirmed) and exit_code in (
                0,
                130,
            ):
                exit_code = 70
            self.writer.emit(
                "cleanup",
                cause.value,
                "OK" if cleanup.ok and cleanup.stop_confirmed else "UNCONFIRMED",
                core=self.core,
                scenario=self.name,
                error_code=(cleanup.errors[0] if cleanup.errors else None),
                cleanup_errors=cleanup.errors,
            )
            self.writer.emit(
                "summary",
                self.name,
                "COMPLETE",
                core=self.core,
                scenario=self.name,
                exit_code=exit_code,
                cleanup_ok=cleanup.ok,
                stop_confirmed=cleanup.stop_confirmed,
                cause=cause,
            )
        return SessionResult(self.name, exit_code, cleanup, cause, error_text)

    def execute(self, words: list[str], *, line_number: int = 0) -> None:
        command = words[0].lower()
        try:
            if command == "connect" and len(words) == 1:
                self.core.connect()
                self._emit_command(words, "OK")
                return
            if command == "lease":
                self._execute_lease(words)
                return
            if command == "arm" and len(words) == 2:
                result = self.core.arm(self._token(words[1]))
                self._emit_command(words, result.status.value)
                return
            if command == "disarm" and len(words) == 2:
                result = self.core.disarm(self._token(words[1]))
                self._emit_command(words, result.status.value)
                return
            if command == "press" and len(words) == 3:
                result = apply_control_key(
                    self.core, self._token(words[1]), words[2]
                )
                self._emit_command(words, result.status.value)
                return
            if command == "safe-stop" and len(words) in (1, 2):
                reason = (
                    SafeStopReason.USER_REQUEST
                    if len(words) == 1
                    else self._enum_value(SafeStopReason, words[1], "reason")
                )
                result = self.core.safe_stop(reason)
                self._emit_command(words, result.status.value)
                return
            if command == "advance" and len(words) == 2:
                milliseconds = self._nonnegative_number(words[1], "milliseconds")
                self.clock.advance(milliseconds / 1_000.0)
                self._emit_command(words, "OK")
                return
            if command == "poll" and len(words) == 1:
                self.core.poll()
                self._emit_command(words, "OK")
                return
            if command == "fault":
                self._execute_fault(words)
                return
            if command == "result":
                self._execute_result(words)
                return
            if command == "disconnect" and len(words) == 1:
                self.fake.inject_disconnect()
                self._emit_command(words, "INJECTED")
                return
            if command == "feedback" and len(words) == 2:
                validity = self._enum_value(
                    FieldValidity, words[1], "feedback validity"
                )
                self.fake.set_feedback_validity(validity)
                self._emit_command(words, "INJECTED")
                return
            if command == "status" and len(words) == 1:
                self._emit_command(words, "OK")
                return
            if command == "expect":
                self._execute_expect(words, line_number=line_number)
                return
            if command == "terminate" and len(words) == 2:
                kind = words[1].lower()
                if kind == "eof":
                    raise _SyntheticEOF()
                if kind in ("keyboard-interrupt", "ctrl-c"):
                    raise _SyntheticKeyboardInterrupt()
                if kind in ("exception", "unhandled"):
                    raise _SyntheticUnhandled("synthetic unhandled exception")
                raise ScenarioSyntaxError(f"unknown termination kind: {words[1]}")
            if command == "shutdown" and len(words) == 1:
                cleanup = self.core.shutdown(SafeStopReason.SHUTDOWN)
                self._emit_command(
                    words,
                    "OK" if cleanup.ok and cleanup.stop_confirmed else "UNCONFIRMED",
                )
                return
        except ValueError as exc:
            raise ScenarioSyntaxError(str(exc)) from exc
        location = f"line {line_number}: " if line_number else ""
        raise ScenarioSyntaxError(
            f"{location}unknown or malformed command: {' '.join(words)}"
        )

    def _execute_lease(self, words: list[str]) -> None:
        if (
            len(words) == 6
            and words[1].lower() == "acquire"
            and words[4].lower() == "as"
        ):
            owner = words[2]
            duration_ms = self._positive_number(words[3], "duration_ms")
            alias = words[5]
            if alias in self.tokens:
                raise ScenarioSyntaxError(f"token alias already exists: {alias}")
            self.tokens[alias] = self.core.acquire_lease(
                owner, duration_ms / 1_000.0
            )
            self._emit_command(words, "OK", alias=alias)
            return
        if len(words) in (3, 4) and words[1].lower() == "renew":
            token = self._token(words[2])
            duration = (
                self._positive_number(words[3], "duration_ms") / 1_000.0
                if len(words) == 4
                else None
            )
            self.core.renew_lease(token, duration)
            self._emit_command(words, "OK")
            return
        if len(words) == 3 and words[1].lower() == "release":
            self.core.release_lease(self._token(words[2]))
            self._emit_command(words, "OK")
            return
        raise ScenarioSyntaxError("malformed lease command")

    def _execute_fault(self, words: list[str]) -> None:
        if len(words) != 4 or words[1].lower() != "next":
            raise ScenarioSyntaxError("use: fault next OPERATION KIND")
        operation = words[2]
        kind_name = words[3].replace("-", "_").upper()
        aliases = {
            "TIMEOUT_BEFORE": "TIMEOUT_BEFORE_EFFECT",
            "TIMEOUT_AFTER": "TIMEOUT_AFTER_EFFECT",
            "PROTOCOL": "PROTOCOL_ERROR",
            "UNEXPECTED": "UNEXPECTED_ERROR",
        }
        kind_name = aliases.get(kind_name, kind_name)
        try:
            kind = FaultKind[kind_name]
        except KeyError as exc:
            raise ScenarioSyntaxError(f"unknown fault kind: {words[3]}") from exc
        self.fake.fail_next(operation, kind)
        self._emit_command(words, "INJECTED")

    def _execute_result(self, words: list[str]) -> None:
        if len(words) != 4 or words[1].lower() != "next":
            raise ScenarioSyntaxError("use: result next OPERATION STATUS")
        status = self._enum_value(CommandStatus, words[3], "command status")
        self.fake.result_next(words[2], status)
        self._emit_command(words, "INJECTED")

    def _execute_expect(self, words: list[str], *, line_number: int) -> None:
        if len(words) >= 4 and words[1].lower() == "error":
            expected = words[2].upper()
            expected_delivery: DeliveryState | None = None
            if len(words) >= 6 and words[3].lower() == "delivery":
                expected_delivery = self._enum_value(
                    DeliveryState, words[4], "delivery state"
                )
                nested = words[5:]
            else:
                nested = words[3:]
            try:
                self.execute(nested, line_number=line_number)
            except CoreError as exc:
                if exc.code.value != expected:
                    raise ScenarioAssertionError(
                        f"expected {expected}, got {exc.code.value}"
                    ) from exc
                if expected_delivery is not None:
                    actual_delivery = (
                        exc.delivery
                        if isinstance(exc, CoreBackendError)
                        else None
                    )
                    if actual_delivery is not expected_delivery:
                        raise ScenarioAssertionError(
                            "delivery: expected "
                            f"{expected_delivery.value}, got "
                            f"{_normalize(actual_delivery)!r}"
                        ) from exc
                self._emit_command(
                    words,
                    "EXPECTED",
                    error_code=expected,
                    **_core_error_details(exc),
                )
                return
            raise ScenarioAssertionError(
                f"expected {expected}, but command succeeded"
            )

        snapshot = self.core.snapshot()
        if len(words) == 3 and words[1].lower() == "mode":
            expected_mode = self._enum_value(CoreMode, words[2], "core mode")
            self._assert_equal("mode", snapshot.mode, expected_mode)
        elif len(words) == 3 and words[1].lower() == "lease":
            expected = words[2]
            actual = snapshot.lease.owner if snapshot.lease else "none"
            self._assert_equal("lease", actual, expected)
        elif len(words) == 3 and words[1].lower() == "safety":
            expected_reason = self._enum_value(
                SafeStopReason, words[2], "safe-stop reason"
            )
            self._assert_equal(
                "safety reason", snapshot.last_safety_reason, expected_reason
            )
        elif len(words) == 3 and words[1].lower() == "stop-confirmed":
            expected_bool = self._parse_bool(words[2])
            self._assert_equal(
                "stop confirmation", snapshot.stop_confirmed, expected_bool
            )
        elif len(words) == 5 and words[1].lower() == "backend":
            if words[3].lower() != "count":
                raise ScenarioSyntaxError(
                    "use: expect backend OPERATION count NUMBER"
                )
            expected_count = self._nonnegative_integer(words[4], "count")
            self._assert_equal(
                f"backend {words[2]} count",
                self.fake.count(words[2]),
                expected_count,
            )
        elif len(words) == 4 and words[1].lower() == "motion":
            linear = self._integer(words[2], "linear")
            angular = self._integer(words[3], "angular")
            self._assert_equal(
                "linear motion",
                snapshot.commanded_linear_velocity_mm_s,
                linear,
            )
            self._assert_equal(
                "angular motion",
                snapshot.commanded_angular_velocity_mrad_s,
                angular,
            )
        else:
            raise ScenarioSyntaxError("unknown expect expression")
        self._emit_command(words, "PASSED")

    def _token(self, alias: str) -> LeaseToken:
        try:
            return self.tokens[alias]
        except KeyError as exc:
            raise ScenarioSyntaxError(f"unknown token alias: {alias}") from exc

    def _emit_command(
        self,
        words: list[str],
        outcome: str,
        *,
        error_code: str | None = None,
        **details: object,
    ) -> None:
        self.writer.emit(
            "command",
            " ".join(words),
            outcome,
            core=self.core,
            scenario=self.name,
            error_code=error_code,
            **details,
        )

    @staticmethod
    def _enum_value(enum_type, raw: str, label: str):
        normalized = raw.replace("-", "_").upper()
        try:
            return enum_type[normalized]
        except KeyError as exc:
            raise ScenarioSyntaxError(f"unknown {label}: {raw}") from exc

    @staticmethod
    def _positive_number(raw: str, label: str) -> float:
        value = ScenarioRunner._number(raw, label)
        if value <= 0:
            raise ScenarioSyntaxError(f"{label} must be positive")
        return value

    @staticmethod
    def _nonnegative_number(raw: str, label: str) -> float:
        value = ScenarioRunner._number(raw, label)
        if value < 0:
            raise ScenarioSyntaxError(f"{label} must be non-negative")
        return value

    @staticmethod
    def _number(raw: str, label: str) -> float:
        try:
            value = float(raw)
        except ValueError as exc:
            raise ScenarioSyntaxError(f"{label} must be numeric") from exc
        if not math.isfinite(value):
            raise ScenarioSyntaxError(f"{label} must be finite")
        return value

    @staticmethod
    def _integer(raw: str, label: str) -> int:
        try:
            return int(raw, 10)
        except ValueError as exc:
            raise ScenarioSyntaxError(f"{label} must be an integer") from exc

    @staticmethod
    def _nonnegative_integer(raw: str, label: str) -> int:
        value = ScenarioRunner._integer(raw, label)
        if value < 0:
            raise ScenarioSyntaxError(f"{label} must be non-negative")
        return value

    @staticmethod
    def _parse_bool(raw: str) -> bool:
        normalized = raw.lower()
        if normalized in ("true", "yes", "1"):
            return True
        if normalized in ("false", "no", "0"):
            return False
        raise ScenarioSyntaxError(f"expected boolean, got {raw}")

    @staticmethod
    def _assert_equal(label: str, actual: object, expected: object) -> None:
        if actual != expected:
            raise ScenarioAssertionError(
                f"{label}: expected {_normalize(expected)!r}, "
                f"got {_normalize(actual)!r}"
            )


class TerminalInputSource:
    """Lazy POSIX cbreak/select adapter; importing the package has no TTY effect."""

    def __init__(self, stream: TextIO = sys.stdin) -> None:
        self._stream = stream
        self._fd: int | None = None
        self._settings = None

    def __enter__(self) -> "TerminalInputSource":
        import termios
        import tty

        if not self._stream.isatty():
            raise RuntimeError("interactive mode requires a TTY")
        self._fd = self._stream.fileno()
        self._settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def read(self, timeout_s: float) -> str | None:
        import select

        ready, _, _ = select.select([self._stream], [], [], timeout_s)
        if not ready:
            return None
        return self._stream.read(1)

    def __exit__(self, exc_type, exc, traceback) -> None:
        if self._fd is not None and self._settings is not None:
            import termios

            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)


def _emit_interactive_help(
    writer: OutputWriter,
    core: ControlCore,
    *,
    event: str,
) -> None:
    writer.emit(
        event,
        "STATUS_HELP",
        "OK",
        core=core,
        keys=INTERACTIVE_KEY_BINDINGS,
        reliable_key_up=False,
        lease_timeout_ms=round(INTERACTIVE_LEASE_S * 1_000),
        motion_ttl_ms=MOTION_TTL_MS,
        message=INTERACTIVE_HELP_MESSAGE,
    )


def run_interactive_session(
    core: ControlCore,
    source,
    writer: OutputWriter,
    *,
    owner: str = "interactive",
    poll_interval_s: float = 0.05,
) -> int:
    """Run an injectable event loop; tests can provide a clock-free input source."""
    token: LeaseToken | None = None
    exit_code = 0
    cause = SafeStopReason.SHUTDOWN
    entered = False
    try:
        core.connect()
        writer.emit("interactive", "connect", "OK", core=core)
        source.__enter__()
        entered = True
        _emit_interactive_help(writer, core, event="interactive_start")
        while True:
            key = source.read(poll_interval_s)
            core.poll()
            if key is None:
                continue
            if key == "":
                cause = SafeStopReason.EOF
                break
            normalized = key.lower()
            try:
                if normalized == "q":
                    cause = SafeStopReason.SHUTDOWN
                    break
                if normalized == "l":
                    if token is not None and core.snapshot().lease is not None:
                        core.renew_lease(token, INTERACTIVE_LEASE_S)
                    else:
                        token = core.acquire_lease(owner, INTERACTIVE_LEASE_S)
                    writer.emit("key", "LEASE", "OK", core=core)
                    continue
                if normalized == "r":
                    if token is None:
                        raise InvalidControlInput(
                            CoreErrorCode.INVALID_INPUT,
                            "press L to acquire a lease before ARM",
                        )
                    result = core.arm(token)
                    writer.emit("key", "ARM", result.status.value, core=core)
                    continue
                if normalized == "u":
                    if token is None:
                        raise InvalidControlInput(
                            CoreErrorCode.INVALID_INPUT, "no lease token"
                        )
                    result = core.disarm(token)
                    writer.emit("key", "DISARM", result.status.value, core=core)
                    continue
                if normalized == "v":
                    if token is None:
                        raise InvalidControlInput(
                            CoreErrorCode.INVALID_INPUT, "no lease token"
                        )
                    core.release_lease(token)
                    token = None
                    writer.emit("key", "RELEASE", "OK", core=core)
                    continue
                if normalized == "e":
                    result = core.safe_stop(SafeStopReason.USER_REQUEST)
                    token = None
                    writer.emit(
                        "key", "SAFE_STOP", result.status.value, core=core
                    )
                    continue
                if normalized == "?":
                    _emit_interactive_help(writer, core, event="key")
                    continue
                if token is None:
                    raise InvalidControlInput(
                        CoreErrorCode.INVALID_INPUT,
                        "press L to acquire a lease before control",
                    )
                result = apply_control_key(core, token, key)
                writer.emit(
                    "key", _key_name(key), result.status.value, core=core
                )
            except CoreError as exc:
                if core.snapshot().lease is None:
                    token = None
                writer.emit(
                    "key",
                    _key_name(key),
                    "REJECTED",
                    core=core,
                    error_code=exc.code.value,
                    message=str(exc),
                    **_core_error_details(exc),
                )
    except KeyboardInterrupt:
        cause = SafeStopReason.KEYBOARD_INTERRUPT
        exit_code = 130
    except Exception as exc:
        cause = SafeStopReason.UNHANDLED_EXCEPTION
        exit_code = 70
        try:
            writer.emit(
                "interactive",
                type(exc).__name__,
                "ERROR",
                core=core,
                error_code="UNEXPECTED_RUNTIME_ERROR",
                message=str(exc),
            )
        except Exception:
            pass
    finally:
        cleanup = None
        cleanup_error: Exception | None = None
        try:
            try:
                cleanup = core.shutdown(cause)
            except Exception as exc:
                cleanup_error = exc
                exit_code = 70
            if cleanup is None or not cleanup.ok:
                exit_code = 70
            try:
                writer.emit(
                    "cleanup",
                    cause.value,
                    "OK" if cleanup is not None and cleanup.ok else "UNCONFIRMED",
                    core=core,
                    error_code=(
                        cleanup.errors[0]
                        if cleanup is not None and cleanup.errors
                        else "CLEANUP_FAILED" if cleanup_error else None
                    ),
                    message=str(cleanup_error) if cleanup_error else None,
                )
            except Exception:
                exit_code = 70
        finally:
            if entered:
                try:
                    source.__exit__(None, None, None)
                except BaseException as exc:
                    exit_code = 70
                    try:
                        writer.emit(
                            "cleanup",
                            "TERMINAL_RESTORE",
                            "ERROR",
                            core=core,
                            error_code="TERMINAL_RESTORE_FAILED",
                            message=str(exc),
                        )
                    except Exception:
                        pass
        try:
            writer.emit(
                "summary",
                "interactive",
                "COMPLETE",
                core=core,
                exit_code=exit_code,
            )
        except Exception:
            exit_code = 70
    return exit_code


def _key_name(key: str) -> str:
    if key == " ":
        return "SPACE"
    if not key:
        return "EOF"
    return key.upper()


BUILTIN_SCENARIOS: dict[str, tuple[str, int]] = {
    "normal": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        press p w
        expect motion 200 0
        press p x
        press p s
        press p space
        press p a
        press p x
        press p d
        press p x
        press p o
        press p c
        lease renew p 500
        disarm p
        lease release p
        expect mode DISARMED
        """,
        0,
    ),
    "lease-conflict": (
        """
        connect
        lease acquire pilot 500 as p
        expect error LEASE_CONFLICT lease acquire intruder 500 as i
        lease renew p 500
        arm p
        lease release p
        expect lease none
        """,
        0,
    ),
    "lease-expiry": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        press p w
        advance 499
        poll
        expect mode ARMED
        advance 1
        poll
        expect mode SAFE_STOPPED
        expect lease none
        expect safety COMMAND_TIMEOUT
        expect backend safe_stop count 1
        """,
        0,
    ),
    "disconnect": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        press p w
        disconnect
        poll
        expect mode DISCONNECTED
        expect lease none
        expect safety LINK_DISCONNECTED
        expect stop-confirmed false
        """,
        70,
    ),
    "feedback-stale": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        press p w
        feedback stale
        poll
        expect mode SAFE_STOPPED
        expect safety FEEDBACK_STALE
        """,
        0,
    ),
    "timeout-before": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        fault next set_chassis timeout-before-effect
        expect error BACKEND_TIMEOUT delivery NOT_SENT press p w
        expect mode SAFE_STOPPED
        expect lease none
        expect backend safe_stop count 1
        """,
        0,
    ),
    "timeout-after": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        fault next set_chassis timeout-after-effect
        expect error BACKEND_TIMEOUT delivery MAY_HAVE_APPLIED press p w
        expect mode SAFE_STOPPED
        expect motion 0 0
        expect backend safe_stop count 1
        """,
        0,
    ),
    "backend-exception": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        fault next set_gripper unexpected-error
        expect error BACKEND_INTERNAL press p o
        expect mode SAFE_STOPPED
        expect lease none
        """,
        0,
    ),
    "backend-disconnect": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        fault next set_chassis disconnect
        expect error BACKEND_DISCONNECTED press p w
        expect mode DISCONNECTED
        expect stop-confirmed false
        """,
        70,
    ),
    "manual-safe-stop": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        press p w
        safe-stop USER_REQUEST
        expect mode SAFE_STOPPED
        expect lease none
        expect safety USER_REQUEST
        expect stop-confirmed true
        expect backend safe_stop count 1
        """,
        0,
    ),
    "invalid-input": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        expect error INVALID_INPUT press p z
        expect mode SAFE_STOPPED
        expect lease none
        expect safety INVALID_COMMAND
        expect stop-confirmed true
        expect backend safe_stop count 1
        """,
        0,
    ),
    "eof": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        press p w
        terminate eof
        """,
        0,
    ),
    "ctrl-c": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        press p w
        terminate keyboard-interrupt
        """,
        130,
    ),
    "unhandled-exception": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        press p w
        terminate exception
        """,
        70,
    ),
    "cleanup-failure": (
        """
        connect
        lease acquire pilot 500 as p
        arm p
        press p w
        fault next safe_stop timeout-before-effect
        terminate eof
        """,
        70,
    ),
}


BUILTIN_OUTCOME_EXPECTATIONS: dict[
    str, tuple[bool, bool, SafeStopReason]
] = {
    "normal": (True, True, SafeStopReason.SHUTDOWN),
    "lease-conflict": (True, True, SafeStopReason.SHUTDOWN),
    "lease-expiry": (True, True, SafeStopReason.SHUTDOWN),
    "disconnect": (False, False, SafeStopReason.SHUTDOWN),
    "feedback-stale": (True, True, SafeStopReason.SHUTDOWN),
    "timeout-before": (True, True, SafeStopReason.SHUTDOWN),
    "timeout-after": (True, True, SafeStopReason.SHUTDOWN),
    "backend-exception": (True, True, SafeStopReason.SHUTDOWN),
    "backend-disconnect": (False, False, SafeStopReason.SHUTDOWN),
    "manual-safe-stop": (True, True, SafeStopReason.SHUTDOWN),
    "invalid-input": (True, True, SafeStopReason.SHUTDOWN),
    "eof": (True, True, SafeStopReason.EOF),
    "ctrl-c": (True, True, SafeStopReason.KEYBOARD_INTERRUPT),
    "unhandled-exception": (True, True, SafeStopReason.UNHANDLED_EXCEPTION),
    "cleanup-failure": (False, False, SafeStopReason.EOF),
}


def run_builtin_scenarios(
    selection: str, *, writer: OutputWriter
) -> int:
    names = list(BUILTIN_SCENARIOS) if selection == "all" else [selection]
    failures = 0
    for name in names:
        script, expected_exit = BUILTIN_SCENARIOS[name]
        runner = ScenarioRunner(name, writer)
        result = runner.run(script)
        (
            expected_cleanup_ok,
            expected_stop_confirmed,
            expected_cause,
        ) = BUILTIN_OUTCOME_EXPECTATIONS[name]
        passed = (
            result.exit_code == expected_exit
            and result.cleanup.ok is expected_cleanup_ok
            and result.cleanup.stop_confirmed is expected_stop_confirmed
            and result.cause is expected_cause
        )
        if not passed:
            failures += 1
        writer.emit(
            "scenario_check",
            name,
            "PASSED" if passed else "FAILED",
            core=runner.core,
            scenario=name,
            expected_exit=expected_exit,
            actual_exit=result.exit_code,
            expected_cleanup_ok=expected_cleanup_ok,
            actual_cleanup_ok=result.cleanup.ok,
            expected_stop_confirmed=expected_stop_confirmed,
            actual_stop_confirmed=result.cleanup.stop_confirmed,
            expected_cause=expected_cause,
            actual_cause=result.cause,
            passed=passed,
        )
    writer.emit(
        "aggregate_summary",
        selection,
        "PASSED" if failures == 0 else "FAILED",
        scenarios=len(names),
        failures=failures,
        passed=failures == 0,
    )
    return 0 if failures == 0 else 3


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 -m rescue_control",
        description="Hardware-free Rescue Robot ControlCore rehearsal",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    interactive = subparsers.add_parser(
        "interactive", help="Fake backend, deadman-pulse keyboard rehearsal"
    )
    interactive.add_argument("--json", action="store_true", help="emit NDJSON")

    script = subparsers.add_parser(
        "script", help="run a deterministic scenario DSL file"
    )
    script.add_argument("path", help="scenario path, or - for stdin")
    script.add_argument("--json", action="store_true", help="emit NDJSON")

    scenario = subparsers.add_parser(
        "scenario", help="run a built-in normal or safety rehearsal"
    )
    scenario.add_argument(
        "name", choices=("all", *BUILTIN_SCENARIOS.keys())
    )
    scenario.add_argument("--json", action="store_true", help="emit NDJSON")
    return parser


def main(
    argv: list[str] | None = None,
    *,
    stdout: TextIO = sys.stdout,
    stdin: TextIO = sys.stdin,
) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == "interactive":
        clock = time.monotonic
        fake = FakeLowerLink(clock)
        core = ControlCore(fake, clock=clock)
        writer = OutputWriter(stdout, json_mode=args.json, clock=clock)
        return run_interactive_session(
            core, TerminalInputSource(stdin), writer
        )
    if args.command == "script":
        clock = ManualClock()
        writer = OutputWriter(stdout, json_mode=args.json, clock=clock)
        if args.path == "-":
            text = stdin.read()
            name = "stdin"
        else:
            try:
                text = Path(args.path).read_text(encoding="utf-8")
            except OSError as exc:
                parser.error(str(exc))
            name = Path(args.path).name
        return ScenarioRunner(name, writer, clock=clock).run(text).exit_code
    if args.command == "scenario":
        clock = ManualClock()
        writer = OutputWriter(stdout, json_mode=args.json, clock=clock)
        return run_builtin_scenarios(args.name, writer=writer)
    parser.error("unknown command")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
