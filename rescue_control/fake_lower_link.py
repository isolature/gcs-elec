"""Deterministic, hardware-free LowerLink implementation and fault injector."""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import Callable, TypeVar

from .models import (
    ArmState,
    ChassisSetpoint,
    CommandResult,
    CommandStatus,
    DeliveryState,
    FieldValidity,
    GripperState,
    GripperTarget,
    HealthSnapshot,
    LinkSnapshot,
    LowerLinkError,
    LowerLinkErrorCode,
    RobotStateSnapshot,
    SafeStopReason,
    StateField,
)


class ManualClock:
    """A monotonic clock advanced explicitly by tests and scripted rehearsals."""

    def __init__(self, initial: float = 0.0) -> None:
        if not isinstance(initial, (int, float)) or isinstance(initial, bool):
            raise TypeError("initial time must be numeric")
        if not math.isfinite(initial):
            raise ValueError("initial time must be finite")
        self._now = float(initial)

    def __call__(self) -> float:
        return self._now

    def advance(self, seconds: float) -> float:
        if not isinstance(seconds, (int, float)) or isinstance(seconds, bool):
            raise TypeError("advance must be numeric")
        if not math.isfinite(seconds):
            raise ValueError("advance must be finite")
        if seconds < 0:
            raise ValueError("monotonic time cannot move backwards")
        self._now += float(seconds)
        return self._now


class FaultKind(str, Enum):
    REJECT = "REJECT"
    DISCONNECT = "DISCONNECT"
    TIMEOUT_BEFORE_EFFECT = "TIMEOUT_BEFORE_EFFECT"
    TIMEOUT_AFTER_EFFECT = "TIMEOUT_AFTER_EFFECT"
    PROTOCOL_ERROR = "PROTOCOL_ERROR"
    UNEXPECTED_ERROR = "UNEXPECTED_ERROR"


@dataclass(frozen=True)
class FaultPlan:
    kind: FaultKind
    message: str = "injected fault"


@dataclass(frozen=True)
class FakeStateSnapshot:
    connected: bool
    arm_state: ArmState
    linear_velocity_mm_s: int
    angular_velocity_mrad_s: int
    gripper_target: GripperTarget | None
    feedback_validity: FieldValidity


@dataclass(frozen=True)
class CommandRecord:
    sequence: int
    time_s: float
    operation: str
    parameters: tuple[tuple[str, object], ...]
    outcome: str
    before: FakeStateSnapshot
    after: FakeStateSnapshot
    error_code: str | None = None
    delivery: DeliveryState | None = None


R = TypeVar("R")


class FakeLowerLink:
    """Stateful fake whose only time and faults are explicitly injected."""

    OPERATIONS = frozenset(
        {
            "connect",
            "close",
            "arm",
            "disarm",
            "set_chassis",
            "stop",
            "safe_stop",
            "set_gripper",
            "get_health",
            "get_robot_state",
        }
    )
    COMMAND_OPERATIONS = frozenset(
        {
            "arm",
            "disarm",
            "set_chassis",
            "stop",
            "safe_stop",
            "set_gripper",
        }
    )

    def __init__(self, clock: Callable[[], float] | None = None) -> None:
        self._clock = clock or ManualClock()
        self._connected = False
        self._arm_state = ArmState.DISARMED
        self._linear = 0
        self._angular = 0
        self._gripper_target: GripperTarget | None = None
        self._feedback_value: bool | None = None
        self._feedback_validity = FieldValidity.UNKNOWN
        self._feedback_observed_at: float | None = None
        self._feedback_reason = "no feedback observed"
        self._reported_arm_state: ArmState | None = None
        self._reported_arm_validity = FieldValidity.UNKNOWN
        self._reported_arm_observed_at: float | None = None
        self._reported_arm_reason = "no arm feedback observed"
        self._fault_bits_value: int | None = None
        self._fault_bits_validity = FieldValidity.UNKNOWN
        self._fault_bits_observed_at: float | None = None
        self._fault_bits_reason = "no fault feedback observed"
        self._history: list[CommandRecord] = []
        self._faults: dict[str, deque[FaultPlan]] = defaultdict(deque)
        self._results: dict[str, deque[CommandStatus]] = defaultdict(deque)
        self._sequence = 0

    @property
    def history(self) -> tuple[CommandRecord, ...]:
        return tuple(self._history)

    @property
    def state(self) -> FakeStateSnapshot:
        return self._snapshot()

    def count(self, operation: str) -> int:
        return sum(record.operation == operation for record in self._history)

    def fail_next(
        self, operation: str, kind: FaultKind, message: str = "injected fault"
    ) -> None:
        self._validate_operation(operation)
        if not isinstance(kind, FaultKind):
            raise TypeError("kind must be FaultKind")
        if kind is FaultKind.REJECT and operation not in self.COMMAND_OPERATIONS:
            raise ValueError("REJECT is only valid for command operations")
        self._faults[operation].append(FaultPlan(kind, message))

    def result_next(self, operation: str, status: CommandStatus) -> None:
        self._validate_operation(operation)
        if operation not in self.COMMAND_OPERATIONS:
            raise ValueError("result injection is only valid for command operations")
        if not isinstance(status, CommandStatus):
            raise TypeError("status must be CommandStatus")
        if status is CommandStatus.REJECTED:
            self.fail_next(operation, FaultKind.REJECT, "injected rejection")
            return
        if not isinstance(status, CommandStatus):
            raise TypeError("status must be CommandStatus")
        self._results[operation].append(status)

    def inject_disconnect(self) -> None:
        """Drop observability without pretending the physical target became safe."""
        self._connected = False
        self._mark_reports_stale("link disconnected")

    def inject_health(
        self,
        *,
        feedback_value: bool | None,
        feedback_validity: FieldValidity,
        reported_arm_state: ArmState | None,
        reported_arm_validity: FieldValidity,
        fault_bits_value: int | None,
        fault_bits_validity: FieldValidity,
        reason: str = "injected health state",
    ) -> None:
        """Replace reported health fields without changing physical fake state."""
        if feedback_value is not None and not isinstance(feedback_value, bool):
            raise TypeError("feedback_value must be bool or None")
        if not isinstance(feedback_validity, FieldValidity):
            raise TypeError("feedback_validity must be FieldValidity")
        if reported_arm_state is not None and not isinstance(
            reported_arm_state, ArmState
        ):
            raise TypeError("reported_arm_state must be ArmState or None")
        if not isinstance(reported_arm_validity, FieldValidity):
            raise TypeError("reported_arm_validity must be FieldValidity")
        if fault_bits_value is not None and type(fault_bits_value) is not int:
            raise TypeError("fault_bits_value must be int or None")
        if fault_bits_value is not None and fault_bits_value < 0:
            raise ValueError("fault_bits_value must be non-negative")
        if not isinstance(fault_bits_validity, FieldValidity):
            raise TypeError("fault_bits_validity must be FieldValidity")
        if feedback_validity is FieldValidity.VALID and feedback_value is None:
            raise ValueError("VALID feedback requires a bool value")
        if (
            reported_arm_validity is FieldValidity.VALID
            and reported_arm_state is None
        ):
            raise ValueError("VALID reported arm state requires a value")
        if fault_bits_validity is FieldValidity.VALID and fault_bits_value is None:
            raise ValueError("VALID fault bits require an int value")
        if not isinstance(reason, str):
            raise TypeError("reason must be str")

        self._feedback_value = feedback_value
        self._feedback_observed_at = self._injected_observed_at(
            feedback_validity, self._feedback_observed_at
        )
        self._feedback_validity = feedback_validity
        self._feedback_reason = reason
        self._reported_arm_state = reported_arm_state
        self._reported_arm_observed_at = self._injected_observed_at(
            reported_arm_validity, self._reported_arm_observed_at
        )
        self._reported_arm_validity = reported_arm_validity
        self._reported_arm_reason = reason
        self._fault_bits_value = fault_bits_value
        self._fault_bits_observed_at = self._injected_observed_at(
            fault_bits_validity, self._fault_bits_observed_at
        )
        self._fault_bits_validity = fault_bits_validity
        self._fault_bits_reason = reason

    def set_feedback_validity(
        self, validity: FieldValidity, reason: str = "injected feedback state"
    ) -> None:
        if not isinstance(validity, FieldValidity):
            raise TypeError("validity must be FieldValidity")
        if validity is FieldValidity.VALID and self._feedback_value is None:
            self._feedback_value = True
        self._feedback_observed_at = self._injected_observed_at(
            validity, self._feedback_observed_at
        )
        self._feedback_validity = validity
        self._feedback_reason = reason

    def make_feedback_stale(self, reason: str = "feedback timeout") -> None:
        self.set_feedback_validity(FieldValidity.STALE, reason)

    def connect(self) -> LinkSnapshot:
        def effect() -> LinkSnapshot:
            self._connected = True
            now = self._clock()
            self._feedback_value = True
            self._feedback_validity = FieldValidity.VALID
            self._feedback_observed_at = now
            self._feedback_reason = ""
            self._reported_arm_state = self._arm_state
            self._reported_arm_validity = FieldValidity.VALID
            self._reported_arm_observed_at = now
            self._reported_arm_reason = ""
            if self._fault_bits_value is None:
                self._fault_bits_value = 0
            self._fault_bits_validity = FieldValidity.VALID
            self._fault_bits_observed_at = now
            self._fault_bits_reason = ""
            return LinkSnapshot(True, now, "fake link connected")

        return self._invoke("connect", {}, effect, requires_connection=False)

    def close(self) -> None:
        def effect() -> None:
            self._connected = False
            self._mark_reports_stale("link closed")
            return None

        return self._invoke("close", {}, effect, requires_connection=False)

    def arm(self) -> CommandResult:
        return self._command("arm", {}, lambda: self._set_arm(ArmState.ARMED))

    def disarm(self) -> CommandResult:
        def effect() -> None:
            self._set_arm(ArmState.DISARMED)
            self._linear = 0
            self._angular = 0

        return self._command("disarm", {}, effect)

    def set_chassis(self, target: ChassisSetpoint) -> CommandResult:
        if not isinstance(target, ChassisSetpoint):
            raise TypeError("target must be ChassisSetpoint")

        def effect() -> None:
            if self._arm_state is not ArmState.ARMED:
                raise _FakeReject("lower controller is not armed")
            self._linear = target.linear_velocity_mm_s
            self._angular = target.angular_velocity_mrad_s

        return self._command(
            "set_chassis",
            {
                "linear_velocity_mm_s": target.linear_velocity_mm_s,
                "angular_velocity_mrad_s": target.angular_velocity_mrad_s,
                "ttl_ms": target.ttl_ms,
            },
            effect,
        )

    def stop(self) -> CommandResult:
        def effect() -> None:
            self._linear = 0
            self._angular = 0

        return self._command("stop", {}, effect)

    def safe_stop(self, reason: SafeStopReason) -> CommandResult:
        if not isinstance(reason, SafeStopReason):
            raise TypeError("reason must be SafeStopReason")

        def effect() -> None:
            self._linear = 0
            self._angular = 0
            self._set_arm(ArmState.SAFE_STOP)

        return self._command("safe_stop", {"reason": reason.value}, effect)

    def set_gripper(self, target: GripperTarget) -> CommandResult:
        if not isinstance(target, GripperTarget):
            raise TypeError("target must be GripperTarget")

        def effect() -> None:
            if self._arm_state is not ArmState.ARMED:
                raise _FakeReject("lower controller is not armed")
            self._gripper_target = target

        return self._command(
            "set_gripper", {"target": target.value}, effect
        )

    def get_health(self) -> HealthSnapshot:
        def effect() -> HealthSnapshot:
            now = self._clock()
            return HealthSnapshot(
                connected=self._connected,
                arm_state=StateField(
                    self._reported_arm_validity,
                    self._reported_arm_state,
                    self._reported_arm_observed_at,
                    self._reported_arm_reason or None,
                ),
                feedback=StateField(
                    self._feedback_validity,
                    self._feedback_value,
                    self._feedback_observed_at,
                    self._feedback_reason or None,
                ),
                fault_bits=StateField(
                    self._fault_bits_validity,
                    self._fault_bits_value,
                    self._fault_bits_observed_at,
                    self._fault_bits_reason or None,
                ),
                observed_at=now,
            )

        return self._invoke(
            "get_health", {}, effect, requires_connection=False
        )

    def get_robot_state(self) -> RobotStateSnapshot:
        def effect() -> RobotStateSnapshot:
            now = self._clock()
            reason = self._feedback_reason or None
            gripper_state = None
            if self._gripper_target is GripperTarget.OPEN:
                gripper_state = GripperState.OPEN
            elif self._gripper_target is GripperTarget.CLOSED:
                gripper_state = GripperState.CLOSED
            return RobotStateSnapshot(
                linear_velocity_mm_s=StateField(
                    self._feedback_validity,
                    self._linear,
                    self._feedback_observed_at,
                    reason,
                ),
                angular_velocity_mrad_s=StateField(
                    self._feedback_validity,
                    self._angular,
                    self._feedback_observed_at,
                    reason,
                ),
                gripper_state=StateField(
                    self._feedback_validity
                    if gripper_state is not None
                    else FieldValidity.UNKNOWN,
                    gripper_state,
                    self._feedback_observed_at if gripper_state is not None else None,
                    reason if gripper_state is not None else "gripper not commanded",
                ),
                arm_state=StateField(
                    self._reported_arm_validity,
                    self._reported_arm_state,
                    self._reported_arm_observed_at,
                    self._reported_arm_reason or None,
                ),
                observed_at=now,
            )

        return self._invoke(
            "get_robot_state", {}, effect, requires_connection=False
        )

    def _command(
        self,
        operation: str,
        parameters: dict[str, object],
        effect: Callable[[], None],
    ) -> CommandResult:
        def command_effect() -> CommandResult:
            status = (
                self._results[operation].popleft()
                if self._results[operation]
                else CommandStatus.COMPLETED
            )
            effect()
            return CommandResult(status, operation, "fake command result")

        return self._invoke(operation, parameters, command_effect)

    def _invoke(
        self,
        operation: str,
        parameters: dict[str, object],
        effect: Callable[[], R],
        *,
        requires_connection: bool = True,
    ) -> R:
        before = self._snapshot()
        if requires_connection and not self._connected:
            error = LowerLinkError(
                operation,
                LowerLinkErrorCode.DISCONNECTED,
                "fake lower link is disconnected",
            )
            self._record_error(operation, parameters, before, error)
            raise error

        plan = self._faults[operation].popleft() if self._faults[operation] else None

        if plan is not None:
            if plan.kind is FaultKind.REJECT:
                result = CommandResult(
                    CommandStatus.REJECTED, operation, plan.message
                )
                self._record(operation, parameters, result.status.value, before)
                return result  # type: ignore[return-value]
            if plan.kind is FaultKind.DISCONNECT:
                self.inject_disconnect()
                error = LowerLinkError(
                    operation,
                    LowerLinkErrorCode.DISCONNECTED,
                    plan.message,
                )
                self._record_error(operation, parameters, before, error)
                raise error
            if plan.kind is FaultKind.TIMEOUT_BEFORE_EFFECT:
                error = LowerLinkError(
                    operation,
                    LowerLinkErrorCode.TIMEOUT,
                    plan.message,
                    retryable=True,
                )
                self._record_error(operation, parameters, before, error)
                raise error
            if plan.kind is FaultKind.TIMEOUT_AFTER_EFFECT:
                try:
                    effect()
                except _FakeReject as exc:
                    result = CommandResult(CommandStatus.REJECTED, operation, str(exc))
                    self._record(operation, parameters, result.status.value, before)
                    return result  # type: ignore[return-value]
                error = LowerLinkError(
                    operation,
                    LowerLinkErrorCode.TIMEOUT,
                    plan.message,
                    delivery=DeliveryState.MAY_HAVE_APPLIED,
                    retryable=False,
                )
                self._record_error(operation, parameters, before, error)
                raise error
            if plan.kind is FaultKind.PROTOCOL_ERROR:
                error = LowerLinkError(
                    operation,
                    LowerLinkErrorCode.PROTOCOL,
                    plan.message,
                )
                self._record_error(operation, parameters, before, error)
                raise error
            if plan.kind is FaultKind.UNEXPECTED_ERROR:
                error = RuntimeError(plan.message)
                self._record(
                    operation,
                    parameters,
                    "ERROR",
                    before,
                    error_code=LowerLinkErrorCode.INTERNAL.value,
                    delivery=DeliveryState.NOT_SENT,
                )
                raise error

        try:
            result = effect()
        except _FakeReject as exc:
            result = CommandResult(CommandStatus.REJECTED, operation, str(exc))
            self._record(operation, parameters, result.status.value, before)
            return result  # type: ignore[return-value]
        self._record(
            operation,
            parameters,
            result.status.value if isinstance(result, CommandResult) else "OK",
            before,
        )
        return result

    def _record_error(
        self,
        operation: str,
        parameters: dict[str, object],
        before: FakeStateSnapshot,
        error: LowerLinkError,
    ) -> None:
        self._record(
            operation,
            parameters,
            "ERROR",
            before,
            error_code=error.code.value,
            delivery=error.delivery,
        )

    def _record(
        self,
        operation: str,
        parameters: dict[str, object],
        outcome: str,
        before: FakeStateSnapshot,
        *,
        error_code: str | None = None,
        delivery: DeliveryState | None = None,
    ) -> None:
        self._sequence += 1
        self._history.append(
            CommandRecord(
                self._sequence,
                self._clock(),
                operation,
                tuple(sorted(parameters.items())),
                outcome,
                before,
                self._snapshot(),
                error_code,
                delivery,
            )
        )

    def _snapshot(self) -> FakeStateSnapshot:
        return FakeStateSnapshot(
            self._connected,
            self._arm_state,
            self._linear,
            self._angular,
            self._gripper_target,
            self._feedback_validity,
        )

    def _set_arm(self, state: ArmState) -> None:
        self._arm_state = state
        if self._connected:
            self._reported_arm_state = state
            self._reported_arm_validity = FieldValidity.VALID
            self._reported_arm_observed_at = self._clock()
            self._reported_arm_reason = ""

    def _mark_reports_stale(self, reason: str) -> None:
        for validity_name, observed_at in (
            ("_feedback_validity", self._feedback_observed_at),
            ("_reported_arm_validity", self._reported_arm_observed_at),
            ("_fault_bits_validity", self._fault_bits_observed_at),
        ):
            setattr(
                self,
                validity_name,
                FieldValidity.STALE
                if observed_at is not None
                else FieldValidity.UNKNOWN,
            )
        self._feedback_reason = reason
        self._reported_arm_reason = reason
        self._fault_bits_reason = reason

    def _injected_observed_at(
        self, validity: FieldValidity, previous: float | None
    ) -> float | None:
        if validity is FieldValidity.STALE:
            return previous
        if validity is FieldValidity.UNKNOWN:
            return None
        return self._clock()

    @classmethod
    def _validate_operation(cls, operation: str) -> None:
        if operation not in cls.OPERATIONS:
            raise ValueError(f"unknown fake operation: {operation}")


class _FakeReject(RuntimeError):
    pass
