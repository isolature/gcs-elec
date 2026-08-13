"""Single-owner control gate and safety state machine."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable

from .lower_link import LowerLink
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


class CoreMode(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    DISARMED = "DISARMED"
    ARMED = "ARMED"
    SAFE_STOPPED = "SAFE_STOPPED"
    FAULT = "FAULT"
    CLOSED = "CLOSED"


class CoreErrorCode(str, Enum):
    NOT_CONNECTED = "NOT_CONNECTED"
    INVALID_STATE = "INVALID_STATE"
    LEASE_CONFLICT = "LEASE_CONFLICT"
    INVALID_LEASE = "INVALID_LEASE"
    LEASE_EXPIRED = "LEASE_EXPIRED"
    INVALID_INPUT = "INVALID_INPUT"
    COMMAND_REJECTED = "COMMAND_REJECTED"
    BACKEND_DISCONNECTED = "BACKEND_DISCONNECTED"
    BACKEND_TIMEOUT = "BACKEND_TIMEOUT"
    BACKEND_IO = "BACKEND_IO"
    BACKEND_PROTOCOL = "BACKEND_PROTOCOL"
    BACKEND_INTERNAL = "BACKEND_INTERNAL"


class CoreError(RuntimeError):
    def __init__(self, code: CoreErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code


class LeaseConflictError(CoreError):
    pass


class InvalidLeaseError(CoreError):
    pass


class InvalidControlInput(CoreError):
    pass


class InvalidCoreState(CoreError):
    pass


class CommandRejectedError(CoreError):
    def __init__(self, operation: str, message: str) -> None:
        super().__init__(CoreErrorCode.COMMAND_REJECTED, message)
        self.operation = operation


class CoreBackendError(CoreError):
    _CODE_MAP = {
        LowerLinkErrorCode.DISCONNECTED: CoreErrorCode.BACKEND_DISCONNECTED,
        LowerLinkErrorCode.TIMEOUT: CoreErrorCode.BACKEND_TIMEOUT,
        LowerLinkErrorCode.IO: CoreErrorCode.BACKEND_IO,
        LowerLinkErrorCode.PROTOCOL: CoreErrorCode.BACKEND_PROTOCOL,
        LowerLinkErrorCode.INTERNAL: CoreErrorCode.BACKEND_INTERNAL,
    }

    def __init__(self, error: LowerLinkError) -> None:
        super().__init__(self._CODE_MAP[error.code], str(error))
        self.operation = error.operation
        self.delivery = error.delivery
        self.retryable = error.retryable


@dataclass(frozen=True)
class CoreConfig:
    default_lease_duration_s: float = 0.5
    max_lease_duration_s: float = 10.0
    max_abs_linear_velocity_mm_s: int = 500
    max_abs_angular_velocity_mrad_s: int = 2_000
    min_ttl_ms: int = 1
    max_ttl_ms: int = 1_000

    def __post_init__(self) -> None:
        if self.default_lease_duration_s <= 0:
            raise ValueError("default lease duration must be positive")
        if self.max_lease_duration_s < self.default_lease_duration_s:
            raise ValueError("max lease duration must cover the default")
        if self.max_abs_linear_velocity_mm_s <= 0:
            raise ValueError("linear velocity limit must be positive")
        if self.max_abs_angular_velocity_mrad_s <= 0:
            raise ValueError("angular velocity limit must be positive")
        if not 1 <= self.min_ttl_ms <= self.max_ttl_ms:
            raise ValueError("invalid TTL limits")


@dataclass(frozen=True)
class LeaseToken:
    owner: str
    generation: int


@dataclass(frozen=True)
class LeaseSnapshot:
    owner: str
    generation: int
    deadline_s: float
    remaining_s: float


@dataclass(frozen=True)
class CoreSnapshot:
    time_s: float
    mode: CoreMode
    connected: bool
    lease: LeaseSnapshot | None
    armed_generation: int | None
    commanded_linear_velocity_mm_s: int
    commanded_angular_velocity_mrad_s: int
    commanded_gripper_target: GripperTarget | None
    last_safety_reason: SafeStopReason | None
    stop_attempted: bool
    stop_confirmed: bool
    last_error: str | None

    @property
    def armed(self) -> bool:
        return self.mode is CoreMode.ARMED


@dataclass(frozen=True)
class CoreEvent:
    sequence: int
    time_s: float
    event: str
    outcome: str
    details: tuple[tuple[str, object], ...]


@dataclass(frozen=True)
class SafetyAttempt:
    reason: SafeStopReason
    result: CommandResult | None
    error: CoreError | None

    @property
    def confirmed(self) -> bool:
        return self.result is not None and self.result.confirmed


@dataclass(frozen=True)
class CleanupResult:
    stop_attempted: bool
    stop_confirmed: bool
    errors: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return not self.errors and self.stop_confirmed


@dataclass
class _LeaseRecord:
    token: LeaseToken
    deadline_s: float
    duration_s: float


class ControlCore:
    """The only owner of leases, command gating and upper-layer safety policy."""

    def __init__(
        self,
        lower_link: LowerLink,
        *,
        clock: Callable[[], float] = time.monotonic,
        config: CoreConfig | None = None,
    ) -> None:
        self._link = lower_link
        self._clock = clock
        self._config = config or CoreConfig()
        self._lock = threading.RLock()
        self._mode = CoreMode.DISCONNECTED
        self._connected = False
        self._lease: _LeaseRecord | None = None
        self._armed_generation: int | None = None
        self._next_generation = 1
        self._commanded_linear = 0
        self._commanded_angular = 0
        self._commanded_gripper: GripperTarget | None = None
        self._last_safety_reason: SafeStopReason | None = None
        self._stop_attempted = False
        self._stop_confirmed = False
        self._last_error: str | None = None
        self._events: list[CoreEvent] = []
        self._event_sequence = 0
        self._cleanup_result: CleanupResult | None = None

    @property
    def events(self) -> tuple[CoreEvent, ...]:
        with self._lock:
            return tuple(self._events)

    def snapshot(self) -> CoreSnapshot:
        with self._lock:
            return self._snapshot_locked()

    def connect(self) -> LinkSnapshot:
        with self._lock:
            if self._mode is CoreMode.CLOSED:
                raise InvalidCoreState(
                    CoreErrorCode.INVALID_STATE, "closed core cannot reconnect"
                )
            if self._connected:
                return LinkSnapshot(True, self._clock(), "already connected")
            try:
                snapshot = self._link.connect()
                self._validate_link_snapshot_locked(snapshot)
            except Exception as exc:
                error = self._as_backend_error("connect", exc)
                self._mode = CoreMode.DISCONNECTED
                self._connected = False
                self._last_error = error.code.value
                self._record_event("connect", "ERROR", error=error.code.value)
                raise error from exc
            if not snapshot.connected:
                error = CoreBackendError(
                    LowerLinkError(
                        "connect",
                        LowerLinkErrorCode.DISCONNECTED,
                        "lower link returned a disconnected connect snapshot",
                    )
                )
                self._last_error = error.code.value
                self._record_event("connect", "ERROR", error=error.code.value)
                raise error

            self._connected = True
            self._lease = None
            self._armed_generation = None
            operation = "get_health"
            try:
                health = self._link.get_health()
                self._validate_health_snapshot_locked(health)
                operation = "get_robot_state"
                robot_state = self._link.get_robot_state()
                self._validate_robot_state_snapshot_locked(robot_state)
            except Exception as exc:
                error = self._as_backend_error(operation, exc)
                self._attempt_safe_stop_locked(
                    SafeStopReason.BACKEND_FAILURE, clear_lease=True
                )
                self._mode = CoreMode.FAULT
                self._last_error = error.code.value
                self._record_event("connect", "ERROR", error=error.code.value)
                raise error from exc

            if not health.connected:
                self._attempt_safe_stop_locked(
                    SafeStopReason.LINK_DISCONNECTED, clear_lease=True
                )
                self._connected = False
                self._mode = CoreMode.DISCONNECTED
                error = CoreBackendError(
                    LowerLinkError(
                        "get_health",
                        LowerLinkErrorCode.DISCONNECTED,
                        "lower link disconnected during connect recovery",
                    )
                )
                self._last_error = error.code.value
                self._record_event("connect", "ERROR", error=error.code.value)
                raise error

            issue = self._health_safety_issue_locked(health)
            state = health.arm_state.value
            motion_safe = self._robot_state_confirms_safe_motion_locked(
                robot_state, expected_arm=state
            )
            if (
                issue is None
                and state is ArmState.DISARMED
                and motion_safe
            ):
                self._mode = CoreMode.DISARMED
                self._commanded_linear = 0
                self._commanded_angular = 0
                self._last_error = None
                self._record_event("connect", "CONNECTED")
                return snapshot
            if (
                issue is None
                and state is ArmState.SAFE_STOP
                and motion_safe
            ):
                self._mode = CoreMode.SAFE_STOPPED
                self._commanded_linear = 0
                self._commanded_angular = 0
                self._last_error = None
                self._record_event("connect", "CONNECTED_SAFE_STOPPED")
                return snapshot

            attempt = self._attempt_safe_stop_locked(
                issue or SafeStopReason.FEEDBACK_INVALID, clear_lease=True
            )
            if attempt.error is not None:
                self._record_event(
                    "connect", "RECOVERY_ERROR", error=attempt.error.code.value
                )
                raise attempt.error
            if not attempt.confirmed:
                self._record_event("connect", "RECOVERY_UNCONFIRMED")
                raise InvalidCoreState(
                    CoreErrorCode.INVALID_STATE,
                    "connected backend state was not safe and recovery was unconfirmed",
                )
            self._last_error = None
            self._record_event("connect", "RECOVERED_SAFE_STOP")
            return snapshot

    def acquire_lease(
        self, owner: str, duration_s: float | None = None
    ) -> LeaseToken:
        with self._lock:
            self._require_connected_locked()
            self._require_operable_locked()
            self._expire_lease_locked()
            if self._mode in (CoreMode.FAULT, CoreMode.DISCONNECTED):
                raise InvalidCoreState(
                    CoreErrorCode.INVALID_STATE,
                    "cannot acquire a lease after an unconfirmed safety failure",
                )
            normalized_owner = self._validate_owner(owner)
            duration = self._validate_duration(duration_s)
            if self._lease is not None:
                raise LeaseConflictError(
                    CoreErrorCode.LEASE_CONFLICT,
                    f"control is held by {self._lease.token.owner}",
                )
            token = LeaseToken(normalized_owner, self._next_generation)
            self._next_generation += 1
            self._lease = _LeaseRecord(
                token, self._clock() + duration, duration
            )
            self._record_event(
                "lease_acquired",
                "OK",
                owner=token.owner,
                generation=token.generation,
                duration_s=duration,
            )
            return token

    def renew_lease(
        self, token: LeaseToken, duration_s: float | None = None
    ) -> LeaseToken:
        with self._lock:
            record = self._require_lease_locked(token)
            duration = self._validate_duration(
                record.duration_s if duration_s is None else duration_s
            )
            record.duration_s = duration
            record.deadline_s = self._clock() + duration
            self._record_event(
                "lease_renewed",
                "OK",
                owner=token.owner,
                generation=token.generation,
                duration_s=duration,
            )
            return token

    def release_lease(self, token: LeaseToken) -> None:
        with self._lock:
            self._require_lease_locked(token)
            if self._mode is CoreMode.ARMED and self._armed_generation is not None:
                if self._armed_generation != token.generation:
                    attempt = self._attempt_safe_stop_locked(
                        SafeStopReason.BACKEND_FAILURE, clear_lease=True
                    )
                    if attempt.error is not None:
                        raise attempt.error
                    raise InvalidCoreState(
                        CoreErrorCode.INVALID_STATE,
                        "armed generation did not match the releasing lease",
                    )
                stop_result = self.stop(token)
                if not stop_result.confirmed:
                    attempt = self._attempt_safe_stop_locked(
                        SafeStopReason.BACKEND_FAILURE, clear_lease=True
                    )
                    if attempt.error is not None:
                        raise attempt.error
                    raise InvalidCoreState(
                        CoreErrorCode.INVALID_STATE,
                        "lease release stop was not confirmed; safe-stop required",
                    )
            self._lease = None
            self._armed_generation = None
            self._record_event(
                "lease_released",
                "OK",
                owner=token.owner,
                generation=token.generation,
            )

    def arm(self, token: LeaseToken) -> CommandResult:
        with self._lock:
            record = self._require_lease_locked(token)
            if self._mode not in (
                CoreMode.DISARMED,
                CoreMode.SAFE_STOPPED,
                CoreMode.ARMED,
            ):
                raise InvalidCoreState(
                    CoreErrorCode.INVALID_STATE,
                    f"cannot arm while core mode is {self._mode.value}",
                )
            if (
                self._mode is CoreMode.ARMED
                and self._armed_generation not in (None, token.generation)
            ):
                raise InvalidCoreState(
                    CoreErrorCode.INVALID_STATE,
                    "another lease generation is bound to the armed state",
                )
            result = self._call_command_locked("arm", self._link.arm)
            self._ensure_lease_fresh_after_backend_locked(record)
            if not result.confirmed:
                self._raise_after_unconfirmed_critical_locked("arm")
            self._mode = CoreMode.ARMED
            self._armed_generation = token.generation
            self._renew_after_success_locked(record)
            self._record_event("arm", result.status.value)
            return result

    def disarm(self, token: LeaseToken) -> CommandResult:
        with self._lock:
            record = self._require_lease_locked(token)
            self._require_armed_locked(token)
            result = self._call_command_locked("disarm", self._link.disarm)
            self._ensure_lease_fresh_after_backend_locked(record)
            if not result.confirmed:
                self._raise_after_unconfirmed_critical_locked("disarm")
            self._mode = CoreMode.DISARMED
            self._armed_generation = None
            self._commanded_linear = 0
            self._commanded_angular = 0
            self._renew_after_success_locked(record)
            self._record_event("disarm", result.status.value)
            return result

    def set_chassis(
        self,
        token: LeaseToken,
        linear_velocity_mm_s: object,
        angular_velocity_mrad_s: object,
        ttl_ms: object,
    ) -> CommandResult:
        with self._lock:
            record = self._require_lease_locked(token)
            self._require_armed_locked(token)
            try:
                target = self._validate_chassis_locked(
                    linear_velocity_mm_s,
                    angular_velocity_mrad_s,
                    ttl_ms,
                    record,
                )
            except InvalidControlInput:
                self._attempt_safe_stop_locked(
                    SafeStopReason.INVALID_COMMAND, clear_lease=True
                )
                raise
            result = self._call_command_locked(
                "set_chassis", lambda: self._link.set_chassis(target)
            )
            self._ensure_lease_fresh_after_backend_locked(record)
            self._commanded_linear = target.linear_velocity_mm_s
            self._commanded_angular = target.angular_velocity_mrad_s
            self._renew_after_success_locked(record)
            self._record_event(
                "set_chassis",
                result.status.value,
                linear_velocity_mm_s=target.linear_velocity_mm_s,
                angular_velocity_mrad_s=target.angular_velocity_mrad_s,
                ttl_ms=target.ttl_ms,
            )
            return result

    def stop(self, token: LeaseToken) -> CommandResult:
        with self._lock:
            record = self._require_lease_locked(token)
            self._require_armed_locked(token)
            if self._commanded_linear == 0 and self._commanded_angular == 0:
                result = CommandResult(
                    CommandStatus.COMPLETED, "stop", "already stopped"
                )
                self._renew_after_success_locked(record)
                self._record_event("stop", "DEDUPLICATED")
                return result
            result = self._call_command_locked("stop", self._link.stop)
            self._ensure_lease_fresh_after_backend_locked(record)
            if result.confirmed:
                self._commanded_linear = 0
                self._commanded_angular = 0
                self._renew_after_success_locked(record)
            self._record_event("stop", result.status.value)
            return result

    def set_gripper(
        self, token: LeaseToken, target: GripperTarget
    ) -> CommandResult:
        with self._lock:
            record = self._require_lease_locked(token)
            self._require_armed_locked(token)
            if not isinstance(target, GripperTarget):
                self._attempt_safe_stop_locked(
                    SafeStopReason.INVALID_COMMAND, clear_lease=True
                )
                raise InvalidControlInput(
                    CoreErrorCode.INVALID_INPUT,
                    "gripper target must be OPEN or CLOSED",
                )
            result = self._call_command_locked(
                "set_gripper", lambda: self._link.set_gripper(target)
            )
            self._ensure_lease_fresh_after_backend_locked(record)
            self._commanded_gripper = target
            self._renew_after_success_locked(record)
            self._record_event(
                "set_gripper", result.status.value, target=target.value
            )
            return result

    def safe_stop(self, reason: SafeStopReason) -> CommandResult:
        with self._lock:
            if not isinstance(reason, SafeStopReason):
                raise InvalidControlInput(
                    CoreErrorCode.INVALID_INPUT,
                    "safe-stop reason must be SafeStopReason",
                )
            attempt = self._attempt_safe_stop_locked(reason, clear_lease=True)
            if attempt.error is not None:
                raise attempt.error
            assert attempt.result is not None
            return attempt.result

    def poll(self) -> CoreSnapshot:
        """Process lease and health deadlines without sleeping or creating a thread."""
        with self._lock:
            if self._mode is CoreMode.CLOSED:
                return self._snapshot_locked()
            if self._expire_lease_locked():
                return self._snapshot_locked()
            if not self._connected:
                return self._snapshot_locked()
            try:
                health = self._link.get_health()
                self._validate_health_snapshot_locked(health)
            except Exception as exc:
                primary = self._as_backend_error("get_health", exc)
                self._attempt_safe_stop_locked(
                    SafeStopReason.BACKEND_FAILURE, clear_lease=True
                )
                self._last_error = primary.code.value
                self._record_event(
                    "health_check", "ERROR", error=primary.code.value
                )
                return self._snapshot_locked()

            if not health.connected:
                self._attempt_safe_stop_locked(
                    SafeStopReason.LINK_DISCONNECTED, clear_lease=True
                )
                self._connected = False
                self._mode = CoreMode.DISCONNECTED
                self._record_event("health_check", "DISCONNECTED")
                return self._snapshot_locked()

            issue = self._health_safety_issue_locked(health)
            if issue is not None:
                self._attempt_safe_stop_locked(issue, clear_lease=True)
                self._record_event("health_check", issue.value)
                return self._snapshot_locked()

            self._reconcile_arm_state_locked(health)
            self._record_event("health_check", "OK")
            return self._snapshot_locked()

    def get_robot_state(self) -> RobotStateSnapshot:
        with self._lock:
            self._require_connected_locked()
            try:
                state = self._link.get_robot_state()
                self._validate_robot_state_snapshot_locked(state)
            except Exception as exc:
                primary = self._as_backend_error("get_robot_state", exc)
                if self._mode is CoreMode.ARMED or self._lease is not None:
                    self._attempt_safe_stop_locked(
                        SafeStopReason.BACKEND_FAILURE, clear_lease=True
                    )
                self._last_error = primary.code.value
                raise primary from exc
            return state

    def shutdown(
        self, cause: SafeStopReason = SafeStopReason.SHUTDOWN
    ) -> CleanupResult:
        with self._lock:
            if self._cleanup_result is not None:
                return self._cleanup_result
            if not isinstance(cause, SafeStopReason):
                cause = SafeStopReason.UNHANDLED_EXCEPTION
            errors: list[str] = []
            attempt = self._attempt_safe_stop_locked(cause, clear_lease=True)
            if attempt.error is not None:
                errors.append(attempt.error.code.value)
            try:
                self._link.close()
            except Exception as exc:
                error = self._as_backend_error("close", exc)
                errors.append(error.code.value)
                if self._last_error is None:
                    self._last_error = error.code.value
            self._connected = False
            self._mode = CoreMode.CLOSED
            self._lease = None
            self._armed_generation = None
            self._cleanup_result = CleanupResult(
                True, attempt.confirmed, tuple(errors)
            )
            self._record_event(
                "shutdown",
                "OK" if not errors else "ERROR",
                reason=cause.value,
                stop_confirmed=attempt.confirmed,
                errors=",".join(errors),
            )
            return self._cleanup_result

    def _call_command_locked(
        self, operation: str, call: Callable[[], CommandResult]
    ) -> CommandResult:
        try:
            result = call()
            self._validate_command_result_locked(operation, result)
        except Exception as exc:
            primary = self._as_backend_error(operation, exc)
            self._attempt_safe_stop_locked(
                SafeStopReason.BACKEND_FAILURE, clear_lease=True
            )
            self._last_error = primary.code.value
            self._record_event(operation, "ERROR", error=primary.code.value)
            raise primary from exc
        if result.status is CommandStatus.REJECTED:
            primary = CommandRejectedError(
                operation, result.message or f"{operation} was rejected"
            )
            self._attempt_safe_stop_locked(
                SafeStopReason.BACKEND_FAILURE, clear_lease=True
            )
            self._last_error = primary.code.value
            self._record_event(operation, "REJECTED")
            raise primary
        return result

    def _attempt_safe_stop_locked(
        self, reason: SafeStopReason, *, clear_lease: bool
    ) -> SafetyAttempt:
        self._stop_attempted = True
        self._stop_confirmed = False
        self._last_safety_reason = reason
        if clear_lease:
            self._lease = None
        self._armed_generation = None
        try:
            result = self._link.safe_stop(reason)
            self._validate_command_result_locked("safe_stop", result)
        except Exception as exc:
            error = self._as_backend_error("safe_stop", exc)
            self._last_error = error.code.value
            if error.code is CoreErrorCode.BACKEND_DISCONNECTED:
                self._connected = False
                self._mode = CoreMode.DISCONNECTED
            else:
                self._mode = CoreMode.FAULT
            self._record_event(
                "safe_stop",
                "ERROR",
                reason=reason.value,
                error=error.code.value,
                confirmed=False,
            )
            return SafetyAttempt(reason, None, error)
        if result.status is CommandStatus.REJECTED:
            error = CommandRejectedError(
                "safe_stop", result.message or "safe-stop was rejected"
            )
            self._mode = CoreMode.FAULT
            self._last_error = error.code.value
            self._record_event(
                "safe_stop", "REJECTED", reason=reason.value, confirmed=False
            )
            return SafetyAttempt(reason, result, error)
        self._stop_confirmed = result.confirmed
        if result.confirmed:
            self._commanded_linear = 0
            self._commanded_angular = 0
            self._mode = CoreMode.SAFE_STOPPED
        else:
            self._mode = CoreMode.FAULT
            self._last_error = "SAFE_STOP_UNCONFIRMED"
        self._record_event(
            "safe_stop",
            result.status.value,
            reason=reason.value,
            confirmed=result.confirmed,
        )
        return SafetyAttempt(reason, result, None)

    def _expire_lease_locked(self) -> bool:
        if self._lease is None or self._clock() < self._lease.deadline_s:
            return False
        expired = self._lease.token
        self._attempt_safe_stop_locked(
            SafeStopReason.COMMAND_TIMEOUT, clear_lease=True
        )
        self._record_event(
            "lease_expired",
            "SAFE_STOP",
            owner=expired.owner,
            generation=expired.generation,
        )
        return True

    def _require_lease_locked(self, token: LeaseToken) -> _LeaseRecord:
        expired = self._expire_lease_locked()
        if expired:
            raise InvalidLeaseError(
                CoreErrorCode.LEASE_EXPIRED,
                "control lease has expired and caused a safe-stop",
            )
        if not isinstance(token, LeaseToken):
            raise InvalidLeaseError(
                CoreErrorCode.INVALID_LEASE, "a LeaseToken is required"
            )
        if self._lease is None or token != self._lease.token:
            raise InvalidLeaseError(
                CoreErrorCode.INVALID_LEASE,
                "token is not the current control lease",
            )
        return self._lease

    def _validate_chassis_locked(
        self,
        linear: object,
        angular: object,
        ttl_ms: object,
        record: _LeaseRecord,
    ) -> ChassisSetpoint:
        for name, value in (
            ("linear_velocity_mm_s", linear),
            ("angular_velocity_mrad_s", angular),
            ("ttl_ms", ttl_ms),
        ):
            if type(value) is not int:
                raise InvalidControlInput(
                    CoreErrorCode.INVALID_INPUT, f"{name} must be an integer"
                )
        assert isinstance(linear, int)
        assert isinstance(angular, int)
        assert isinstance(ttl_ms, int)
        if abs(linear) > self._config.max_abs_linear_velocity_mm_s:
            raise InvalidControlInput(
                CoreErrorCode.INVALID_INPUT, "linear velocity exceeds core limit"
            )
        if abs(angular) > self._config.max_abs_angular_velocity_mrad_s:
            raise InvalidControlInput(
                CoreErrorCode.INVALID_INPUT, "angular velocity exceeds core limit"
            )
        if not self._config.min_ttl_ms <= ttl_ms <= self._config.max_ttl_ms:
            raise InvalidControlInput(
                CoreErrorCode.INVALID_INPUT, "TTL exceeds core limits"
            )
        if ttl_ms > record.duration_s * 1_000:
            raise InvalidControlInput(
                CoreErrorCode.INVALID_INPUT,
                "motion TTL cannot outlive the renewed control lease",
            )
        return ChassisSetpoint(linear, angular, ttl_ms)

    def _reconcile_arm_state_locked(self, health: HealthSnapshot) -> None:
        state = health.arm_state.value
        if state is ArmState.ARMED:
            self._mode = CoreMode.ARMED
        elif state is ArmState.DISARMED:
            self._mode = CoreMode.DISARMED
            self._armed_generation = None
            self._commanded_linear = 0
            self._commanded_angular = 0
        elif state is ArmState.SAFE_STOP:
            self._mode = CoreMode.SAFE_STOPPED
            self._lease = None
            self._armed_generation = None
            self._commanded_linear = 0
            self._commanded_angular = 0

    def _validate_link_snapshot_locked(self, snapshot: object) -> None:
        if not isinstance(snapshot, LinkSnapshot):
            raise TypeError("backend returned a non-LinkSnapshot")
        if type(snapshot.connected) is not bool:
            raise TypeError("LinkSnapshot.connected must be bool")
        if (
            isinstance(snapshot.observed_at, bool)
            or not isinstance(snapshot.observed_at, (int, float))
            or not math.isfinite(float(snapshot.observed_at))
        ):
            raise TypeError("LinkSnapshot.observed_at must be finite")

    def _validate_health_snapshot_locked(self, health: object) -> None:
        if not isinstance(health, HealthSnapshot):
            raise TypeError("backend returned a non-HealthSnapshot")
        if type(health.connected) is not bool:
            raise TypeError("HealthSnapshot.connected must be bool")
        self._validate_finite_time_locked(
            "HealthSnapshot.observed_at", health.observed_at, required=True
        )
        self._validate_state_field_locked(
            "health.feedback", health.feedback, bool, strict_type=True
        )
        self._validate_state_field_locked(
            "health.arm_state", health.arm_state, ArmState
        )
        self._validate_state_field_locked(
            "health.fault_bits", health.fault_bits, int, strict_type=True
        )

    def _health_safety_issue_locked(
        self, health: HealthSnapshot
    ) -> SafeStopReason | None:
        if health.feedback.validity is FieldValidity.STALE:
            return SafeStopReason.FEEDBACK_STALE
        if (
            health.feedback.validity is not FieldValidity.VALID
            or health.feedback.value is not True
        ):
            return SafeStopReason.FEEDBACK_INVALID
        if (
            health.arm_state.validity is not FieldValidity.VALID
            or not isinstance(health.arm_state.value, ArmState)
            or health.arm_state.value is ArmState.FAULT
        ):
            return SafeStopReason.FEEDBACK_INVALID
        if (
            health.fault_bits.validity is not FieldValidity.VALID
            or type(health.fault_bits.value) is not int
            or health.fault_bits.value != 0
        ):
            return SafeStopReason.FEEDBACK_INVALID
        return None

    def _validate_robot_state_snapshot_locked(self, state: object) -> None:
        if not isinstance(state, RobotStateSnapshot):
            raise TypeError("backend returned a non-RobotStateSnapshot")
        self._validate_finite_time_locked(
            "RobotStateSnapshot.observed_at", state.observed_at, required=True
        )
        self._validate_state_field_locked(
            "linear_velocity_mm_s",
            state.linear_velocity_mm_s,
            int,
            strict_type=True,
        )
        self._validate_state_field_locked(
            "angular_velocity_mrad_s",
            state.angular_velocity_mrad_s,
            int,
            strict_type=True,
        )
        self._validate_state_field_locked(
            "gripper_state", state.gripper_state, GripperState
        )
        self._validate_state_field_locked(
            "arm_state", state.arm_state, ArmState
        )

    def _validate_state_field_locked(
        self,
        name: str,
        field: object,
        expected_type: type[object],
        *,
        strict_type: bool = False,
    ) -> None:
        if not isinstance(field, StateField):
            raise TypeError(f"{name} must be StateField")
        if not isinstance(field.validity, FieldValidity):
            raise TypeError(f"{name}.validity must be FieldValidity")
        self._validate_finite_time_locked(
            f"{name}.observed_at",
            field.observed_at,
            required=field.validity is FieldValidity.VALID,
        )
        if field.value is None:
            if field.validity is FieldValidity.VALID:
                raise TypeError(f"valid {name}.value cannot be None")
            return
        if strict_type:
            valid_value = type(field.value) is expected_type
        else:
            valid_value = isinstance(field.value, expected_type)
        if not valid_value:
            raise TypeError(
                f"{name}.value must be {expected_type.__name__} when present"
            )

    def _validate_finite_time_locked(
        self, name: str, value: object, *, required: bool
    ) -> None:
        if value is None and not required:
            return
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            raise TypeError(f"{name} must be finite")

    def _robot_state_confirms_safe_motion_locked(
        self,
        state: RobotStateSnapshot,
        *,
        expected_arm: object,
    ) -> bool:
        return (
            state.linear_velocity_mm_s.validity is FieldValidity.VALID
            and type(state.linear_velocity_mm_s.value) is int
            and state.linear_velocity_mm_s.value == 0
            and state.angular_velocity_mrad_s.validity is FieldValidity.VALID
            and type(state.angular_velocity_mrad_s.value) is int
            and state.angular_velocity_mrad_s.value == 0
            and state.arm_state.validity is FieldValidity.VALID
            and state.arm_state.value is expected_arm
        )

    def _validate_command_result_locked(
        self, operation: str, result: object
    ) -> None:
        if not isinstance(result, CommandResult):
            raise TypeError("backend returned a non-CommandResult")
        if not isinstance(result.status, CommandStatus):
            raise TypeError("CommandResult.status must be CommandStatus")
        if result.operation != operation:
            raise TypeError(
                f"CommandResult.operation {result.operation!r} does not match {operation!r}"
            )

    def _raise_after_unconfirmed_critical_locked(self, operation: str) -> None:
        attempt = self._attempt_safe_stop_locked(
            SafeStopReason.BACKEND_FAILURE, clear_lease=True
        )
        if attempt.error is not None:
            raise attempt.error
        raise InvalidCoreState(
            CoreErrorCode.INVALID_STATE,
            f"{operation} was not confirmed; safe-stop applied",
        )

    def _ensure_lease_fresh_after_backend_locked(
        self, record: _LeaseRecord
    ) -> None:
        if self._lease is not record:
            raise InvalidLeaseError(
                CoreErrorCode.INVALID_LEASE,
                "control lease changed during backend command",
            )
        if self._clock() >= record.deadline_s:
            self._expire_lease_locked()
            raise InvalidLeaseError(
                CoreErrorCode.LEASE_EXPIRED,
                "control lease expired during backend command",
            )

    def _renew_after_success_locked(self, record: _LeaseRecord) -> None:
        if self._lease is record:
            record.deadline_s = self._clock() + record.duration_s

    def _require_connected_locked(self) -> None:
        if not self._connected:
            raise InvalidCoreState(
                CoreErrorCode.NOT_CONNECTED, "lower link is not connected"
            )

    def _require_operable_locked(self) -> None:
        if self._mode in (CoreMode.FAULT, CoreMode.CLOSED):
            raise InvalidCoreState(
                CoreErrorCode.INVALID_STATE,
                f"core mode {self._mode.value} does not accept control",
            )

    def _require_armed_locked(self, token: LeaseToken) -> None:
        if self._mode is not CoreMode.ARMED:
            raise InvalidCoreState(
                CoreErrorCode.INVALID_STATE,
                f"control requires ARMED, got {self._mode.value}",
            )
        if self._armed_generation != token.generation:
            raise InvalidCoreState(
                CoreErrorCode.INVALID_STATE,
                "current lease generation has not completed ARM",
            )

    def _validate_owner(self, owner: str) -> str:
        if not isinstance(owner, str) or not owner.strip():
            raise InvalidControlInput(
                CoreErrorCode.INVALID_INPUT, "lease owner must be non-empty"
            )
        normalized = owner.strip()
        if len(normalized) > 128:
            raise InvalidControlInput(
                CoreErrorCode.INVALID_INPUT, "lease owner is too long"
            )
        return normalized

    def _validate_duration(self, duration_s: float | None) -> float:
        duration = (
            self._config.default_lease_duration_s
            if duration_s is None
            else duration_s
        )
        if not isinstance(duration, (int, float)) or isinstance(duration, bool):
            raise InvalidControlInput(
                CoreErrorCode.INVALID_INPUT, "lease duration must be numeric"
            )
        duration = float(duration)
        if (
            not math.isfinite(duration)
            or not 0 < duration <= self._config.max_lease_duration_s
        ):
            raise InvalidControlInput(
                CoreErrorCode.INVALID_INPUT,
                "lease duration is outside configured limits",
            )
        return duration

    def _as_backend_error(
        self, operation: str, exc: Exception
    ) -> CoreBackendError:
        if isinstance(exc, LowerLinkError):
            return CoreBackendError(exc)
        return CoreBackendError(
            LowerLinkError(
                operation,
                LowerLinkErrorCode.INTERNAL,
                str(exc) or type(exc).__name__,
                delivery=DeliveryState.NOT_SENT,
            )
        )

    def _snapshot_locked(self) -> CoreSnapshot:
        now = self._clock()
        lease = None
        if self._lease is not None:
            lease = LeaseSnapshot(
                self._lease.token.owner,
                self._lease.token.generation,
                self._lease.deadline_s,
                max(0.0, self._lease.deadline_s - now),
            )
        return CoreSnapshot(
            now,
            self._mode,
            self._connected,
            lease,
            self._armed_generation,
            self._commanded_linear,
            self._commanded_angular,
            self._commanded_gripper,
            self._last_safety_reason,
            self._stop_attempted,
            self._stop_confirmed,
            self._last_error,
        )

    def _record_event(self, event: str, outcome: str, **details: object) -> None:
        self._event_sequence += 1
        self._events.append(
            CoreEvent(
                self._event_sequence,
                self._clock(),
                event,
                outcome,
                tuple(sorted(details.items())),
            )
        )
