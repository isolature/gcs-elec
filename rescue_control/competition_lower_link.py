"""Business-semantic adapter for the formal RescueCar runtime client."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable

import rescue_car_client as runtime
import rescue_car_protocol as protocol

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


@dataclass(frozen=True)
class CompetitionLinkConfig:
    connect_timeout_s: float = 3.0
    initial_feedback_timeout_s: float = 1.0
    confirmation_timeout_s: float = 0.25
    feedback_stale_after_s: float = 0.5
    # Idle serial-read slice of the runtime client; bounds how long a queued
    # command can wait behind an idle read before it is dispatched.  Only
    # applied when this adapter constructs the client itself.
    io_slice_s: float = 0.005

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TypeError(f"{name} must be a finite positive number")
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a finite positive number")


class CompetitionLowerLink:
    """Translate the formal protocol runtime into the stable LowerLink contract.

    RescueCarClient remains the only owner of serial I/O, framing, HELLO,
    heartbeat, motion refresh, receive handling and reconnect.  This adapter
    owns only business completion, freshness and authority semantics.
    """

    def __init__(
        self,
        port: str | None = None,
        *,
        client: object | None = None,
        serial_factory: Callable[..., object] | None = None,
        reconnect: bool = True,
        clock: Callable[[], float] = time.monotonic,
        config: CompetitionLinkConfig | None = None,
    ) -> None:
        if client is not None and (port is not None or serial_factory is not None):
            raise ValueError("an injected client cannot be combined with port options")
        self._config = config or CompetitionLinkConfig()
        self._client = (
            client
            if client is not None
            else runtime.RescueCarClient(
                port=port,
                serial_factory=serial_factory,
                reconnect=reconnect,
                io_slice_s=self._config.io_slice_s,
            )
        )
        self._clock = clock
        self._authority: tuple[int, int, int] | None = None
        self._last_safe_stop_reason: SafeStopReason | None = None

    @property
    def last_safe_stop_reason(self) -> SafeStopReason | None:
        return self._last_safe_stop_reason

    def connect(self) -> LinkSnapshot:
        try:
            self._client.connect(timeout=self._config.connect_timeout_s)
            initial = self._client.snapshot()
            authority = self._identity(initial)
            if authority is None:
                raise runtime.NotConnectedError("HELLO session identity is incomplete")
            self._authority = authority
            snapshot = self._client.wait_for_snapshot(
                lambda value: (
                    not self._same_authority(value)
                    or self._initial_feedback_is_valid(value)
                ),
                self._config.initial_feedback_timeout_s,
            )
            if not self._same_authority(snapshot):
                raise runtime.NotConnectedError(
                    "connection changed before initial state became valid"
                )
            if not self._initial_feedback_is_valid(snapshot):
                raise runtime.RequestTimeoutError("initial_feedback")
        except Exception as exc:
            if self._authority is not None:
                self._last_safe_stop_reason = SafeStopReason.FEEDBACK_INVALID
                try:
                    self._client.safe_stop()
                except Exception:
                    pass
            try:
                self._client.close(stop=False)
            except Exception:
                pass
            self._authority = None
            raise self._map_error("connect", exc, command=False) from exc
        now = self._clock()
        return LinkSnapshot(True, now, self._identity_detail(snapshot))

    def close(self) -> None:
        try:
            # ControlCore performs the one deliberate safe-stop attempt.  Do not
            # let RescueCarClient add a second, reason-less attempt on close.
            self._client.close(stop=False)
        except Exception as exc:
            raise self._map_error("close", exc, command=False) from exc
        finally:
            self._authority = None

    def arm(self) -> CommandResult:
        before = self._require_authority("arm")
        result = self._discrete_call("arm", self._client.arm)
        if result.status is CommandStatus.REJECTED:
            return result
        return self._confirm(
            result,
            before,
            lambda snapshot: self._safety_confirms(snapshot, ArmState.ARMED),
            require_safety=True,
        )

    def disarm(self) -> CommandResult:
        before = self._require_authority("disarm")
        result = self._discrete_call("disarm", self._client.disarm)
        if result.status is CommandStatus.REJECTED:
            return result
        return self._confirm(
            result,
            before,
            lambda snapshot: (
                self._safety_confirms(snapshot, ArmState.DISARMED)
                and self._robot_confirms_zero(snapshot)
            ),
            require_safety=True,
            require_robot=True,
        )

    def set_chassis(self, target: ChassisSetpoint) -> CommandResult:
        self._require_authority("set_chassis")
        try:
            self._client.set_velocity(
                target.linear_velocity_mm_s,
                target.angular_velocity_mrad_s,
                ttl_ms=target.ttl_ms,
            )
        except Exception as exc:
            raise self._map_error("set_chassis", exc, command=True) from exc
        self._ensure_authority_after_command("set_chassis")
        return CommandResult(CommandStatus.SENT, "set_chassis")

    def stop(self) -> CommandResult:
        before = self._require_authority("stop")
        try:
            self._client.stop()
        except Exception as exc:
            raise self._map_error("stop", exc, command=True) from exc
        sent = CommandResult(CommandStatus.SENT, "stop")
        return self._confirm(
            sent, before, self._robot_confirms_zero, require_robot=True
        )

    def safe_stop(self, reason: SafeStopReason) -> CommandResult:
        if not isinstance(reason, SafeStopReason):
            raise TypeError("reason must be SafeStopReason")
        self._last_safe_stop_reason = reason
        before = self._require_authority("safe_stop")
        result = self._discrete_call("safe_stop", self._client.safe_stop)
        result = CommandResult(
            result.status,
            result.operation,
            f"reason={reason.value}",
            result.command_id,
        )
        if result.status is CommandStatus.REJECTED:
            return result
        return self._confirm(
            result,
            before,
            lambda snapshot: (
                self._safety_confirms(snapshot, ArmState.SAFE_STOP)
                and self._robot_confirms_zero(snapshot)
            ),
            require_safety=True,
            require_robot=True,
        )

    def set_gripper(self, target: GripperTarget) -> CommandResult:
        if not isinstance(target, GripperTarget):
            raise TypeError("target must be GripperTarget")
        before = self._require_authority("set_gripper")
        call = (
            self._client.open_gripper
            if target is GripperTarget.OPEN
            else self._client.close_gripper
        )
        result = self._discrete_call("set_gripper", call)
        if result.status is CommandStatus.REJECTED:
            return result
        expected = (
            GripperState.OPEN
            if target is GripperTarget.OPEN
            else GripperState.CLOSED
        )
        return self._confirm(
            result,
            before,
            lambda snapshot: self._gripper_confirms(snapshot, expected),
            require_robot=True,
        )

    def get_health(self) -> HealthSnapshot:
        now = self._clock()
        snapshot = self._client.snapshot()
        if not self._same_authority(snapshot):
            unavailable = StateField.unknown("connection authority changed")
            return HealthSnapshot(False, unavailable, unavailable, unavailable, now)
        status = snapshot.safety_status
        received_at = snapshot.safety_status_received_at
        if status is None or received_at <= 0:
            unavailable = StateField.unknown("no SAFETY_STATUS received")
            return HealthSnapshot(True, unavailable, unavailable, unavailable, now)
        validity_reported = (
            isinstance(status.validity_flags, int) and status.validity_flags > 0
        )
        if not validity_reported:
            arm_field = StateField(
                FieldValidity.UNKNOWN,
                observed_at=received_at,
                reason="safety validity is not reported",
            )
            feedback = StateField(
                FieldValidity.UNKNOWN,
                observed_at=received_at,
                reason="safety validity is not reported",
            )
            faults = StateField(
                FieldValidity.UNKNOWN,
                observed_at=received_at,
                reason="fault validity is not reported",
            )
        else:
            arm = self._arm_state(status)
            if arm is None:
                arm_field = StateField(
                    FieldValidity.INVALID,
                    observed_at=received_at,
                    reason="invalid or inconsistent safety state",
                )
            else:
                arm_field = StateField.valid(arm, received_at)
            feedback = StateField.valid(bool(status.link_ready), received_at)
            faults = StateField.valid(status.fault_bits, received_at)
        if self._is_stale(received_at, now):
            reason = "SAFETY_STATUS is stale"
            arm_field = arm_field.with_validity(FieldValidity.STALE, reason=reason)
            feedback = feedback.with_validity(FieldValidity.STALE, reason=reason)
            faults = faults.with_validity(FieldValidity.STALE, reason=reason)
        return HealthSnapshot(True, arm_field, feedback, faults, now)

    def get_robot_state(self) -> RobotStateSnapshot:
        now = self._clock()
        snapshot = self._client.snapshot()
        if not self._same_authority(snapshot):
            unavailable = StateField.unknown("connection authority changed")
            return RobotStateSnapshot(
                unavailable, unavailable, unavailable, unavailable, now
            )
        state = snapshot.robot_state
        received_at = snapshot.robot_state_received_at
        if state is None or received_at <= 0:
            unavailable = StateField.unknown("no ROBOT_STATE received")
            return RobotStateSnapshot(
                unavailable,
                unavailable,
                unavailable,
                self.get_health().arm_state,
                now,
            )
        if isinstance(state.validity_flags, int) and state.validity_flags > 0:
            linear = StateField.valid(state.linear_mm_s, received_at)
            angular = StateField.valid(state.angular_mrad_s, received_at)
            gripper = self._gripper_field(state.gripper_state, received_at)
        else:
            linear = StateField(
                FieldValidity.UNKNOWN,
                observed_at=received_at,
                reason="velocity validity is not reported",
            )
            angular = StateField(
                FieldValidity.UNKNOWN,
                observed_at=received_at,
                reason="velocity validity is not reported",
            )
            gripper = StateField(
                FieldValidity.UNKNOWN,
                observed_at=received_at,
                reason="gripper validity is not reported",
            )
        if self._is_stale(received_at, now):
            reason = "ROBOT_STATE is stale"
            linear = linear.with_validity(FieldValidity.STALE, reason=reason)
            angular = angular.with_validity(FieldValidity.STALE, reason=reason)
            gripper = gripper.with_validity(FieldValidity.STALE, reason=reason)
        return RobotStateSnapshot(
            linear, angular, gripper, self.get_health().arm_state, now
        )

    def _discrete_call(self, operation: str, call: Callable[[], object]) -> CommandResult:
        try:
            raw = call()
        except runtime.CommandRejectedError as exc:
            return CommandResult(
                CommandStatus.REJECTED,
                operation,
                str(exc),
                str(exc.result.command_id),
            )
        except Exception as exc:
            raise self._map_error(operation, exc, command=True) from exc
        self._ensure_authority_after_command(operation)
        status = (
            CommandStatus.COMPLETED
            if raw.status == protocol.COMMAND_RESULT_COMPLETED
            else CommandStatus.ACCEPTED
        )
        return CommandResult(status, operation, command_id=str(raw.command_id))

    def _confirm(
        self,
        result: CommandResult,
        before: object,
        predicate: Callable[[object], bool],
        *,
        require_safety: bool = False,
        require_robot: bool = False,
    ) -> CommandResult:
        operation = result.operation
        def confirmed(value: object) -> bool:
            return (
                (not require_safety or self._safety_is_newer(value, before))
                and (not require_robot or self._robot_is_newer(value, before))
                and predicate(value)
            )

        snapshot = self._client.wait_for_snapshot(
            lambda value: not self._same_authority(value)
            or confirmed(value),
            self._config.confirmation_timeout_s,
        )
        if not self._same_authority(snapshot):
            raise LowerLinkError(
                operation,
                LowerLinkErrorCode.DISCONNECTED,
                "connection, session or boot changed while confirming command",
                delivery=DeliveryState.MAY_HAVE_APPLIED,
                retryable=False,
            )
        if confirmed(snapshot):
            return CommandResult(
                CommandStatus.COMPLETED,
                operation,
                result.message,
                result.command_id,
            )
        return CommandResult(
            CommandStatus.ACCEPTED
            if result.status is not CommandStatus.SENT
            else CommandStatus.SENT,
            operation,
            result.message or "fresh completion feedback not observed",
            result.command_id,
        )

    def _require_authority(self, operation: str) -> object:
        snapshot = self._client.snapshot()
        if not self._same_authority(snapshot):
            raise LowerLinkError(
                operation,
                LowerLinkErrorCode.DISCONNECTED,
                "connection authority is absent or changed; reconnect and ARM again",
                delivery=DeliveryState.NOT_SENT,
                retryable=False,
            )
        return snapshot

    def _ensure_authority_after_command(self, operation: str) -> None:
        if not self._same_authority(self._client.snapshot()):
            raise LowerLinkError(
                operation,
                LowerLinkErrorCode.DISCONNECTED,
                "connection authority changed during command",
                delivery=DeliveryState.MAY_HAVE_APPLIED,
                retryable=False,
            )

    def _map_error(
        self, operation: str, exc: Exception, *, command: bool
    ) -> LowerLinkError:
        delivery = DeliveryState.MAY_HAVE_APPLIED if command else DeliveryState.NOT_SENT
        if isinstance(exc, runtime.RequestTimeoutError):
            return LowerLinkError(
                operation,
                LowerLinkErrorCode.TIMEOUT,
                str(exc),
                delivery=DeliveryState.MAY_HAVE_APPLIED,
                retryable=False,
            )
        if isinstance(exc, runtime.NotConnectedError):
            return LowerLinkError(
                operation,
                LowerLinkErrorCode.DISCONNECTED,
                str(exc),
                delivery=delivery,
                retryable=not command,
            )
        if isinstance(exc, runtime.NotArmedError):
            return LowerLinkError(
                operation,
                LowerLinkErrorCode.PROTOCOL,
                str(exc),
                delivery=DeliveryState.NOT_SENT,
                retryable=False,
            )
        if isinstance(exc, protocol.ProtocolError):
            code = LowerLinkErrorCode.PROTOCOL
        elif isinstance(exc, OSError):
            code = LowerLinkErrorCode.IO
        elif isinstance(exc, runtime.RescueCarError) and (
            "no result" in str(exc).lower() or "timed out" in str(exc).lower()
        ):
            code = LowerLinkErrorCode.TIMEOUT
        else:
            code = LowerLinkErrorCode.INTERNAL
        return LowerLinkError(
            operation,
            code,
            str(exc),
            delivery=delivery,
            retryable=False,
        )

    def _initial_feedback_is_valid(self, snapshot: object) -> bool:
        status = snapshot.safety_status
        state = snapshot.robot_state
        return bool(
            self._same_authority(snapshot)
            and status is not None
            and state is not None
            and snapshot.safety_status_received_at >= snapshot.connected_at
            and snapshot.robot_state_received_at >= snapshot.connected_at
            and status.link_ready
            and self._arm_state(status) is not None
            and isinstance(status.validity_flags, int)
            and status.validity_flags > 0
            and state.state_seq > 0
            and state.timestamp_us > 0
            and state.coordinate_frame == protocol.COORDINATE_FRAME_BASE_LINK
            and isinstance(state.validity_flags, int)
            and state.validity_flags > 0
            and not self._is_stale(
                snapshot.safety_status_received_at, self._clock()
            )
            and not self._is_stale(
                snapshot.robot_state_received_at, self._clock()
            )
        )

    def _safety_confirms(self, snapshot: object, expected: ArmState) -> bool:
        received_at = snapshot.safety_status_received_at
        return bool(
            snapshot.safety_status is not None
            and not self._is_stale(received_at, self._clock())
            and snapshot.safety_status.link_ready
            and isinstance(snapshot.safety_status.validity_flags, int)
            and snapshot.safety_status.validity_flags > 0
            and self._arm_state(snapshot.safety_status) is expected
        )

    def _robot_confirms_zero(self, snapshot: object) -> bool:
        state = snapshot.robot_state
        received_at = snapshot.robot_state_received_at
        return bool(
            state is not None
            and not self._is_stale(received_at, self._clock())
            and state.validity_flags > 0
            and state.linear_mm_s == 0
            and state.angular_mrad_s == 0
        )

    def _gripper_confirms(self, snapshot: object, expected: GripperState) -> bool:
        state = snapshot.robot_state
        received_at = snapshot.robot_state_received_at
        if (
            state is None
            or self._is_stale(received_at, self._clock())
            or state.validity_flags <= 0
        ):
            return False
        field = self._gripper_field(state.gripper_state, received_at)
        return field.validity is FieldValidity.VALID and field.value is expected

    @staticmethod
    def _safety_is_newer(snapshot: object, before: object) -> bool:
        return snapshot.safety_status_received_at > before.safety_status_received_at

    @staticmethod
    def _robot_is_newer(snapshot: object, before: object) -> bool:
        return snapshot.robot_state_received_at > before.robot_state_received_at

    @staticmethod
    def _identity(snapshot: object) -> tuple[int, int, int] | None:
        if not snapshot.connected:
            return None
        values = (
            snapshot.connection_generation,
            snapshot.session_id,
            snapshot.boot_id,
        )
        if not all(isinstance(value, int) and value > 0 for value in values):
            return None
        return values

    def _same_authority(self, snapshot: object) -> bool:
        return self._authority is not None and self._identity(snapshot) == self._authority

    @staticmethod
    def _identity_detail(snapshot: object) -> str:
        return (
            f"port={snapshot.port} generation={snapshot.connection_generation} "
            f"session={snapshot.session_id} boot={snapshot.boot_id}"
        )

    @staticmethod
    def _arm_state(status: object) -> ArmState | None:
        mapping = {
            protocol.SAFETY_STATE_DISARMED: ArmState.DISARMED,
            protocol.SAFETY_STATE_ARMED: ArmState.ARMED,
            protocol.SAFETY_STATE_SAFE_STOP: ArmState.SAFE_STOP,
            protocol.SAFETY_STATE_FAULT: ArmState.FAULT,
        }
        state = mapping.get(status.safety_state)
        if state is None:
            return None
        if bool(status.armed) is (state is ArmState.ARMED):
            return state
        return None

    @staticmethod
    def _gripper_field(value: int, observed_at: float) -> StateField[GripperState]:
        mapping = {
            protocol.GRIPPER_STATE_OPEN: GripperState.OPEN,
            protocol.GRIPPER_STATE_CLOSED: GripperState.CLOSED,
            protocol.GRIPPER_STATE_OPENING: GripperState.OPENING,
            protocol.GRIPPER_STATE_CLOSING: GripperState.CLOSING,
            protocol.GRIPPER_STATE_FAULT: GripperState.FAULT,
        }
        if value == protocol.GRIPPER_STATE_UNKNOWN:
            return StateField(
                FieldValidity.UNKNOWN,
                observed_at=observed_at,
                reason="gripper state is UNKNOWN",
            )
        if value not in mapping:
            return StateField(
                FieldValidity.INVALID,
                observed_at=observed_at,
                reason=f"unsupported gripper state {value}",
            )
        return StateField.valid(mapping[value], observed_at)

    def _is_stale(self, observed_at: float, now: float) -> bool:
        return (
            not isinstance(observed_at, (int, float))
            or not math.isfinite(observed_at)
            or observed_at <= 0
            or now < observed_at
            or now - observed_at > self._config.feedback_stale_after_s
        )
