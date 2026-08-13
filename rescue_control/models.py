"""Stable business-domain values shared by the rescue control boundary."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Generic, TypeVar


class CommandStatus(str, Enum):
    """How much of a command's business effect has been confirmed."""

    SENT = "SENT"
    ACCEPTED = "ACCEPTED"
    COMPLETED = "COMPLETED"
    REJECTED = "REJECTED"


class FieldValidity(str, Enum):
    VALID = "VALID"
    UNKNOWN = "UNKNOWN"
    STALE = "STALE"
    INVALID = "INVALID"


class ArmState(str, Enum):
    DISARMED = "DISARMED"
    ARMED = "ARMED"
    SAFE_STOP = "SAFE_STOP"
    FAULT = "FAULT"


class GripperTarget(str, Enum):
    OPEN = "OPEN"
    CLOSED = "CLOSED"


class GripperState(str, Enum):
    OPEN = "OPEN"
    CLOSED = "CLOSED"
    OPENING = "OPENING"
    CLOSING = "CLOSING"
    FAULT = "FAULT"


class SafeStopReason(str, Enum):
    USER_REQUEST = "USER_REQUEST"
    COMMAND_TIMEOUT = "COMMAND_TIMEOUT"
    LINK_DISCONNECTED = "LINK_DISCONNECTED"
    FEEDBACK_STALE = "FEEDBACK_STALE"
    FEEDBACK_INVALID = "FEEDBACK_INVALID"
    INVALID_COMMAND = "INVALID_COMMAND"
    BACKEND_FAILURE = "BACKEND_FAILURE"
    EOF = "EOF"
    KEYBOARD_INTERRUPT = "KEYBOARD_INTERRUPT"
    UNHANDLED_EXCEPTION = "UNHANDLED_EXCEPTION"
    SHUTDOWN = "SHUTDOWN"


T = TypeVar("T")


@dataclass(frozen=True)
class StateField(Generic[T]):
    """A reported value plus an explicit statement of whether it is usable."""

    validity: FieldValidity
    value: T | None = None
    observed_at: float | None = None
    reason: str | None = None

    @classmethod
    def valid(cls, value: T, observed_at: float) -> "StateField[T]":
        return cls(FieldValidity.VALID, value, observed_at)

    @classmethod
    def unknown(cls, reason: str) -> "StateField[T]":
        return cls(FieldValidity.UNKNOWN, reason=reason)

    def with_validity(
        self, validity: FieldValidity, *, reason: str | None = None
    ) -> "StateField[T]":
        return StateField(validity, self.value, self.observed_at, reason)


@dataclass(frozen=True)
class ChassisSetpoint:
    """A ControlCore-validated motion target in stable business units."""

    linear_velocity_mm_s: int
    angular_velocity_mrad_s: int
    ttl_ms: int


@dataclass(frozen=True)
class CommandResult:
    status: CommandStatus
    operation: str
    message: str = ""
    command_id: str | None = None

    @property
    def confirmed(self) -> bool:
        return self.status is CommandStatus.COMPLETED


@dataclass(frozen=True)
class LinkSnapshot:
    connected: bool
    observed_at: float
    detail: str = ""


@dataclass(frozen=True)
class HealthSnapshot:
    connected: bool
    arm_state: StateField[ArmState]
    feedback: StateField[bool]
    fault_bits: StateField[int]
    observed_at: float


@dataclass(frozen=True)
class RobotStateSnapshot:
    linear_velocity_mm_s: StateField[int]
    angular_velocity_mrad_s: StateField[int]
    gripper_state: StateField[GripperState]
    arm_state: StateField[ArmState]
    observed_at: float


class LowerLinkErrorCode(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    TIMEOUT = "TIMEOUT"
    IO = "IO"
    PROTOCOL = "PROTOCOL"
    INTERNAL = "INTERNAL"


class DeliveryState(str, Enum):
    NOT_SENT = "NOT_SENT"
    MAY_HAVE_APPLIED = "MAY_HAVE_APPLIED"
    SENT_UNCONFIRMED = "SENT_UNCONFIRMED"


class LowerLinkError(RuntimeError):
    """Transport/adapter failure; explicit command rejection is not an error."""

    def __init__(
        self,
        operation: str,
        code: LowerLinkErrorCode,
        message: str,
        *,
        delivery: DeliveryState = DeliveryState.NOT_SENT,
        retryable: bool = False,
    ) -> None:
        super().__init__(message)
        self.operation = operation
        self.code = code
        self.delivery = delivery
        self.retryable = retryable
