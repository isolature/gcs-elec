"""Business interface between ControlCore and a lower-controller adapter."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from .models import (
    ChassisSetpoint,
    CommandResult,
    GripperTarget,
    HealthSnapshot,
    LinkSnapshot,
    RobotStateSnapshot,
    SafeStopReason,
)


@runtime_checkable
class LowerLink(Protocol):
    """Stable semantics; framing, serial ports and schema details stay in adapters."""

    def connect(self) -> LinkSnapshot: ...

    def close(self) -> None: ...

    def arm(self) -> CommandResult: ...

    def disarm(self) -> CommandResult: ...

    def set_chassis(self, target: ChassisSetpoint) -> CommandResult: ...

    def stop(self) -> CommandResult: ...

    def safe_stop(self, reason: SafeStopReason) -> CommandResult: ...

    def set_gripper(self, target: GripperTarget) -> CommandResult: ...

    def get_health(self) -> HealthSnapshot: ...

    def get_robot_state(self) -> RobotStateSnapshot: ...
