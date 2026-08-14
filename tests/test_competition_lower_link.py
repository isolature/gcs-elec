from __future__ import annotations

import unittest
from dataclasses import replace
from unittest.mock import patch

import rescue_car_client as runtime
import rescue_car_protocol as protocol
from rescue_control import (
    ArmState,
    ChassisSetpoint,
    CommandStatus,
    CompetitionLinkConfig,
    CompetitionLowerLink,
    ControlCore,
    CoreMode,
    DeliveryState,
    FieldValidity,
    GripperState,
    GripperTarget,
    LowerLinkError,
    LowerLinkErrorCode,
    ManualClock,
    SafeStopReason,
)


def safety(state: int = protocol.SAFETY_STATE_DISARMED, *, flags: int = 15):
    return protocol.SafetyStatus(
        True,
        state == protocol.SAFETY_STATE_ARMED,
        state,
        1,
        0,
        0,
        12_000,
        False,
        flags,
    )


def robot(
    *,
    seq: int = 1,
    linear: int = 0,
    angular: int = 0,
    gripper: int = protocol.GRIPPER_STATE_UNKNOWN,
    flags: int = 15,
):
    return protocol.RobotState(
        seq,
        seq * 1_000,
        protocol.COORDINATE_FRAME_BASE_LINK,
        0,
        linear,
        angular,
        0,
        0,
        0,
        0,
        0,
        gripper,
        flags,
        0,
        0,
        0,
    )


def connected_snapshot():
    return runtime.ClientSnapshot(
        connected=True,
        armed=False,
        port="/dev/serial/by-id/usb-STM32-test",
        session_id=11,
        boot_id=22,
        last_error="",
        safety_status=safety(),
        robot_state=robot(),
        last_heartbeat_ack_time=2.0,
        connection_generation=1,
        connected_at=1.0,
        safety_status_received_at=2.0,
        robot_state_received_at=2.0,
    )


class FakeRuntimeClient:
    def __init__(self, clock: ManualClock, snapshot=None):
        self.clock = clock
        self.current = snapshot or connected_snapshot()
        self.updates = []
        self.calls = []
        self.errors = {}
        self.statuses = {}
        self.command_id = 1

    def connect(self, timeout):
        self.calls.append(("connect", timeout))
        error = self.errors.pop("connect", None)
        if error:
            raise error
        return self

    def close(self, stop=True):
        self.calls.append(("close", stop))
        error = self.errors.pop("close", None)
        if error:
            raise error

    def snapshot(self):
        return self.current

    def wait_for_snapshot(self, predicate, timeout):
        self.calls.append(("wait_for_snapshot", timeout))
        if predicate(self.current):
            return self.current
        while self.updates:
            self.current = self.updates.pop(0)
            observed = max(
                self.current.safety_status_received_at,
                self.current.robot_state_received_at,
            )
            if observed > self.clock():
                self.clock.advance(observed - self.clock())
            if predicate(self.current):
                break
        return self.current

    def queue(self, snapshot):
        self.updates.append(snapshot)

    def arm(self):
        return self._discrete("arm")

    def disarm(self):
        return self._discrete("disarm")

    def safe_stop(self):
        return self._discrete("safe_stop")

    def open_gripper(self):
        return self._discrete("open_gripper")

    def close_gripper(self):
        return self._discrete("close_gripper")

    def set_velocity(self, linear, angular, *, ttl_ms):
        self.calls.append(("set_velocity", linear, angular, ttl_ms))
        error = self.errors.pop("set_velocity", None)
        if error:
            raise error
        return True

    def stop(self):
        self.calls.append(("stop",))
        error = self.errors.pop("stop", None)
        if error:
            raise error
        return True

    def _discrete(self, operation):
        self.calls.append((operation,))
        error = self.errors.pop(operation, None)
        if error:
            raise error
        status = self.statuses.get(operation, protocol.COMMAND_RESULT_ACCEPTED)
        result = protocol.CommandResult(self.command_id, status, 1)
        self.command_id += 1
        return result


class CompetitionLowerLinkTests(unittest.TestCase):
    def setUp(self):
        self.clock = ManualClock(2.0)
        self.client = FakeRuntimeClient(self.clock)
        self.link = CompetitionLowerLink(
            client=self.client,
            clock=self.clock,
            config=CompetitionLinkConfig(
                connect_timeout_s=0.1,
                initial_feedback_timeout_s=0.1,
                confirmation_timeout_s=0.1,
                feedback_stale_after_s=0.5,
            ),
        )

    def test_connect_requires_valid_initial_safety_and_robot_feedback(self):
        result = self.link.connect()
        self.assertTrue(result.connected)
        self.assertIn("/dev/serial/by-id/", result.detail)
        self.assertEqual(self.link.get_health().arm_state.value, ArmState.DISARMED)

        missing = replace(connected_snapshot(), robot_state=None)
        failed_client = FakeRuntimeClient(self.clock, missing)
        failed = CompetitionLowerLink(
            client=failed_client,
            clock=self.clock,
            config=CompetitionLinkConfig(
                connect_timeout_s=0.1,
                initial_feedback_timeout_s=0.1,
                confirmation_timeout_s=0.1,
                feedback_stale_after_s=0.5,
            ),
        )
        with self.assertRaises(LowerLinkError) as caught:
            failed.connect()
        self.assertEqual(caught.exception.code, LowerLinkErrorCode.TIMEOUT)
        self.assertIn(("safe_stop",), failed_client.calls)
        self.assertIn(("close", False), failed_client.calls)

        self.clock.advance(1.0)
        stale_client = FakeRuntimeClient(self.clock, connected_snapshot())
        stale = CompetitionLowerLink(
            client=stale_client,
            clock=self.clock,
            config=CompetitionLinkConfig(
                connect_timeout_s=0.1,
                initial_feedback_timeout_s=0.1,
                confirmation_timeout_s=0.1,
                feedback_stale_after_s=0.5,
            ),
        )
        with self.assertRaises(LowerLinkError):
            stale.connect()

    def test_accepted_or_protocol_completed_needs_corresponding_fresh_state(self):
        self.link.connect()
        self.client.statuses["arm"] = protocol.COMMAND_RESULT_COMPLETED
        robot_only = replace(
            self.client.current,
            robot_state=robot(seq=2),
            robot_state_received_at=3.0,
        )
        self.client.queue(robot_only)
        self.assertEqual(self.link.arm().status, CommandStatus.ACCEPTED)

        invalid = replace(
            self.client.current,
            armed=True,
            safety_status=safety(protocol.SAFETY_STATE_ARMED, flags=0),
            safety_status_received_at=3.1,
        )
        self.client.queue(invalid)
        self.assertEqual(self.link.arm().status, CommandStatus.ACCEPTED)

        armed = replace(
            self.client.current,
            armed=True,
            safety_status=safety(protocol.SAFETY_STATE_ARMED),
            safety_status_received_at=4.0,
        )
        self.client.queue(armed)
        self.assertEqual(self.link.arm().status, CommandStatus.COMPLETED)

    def test_safe_stop_is_not_deduplicated_preserves_reason_and_needs_two_streams(self):
        self.link.connect()
        self.client.current = replace(
            self.client.current,
            armed=True,
            safety_status=safety(protocol.SAFETY_STATE_ARMED),
            robot_state=robot(seq=2, linear=120),
            safety_status_received_at=3.0,
            robot_state_received_at=3.0,
        )
        self.clock.advance(1.0)
        safety_only = replace(
            self.client.current,
            armed=False,
            safety_status=safety(protocol.SAFETY_STATE_SAFE_STOP),
            safety_status_received_at=4.0,
        )
        self.client.queue(safety_only)
        first = self.link.safe_stop(SafeStopReason.COMMAND_TIMEOUT)
        self.assertEqual(first.status, CommandStatus.ACCEPTED)
        self.assertIn("COMMAND_TIMEOUT", first.message)

        completed = replace(
            self.client.current,
            safety_status=safety(protocol.SAFETY_STATE_SAFE_STOP),
            robot_state=robot(seq=3),
            safety_status_received_at=5.0,
            robot_state_received_at=5.0,
        )
        self.client.queue(completed)
        second = self.link.safe_stop(SafeStopReason.SHUTDOWN)
        self.assertEqual(second.status, CommandStatus.COMPLETED)
        self.assertEqual(self.link.last_safe_stop_reason, SafeStopReason.SHUTDOWN)
        self.assertEqual(sum(call == ("safe_stop",) for call in self.client.calls), 2)

    def test_stale_unknown_and_invalid_feedback_are_explicit(self):
        self.link.connect()
        self.clock.advance(1.0)
        self.assertEqual(self.link.get_health().arm_state.validity, FieldValidity.STALE)
        self.assertEqual(
            self.link.get_robot_state().linear_velocity_mm_s.validity,
            FieldValidity.STALE,
        )

        self.client.current = replace(
            self.client.current,
            safety_status=safety(flags=0),
            robot_state=robot(seq=2, flags=0),
            safety_status_received_at=3.0,
            robot_state_received_at=3.0,
        )
        self.assertEqual(
            self.link.get_health().arm_state.validity, FieldValidity.UNKNOWN
        )
        self.assertEqual(
            self.link.get_health().feedback.validity,
            FieldValidity.UNKNOWN,
        )
        self.assertEqual(self.link.get_health().fault_bits.validity, FieldValidity.UNKNOWN)
        self.assertEqual(
            self.link.get_robot_state().gripper_state.validity,
            FieldValidity.UNKNOWN,
        )

        self.client.current = replace(
            self.client.current,
            safety_status=safety(99),
        )
        self.assertEqual(self.link.get_health().arm_state.validity, FieldValidity.INVALID)

    def test_command_timeout_is_may_have_applied_and_core_fails_closed(self):
        core = ControlCore(self.link, clock=self.clock)
        core.connect()
        token = core.acquire_lease("pilot")
        self.client.queue(
            replace(
                self.client.current,
                armed=True,
                safety_status=safety(protocol.SAFETY_STATE_ARMED),
                safety_status_received_at=2.1,
            )
        )
        core.arm(token)
        self.client.errors["set_velocity"] = runtime.RequestTimeoutError("velocity")
        with self.assertRaises(Exception) as caught:
            core.set_chassis(token, 100, 0, 200)
        self.assertEqual(caught.exception.delivery, DeliveryState.MAY_HAVE_APPLIED)
        self.assertEqual(caught.exception.code.value, "BACKEND_TIMEOUT")
        self.assertEqual(core.snapshot().mode, CoreMode.FAULT)
        self.assertIsNone(core.snapshot().lease)

    def test_reconnect_identity_invalidates_old_authority_and_arm(self):
        core = ControlCore(self.link, clock=self.clock)
        core.connect()
        token = core.acquire_lease("pilot")
        self.client.queue(
            replace(
                self.client.current,
                armed=True,
                safety_status=safety(protocol.SAFETY_STATE_ARMED),
                safety_status_received_at=2.1,
            )
        )
        core.arm(token)
        self.client.current = replace(
            self.client.current,
            armed=False,
            connection_generation=2,
            session_id=33,
            boot_id=44,
        )
        snapshot = core.poll()
        self.assertEqual(snapshot.mode, CoreMode.DISCONNECTED)
        self.assertIsNone(snapshot.lease)
        with self.assertRaises(LowerLinkError) as caught:
            self.link.set_chassis(ChassisSetpoint(100, 0, 200))
        self.assertEqual(caught.exception.delivery, DeliveryState.NOT_SENT)

    def test_gripper_completion_needs_fresh_matching_state(self):
        self.link.connect()
        self.assertEqual(
            self.link.set_gripper(GripperTarget.OPEN).status,
            CommandStatus.ACCEPTED,
        )
        self.client.queue(
            replace(
                self.client.current,
                robot_state=robot(
                    seq=2, gripper=protocol.GRIPPER_STATE_OPEN
                ),
                robot_state_received_at=3.0,
            )
        )
        self.assertEqual(
            self.link.set_gripper(GripperTarget.OPEN).status,
            CommandStatus.COMPLETED,
        )
        self.assertEqual(
            self.link.get_robot_state().gripper_state.value, GripperState.OPEN
        )

    def test_close_never_adds_an_unattributed_safe_stop(self):
        self.link.connect()
        self.link.close()
        self.assertEqual(self.client.calls[-1], ("close", False))
        self.assertFalse(any(call == ("safe_stop",) for call in self.client.calls))


class IoSliceWiringTests(unittest.TestCase):
    def test_adapter_constructs_client_with_configured_io_slice(self):
        constructed = []

        class RecordingClient(FakeRuntimeClient):
            def __init__(self, **kwargs):
                super().__init__(ManualClock(2.0))
                constructed.append(kwargs)

        with patch(
            "rescue_control.competition_lower_link.runtime.RescueCarClient",
            RecordingClient,
        ):
            CompetitionLowerLink(port="/dev/serial/by-id/usb-STM32-test")
            CompetitionLowerLink(
                port="/dev/serial/by-id/usb-STM32-test",
                config=CompetitionLinkConfig(io_slice_s=0.02),
            )
        self.assertEqual(constructed[0]["io_slice_s"], 0.005)
        self.assertEqual(constructed[1]["io_slice_s"], 0.02)

    def test_injected_client_ignores_io_slice_and_invalid_values_fail(self):
        clock = ManualClock(2.0)
        link = CompetitionLowerLink(
            client=FakeRuntimeClient(clock),
            clock=clock,
            config=CompetitionLinkConfig(io_slice_s=0.001),
        )
        self.assertIsNotNone(link)
        for bad in (0.0, -1.0, float("nan")):
            with self.subTest(bad=bad):
                with self.assertRaises((TypeError, ValueError)):
                    CompetitionLinkConfig(io_slice_s=bad)


if __name__ == "__main__":
    unittest.main()
