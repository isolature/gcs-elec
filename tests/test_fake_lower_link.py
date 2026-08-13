from __future__ import annotations

import math
import unittest

from rescue_control import (
    ArmState,
    ChassisSetpoint,
    CommandStatus,
    DeliveryState,
    FakeLowerLink,
    FieldValidity,
    FaultKind,
    GripperTarget,
    LowerLinkError,
    ManualClock,
    SafeStopReason,
)


class FakeLowerLinkTests(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = ManualClock(2.0)
        self.fake = FakeLowerLink(self.clock)
        self.fake.connect()
        self.fake.arm()

    def test_history_is_immutable_sequenced_and_contains_before_after(self) -> None:
        self.clock.advance(0.1)
        self.fake.set_chassis(ChassisSetpoint(100, 20, 50))
        history = self.fake.history
        self.assertIsInstance(history, tuple)
        self.assertEqual([record.sequence for record in history], [1, 2, 3])
        last = history[-1]
        self.assertEqual(last.time_s, 2.1)
        self.assertEqual(last.before.linear_velocity_mm_s, 0)
        self.assertEqual(last.after.linear_velocity_mm_s, 100)

    def test_fault_fifo_only_consumes_matching_operation(self) -> None:
        self.fake.fail_next("set_gripper", FaultKind.PROTOCOL_ERROR, "bad feedback")
        self.fake.stop()
        with self.assertRaises(LowerLinkError) as caught:
            self.fake.set_gripper(GripperTarget.OPEN)
        self.assertEqual(caught.exception.operation, "set_gripper")
        self.assertEqual(self.fake.history[-1].error_code, "PROTOCOL")

    def test_disconnected_precondition_does_not_consume_fault_plan(self) -> None:
        fake = FakeLowerLink(self.clock)
        fake.fail_next("arm", FaultKind.TIMEOUT_BEFORE_EFFECT, "planned timeout")

        with self.assertRaises(LowerLinkError) as disconnected:
            fake.arm()
        self.assertEqual(disconnected.exception.code.value, "DISCONNECTED")
        self.assertEqual(disconnected.exception.delivery, DeliveryState.NOT_SENT)

        fake.connect()
        with self.assertRaises(LowerLinkError) as planned:
            fake.arm()
        self.assertEqual(planned.exception.code.value, "TIMEOUT")
        self.assertEqual(
            [record.operation for record in fake.history],
            ["arm", "connect", "arm"],
        )
        self.assertEqual([record.sequence for record in fake.history], [1, 2, 3])
        self.assertEqual(fake.history[0].delivery, DeliveryState.NOT_SENT)
        self.assertEqual(fake.history[2].delivery, DeliveryState.NOT_SENT)

    def test_timeout_after_effect_records_delivery_uncertainty(self) -> None:
        self.fake.fail_next("set_chassis", FaultKind.TIMEOUT_AFTER_EFFECT)
        with self.assertRaises(LowerLinkError) as caught:
            self.fake.set_chassis(ChassisSetpoint(123, 456, 200))
        self.assertEqual(caught.exception.delivery, DeliveryState.MAY_HAVE_APPLIED)
        self.assertEqual(self.fake.state.linear_velocity_mm_s, 123)
        self.assertEqual(self.fake.history[-1].delivery, DeliveryState.MAY_HAVE_APPLIED)

    def test_result_plan_survives_a_prior_fault(self) -> None:
        self.fake.result_next("set_gripper", CommandStatus.ACCEPTED)
        self.fake.fail_next("set_gripper", FaultKind.TIMEOUT_BEFORE_EFFECT)
        with self.assertRaises(LowerLinkError):
            self.fake.set_gripper(GripperTarget.OPEN)
        result = self.fake.set_gripper(GripperTarget.OPEN)
        self.assertEqual(result.status, CommandStatus.ACCEPTED)

    def test_health_fields_are_independently_and_deterministically_injected(self) -> None:
        self.clock.advance(0.5)
        self.fake.inject_health(
            feedback_value=False,
            feedback_validity=FieldValidity.VALID,
            reported_arm_state=ArmState.FAULT,
            reported_arm_validity=FieldValidity.INVALID,
            fault_bits_value=0x24,
            fault_bits_validity=FieldValidity.VALID,
            reason="injected health mismatch",
        )

        health = self.fake.get_health()
        self.assertFalse(health.feedback.value)
        self.assertEqual(health.feedback.validity, FieldValidity.VALID)
        self.assertEqual(health.feedback.observed_at, 2.5)
        self.assertEqual(health.arm_state.value, ArmState.FAULT)
        self.assertEqual(health.arm_state.validity, FieldValidity.INVALID)
        self.assertEqual(health.arm_state.observed_at, 2.5)
        self.assertEqual(health.fault_bits.value, 0x24)
        self.assertEqual(health.fault_bits.validity, FieldValidity.VALID)
        self.assertEqual(health.fault_bits.observed_at, 2.5)
        self.assertEqual(self.fake.state.arm_state, ArmState.ARMED)

    def test_stale_health_fields_keep_last_observation_time(self) -> None:
        observed = self.fake.get_health()
        observed_state = self.fake.get_robot_state()
        self.clock.advance(4.0)
        self.fake.inject_disconnect()
        self.clock.advance(3.0)

        stale = self.fake.get_health()
        stale_state = self.fake.get_robot_state()
        self.assertEqual(stale.observed_at, 9.0)
        self.assertEqual(stale_state.observed_at, 9.0)
        for before, after in (
            (observed.feedback, stale.feedback),
            (observed.arm_state, stale.arm_state),
            (observed.fault_bits, stale.fault_bits),
            (
                observed_state.linear_velocity_mm_s,
                stale_state.linear_velocity_mm_s,
            ),
            (
                observed_state.angular_velocity_mrad_s,
                stale_state.angular_velocity_mrad_s,
            ),
            (observed_state.arm_state, stale_state.arm_state),
        ):
            self.assertEqual(after.validity, FieldValidity.STALE)
            self.assertEqual(after.observed_at, before.observed_at)
            self.assertNotEqual(after.observed_at, 9.0)

    def test_disconnect_and_reconnect_do_not_fake_physical_reset(self) -> None:
        self.fake.set_chassis(ChassisSetpoint(100, 20, 50))
        self.fake.inject_disconnect()
        self.assertEqual(self.fake.state.arm_state, ArmState.ARMED)
        self.assertEqual(self.fake.state.linear_velocity_mm_s, 100)
        self.fake.connect()
        self.assertEqual(self.fake.state.arm_state, ArmState.ARMED)
        self.assertEqual(self.fake.state.linear_velocity_mm_s, 100)

    def test_injection_control_plane_is_not_part_of_business_interface(self) -> None:
        with self.assertRaises(ValueError):
            self.fake.fail_next("get_health", FaultKind.REJECT)
        with self.assertRaises(ValueError):
            self.fake.result_next("get_health", CommandStatus.SENT)
        with self.assertRaises(ValueError):
            self.fake.fail_next("unknown", FaultKind.DISCONNECT)

    def test_manual_clock_rejects_non_monotonic_or_non_finite_time(self) -> None:
        for value in (-1, math.nan, math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    ManualClock().advance(value)
        with self.assertRaises(TypeError):
            ManualClock().advance(True)

    def test_safe_stop_always_records_even_from_safe_state(self) -> None:
        self.fake.safe_stop(SafeStopReason.USER_REQUEST)
        self.fake.safe_stop(SafeStopReason.USER_REQUEST)
        self.assertEqual(self.fake.count("safe_stop"), 2)


if __name__ == "__main__":
    unittest.main()
