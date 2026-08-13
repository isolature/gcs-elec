from __future__ import annotations

import math
import unittest

from rescue_control import (
    ArmState,
    ChassisSetpoint,
    CommandResult,
    CommandStatus,
    ControlCore,
    CoreConfig,
    CoreErrorCode,
    CoreMode,
    CoreBackendError,
    DeliveryState,
    FieldValidity,
    FakeLowerLink,
    FaultKind,
    GripperState,
    GripperTarget,
    HealthSnapshot,
    InvalidControlInput,
    InvalidCoreState,
    InvalidLeaseError,
    LeaseConflictError,
    ManualClock,
    RobotStateSnapshot,
    SafeStopReason,
    StateField,
)


class CoreHarness(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = ManualClock()
        self.fake = FakeLowerLink(self.clock)
        self.core = ControlCore(self.fake, clock=self.clock)
        self.core.connect()

    def acquire_and_arm(self, owner: str = "pilot", duration: float = 0.5):
        token = self.core.acquire_lease(owner, duration)
        result = self.core.arm(token)
        self.assertEqual(result.status, CommandStatus.COMPLETED)
        return token


class LeaseTests(CoreHarness):
    def test_single_owner_conflict_is_non_disruptive_and_no_preemption(self) -> None:
        token = self.core.acquire_lease("pilot", 0.5)
        before = self.core.snapshot()
        with self.assertRaises(LeaseConflictError) as caught:
            self.core.acquire_lease("intruder", 1.0)
        self.assertEqual(caught.exception.code, CoreErrorCode.LEASE_CONFLICT)
        after = self.core.snapshot()
        self.assertEqual(after.lease, before.lease)
        self.assertEqual(self.fake.count("safe_stop"), 0)
        self.core.arm(token)

    def test_renew_release_and_generation_make_old_token_permanently_stale(self) -> None:
        old = self.core.acquire_lease("pilot", 0.2)
        self.clock.advance(0.1)
        self.core.renew_lease(old, 0.4)
        self.assertAlmostEqual(self.core.snapshot().lease.remaining_s, 0.4)
        self.core.release_lease(old)
        fresh = self.core.acquire_lease("pilot", 0.2)
        self.assertGreater(fresh.generation, old.generation)
        with self.assertRaises(InvalidLeaseError):
            self.core.renew_lease(old)

    def test_zero_nan_infinite_and_bool_durations_are_rejected(self) -> None:
        token = self.core.acquire_lease("pilot")
        for duration in (0, math.nan, math.inf, -math.inf, True):
            with self.subTest(duration=duration):
                with self.assertRaises(InvalidControlInput):
                    self.core.renew_lease(token, duration)

    def test_deadline_boundary_expires_and_old_token_cannot_reactivate(self) -> None:
        token = self.acquire_and_arm(duration=0.5)
        self.core.set_chassis(token, 200, 0, 200)
        self.clock.advance(0.499)
        self.assertEqual(self.core.poll().mode, CoreMode.ARMED)
        self.clock.advance(0.001)
        snapshot = self.core.poll()
        self.assertEqual(snapshot.mode, CoreMode.SAFE_STOPPED)
        self.assertIsNone(snapshot.lease)
        self.assertEqual(snapshot.last_safety_reason, SafeStopReason.COMMAND_TIMEOUT)
        with self.assertRaises(InvalidLeaseError) as caught:
            self.core.set_chassis(token, 200, 0, 200)
        self.assertEqual(caught.exception.code, CoreErrorCode.INVALID_LEASE)

    def test_successful_controls_renew_but_failed_owner_does_not(self) -> None:
        token = self.acquire_and_arm(duration=0.5)
        self.clock.advance(0.2)
        self.core.set_chassis(token, 200, 0, 200)
        renewed = self.core.snapshot().lease.deadline_s
        bad = type(token)("intruder", token.generation)
        self.clock.advance(0.1)
        with self.assertRaises(InvalidLeaseError):
            self.core.set_chassis(bad, 0, 800, 200)
        self.assertEqual(self.core.snapshot().lease.deadline_s, renewed)
        self.assertEqual(self.fake.count("safe_stop"), 0)


class SafetyStateTests(CoreHarness):
    def test_every_control_semantic_and_normal_stop_dedup(self) -> None:
        token = self.acquire_and_arm()
        for linear, angular in ((200, 0), (-200, 0), (0, 800), (0, -800)):
            result = self.core.set_chassis(token, linear, angular, 200)
            self.assertEqual(result.status, CommandStatus.COMPLETED)
            self.assertEqual(
                (self.fake.state.linear_velocity_mm_s, self.fake.state.angular_velocity_mrad_s),
                (linear, angular),
            )
            self.core.stop(token)
        stop_calls = self.fake.count("stop")
        self.core.stop(token)
        self.assertEqual(self.fake.count("stop"), stop_calls)
        self.core.set_gripper(token, GripperTarget.OPEN)
        self.assertEqual(self.fake.state.gripper_target, GripperTarget.OPEN)
        self.core.set_gripper(token, GripperTarget.CLOSED)
        self.assertEqual(self.fake.state.gripper_target, GripperTarget.CLOSED)

    def test_safe_stop_is_never_deduplicated_and_disarms(self) -> None:
        self.acquire_and_arm()
        self.core.safe_stop(SafeStopReason.USER_REQUEST)
        self.core.safe_stop(SafeStopReason.USER_REQUEST)
        self.assertEqual(self.fake.count("safe_stop"), 2)
        self.assertEqual(self.core.snapshot().mode, CoreMode.SAFE_STOPPED)
        self.assertIsNone(self.core.snapshot().lease)

    def test_invalid_raw_values_reach_core_and_force_safe_stop(self) -> None:
        invalid_values = (
            (True, 0, 200),
            (501, 0, 200),
            (0, 2_001, 200),
            (0, 0, 0),
            (0, 0, 501),
        )
        for linear, angular, ttl in invalid_values:
            with self.subTest(values=(linear, angular, ttl)):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                token = core.acquire_lease("pilot", 0.5)
                core.arm(token)
                with self.assertRaises(InvalidControlInput):
                    core.set_chassis(token, linear, angular, ttl)
                snapshot = core.snapshot()
                self.assertEqual(snapshot.mode, CoreMode.SAFE_STOPPED)
                self.assertIsNone(snapshot.lease)
                self.assertEqual(snapshot.last_safety_reason, SafeStopReason.INVALID_COMMAND)
                self.assertEqual(fake.count("set_chassis"), 0)
                self.assertEqual(fake.count("safe_stop"), 1)

    def test_gripper_invalid_value_for_holder_forces_safe_stop(self) -> None:
        token = self.acquire_and_arm()
        with self.assertRaises(InvalidControlInput):
            self.core.set_gripper(token, "TOGGLE")  # type: ignore[arg-type]
        self.assertEqual(self.core.snapshot().mode, CoreMode.SAFE_STOPPED)
        self.assertEqual(self.fake.count("safe_stop"), 1)

    def test_stale_and_invalid_feedback_are_detected_by_poll(self) -> None:
        for validity, reason in (("STALE", SafeStopReason.FEEDBACK_STALE), ("INVALID", SafeStopReason.FEEDBACK_INVALID)):
            with self.subTest(validity=validity):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                token = core.acquire_lease("pilot")
                core.arm(token)
                from rescue_control import FieldValidity

                fake.set_feedback_validity(FieldValidity[validity])
                snapshot = core.poll()
                self.assertEqual(snapshot.mode, CoreMode.SAFE_STOPPED)
                self.assertEqual(snapshot.last_safety_reason, reason)
                self.assertIsNone(snapshot.lease)

    def test_disconnect_does_not_fabricate_physical_stop(self) -> None:
        token = self.acquire_and_arm()
        self.core.set_chassis(token, 200, 0, 200)
        self.fake.inject_disconnect()
        self.assertEqual(self.fake.state.linear_velocity_mm_s, 200)
        snapshot = self.core.poll()
        self.assertEqual(snapshot.mode, CoreMode.DISCONNECTED)
        self.assertFalse(snapshot.stop_confirmed)
        self.assertEqual(snapshot.last_safety_reason, SafeStopReason.LINK_DISCONNECTED)
        self.assertIsNone(snapshot.lease)



class HardeningInvariantTests(CoreHarness):
    def test_release_preserves_physical_arm_but_new_generation_must_rearm(self) -> None:
        old = self.acquire_and_arm()
        self.core.set_chassis(old, 200, 0, 200)
        self.core.release_lease(old)
        released = self.core.snapshot()
        self.assertEqual(released.mode, CoreMode.ARMED)
        self.assertIsNone(released.lease)
        self.assertIsNone(released.armed_generation)
        self.assertEqual(self.fake.state.arm_state, ArmState.ARMED)
        self.assertEqual(
            dict(self.core.events[-1].details),
            {"generation": old.generation, "owner": old.owner},
        )
        fresh = self.core.acquire_lease("pilot")
        with self.assertRaises(InvalidCoreState):
            self.core.set_chassis(fresh, 200, 0, 200)
        self.core.arm(fresh)
        self.core.set_chassis(fresh, 200, 0, 200)
        with self.assertRaises(InvalidLeaseError):
            self.core.renew_lease(old)

    def test_events_property_returns_immutable_snapshot(self) -> None:
        token = self.core.acquire_lease("pilot")
        events = self.core.events
        self.assertIsInstance(events, tuple)
        self.assertEqual(events[-1].event, "lease_acquired")
        self.core.release_lease(token)
        self.assertEqual(events[-1].event, "lease_acquired")
        self.assertEqual(self.core.events[-1].event, "lease_released")

    def test_unconfirmed_stop_is_not_cached_or_deduplicated(self) -> None:
        token = self.acquire_and_arm()
        self.core.set_chassis(token, 200, 0, 200)
        deadline = self.core.snapshot().lease.deadline_s
        self.fake.result_next("stop", CommandStatus.SENT)
        first = self.core.stop(token)
        self.assertEqual(first.status, CommandStatus.SENT)
        self.assertEqual(self.core.snapshot().commanded_linear_velocity_mm_s, 200)
        self.assertEqual(self.core.snapshot().lease.deadline_s, deadline)
        self.core.stop(token)
        self.assertEqual(self.fake.count("stop"), 2)
        self.assertEqual(self.core.snapshot().commanded_linear_velocity_mm_s, 0)

    def test_backend_return_at_deadline_expires_instead_of_renewing(self) -> None:
        token = self.acquire_and_arm(duration=0.2)
        original = self.fake.set_chassis

        def late_command(target: ChassisSetpoint) -> CommandResult:
            self.clock.advance(0.2)
            return original(target)

        self.fake.set_chassis = late_command  # type: ignore[method-assign]
        with self.assertRaises(InvalidLeaseError) as caught:
            self.core.set_chassis(token, 200, 0, 200)
        self.assertEqual(caught.exception.code, CoreErrorCode.LEASE_EXPIRED)
        snapshot = self.core.snapshot()
        self.assertIsNone(snapshot.lease)
        self.assertEqual(snapshot.mode, CoreMode.SAFE_STOPPED)
        self.assertEqual(snapshot.last_safety_reason, SafeStopReason.COMMAND_TIMEOUT)

    def test_poll_fails_closed_for_unknown_false_arm_unknown_and_fault_bits(self) -> None:
        cases = (
            HealthSnapshot(
                connected=True,
                arm_state=StateField.valid(ArmState.ARMED, 0.0),
                feedback=StateField.unknown("feedback unavailable"),
                fault_bits=StateField.valid(0, 0.0),
                observed_at=0.0,
            ),
            HealthSnapshot(
                connected=True,
                arm_state=StateField.valid(ArmState.ARMED, 0.0),
                feedback=StateField.valid(False, 0.0),
                fault_bits=StateField.valid(0, 0.0),
                observed_at=0.0,
            ),
            HealthSnapshot(
                connected=True,
                arm_state=StateField.unknown("arm unavailable"),
                feedback=StateField.valid(True, 0.0),
                fault_bits=StateField.valid(0, 0.0),
                observed_at=0.0,
            ),
            HealthSnapshot(
                connected=True,
                arm_state=StateField.valid(ArmState.ARMED, 0.0),
                feedback=StateField.valid(True, 0.0),
                fault_bits=StateField.valid(1, 0.0),
                observed_at=0.0,
            ),
        )
        for health in cases:
            with self.subTest(health=health):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                token = core.acquire_lease("pilot")
                core.arm(token)
                fake.get_health = lambda health=health: health  # type: ignore[method-assign]
                snapshot = core.poll()
                self.assertEqual(snapshot.mode, CoreMode.SAFE_STOPPED)
                self.assertIsNone(snapshot.lease)
                self.assertEqual(fake.count("safe_stop"), 1)

    def test_get_robot_state_validates_structure_inactive_and_active(self) -> None:
        self.fake.get_robot_state = lambda: object()  # type: ignore[method-assign]
        with self.assertRaises(CoreBackendError) as inactive:
            self.core.get_robot_state()
        self.assertEqual(inactive.exception.code, CoreErrorCode.BACKEND_INTERNAL)
        self.assertEqual(self.fake.count("safe_stop"), 0)

        malformed_states = (
            RobotStateSnapshot(
                linear_velocity_mm_s=StateField.valid(True, 0.0),
                angular_velocity_mrad_s=StateField.valid(0, 0.0),
                gripper_state=StateField.unknown("not commanded"),
                arm_state=StateField.valid(ArmState.ARMED, 0.0),
                observed_at=0.0,
            ),
            RobotStateSnapshot(
                linear_velocity_mm_s=StateField(
                    FieldValidity.VALID, 0, math.inf
                ),
                angular_velocity_mrad_s=StateField.valid(0, 0.0),
                gripper_state=StateField.valid("OPEN", 0.0),
                arm_state=StateField.valid(ArmState.ARMED, 0.0),
                observed_at=0.0,
            ),
            RobotStateSnapshot(
                linear_velocity_mm_s=StateField.valid(0, 0.0),
                angular_velocity_mrad_s=StateField(
                    "VALID", 0, 0.0  # type: ignore[arg-type]
                ),
                gripper_state=StateField.valid(GripperState.OPEN, 0.0),
                arm_state=StateField.valid("ARMED", 0.0),
                observed_at=math.nan,
            ),
        )
        for malformed in malformed_states:
            with self.subTest(malformed=malformed):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                token = core.acquire_lease("pilot")
                core.arm(token)
                fake.get_robot_state = (  # type: ignore[method-assign]
                    lambda malformed=malformed: malformed
                )
                with self.assertRaises(CoreBackendError) as active:
                    core.get_robot_state()
                self.assertEqual(
                    active.exception.code, CoreErrorCode.BACKEND_INTERNAL
                )
                self.assertEqual(core.snapshot().mode, CoreMode.SAFE_STOPPED)
                self.assertIsNone(core.snapshot().lease)
                self.assertEqual(fake.count("safe_stop"), 1)

    def test_get_robot_state_returns_valid_snapshot(self) -> None:
        state = self.core.get_robot_state()
        self.assertIsInstance(state, RobotStateSnapshot)
        self.assertEqual(state.linear_velocity_mm_s.value, 0)
        self.assertEqual(state.angular_velocity_mrad_s.value, 0)

    def test_connect_requires_valid_zero_motion_before_safe_mode_claim(self) -> None:
        states = (
            RobotStateSnapshot(
                linear_velocity_mm_s=StateField.valid(1, 0.0),
                angular_velocity_mrad_s=StateField.valid(0, 0.0),
                gripper_state=StateField.unknown("not commanded"),
                arm_state=StateField.valid(ArmState.DISARMED, 0.0),
                observed_at=0.0,
            ),
            RobotStateSnapshot(
                linear_velocity_mm_s=StateField.unknown("velocity unavailable"),
                angular_velocity_mrad_s=StateField.valid(0, 0.0),
                gripper_state=StateField.unknown("not commanded"),
                arm_state=StateField.valid(ArmState.DISARMED, 0.0),
                observed_at=0.0,
            ),
        )
        for reported in states:
            with self.subTest(reported=reported):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                fake.get_robot_state = (  # type: ignore[method-assign]
                    lambda reported=reported: reported
                )
                core = ControlCore(fake, clock=clock)
                core.connect()
                self.assertEqual(core.snapshot().mode, CoreMode.SAFE_STOPPED)
                self.assertTrue(core.snapshot().stop_confirmed)
                self.assertEqual(fake.count("safe_stop"), 1)

    def test_connect_accepts_safe_stop_only_with_valid_zero_motion(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        fake.connect()
        fake.safe_stop(SafeStopReason.USER_REQUEST)
        core = ControlCore(fake, clock=clock)
        core.connect()
        self.assertEqual(core.snapshot().mode, CoreMode.SAFE_STOPPED)
        self.assertEqual(fake.count("safe_stop"), 1)

        clock = ManualClock()
        fake = FakeLowerLink(clock)
        fake.connect()
        fake.safe_stop(SafeStopReason.USER_REQUEST)
        reported = RobotStateSnapshot(
            linear_velocity_mm_s=StateField.valid(1, 0.0),
            angular_velocity_mrad_s=StateField.valid(0, 0.0),
            gripper_state=StateField.unknown("not commanded"),
            arm_state=StateField.valid(ArmState.SAFE_STOP, 0.0),
            observed_at=0.0,
        )
        fake.get_robot_state = lambda: reported  # type: ignore[method-assign]
        core = ControlCore(fake, clock=clock)
        core.connect()
        self.assertEqual(core.snapshot().mode, CoreMode.SAFE_STOPPED)
        self.assertEqual(fake.count("safe_stop"), 2)

    def test_poll_rejects_valid_health_field_with_nan_observed_at(self) -> None:
        token = self.acquire_and_arm()
        malformed = HealthSnapshot(
            connected=True,
            arm_state=StateField.valid(ArmState.ARMED, 0.0),
            feedback=StateField(FieldValidity.VALID, True, math.nan),
            fault_bits=StateField.valid(0, 0.0),
            observed_at=0.0,
        )
        self.fake.get_health = lambda: malformed  # type: ignore[method-assign]
        snapshot = self.core.poll()
        self.assertEqual(snapshot.mode, CoreMode.SAFE_STOPPED)
        self.assertIsNone(snapshot.lease)
        self.assertEqual(snapshot.last_error, CoreErrorCode.BACKEND_INTERNAL.value)
        self.assertEqual(snapshot.last_safety_reason, SafeStopReason.BACKEND_FAILURE)
        self.assertEqual(self.fake.count("safe_stop"), 1)
        with self.assertRaises(InvalidLeaseError):
            self.core.renew_lease(token)

    def test_malformed_command_result_fails_closed(self) -> None:
        token = self.acquire_and_arm()
        self.fake.set_gripper = lambda target: CommandResult(  # type: ignore[method-assign]
            "BOGUS", "set_gripper"
        )
        with self.assertRaises(CoreBackendError) as caught:
            self.core.set_gripper(token, GripperTarget.OPEN)
        self.assertEqual(caught.exception.code, CoreErrorCode.BACKEND_INTERNAL)
        self.assertEqual(self.core.snapshot().mode, CoreMode.SAFE_STOPPED)

        clock = ManualClock()
        fake = FakeLowerLink(clock)
        core = ControlCore(fake, clock=clock)
        core.connect()
        token = core.acquire_lease("pilot")
        core.arm(token)
        fake.set_gripper = lambda target: CommandResult(  # type: ignore[method-assign]
            CommandStatus.COMPLETED, "wrong_operation"
        )
        with self.assertRaises(CoreBackendError):
            core.set_gripper(token, GripperTarget.OPEN)
        self.assertEqual(core.snapshot().mode, CoreMode.SAFE_STOPPED)

    def test_connect_recovers_unknown_armed_backend_with_confirmed_safe_stop(self) -> None:
        clock = ManualClock()
        fake = FakeLowerLink(clock)
        fake.connect()
        fake.arm()
        fake.set_chassis(ChassisSetpoint(200, 0, 200))
        core = ControlCore(fake, clock=clock)
        core.connect()
        snapshot = core.snapshot()
        self.assertEqual(snapshot.mode, CoreMode.SAFE_STOPPED)
        self.assertTrue(snapshot.stop_confirmed)
        self.assertEqual(fake.state.linear_velocity_mm_s, 0)
        self.assertEqual(fake.count("safe_stop"), 1)


class BackendFailureTests(CoreHarness):
    def test_timeout_before_and_after_effect_both_trigger_fallback(self) -> None:
        for kind, expected_delivery in (
            (FaultKind.TIMEOUT_BEFORE_EFFECT, DeliveryState.NOT_SENT),
            (FaultKind.TIMEOUT_AFTER_EFFECT, DeliveryState.MAY_HAVE_APPLIED),
        ):
            with self.subTest(kind=kind):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                token = core.acquire_lease("pilot")
                core.arm(token)
                fake.fail_next("set_chassis", kind)
                with self.assertRaises(Exception) as caught:
                    core.set_chassis(token, 200, 0, 200)
                self.assertEqual(caught.exception.code, CoreErrorCode.BACKEND_TIMEOUT)
                self.assertEqual(caught.exception.delivery, expected_delivery)
                self.assertEqual(core.snapshot().mode, CoreMode.SAFE_STOPPED)
                self.assertEqual(fake.count("safe_stop"), 1)
                records = [r for r in fake.history if r.operation == "set_chassis"]
                if kind is FaultKind.TIMEOUT_AFTER_EFFECT:
                    self.assertEqual(records[0].after.linear_velocity_mm_s, 200)

    def test_rejection_unexpected_protocol_and_disconnect_are_mapped(self) -> None:
        cases = (
            (FaultKind.REJECT, CoreErrorCode.COMMAND_REJECTED),
            (FaultKind.UNEXPECTED_ERROR, CoreErrorCode.BACKEND_INTERNAL),
            (FaultKind.PROTOCOL_ERROR, CoreErrorCode.BACKEND_PROTOCOL),
            (FaultKind.DISCONNECT, CoreErrorCode.BACKEND_DISCONNECTED),
        )
        for kind, code in cases:
            with self.subTest(kind=kind):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                token = core.acquire_lease("pilot")
                core.arm(token)
                fake.fail_next("set_gripper", kind)
                with self.assertRaises(Exception) as caught:
                    core.set_gripper(token, GripperTarget.OPEN)
                self.assertEqual(caught.exception.code, code)
                self.assertIsNone(core.snapshot().lease)
                self.assertTrue(core.snapshot().stop_attempted)

    def test_sent_and_accepted_arm_fail_closed_and_never_become_operable(self) -> None:
        for status in (CommandStatus.SENT, CommandStatus.ACCEPTED):
            with self.subTest(status=status):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                token = core.acquire_lease("pilot")
                fake.result_next("arm", status)
                with self.assertRaises(InvalidCoreState):
                    core.arm(token)
                snapshot = core.snapshot()
                self.assertEqual(snapshot.mode, CoreMode.SAFE_STOPPED)
                self.assertIsNone(snapshot.lease)
                self.assertIsNone(snapshot.armed_generation)
                self.assertEqual(fake.count("safe_stop"), 1)

    def test_safe_stop_sent_or_accepted_latches_fault(self) -> None:
        for status in (CommandStatus.SENT, CommandStatus.ACCEPTED):
            with self.subTest(status=status):
                clock = ManualClock()
                fake = FakeLowerLink(clock)
                core = ControlCore(fake, clock=clock)
                core.connect()
                fake.result_next("safe_stop", status)
                result = core.safe_stop(SafeStopReason.USER_REQUEST)
                self.assertEqual(result.status, status)
                self.assertEqual(core.snapshot().mode, CoreMode.FAULT)
                self.assertFalse(core.snapshot().stop_confirmed)
                with self.assertRaises(InvalidCoreState):
                    core.acquire_lease("pilot")

    def test_shutdown_always_attempts_safe_stop_then_close_and_is_idempotent(self) -> None:
        token = self.acquire_and_arm()
        self.core.set_chassis(token, 200, 0, 200)
        result = self.core.shutdown(SafeStopReason.EOF)
        self.assertTrue(result.ok)
        operations = [record.operation for record in self.fake.history]
        self.assertEqual(operations[-2:], ["safe_stop", "close"])
        self.assertEqual(self.core.shutdown(), result)
        self.assertEqual(self.fake.count("safe_stop"), 1)

    def test_shutdown_preserves_safe_stop_and_close_failures_in_order(self) -> None:
        token = self.acquire_and_arm()
        self.core.set_chassis(token, 200, 0, 200)
        self.fake.fail_next("safe_stop", FaultKind.TIMEOUT_BEFORE_EFFECT)
        self.fake.fail_next("close", FaultKind.UNEXPECTED_ERROR)
        result = self.core.shutdown(SafeStopReason.EOF)
        self.assertEqual(
            result.errors,
            (
                CoreErrorCode.BACKEND_TIMEOUT.value,
                CoreErrorCode.BACKEND_INTERNAL.value,
            ),
        )
        self.assertFalse(result.stop_confirmed)
        self.assertFalse(result.ok)
        operations = [record.operation for record in self.fake.history]
        self.assertEqual(operations[-2:], ["safe_stop", "close"])
        snapshot = self.core.snapshot()
        self.assertEqual(snapshot.mode, CoreMode.CLOSED)
        self.assertIsNone(snapshot.lease)
        self.assertFalse(snapshot.stop_confirmed)

    def test_shutdown_closes_after_safe_stop_failure_and_reports_unconfirmed(self) -> None:
        self.acquire_and_arm()
        self.fake.fail_next("safe_stop", FaultKind.TIMEOUT_BEFORE_EFFECT)
        result = self.core.shutdown(SafeStopReason.EOF)
        self.assertFalse(result.ok)
        self.assertFalse(result.stop_confirmed)
        self.assertEqual([r.operation for r in self.fake.history][-2:], ["safe_stop", "close"])


if __name__ == "__main__":
    unittest.main()
