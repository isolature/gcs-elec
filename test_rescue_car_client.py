#!/usr/bin/env python3

import struct
import threading
import time
import unittest

import rescue_car_protocol as protocol
from rescue_car_client import CommandRejectedError, NotArmedError, RescueCarClient
from rescue_control import (
    CommandStatus,
    CompetitionLinkConfig,
    CompetitionLowerLink,
    ControlCore,
    SafeStopReason,
)


def _field(number, value):
    return protocol.encode_varint(number << 3) + protocol.encode_varint(value)


def _fixed32_field(number, value):
    return protocol.encode_varint((number << 3) | 5) + struct.pack("<I", value)


def _fixed64_field(number, value):
    return protocol.encode_varint((number << 3) | 1) + struct.pack("<Q", value)


def _signed_field(number, value):
    return protocol.encode_varint(number << 3) + protocol.encode_sint32(value)


class FakeStm32Serial:
    def __init__(self, timeout=None, **_kwargs):
        self.timeout = 0.02 if timeout is None else timeout
        self.is_open = True
        self.received = []
        self._rx = bytearray()
        self._condition = threading.Condition()
        self._reply_seq = 1
        self._armed = False
        self._safety_state = protocol.SAFETY_STATE_DISARMED
        self._state_seq = 1
        self._linear = 0
        self._angular = 0
        self._gripper = protocol.GRIPPER_STATE_UNKNOWN
        self.command_result_status = protocol.COMMAND_RESULT_ACCEPTED

    def reset_input_buffer(self):
        with self._condition:
            self._rx.clear()

    def write(self, data):
        if not self.is_open:
            raise OSError("fake serial disconnected")
        frame = protocol.parse_wire_frame(bytes(data))
        self.received.append(frame)
        if frame.msg_type == protocol.MSG_HELLO:
            payload = (
                _field(1, protocol.PROTOCOL_VERSION)
                + _fixed32_field(2, protocol.EXPECTED_FIRMWARE_VERSION)
                + _fixed32_field(3, 0x12345678)
                + _fixed32_field(4, protocol.EXPECTED_CONFIG_HASH)
                + _fixed32_field(5, protocol.CAPABILITY_WHEEL_SPEED)
            )
            self._reply(frame, protocol.MSG_HELLO_ACK, payload)
            self._reply_safety(frame)
            self._reply_robot(frame)
        elif frame.msg_type == protocol.MSG_HEARTBEAT:
            fields = protocol._varint_fields(frame.payload)
            payload = _field(1, fields[1]) + _fixed64_field(2, 123456)
            self._reply(frame, protocol.MSG_HEARTBEAT_ACK, payload)
        elif frame.msg_type in (
            protocol.MSG_ARM_COMMAND,
            protocol.MSG_GRIPPER_COMMAND,
            protocol.MSG_SAFE_STOP,
        ):
            fields = protocol._varint_fields(frame.payload)
            command_id = fields[2]
            succeeded = self.command_result_status in (
                protocol.COMMAND_RESULT_ACCEPTED,
                protocol.COMMAND_RESULT_COMPLETED,
            )
            if succeeded:
                if frame.msg_type == protocol.MSG_ARM_COMMAND:
                    self._armed = fields[1] == protocol.ARM_TARGET_ARMED
                    self._safety_state = (
                        protocol.SAFETY_STATE_ARMED
                        if self._armed
                        else protocol.SAFETY_STATE_DISARMED
                    )
                    if not self._armed:
                        self._linear = 0
                        self._angular = 0
                elif frame.msg_type == protocol.MSG_SAFE_STOP:
                    self._armed = False
                    self._safety_state = protocol.SAFETY_STATE_SAFE_STOP
                    self._linear = 0
                    self._angular = 0
                elif frame.msg_type == protocol.MSG_GRIPPER_COMMAND:
                    self._gripper = (
                        protocol.GRIPPER_STATE_OPEN
                        if fields[1] == protocol.GRIPPER_TARGET_OPEN
                        else protocol.GRIPPER_STATE_CLOSED
                    )
            payload = (
                _field(1, command_id)
                + _field(2, self.command_result_status)
                + _field(3, 1)
            )
            self._reply(frame, protocol.MSG_COMMAND_RESULT, payload)
            self._reply_safety(frame)
            self._reply_robot(frame)
        elif frame.msg_type == protocol.MSG_CHASSIS_SETPOINT:
            fields = protocol._varint_fields(frame.payload)
            self._linear = protocol.decode_sint32(fields.get(1, 0))
            self._angular = protocol.decode_sint32(fields.get(2, 0))
            self._reply_robot(frame)
        return len(data)

    def flush(self):
        pass

    def read(self, size):
        deadline = time.monotonic() + self.timeout
        with self._condition:
            while not self._rx and self.is_open:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return b""
                self._condition.wait(remaining)
            chunk = bytes(self._rx[:size])
            del self._rx[:size]
            return chunk

    def close(self):
        with self._condition:
            self.is_open = False
            self._condition.notify_all()

    def _reply(self, request, msg_type, payload):
        wire = protocol.build_wire_frame(
            msg_type,
            payload,
            request.session_id,
            self._reply_seq,
            0,
        )
        self._reply_seq += 1
        with self._condition:
            self._rx.extend(wire)
            self._condition.notify_all()

    def _reply_safety(self, request):
        payload = (
            _field(1, 1)
            + _field(2, int(self._armed))
            + _field(3, self._safety_state)
            + _field(4, 1)
            + _fixed32_field(5, 0)
            + _field(6, 0)
            + _field(7, 12_000)
            + _field(8, 0)
            + _fixed32_field(9, 15)
        )
        self._reply(request, protocol.MSG_SAFETY_STATUS, payload)

    def _reply_robot(self, request):
        payload = (
            _field(1, self._state_seq)
            + _fixed64_field(2, self._state_seq * 1_000)
            + _field(3, protocol.COORDINATE_FRAME_BASE_LINK)
            + _field(4, request.seq)
            + _signed_field(5, self._linear)
            + _signed_field(6, self._angular)
            + _field(12, self._gripper)
            + _fixed32_field(13, 15)
        )
        self._state_seq += 1
        self._reply(request, protocol.MSG_ROBOT_STATE, payload)


class RescueCarProtocolTests(unittest.TestCase):
    def test_golden_empty_hello(self):
        wire = protocol.build_wire_frame(
            protocol.MSG_HELLO, b"", 0x12345678, 1, 0
        )
        decoded = protocol.cobs_decode(wire[:-1])
        self.assertEqual(struct.unpack_from("<I", decoded, 32)[0], 0x76D02E78)

    def test_status_decoders(self):
        safety_payload = (
            _field(1, 1) + _field(2, 1) + _field(3, 2) + _field(4, 1)
            + _fixed32_field(5, 3) + _field(6, 20) + _field(7, 12000)
            + _field(8, 0) + _fixed32_field(9, 15)
        )
        frame = protocol.Frame(protocol.MSG_SAFETY_STATUS, 1, 1, 0, safety_payload)
        status = protocol.parse_safety_status(frame)
        self.assertTrue(status.armed)
        self.assertEqual(status.battery_mv, 12000)
        self.assertEqual(status.validity_flags, 15)


class RescueCarClientTests(unittest.TestCase):
    def setUp(self):
        self.serials = []

        def factory(**kwargs):
            instance = FakeStm32Serial(**kwargs)
            self.serials.append(instance)
            return instance

        self.client = RescueCarClient(
            port="FAKE", serial_factory=factory, reconnect=False
        ).connect()

    def tearDown(self):
        self.client.close(stop=True)

    def test_requires_arm_before_motion(self):
        with self.assertRaises(NotArmedError):
            self.client.set_velocity(150, 0)

    def test_snapshot_exposes_fresh_state_receipt_times_and_generation(self):
        snapshot = self.client.wait_for_snapshot(
            lambda value: (
                value.safety_status is not None and value.robot_state is not None
            ),
            0.5,
        )
        self.assertGreater(snapshot.connection_generation, 0)
        self.assertGreaterEqual(
            snapshot.safety_status_received_at, snapshot.connected_at
        )
        self.assertGreaterEqual(
            snapshot.robot_state_received_at, snapshot.connected_at
        )

    def test_arm_motion_refresh_stop_and_disarm(self):
        self.client.arm()
        self.client.set_velocity(150, 0, ttl_ms=300)
        time.sleep(0.27)
        frames = list(self.serials[0].received)
        chassis_count = sum(
            frame.msg_type == protocol.MSG_CHASSIS_SETPOINT for frame in frames
        )
        heartbeat_count = sum(
            frame.msg_type == protocol.MSG_HEARTBEAT for frame in frames
        )
        self.assertGreaterEqual(chassis_count, 3)
        self.assertGreaterEqual(heartbeat_count, 2)

        self.client.stop()
        time.sleep(0.22)
        after_stop = sum(
            frame.msg_type == protocol.MSG_CHASSIS_SETPOINT
            for frame in self.serials[0].received
        )
        self.assertEqual(after_stop, chassis_count + 1)
        self.client.disarm()
        self.assertFalse(self.client.is_armed)

    def test_gripper_and_close_use_discrete_results(self):
        self.client.arm()
        result = self.client.open_gripper()
        self.assertEqual(result.status, protocol.COMMAND_RESULT_ACCEPTED)
        self.client.safe_stop()
        self.assertFalse(self.client.is_armed)

    def test_completed_arm_and_rejected_disarm_preserve_local_authority(self):
        serial_port = self.serials[0]
        serial_port.command_result_status = protocol.COMMAND_RESULT_COMPLETED
        result = self.client.arm()
        self.assertEqual(result.status, protocol.COMMAND_RESULT_COMPLETED)
        self.assertTrue(self.client.is_armed)
        self.client.set_velocity(50, 0)

        serial_port.command_result_status = protocol.COMMAND_RESULT_REJECTED
        with self.assertRaises(CommandRejectedError):
            self.client.disarm()
        self.assertTrue(self.client.is_armed)
        serial_port.command_result_status = protocol.COMMAND_RESULT_ACCEPTED

    def test_close_sends_safe_stop_when_armed(self):
        serial_port = self.serials[0]
        self.client.arm()
        self.client.close(stop=True)
        self.assertTrue(
            any(
                frame.msg_type == protocol.MSG_SAFE_STOP
                for frame in serial_port.received
            )
        )

    def test_close_preserves_disarmed_state(self):
        serial_port = self.serials[0]
        self.client.close(stop=True)
        self.assertFalse(
            any(
                frame.msg_type == protocol.MSG_SAFE_STOP
                for frame in serial_port.received
            )
        )

    def test_reconnect_stays_disarmed_and_drops_old_motion(self):
        self.client.close(stop=False)
        self.serials.clear()
        client = RescueCarClient(
            port="FAKE",
            serial_factory=lambda **kwargs: self._new_serial(**kwargs),
            reconnect=True,
        ).connect()
        try:
            client.arm()
            client.set_velocity(150, 0)
            self.serials[0].close()
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if len(self.serials) >= 2 and client.is_connected:
                    break
                time.sleep(0.02)
            self.assertGreaterEqual(len(self.serials), 2)
            self.assertTrue(client.is_connected)
            self.assertFalse(client.is_armed)
            time.sleep(0.15)
            self.assertFalse(
                any(
                    frame.msg_type == protocol.MSG_CHASSIS_SETPOINT
                    for frame in self.serials[1].received
                )
            )
        finally:
            client.close(stop=False)

    def _new_serial(self, **kwargs):
        instance = FakeStm32Serial(**kwargs)
        self.serials.append(instance)
        return instance


class IoSliceConfigTests(unittest.TestCase):
    def test_default_slice_preserves_legacy_serial_timeout(self):
        serials = []

        def factory(**kwargs):
            instance = FakeStm32Serial(**kwargs)
            serials.append(instance)
            return instance

        client = RescueCarClient(
            port="FAKE", serial_factory=factory, reconnect=False
        ).connect()
        try:
            self.assertEqual(serials[0].timeout, 0.02)
        finally:
            client.close(stop=False)

    def test_configured_slice_reaches_serial_timeout(self):
        serials = []

        def factory(**kwargs):
            instance = FakeStm32Serial(**kwargs)
            serials.append(instance)
            return instance

        client = RescueCarClient(
            port="FAKE",
            serial_factory=factory,
            reconnect=False,
            io_slice_s=0.005,
        ).connect()
        try:
            self.assertEqual(serials[0].timeout, 0.005)
        finally:
            client.close(stop=False)

    def test_invalid_slice_values_are_rejected(self):
        for bad in (0, -0.01, 0.2, float("nan"), float("inf"), True, "0.01"):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    RescueCarClient(
                        port="FAKE",
                        serial_factory=lambda **kwargs: FakeStm32Serial(**kwargs),
                        reconnect=False,
                        io_slice_s=bad,
                    )


class FormalControlIntegrationTests(unittest.TestCase):
    def test_core_to_formal_client_uses_only_fake_serial_and_confirms_state(self):
        serials = []

        def factory(**kwargs):
            instance = FakeStm32Serial(**kwargs)
            serials.append(instance)
            return instance

        link = CompetitionLowerLink(
            port="/dev/serial/by-id/usb-STM32-fake",
            serial_factory=factory,
            reconnect=False,
            config=CompetitionLinkConfig(
                connect_timeout_s=1.0,
                initial_feedback_timeout_s=1.0,
                confirmation_timeout_s=0.25,
                feedback_stale_after_s=0.5,
            ),
        )
        core = ControlCore(link)
        core.connect()
        token = core.acquire_lease("integration", duration_s=1.0)
        self.assertEqual(core.arm(token).status, CommandStatus.COMPLETED)
        self.assertEqual(
            core.set_chassis(token, 120, 0, 300).status,
            CommandStatus.SENT,
        )
        self.assertEqual(core.stop(token).status, CommandStatus.COMPLETED)
        self.assertEqual(
            core.safe_stop(SafeStopReason.USER_REQUEST).status,
            CommandStatus.COMPLETED,
        )
        self.assertEqual(
            core.safe_stop(SafeStopReason.SHUTDOWN).status,
            CommandStatus.COMPLETED,
        )
        cleanup = core.shutdown(SafeStopReason.SHUTDOWN)
        self.assertTrue(cleanup.ok)
        frames = serials[0].received
        self.assertEqual(
            sum(frame.msg_type == protocol.MSG_SAFE_STOP for frame in frames),
            3,
        )
        self.assertFalse(serials[0].is_open)


if __name__ == "__main__":
    unittest.main()
