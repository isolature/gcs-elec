#!/usr/bin/env python3
"""Production Raspberry Pi client for the RescueCar STM32 protocol."""

from dataclasses import dataclass
import glob
import os
import queue
import secrets
import threading
import time

import rescue_car_protocol as protocol


BAUDRATE = 115200
HEARTBEAT_PERIOD_S = 0.1
HEARTBEAT_ACK_TIMEOUT_S = 0.6
MOTION_REFRESH_PERIOD_S = 0.1
DEFAULT_MOTION_TTL_MS = 300
DEFAULT_LINEAR_SPEED_MM_S = 150
DEFAULT_TURN_RATE_MRAD_S = 1000
RECONNECT_DELAY_S = 0.25


class RescueCarError(RuntimeError):
    pass


class NotConnectedError(RescueCarError):
    pass


class NotArmedError(RescueCarError):
    pass


class CommandRejectedError(RescueCarError):
    def __init__(self, result):
        super().__init__(
            f"command {result.command_id} rejected with reason {result.reason}"
        )
        self.result = result


@dataclass(frozen=True)
class ClientSnapshot:
    connected: bool
    armed: bool
    port: str
    session_id: int
    boot_id: int
    last_error: str
    safety_status: object
    robot_state: object
    last_heartbeat_ack_time: float


@dataclass
class _Request:
    operation: str
    arguments: tuple
    completed: threading.Event
    result: object = None
    error: Exception = None
    cancelled: bool = False


def find_stm32():
    candidates = (
        glob.glob("/dev/serial/by-id/*STM*")
        + glob.glob("/dev/serial/by-id/*STMicroelectronics*")
    )
    if candidates:
        return candidates[0]
    for path in ("/dev/ttyACM0", "/dev/ttyACM1"):
        if os.path.exists(path):
            return path
    raise FileNotFoundError(
        "No STM32 serial device found; run: ls -l /dev/ttyACM*"
    )


class RescueCarClient:
    """Thread-safe high-level client; one worker owns all serial I/O."""

    def __init__(
        self,
        port=None,
        *,
        baudrate=BAUDRATE,
        serial_factory=None,
        reconnect=True,
    ):
        protocol.self_check()
        self._configured_port = port
        self._baudrate = baudrate
        self._serial_factory = serial_factory
        self._reconnect = reconnect

        self._lock = threading.Lock()
        self._state_changed = threading.Condition(self._lock)
        self._requests = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = None
        self._serial = None
        self._receive_buffer = bytearray()

        self._connected = False
        self._armed = False
        self._port = ""
        self._session_id = 0
        self._boot_id = 0
        self._last_error = ""
        self._safety_status = None
        self._robot_state = None
        self._last_heartbeat_ack_time = 0.0

        self._frame_seq = 1
        self._command_id = 1
        self._heartbeat_seq = 1
        self._next_heartbeat_time = 0.0
        self._motion = None
        self._next_motion_time = 0.0

    @property
    def is_connected(self):
        with self._lock:
            return self._connected

    @property
    def is_armed(self):
        with self._lock:
            return self._armed

    @property
    def latest_safety_status(self):
        with self._lock:
            return self._safety_status

    @property
    def latest_robot_state(self):
        with self._lock:
            return self._robot_state

    def snapshot(self):
        with self._lock:
            return ClientSnapshot(
                self._connected,
                self._armed,
                self._port,
                self._session_id,
                self._boot_id,
                self._last_error,
                self._safety_status,
                self._robot_state,
                self._last_heartbeat_ack_time,
            )

    def connect(self, timeout=3.0):
        with self._state_changed:
            if self._thread is None or not self._thread.is_alive():
                self._stop_event.clear()
                self._thread = threading.Thread(
                    target=self._worker,
                    name="rescue-car-client",
                    daemon=True,
                )
                self._thread.start()
            deadline = time.monotonic() + timeout
            while not self._connected:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    detail = f": {self._last_error}" if self._last_error else ""
                    raise NotConnectedError(f"STM32 connection timed out{detail}")
                self._state_changed.wait(remaining)
        return self

    def wait_until_connected(self, timeout=None):
        with self._state_changed:
            deadline = None if timeout is None else time.monotonic() + timeout
            while not self._connected and not self._stop_event.is_set():
                if deadline is None:
                    self._state_changed.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._state_changed.wait(remaining)
            return self._connected

    def arm(self, timeout=2.0):
        result = self._request("arm", timeout=timeout)
        if result.status != protocol.COMMAND_RESULT_ACCEPTED:
            raise CommandRejectedError(result)
        return result

    def disarm(self, timeout=2.0):
        result = self._request("disarm", timeout=timeout)
        if result.status != protocol.COMMAND_RESULT_ACCEPTED:
            raise CommandRejectedError(result)
        return result

    def safe_stop(self, timeout=2.0):
        result = self._request("safe_stop", timeout=timeout)
        if result.status != protocol.COMMAND_RESULT_ACCEPTED:
            raise CommandRejectedError(result)
        return result

    def set_velocity(
        self,
        linear_mm_s,
        angular_mrad_s,
        *,
        ttl_ms=DEFAULT_MOTION_TTL_MS,
        timeout=1.0,
    ):
        if isinstance(linear_mm_s, bool) or not isinstance(linear_mm_s, int):
            raise TypeError("linear_mm_s must be an integer")
        if isinstance(angular_mrad_s, bool) or not isinstance(angular_mrad_s, int):
            raise TypeError("angular_mrad_s must be an integer")
        return self._request(
            "velocity",
            linear_mm_s,
            angular_mrad_s,
            ttl_ms,
            timeout=timeout,
        )

    def stop(self, timeout=1.0):
        return self.set_velocity(0, 0, timeout=timeout)

    def forward(self, speed_mm_s=DEFAULT_LINEAR_SPEED_MM_S, **kwargs):
        return self.set_velocity(speed_mm_s, 0, **kwargs)

    def backward(self, speed_mm_s=DEFAULT_LINEAR_SPEED_MM_S, **kwargs):
        return self.set_velocity(-speed_mm_s, 0, **kwargs)

    def turn_left(self, rate_mrad_s=DEFAULT_TURN_RATE_MRAD_S, **kwargs):
        return self.set_velocity(0, rate_mrad_s, **kwargs)

    def turn_right(self, rate_mrad_s=DEFAULT_TURN_RATE_MRAD_S, **kwargs):
        return self.set_velocity(0, -rate_mrad_s, **kwargs)

    def open_gripper(self, timeout=2.0):
        return self._gripper(True, timeout)

    def close_gripper(self, timeout=2.0):
        return self._gripper(False, timeout)

    def _gripper(self, opened, timeout):
        result = self._request("gripper", opened, timeout=timeout)
        if result.status != protocol.COMMAND_RESULT_ACCEPTED:
            raise CommandRejectedError(result)
        return result

    def close(self, stop=True, timeout=1.0):
        if self._thread is None:
            return
        if stop and self.is_connected:
            try:
                self.safe_stop(timeout=timeout)
            except Exception:
                pass
        self._stop_event.set()
        self._requests.put(None)
        self._thread.join(timeout + 1.0)
        self._close_serial()
        self._mark_disconnected("")

    def __enter__(self):
        return self.connect()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close(stop=True)

    def _request(self, operation, *arguments, timeout):
        if not self.is_connected:
            raise NotConnectedError("STM32 is not connected")
        request = _Request(operation, arguments, threading.Event())
        self._requests.put(request)
        if not request.completed.wait(timeout):
            request.cancelled = True
            raise RescueCarError(f"{operation} request timed out")
        if request.error is not None:
            raise request.error
        return request.result

    def _worker(self):
        while not self._stop_event.is_set():
            try:
                self._open_and_handshake()
                self._run_connected()
            except Exception as exc:
                self._fail_pending(NotConnectedError(str(exc)))
                self._close_serial()
                self._mark_disconnected(str(exc))
                if not self._reconnect:
                    return
                self._stop_event.wait(RECONNECT_DELAY_S)

    def _make_serial(self, port):
        if self._serial_factory is not None:
            return self._serial_factory(
                port=port, baudrate=self._baudrate, timeout=0.02
            )
        try:
            import serial
        except ImportError as exc:
            raise RescueCarError("pyserial is required: pip install pyserial") from exc
        return serial.Serial(port=port, baudrate=self._baudrate, timeout=0.02)

    def _open_and_handshake(self):
        port = self._configured_port or find_stm32()
        self._serial = self._make_serial(port)
        self._serial.reset_input_buffer()
        self._receive_buffer.clear()
        self._session_id = secrets.randbits(32) or 1
        self._frame_seq = 1
        self._command_id = 1
        self._heartbeat_seq = 1
        self._send(protocol.MSG_HELLO, protocol.build_hello_payload())
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not self._stop_event.is_set():
            frame = self._read_frame(deadline)
            if frame is None or frame.msg_type != protocol.MSG_HELLO_ACK:
                continue
            if frame.session_id != self._session_id:
                continue
            ack = protocol.parse_hello_ack(frame)
            with self._state_changed:
                self._connected = True
                self._armed = False
                self._port = port
                self._boot_id = ack.boot_id
                self._last_error = ""
                self._motion = None
                self._last_heartbeat_ack_time = time.monotonic()
                self._next_heartbeat_time = time.monotonic()
                self._state_changed.notify_all()
            return
        raise NotConnectedError("no compatible HELLO_ACK within 1 second")

    def _run_connected(self):
        while not self._stop_event.is_set():
            self._service_periodic()
            self._read_available()
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                continue
            if request is None:
                return
            if request.cancelled:
                request.completed.set()
                continue
            try:
                request.result = self._handle_request(request)
            except Exception as exc:
                request.error = exc
            finally:
                request.completed.set()

    def _handle_request(self, request):
        operation = request.operation
        if operation == "velocity":
            if not self.is_armed:
                raise NotArmedError("ARM is required before motion")
            linear, angular, ttl_ms = request.arguments
            payload = protocol.build_chassis_payload(linear, angular, ttl_ms)
            self._send(protocol.MSG_CHASSIS_SETPOINT, payload)
            with self._lock:
                self._motion = None if (linear == 0 and angular == 0) else (
                    linear,
                    angular,
                    ttl_ms,
                )
                self._next_motion_time = time.monotonic() + MOTION_REFRESH_PERIOD_S
            return True

        command_id = self._next_command_id()
        if operation == "arm":
            msg_type = protocol.MSG_ARM_COMMAND
            payload = protocol.build_arm_payload(True, command_id)
        elif operation == "disarm":
            msg_type = protocol.MSG_ARM_COMMAND
            payload = protocol.build_arm_payload(False, command_id)
        elif operation == "safe_stop":
            msg_type = protocol.MSG_SAFE_STOP
            payload = protocol.build_safe_stop_payload(command_id)
        elif operation == "gripper":
            if not self.is_armed:
                raise NotArmedError("ARM is required before gripper commands")
            msg_type = protocol.MSG_GRIPPER_COMMAND
            payload = protocol.build_gripper_payload(request.arguments[0], command_id)
        else:
            raise RescueCarError(f"unsupported operation: {operation}")

        self._send(msg_type, payload)
        result = self._wait_for_command_result(command_id, 1.0)
        if result is None:
            raise RescueCarError(f"no result for command {command_id}")
        with self._lock:
            if operation == "arm" and result.status == protocol.COMMAND_RESULT_ACCEPTED:
                self._armed = True
                self._next_heartbeat_time = time.monotonic()
            elif operation in ("disarm", "safe_stop"):
                self._armed = False
                self._motion = None
        return result

    def _wait_for_command_result(self, command_id, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._service_periodic()
            frame = self._read_frame(min(deadline, time.monotonic() + 0.03))
            if frame is None:
                continue
            result = self._handle_frame(frame)
            if isinstance(result, protocol.CommandResult):
                if result.command_id == command_id:
                    return result
        return None

    def _service_periodic(self):
        now = time.monotonic()
        with self._lock:
            heartbeat_age = now - self._last_heartbeat_ack_time
        if heartbeat_age > HEARTBEAT_ACK_TIMEOUT_S:
            raise NotConnectedError("HEARTBEAT_ACK timed out")
        if now >= self._next_heartbeat_time:
            payload = protocol.build_heartbeat_payload(self._heartbeat_seq)
            self._heartbeat_seq = self._increment_nonzero(self._heartbeat_seq)
            self._send(protocol.MSG_HEARTBEAT, payload)
            self._next_heartbeat_time = now + HEARTBEAT_PERIOD_S
        with self._lock:
            motion = self._motion
            motion_due = motion is not None and now >= self._next_motion_time
        if motion_due:
            self._send(
                protocol.MSG_CHASSIS_SETPOINT,
                protocol.build_chassis_payload(*motion),
            )
            with self._lock:
                self._next_motion_time = now + MOTION_REFRESH_PERIOD_S

    def _send(self, msg_type, payload):
        if self._serial is None:
            raise NotConnectedError("serial port is not open")
        wire = protocol.build_wire_frame(
            msg_type,
            payload,
            self._session_id,
            self._frame_seq,
            time.monotonic_ns() // 1000,
        )
        self._frame_seq = (self._frame_seq + 1) & 0xFFFFFFFF
        written = self._serial.write(wire)
        self._serial.flush()
        if written != len(wire):
            raise OSError(f"short serial write: {written}/{len(wire)}")

    def _read_available(self):
        deadline = time.monotonic() + 0.02
        frame = self._read_frame(deadline)
        if frame is not None:
            self._handle_frame(frame)

    def _read_frame(self, deadline):
        while time.monotonic() < deadline:
            delimiter = self._receive_buffer.find(0)
            if delimiter >= 0:
                wire = bytes(self._receive_buffer[: delimiter + 1])
                del self._receive_buffer[: delimiter + 1]
                if len(wire) > 1:
                    return protocol.parse_wire_frame(wire)
                continue
            chunk = self._serial.read(256)
            if chunk:
                self._receive_buffer.extend(chunk)
        return None

    def _handle_frame(self, frame):
        if frame.session_id != self._session_id:
            return None
        if frame.msg_type == protocol.MSG_COMMAND_RESULT:
            return protocol.parse_command_result(frame)
        if frame.msg_type == protocol.MSG_HEARTBEAT_ACK:
            protocol.parse_heartbeat_ack(frame)
            with self._lock:
                self._last_heartbeat_ack_time = time.monotonic()
        elif frame.msg_type == protocol.MSG_SAFETY_STATUS:
            status = protocol.parse_safety_status(frame)
            with self._lock:
                self._safety_status = status
                self._armed = status.armed
                if not status.armed:
                    self._motion = None
        elif frame.msg_type == protocol.MSG_ROBOT_STATE:
            state = protocol.parse_robot_state(frame)
            with self._lock:
                self._robot_state = state
        return None

    def _next_command_id(self):
        value = self._command_id
        self._command_id = self._increment_nonzero(self._command_id)
        return value

    @staticmethod
    def _increment_nonzero(value):
        value = (value + 1) & 0xFFFFFFFF
        return value or 1

    def _fail_pending(self, error):
        while True:
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                return
            if request is not None:
                request.error = error
                request.completed.set()

    def _close_serial(self):
        serial_port = self._serial
        self._serial = None
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass

    def _mark_disconnected(self, error):
        with self._state_changed:
            self._connected = False
            self._armed = False
            self._session_id = 0
            self._boot_id = 0
            self._motion = None
            self._last_error = error
            self._state_changed.notify_all()
