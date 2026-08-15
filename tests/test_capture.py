"""Unit tests for the teleop<->camera bridge (no real cameras involved)."""

from __future__ import annotations

import threading
import time
import unittest
from pathlib import Path

from rescue_control.capture import (
    CaptureEvent,
    CaptureEventKind,
    CaptureUnavailableError,
    TeleopCameraSession,
    build_camera_app,
)


class FakeStore:
    session_id = "20260815T120000_fake0001"
    session_dir = Path("/tmp/fake-session-dir")

    def log_event(self, *args, **kwargs) -> None:
        pass


class FakeApp:
    """Scriptable stand-in for rescue_camera_capture.CaptureApplication."""

    def __init__(
        self,
        *,
        key_results: dict[str, str] | None = None,
        poll_error: Exception | None = None,
        init_error: Exception | None = None,
    ) -> None:
        self.store = FakeStore()
        self.keys: list[str] = []
        self.polls = 0
        self.shutdown_calls: list[tuple[str, bool]] = []
        self.key_results = key_results or {
            "p": "photo_captured",
            "v": "recording_started",
        }
        self.poll_error = poll_error
        self.init_error = init_error
        self.gate = threading.Event()

    def initialize(self) -> None:
        if self.init_error is not None:
            raise self.init_error

    def handle_key(self, key: str, now: float | None = None) -> str:
        self.keys.append(key)
        self.gate.wait(5.0)
        return self.key_results.get(key, "ignored")

    def poll(self) -> None:
        self.polls += 1
        if self.poll_error is not None:
            raise self.poll_error

    def shutdown(self, reason: str, graceful: bool) -> list[str]:
        self.shutdown_calls.append((reason, graceful))
        return []


def wait_until(predicate, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return False


def collect_event(
    session: TeleopCameraSession,
    kind: CaptureEventKind,
    *,
    timeout_s: float = 5.0,
) -> CaptureEvent:
    """Drain until an event of ``kind`` appears; fails the test otherwise."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for event in session.drain_events():
            if event.kind is kind:
                return event
        time.sleep(0.005)
    raise AssertionError(f"no {kind.value} event within {timeout_s} s")


class TeleopCameraSessionTests(unittest.TestCase):
    def test_ready_event_carries_session_identity(self) -> None:
        app = FakeApp()
        session = TeleopCameraSession(lambda: app)
        self.addCleanup(session.close)
        session.start()
        event = collect_event(session, CaptureEventKind.READY)
        self.assertEqual(event.session_id, FakeStore.session_id)
        self.assertEqual(event.session_dir, str(FakeStore.session_dir))
        self.assertFalse(session.closed)
        self.assertTrue(wait_until(lambda: app.polls > 0))

    def test_photo_and_video_actions_reach_app_and_report_results(
        self,
    ) -> None:
        app = FakeApp()
        session = TeleopCameraSession(lambda: app)
        self.addCleanup(session.close)
        session.start()
        collect_event(session, CaptureEventKind.READY)

        self.assertTrue(session.photo())
        self.assertTrue(wait_until(lambda: app.keys == ["p"]))
        app.gate.set()
        result = collect_event(session, CaptureEventKind.KEY_RESULT)
        self.assertEqual(result.action, "PHOTO")
        self.assertEqual(result.detail, "photo_captured")

        self.assertTrue(session.video_toggle())
        self.assertTrue(wait_until(lambda: app.keys == ["p", "v"]))
        app.gate.set()
        result = collect_event(session, CaptureEventKind.KEY_RESULT)
        self.assertEqual(result.action, "VIDEO_TOGGLE")
        self.assertEqual(result.detail, "recording_started")

    def test_busy_flag_reflects_in_flight_camera_work(self) -> None:
        app = FakeApp()
        session = TeleopCameraSession(lambda: app)
        self.addCleanup(session.close)
        session.start()
        collect_event(session, CaptureEventKind.READY)

        self.assertTrue(session.photo())
        self.assertTrue(wait_until(lambda: session.busy))
        app.gate.set()
        self.assertTrue(wait_until(lambda: not session.busy))

    def test_factory_failure_emits_unavailable_and_closes(self) -> None:
        def failing_factory():
            raise CaptureUnavailableError("rescue_camera_capture missing")

        session = TeleopCameraSession(failing_factory)
        session.start()
        event = collect_event(session, CaptureEventKind.UNAVAILABLE)
        self.assertIn("missing", event.detail)
        self.assertTrue(wait_until(lambda: session.closed))
        self.assertFalse(session.photo())
        self.assertFalse(session.video_toggle())
        self.assertTrue(session.close())

    def test_handle_key_exception_is_isolated_as_error_result(self) -> None:
        class ExplodingApp(FakeApp):
            def handle_key(self, key, now=None):
                self.keys.append(key)
                raise RuntimeError("sensor blew up")

        app = ExplodingApp()
        session = TeleopCameraSession(lambda: app)
        self.addCleanup(session.close)
        session.start()
        collect_event(session, CaptureEventKind.READY)

        self.assertTrue(session.photo())
        result = collect_event(session, CaptureEventKind.KEY_RESULT)
        self.assertTrue(result.detail.startswith("error:"))
        self.assertIn("sensor blew up", result.detail)

    def test_poll_failure_reports_error_but_session_survives(self) -> None:
        app = FakeApp(poll_error=RuntimeError("poll glitch"))
        session = TeleopCameraSession(lambda: app)
        self.addCleanup(session.close)
        session.start()
        collect_event(session, CaptureEventKind.READY)
        error = collect_event(session, CaptureEventKind.POLL_ERROR)
        self.assertIn("poll glitch", error.detail)

        self.assertTrue(session.photo())
        self.assertTrue(wait_until(lambda: app.keys == ["p"]))
        app.gate.set()
        collect_event(session, CaptureEventKind.KEY_RESULT)
        self.assertFalse(session.closed)

    def test_close_gracefully_shuts_down_and_reports(self) -> None:
        app = FakeApp()
        session = TeleopCameraSession(lambda: app)
        session.start()
        collect_event(session, CaptureEventKind.READY)

        self.assertTrue(session.close(timeout_s=5.0))
        self.assertEqual(app.shutdown_calls, [("teleop_exit", True)])
        closed = collect_event(session, CaptureEventKind.CLOSED)
        self.assertEqual(closed.session_id, FakeStore.session_id)
        self.assertEqual(closed.detail, "")
        self.assertTrue(session.closed)
        self.assertFalse(session.photo())


class BuildCameraAppTests(unittest.TestCase):
    def test_missing_package_is_unavailable_not_a_crash(self) -> None:
        # WSL test host has no rescue_camera_capture installed: the lazy
        # import must degrade into CaptureUnavailableError.
        try:
            import rescue_camera_capture  # noqa: F401

            self.skipTest("rescue_camera_capture importable on this host")
        except ImportError:
            pass
        with self.assertRaises(CaptureUnavailableError):
            build_camera_app("/nonexistent/map.json", None)


if __name__ == "__main__":
    unittest.main()
