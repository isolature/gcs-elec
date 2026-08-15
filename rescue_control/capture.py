"""Bridge between the teleop session and the rescue_camera_capture app.

Teleop owns the STM32 serial link and the 20 ms control loop; camera work
must never block either.  This module runs one dedicated worker thread that
exclusively owns the ``CaptureApplication`` from the ``rescue_camera_capture``
subproject (lazily imported on the Pi), exactly mirroring that project's own
single-owner threading model:

    teleop loop --P/V keys--> action queue --> worker thread
    worker thread: handle_key(...) / poll() / shutdown(...)
    worker thread --mailbox--> teleop loop (NDJSON events)

Nothing here duplicates capture logic: photos, dual-camera recording, state
transitions (photos rejected while recording), rollback, session storage and
inventory all remain in ``rescue_camera_capture``.  Transfer/回传 is also out
of scope for teleop; after exit the existing session inventory and transfer
pipeline handles it.  Cameras are exclusive: while a teleop camera session is
active, the interactive ``rescue_camera_capture`` CLI must not be run.
"""

from __future__ import annotations

import queue
import threading
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Protocol


class CaptureAction(str, Enum):
    PHOTO = "PHOTO"
    VIDEO_TOGGLE = "VIDEO_TOGGLE"
    CLOSE = "CLOSE"


class CaptureEventKind(str, Enum):
    READY = "READY"
    KEY_RESULT = "KEY_RESULT"
    POLL_ERROR = "POLL_ERROR"
    CLOSED = "CLOSED"
    UNAVAILABLE = "UNAVAILABLE"


@dataclass(frozen=True)
class CaptureEvent:
    """Worker -> control loop notification; emitted as NDJSON by the loop."""

    kind: CaptureEventKind
    detail: str = ""
    action: str = ""
    busy: bool = False
    session_id: str | None = None
    session_dir: str | None = None


class CaptureApp(Protocol):
    """Structural subset of rescue_camera_capture.CaptureApplication."""

    def initialize(self) -> None: ...

    def handle_key(self, key: str, now: float | None = None) -> str: ...

    def poll(self) -> None: ...

    def shutdown(self, reason: str, graceful: bool) -> list[str]: ...


class CaptureUnavailableError(RuntimeError):
    """The camera subproject is missing or misconfigured on this host."""


def build_camera_app(camera_map_path: str, output_root: str | None) -> CaptureApp:
    """Lazily import the capture subproject; only called on the Pi.

    Teleop must be started with an interpreter that can import
    ``rescue_camera_capture`` (for example the ~/gcs-camera-capture venv with
    ``--system-site-packages``).  Import/config errors surface as
    ``CaptureUnavailableError`` so the control loop can degrade cleanly.
    """

    try:
        from rescue_camera_capture.app import CaptureApplication
        from rescue_camera_capture.config import load_camera_expectations
        from rescue_camera_capture.models import CaptureSettings
        from rescue_camera_capture.picamera2_backend import Picamera2Factory
        from rescue_camera_capture.storage import SessionStore
    except Exception as exc:  # ImportError or missing Pi dependencies
        raise CaptureUnavailableError(
            f"rescue_camera_capture is not importable: {exc}"
        ) from exc

    try:
        expectations = load_camera_expectations(camera_map_path)
    except Exception as exc:
        raise CaptureUnavailableError(
            f"camera map {camera_map_path} could not be loaded: {exc}"
        ) from exc

    store_kwargs: dict[str, Any] = {}
    if output_root is not None:
        store_kwargs["output_root"] = output_root
    return CaptureApplication(
        camera_factory=Picamera2Factory(),
        expectations=expectations,
        store=SessionStore(**store_kwargs),
        settings=CaptureSettings(),
    )


class TeleopCameraSession:
    """Single worker thread owning the camera app; loop-safe facade.

    All public methods are non-blocking and safe to call from the control
    loop.  Events flow back through :meth:`drain_events` on the loop thread,
    so NDJSON emission stays single-threaded.
    """

    POLL_INTERVAL_S = 0.2

    def __init__(
        self,
        app_factory: Callable[[], CaptureApp],
        *,
        close_reason: str = "teleop_exit",
    ) -> None:
        self._app_factory = app_factory
        self._close_reason = close_reason
        self._actions: "queue.Queue[CaptureAction]" = queue.Queue()
        self._events: "queue.Queue[CaptureEvent]" = queue.Queue()
        self._closed = threading.Event()
        self._busy = False
        self._thread = threading.Thread(
            target=self._run, name="teleop-camera", daemon=True
        )
        self._started = False

    # -- control-loop API (non-blocking) ---------------------------------

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._thread.start()

    def photo(self) -> bool:
        return self._submit(CaptureAction.PHOTO)

    def video_toggle(self) -> bool:
        return self._submit(CaptureAction.VIDEO_TOGGLE)

    def close(self, timeout_s: float = 10.0) -> bool:
        """Request graceful shutdown; True when the worker joined in time."""
        if not self._started:
            return True
        if not self._closed.is_set():
            self._actions.put(CaptureAction.CLOSE)
        self._thread.join(timeout=timeout_s)
        return not self._thread.is_alive()

    def drain_events(self) -> list[CaptureEvent]:
        events: list[CaptureEvent] = []
        while True:
            try:
                events.append(self._events.get_nowait())
            except queue.Empty:
                return events

    @property
    def events_pending(self) -> bool:
        """Non-destructive check used by tests and loop backoff logic."""
        return not self._events.empty()

    @property
    def busy(self) -> bool:
        return self._busy

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    # -- worker side ------------------------------------------------------

    def _submit(self, action: CaptureAction) -> bool:
        if not self._started or self._closed.is_set():
            return False
        try:
            self._actions.put_nowait(action)
        except queue.Full:
            return False
        return True

    def _emit(self, event: CaptureEvent) -> None:
        self._events.put(event)

    def _run(self) -> None:
        app: CaptureApp | None = None
        try:
            app = self._app_factory()
            app.initialize()
        except Exception as exc:
            detail = str(exc) or type(exc).__name__
            self._emit(
                CaptureEvent(kind=CaptureEventKind.UNAVAILABLE, detail=detail)
            )
            app = None

        if app is not None:
            store = getattr(app, "store", None)
            directory = getattr(store, "session_dir", None)
            self._emit(
                CaptureEvent(
                    kind=CaptureEventKind.READY,
                    session_id=getattr(store, "session_id", None),
                    session_dir=(
                        str(directory) if directory is not None else None
                    ),
                )
            )

        while app is not None:
            try:
                action = self._actions.get(timeout=self.POLL_INTERVAL_S)
            except queue.Empty:
                action = None
            if action is CaptureAction.CLOSE:
                break
            if action is not None:
                key = {
                    CaptureAction.PHOTO: "p",
                    CaptureAction.VIDEO_TOGGLE: "v",
                }[action]
                self._busy = True
                try:
                    result = app.handle_key(key)
                except Exception as exc:
                    result = f"error: {type(exc).__name__}: {exc}"
                finally:
                    self._busy = False
                self._emit(
                    CaptureEvent(
                        kind=CaptureEventKind.KEY_RESULT,
                        action=action.value,
                        detail=result,
                    )
                )
            try:
                app.poll()
            except Exception as exc:
                self._emit(
                    CaptureEvent(
                        kind=CaptureEventKind.POLL_ERROR,
                        detail=f"{type(exc).__name__}: {exc}",
                    )
                )

        if app is not None:
            errors: list[str] = []
            try:
                errors = app.shutdown(
                    reason=self._close_reason, graceful=True
                )
            except Exception as exc:
                errors.append(f"{type(exc).__name__}: {exc}")
            store = getattr(app, "store", None)
            directory = getattr(store, "session_dir", None)
            self._emit(
                CaptureEvent(
                    kind=CaptureEventKind.CLOSED,
                    detail=";".join(errors),
                    session_id=getattr(store, "session_id", None),
                    session_dir=(
                        str(directory) if directory is not None else None
                    ),
                )
            )
        self._closed.set()
