"""
Drone Pad API Service — Robot E-stop Monitor

Follows the robot's physical emergency stop. The E-stop button is wired to a
GPIO line owned by the robot control system (xavier-robot.service), which
serves the debounced state at http://127.0.0.1:8770/estop (the "E-stop hub").
This module polls that endpoint and calls:

    on_engage()   when the E-stop becomes active   -> stop everything
    on_release()  when the E-stop is released      -> reset to ready

Polling (rather than being pushed to) means the pad converges on the right
state by itself after either service restarts.

If the hub cannot be reached the monitor keeps the last known state by
default. With fail_safe=True an unreachable hub is treated as an active
E-stop after `unreachable_grace_s`.
"""

from __future__ import annotations

import json
import logging
import threading
import time
import urllib.request
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class EStopMonitor:
    def __init__(
        self,
        url: str,
        on_engage: Callable[[str], None],
        on_release: Callable[[], None],
        poll_interval_s: float = 0.1,
        timeout_s: float = 0.5,
        fail_safe: bool = False,
        unreachable_grace_s: float = 2.0,
    ) -> None:
        self._url = url
        self._on_engage = on_engage
        self._on_release = on_release
        self._poll_interval = max(0.02, poll_interval_s)
        self._timeout = timeout_s
        self._fail_safe = fail_safe
        self._grace = unreachable_grace_s

        self._lock = threading.Lock()
        self._active = False              # what we last acted on
        self._reason: Optional[str] = None
        self._hub_reachable: Optional[bool] = None   # None = never polled yet
        self._last_ok: Optional[float] = None        # monotonic time of last good poll
        self._last_seq: Optional[int] = None
        self._last_error: Optional[str] = None

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ----- Public API -----

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def status(self) -> dict:
        with self._lock:
            return {
                "active": self._active,
                "reason": self._reason,
                "hub_url": self._url,
                "hub_reachable": self._hub_reachable,
                "hub_last_ok_age_s": (
                    round(time.monotonic() - self._last_ok, 2) if self._last_ok is not None else None
                ),
                "hub_error": self._last_error,
                "fail_safe": self._fail_safe,
            }

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="estop-monitor", daemon=True)
        self._thread.start()
        logger.info(
            "E-stop monitor started — following %s every %.0f ms (fail_safe=%s)",
            self._url, self._poll_interval * 1000, self._fail_safe,
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ----- Internals -----

    def _fetch(self) -> dict:
        with urllib.request.urlopen(self._url, timeout=self._timeout) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        if not isinstance(data.get("active"), bool):
            raise ValueError("response has no boolean 'active' field")
        return data

    def _run(self) -> None:
        started = time.monotonic()
        while not self._stop.is_set():
            try:
                data = self._fetch()
            except Exception as exc:
                self._handle_unreachable(str(exc), started)
            else:
                self._handle_state(data)
            self._stop.wait(self._poll_interval)

    def _handle_state(self, data: dict) -> None:
        hub_active = data["active"]
        seq = data.get("seq")
        with self._lock:
            if self._hub_reachable is False:
                logger.info("E-stop hub reachable again at %s", self._url)
            self._hub_reachable = True
            self._last_ok = time.monotonic()
            self._last_error = None
            prev_seq, self._last_seq = self._last_seq, seq
            was_active = self._active
            # Pressed and released again between two polls: seq moved on by 2+
            # although `active` looks unchanged. Still counts as a stop.
            missed_press = (
                not hub_active and not was_active
                and isinstance(seq, int) and isinstance(prev_seq, int)
                and seq - prev_seq >= 2
            )

        if hub_active and not was_active:
            self._set(True, "Physical E-stop pressed")
        elif missed_press:
            self._set(True, "Physical E-stop pressed (brief)")
            self._set(False, None)
        elif not hub_active and was_active:
            self._set(False, None)

    def _handle_unreachable(self, error: str, started: float) -> None:
        now = time.monotonic()
        with self._lock:
            first_failure = self._hub_reachable is not False
            self._hub_reachable = False
            self._last_error = error
            reference = self._last_ok if self._last_ok is not None else started
            overdue = now - reference > self._grace
            was_active = self._active
        if first_failure:
            logger.warning(
                "E-stop hub unreachable at %s (%s) — %s",
                self._url, error,
                "will treat as E-STOP (fail-safe)" if self._fail_safe else "holding last known state",
            )
        if self._fail_safe and overdue and not was_active:
            self._set(True, "E-stop hub unreachable (fail-safe)")

    def _set(self, active: bool, reason: Optional[str]) -> None:
        with self._lock:
            self._active = active
            self._reason = reason
        try:
            if active:
                logger.critical("ROBOT E-STOP ENGAGED — %s", reason)
                self._on_engage(reason or "E-stop")
            else:
                logger.warning("ROBOT E-STOP RELEASED — resetting")
                self._on_release()
        except Exception:
            logger.exception("E-stop %s handler failed", "engage" if active else "release")
