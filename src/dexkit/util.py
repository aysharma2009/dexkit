"""Small shared helpers: logging setup and a fixed-rate loop timer."""

from __future__ import annotations

import logging
import math
import queue
import sys
import threading
import time
from collections.abc import Callable


def windows_read_key(timeout: float) -> bytes | None:
    """Read a console key without termios/select (Windows only)."""
    import msvcrt

    deadline = time.monotonic() + timeout
    while True:
        if msvcrt.kbhit():
            return msvcrt.getch()
        if time.monotonic() >= deadline:
            return None
        time.sleep(min(.01, max(0, deadline - time.monotonic())))


def guarded_wait(duration: float, estop, hand=None) -> None:
    """Wait without suspending stop processing or powered-hand monitoring."""
    if not duration >= 0 or not math.isfinite(duration):
        raise ValueError("wait duration must be finite and nonnegative")
    deadline = time.monotonic() + duration
    while True:
        estop.check()
        if hand is not None and hasattr(hand, "service_safety"):
            hand.service_safety()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(0.05, remaining))


def guarded_input(prompt: str, read, estop, hand=None) -> str:
    """Keep the main thread monitoring hardware while the operator types a line."""
    replies = queue.Queue()

    def reader():
        try:
            replies.put((True, read(prompt)))
        except BaseException as exc:
            replies.put((False, exc))

    threading.Thread(target=reader, daemon=True, name="operator-input").start()
    while True:
        estop.check()
        if hand is not None and hasattr(hand, "service_safety"):
            hand.service_safety()
        try:
            ok, result = replies.get(timeout=0.05)
        except queue.Empty:
            continue
        if not ok:
            raise result
        return result


def setup_logging(verbose: bool = False) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    if not verbose:
        sys.excepthook = _friendly_excepthook


def _friendly_excepthook(etype: type[BaseException], value: BaseException, tb: object) -> None:
    """One readable line instead of a traceback for expected hardware/config problems (-v shows all)."""
    if issubclass(etype, KeyboardInterrupt):
        print("\nstopped (Ctrl+C)", file=sys.stderr)
        return
    if issubclass(etype, (OSError, RuntimeError, ValueError, TimeoutError)):
        print(f"\nerror: {value}\n(run the same command with -v for full details)", file=sys.stderr)
        return
    sys.__excepthook__(etype, value, tb)  # type: ignore[arg-type]


class Rate:
    """Sleep so that successive `sleep()` calls are spaced 1/hz apart."""

    def __init__(
        self,
        hz: float,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if hz <= 0:
            raise ValueError("rate must be positive")
        self.period = 1.0 / hz
        self._clock = clock
        self._sleep = sleep
        self._next = clock() + self.period
        self.last_overrun = 0.0

    def sleep(self) -> None:
        now = self._clock()
        remaining = self._next - now
        if remaining > 0:
            self._sleep(remaining)
            self.last_overrun = 0.0
        else:
            self.last_overrun = -remaining
        self._next = max(self._next + self.period, self._clock())
