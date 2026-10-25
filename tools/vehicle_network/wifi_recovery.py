"""Wi-Fi recovery policy, independent of NetworkManager and vehicle hardware.

Only the adapter reads user preferences or touches the network.  In particular,
temporary AP fallback never changes the saved preferred mode.  ``step()`` is
useful for deterministic tests; ``run()`` supplies the ordinary service loop.
"""

from copy import deepcopy
import math
import time


class RecoveryCoordinator:
    """Coordinate manual AP, preferred STA, and temporary rescue AP.

    Required callbacks are ``read_preference() -> dict`` (``mode`` 1 or 2 and
    ``ssid``), ``try_sta(config, timeout) -> bool``, ``start_ap() -> bool`` and
    ``sta_ready(config) -> bool``. Extra preference keys, such as a password,
    are passed through without logging. Any preference change starts a fresh
    policy cycle. Callbacks receive copies, and this module never writes files.

    ``try_sta`` MUST return within the supplied timeout, include all activation
    waits in that budget, and perform no unbounded internal retry loop. Other
    callbacks must also be bounded. Changes to mode/target are re-read before
    and after network operations, so they take effect within one bounded
    operation plus ``check_interval``. ``target_visible(config)`` is an optional
    scan-only check used before disrupting a running rescue AP. Return False
    only when a successful scan confirms absence; None means scan unavailable
    and still permits an attempt (important for single-radio AP adapters).

    Initial connection and recovery after STA loss get ``startup_budget``
    seconds. After fallback, only one ``attempt_timeout`` attempt is made per
    ``retry_interval``; failure restores AP immediately. Manual AP never makes
    STA attempts. A failed AP activation is retried at ``retry_interval``.
    """

    def __init__(
        self, read_preference, try_sta, start_ap, sta_ready, *,
        target_visible=None, log=None, clock=time.monotonic, sleep=time.sleep,
        should_stop=None, startup_budget=120.0, retry_interval=60.0,
        attempt_timeout=25.0, check_interval=2.0,
    ):
        for name, value in (
            ("startup_budget", startup_budget), ("retry_interval", retry_interval),
            ("attempt_timeout", attempt_timeout), ("check_interval", check_interval),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be positive and finite")
        self.read_preference = read_preference
        self.try_sta = try_sta
        self.start_ap = start_ap
        self.sta_ready = sta_ready
        self.target_visible = target_visible
        self.log = log or (lambda message: None)
        self.clock = clock
        self.sleep = sleep
        self.should_stop = should_stop or (lambda: False)
        self.startup_budget = float(startup_budget)
        self.retry_interval = float(retry_interval)
        self.attempt_timeout = float(attempt_timeout)
        self.check_interval = float(check_interval)
        self.preferred_mode = None
        self.runtime_mode = "unknown"
        self.phase = "idle"
        self._config = None
        self._next_at = 0.0
        self._deadline = None

    def status(self):
        """Return diagnostic state without exposing the Wi-Fi password."""
        return {
            "preferred_mode": self.preferred_mode,
            "runtime_mode": self.runtime_mode,
            "phase": self.phase,
            "ssid": self._config.get("ssid", "") if self._config else "",
        }

    def _log(self, message):
        try:
            self.log(message)
        except Exception:
            # A broken log destination must not interrupt recovery.
            pass

    def _read(self):
        config = deepcopy(self.read_preference())
        if not isinstance(config, dict) or config.get("mode") not in (1, 2):
            raise ValueError("preferred mode must be 1 (AP) or 2 (STA)")
        if config["mode"] == 2 and not isinstance(config.get("ssid"), str):
            raise ValueError("STA preference requires an SSID string")
        if config["mode"] == 2 and not config["ssid"]:
            raise ValueError("STA preference requires a nonempty SSID")
        return config

    def _refresh(self):
        """Return (read_ok, changed); unreadable preference freezes mutations."""
        try:
            config = self._read()
        except Exception as error:
            self._log("Wi-Fi preference read failed (%s); retrying" % type(error).__name__)
            return False, False
        if config == self._config:
            return True, False
        self._config = config
        self.preferred_mode = config["mode"]
        self._next_at = self.clock()
        if self.preferred_mode == 1:
            self.phase = "manual_ap"
            self._deadline = None
            self._log("Preferred mode AP; automatic STA retries disabled")
        else:
            self.phase = "connecting"
            self._deadline = self.clock() + self.startup_budget
            self._log("Preferred mode STA; starting bounded connection recovery")
        return True, True

    def _call(self, name, callback, *args):
        try:
            return bool(callback(*args))
        except Exception as error:
            # Do not include callback exception text: external tools can put
            # passwords or entire connection commands in their error messages.
            self._log("%s failed (%s)" % (name, type(error).__name__))
            return False

    def _after_operation(self):
        if self.should_stop():
            return False
        ok, changed = self._refresh()
        return ok and not changed

    def _activate_ap(self, temporary):
        success = self._call("AP activation", self.start_ap)
        self.runtime_mode = "ap" if success else "unavailable"
        if not self._after_operation():
            return
        self._next_at = self.clock() + self.retry_interval
        if temporary:
            self.phase = "fallback_ap"
            self._log(
                "Preferred mode STA; runtime %s (temporary fallback; preference unchanged)"
                % ("AP" if success else "unavailable")
            )
        elif success:
            self.phase = "manual_ap_ready"
            self._log("Preferred mode AP; runtime AP")
        else:
            self.phase = "manual_ap"
            self._log("Preferred mode AP; activation failed; retry scheduled")

    def _try_sta(self, timeout, from_ap=False):
        self.runtime_mode = "unknown"
        success = self._call(
            "STA activation", self.try_sta, deepcopy(self._config), timeout,
        )
        if not self._after_operation():
            return
        if success:
            self.runtime_mode = "sta"
            self.phase = "online"
            self._deadline = None
            self._next_at = self.clock() + self.check_interval
            self._log("Preferred mode STA; runtime STA")
        elif from_ap or self.clock() >= self._deadline:
            self._activate_ap(temporary=True)
        else:
            self._next_at = self.clock() + self.check_interval

    def _delay(self):
        remaining = max(0.0, self._next_at - self.clock())
        # Every completed operation yields, even when it consumed its deadline.
        return min(self.check_interval, remaining) if remaining else self.check_interval

    def step(self):
        """Perform a bounded unit of work; return sleep seconds or None to stop.

        Callers must honor the returned delay. The preference is polled even
        while a much later retry is scheduled. At most one STA activation is
        attempted per call; restoring AP after failure can happen in that call.
        """
        if self.should_stop():
            return None
        ok, _ = self._refresh()
        if not ok:
            return self.check_interval
        if self.clock() < self._next_at:
            return self._delay()
        if self.preferred_mode == 1:
            if self.phase != "manual_ap_ready":
                self._activate_ap(temporary=False)
            return self.check_interval
        if self.phase == "online":
            ready = self._call("STA status", self.sta_ready, deepcopy(self._config))
            if not self._after_operation():
                return self.check_interval
            if not ready:
                self.runtime_mode = "unavailable"
                self.phase = "connecting"
                self._deadline = self.clock() + self.startup_budget
                self._log("STA connection lost; starting bounded recovery")
            self._next_at = self.clock() + self.check_interval
        elif self.phase == "fallback_ap":
            if self.target_visible is not None:
                try:
                    visible = self.target_visible(deepcopy(self._config))
                except Exception as error:
                    self._log("STA visibility scan unavailable (%s)" % type(error).__name__)
                    visible = None
                if not self._after_operation():
                    return self.check_interval
                if visible is False:
                    if self.runtime_mode != "ap":
                        self._activate_ap(temporary=True)
                    else:
                        self._next_at = self.clock() + self.retry_interval
                        self._log("Preferred STA target absent; preserving temporary AP")
                    return self._delay()
            self._try_sta(self.attempt_timeout, from_ap=True)
        else:
            remaining = self._deadline - self.clock()
            if remaining <= 0:
                self._activate_ap(temporary=True)
            else:
                self._try_sta(min(self.attempt_timeout, remaining))
        return self._delay()

    def run(self):
        """Run until should_stop() is true, polling preferences between waits."""
        while True:
            delay = self.step()
            if delay is None:
                return
            self.sleep(delay)
