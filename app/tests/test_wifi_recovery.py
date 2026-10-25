"""Offline checks for the vehicle's persistent STA preference/recovery policy."""

from copy import deepcopy
import importlib.util
from pathlib import Path
import unittest


_PATH = Path(__file__).resolve().parents[2] / "tools" / "vehicle_network" / "wifi_recovery.py"
_SPEC = importlib.util.spec_from_file_location("vehicle_wifi_recovery", _PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)
RecoveryCoordinator = _MODULE.RecoveryCoordinator


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.config = {"mode": 2, "ssid": "test-network", "password": "do-not-log"}
        self.calls = []
        self.messages = []
        self.connected = False
        self.visible = True
        self.ap_ok = True

    def attempt(self, config, timeout):
        self.calls.append(("sta", config["ssid"], timeout))
        self.clock.sleep(timeout)
        return self.connected

    def ap(self):
        self.calls.append(("ap",))
        return self.ap_ok

    def coordinator(self, **kwargs):
        arguments = dict(
            read_preference=lambda: self.config,
            try_sta=self.attempt, start_ap=self.ap,
            sta_ready=lambda config: self.connected,
            clock=self.clock, sleep=self.clock.sleep, log=self.messages.append,
            startup_budget=20, attempt_timeout=8, retry_interval=60, check_interval=2,
        )
        arguments.update(kwargs)
        return RecoveryCoordinator(**arguments)

    def tick(self, policy):
        delay = policy.step()
        if delay is not None:
            self.assertGreater(delay, 0)
            self.assertLessEqual(delay, policy.check_interval)
            self.clock.sleep(delay)
        return delay

    def fallback(self, policy):
        for _ in range(20):
            self.tick(policy)
            if policy.phase == "fallback_ap":
                return
        self.fail("STA did not fall back within the bounded attempt budget")

    def test_manual_ap_starts_once_and_never_attempts_sta(self):
        self.config["mode"] = 1
        policy = self.coordinator()
        for _ in range(100):
            self.tick(policy)
        self.assertEqual(self.calls, [("ap",)])
        self.assertEqual(policy.status()["runtime_mode"], "ap")

    def test_failed_startup_retains_preference_and_limits_total_budget(self):
        before = deepcopy(self.config)
        policy = self.coordinator()
        self.fallback(policy)
        self.assertEqual(self.config, before)
        self.assertEqual(self.calls, [("sta", "test-network", 8.0),
                                     ("sta", "test-network", 8.0), ("ap",)])
        self.assertEqual(policy.preferred_mode, 2)
        self.assertEqual(policy.runtime_mode, "ap")
        self.assertTrue(any("preference unchanged" in entry for entry in self.messages))

    def test_last_attempt_is_clamped_to_remaining_budget(self):
        policy = self.coordinator(startup_budget=13)
        self.fallback(policy)
        self.assertEqual([call[2] for call in self.calls if call[0] == "sta"], [8, 3])
        self.assertEqual(self.calls[-1], ("ap",))

    def test_ap_retry_waits_then_restores_ap_after_one_failed_attempt(self):
        policy = self.coordinator()
        self.fallback(policy)
        before = list(self.calls)
        retry_at = policy._next_at
        while self.clock() < retry_at:
            self.tick(policy)
        self.assertEqual(self.calls, before)
        self.tick(policy)
        self.assertEqual(self.calls[-2:], [("sta", "test-network", 8.0), ("ap",)])
        self.assertEqual(policy.phase, "fallback_ap")
        self.assertGreaterEqual(policy._next_at, self.clock() + 58)

    def test_absent_target_does_not_disrupt_rescue_ap(self):
        policy = self.coordinator(target_visible=lambda config: self.visible)
        self.fallback(policy)
        self.visible = False
        self.clock.now = policy._next_at
        before = list(self.calls)
        self.tick(policy)
        self.assertEqual(self.calls, before)
        self.assertEqual(policy.runtime_mode, "ap")
        self.assertEqual(policy.phase, "fallback_ap")

    def test_target_returns_and_sta_becomes_online(self):
        policy = self.coordinator(target_visible=lambda config: self.visible)
        self.fallback(policy)
        self.connected = True
        self.clock.now = policy._next_at
        self.tick(policy)
        self.assertEqual(policy.runtime_mode, "sta")
        self.assertEqual(policy.phase, "online")
        self.assertEqual(self.calls[-1][0], "sta")

    def test_unavailable_scan_still_attempts_sta(self):
        for scan in (lambda config: None, lambda config: 1 / 0):
            with self.subTest(scan=scan):
                policy = self.coordinator(target_visible=scan)
                self.connected = False
                self.fallback(policy)
                self.connected = True
                self.clock.now = policy._next_at
                self.tick(policy)
                self.assertEqual(policy.phase, "online")

    def test_mode_change_during_scan_cancels_sta_attempt(self):
        def scan(config):
            self.config["mode"] = 1
            return True
        policy = self.coordinator(target_visible=scan)
        self.fallback(policy)
        before = list(self.calls)
        self.clock.now = policy._next_at
        self.tick(policy)
        self.assertEqual(self.calls, before)
        self.tick(policy)
        self.assertEqual(policy.phase, "manual_ap_ready")

    def test_sta_loss_gets_bounded_recovery_before_fallback(self):
        self.connected = True
        policy = self.coordinator()
        self.tick(policy)
        self.assertEqual(policy.phase, "online")
        self.connected = False
        self.tick(policy)
        self.assertEqual(policy.phase, "connecting")
        self.assertNotIn(("ap",), self.calls)
        self.fallback(policy)
        self.assertEqual(self.calls[-1], ("ap",))

    def test_manual_ap_change_cancels_pending_long_retry(self):
        policy = self.coordinator()
        self.fallback(policy)
        self.config["mode"] = 1
        self.tick(policy)
        count = len(self.calls)
        for _ in range(100):
            self.tick(policy)
        self.assertEqual(len(self.calls), count)
        self.assertEqual(policy.preferred_mode, 1)
        self.assertEqual(policy.phase, "manual_ap_ready")

    def test_mode_change_during_sta_attempt_ignores_stale_success(self):
        def attempt(config, timeout):
            self.calls.append(("sta", config["ssid"], timeout))
            self.config["mode"] = 1
            return True
        policy = self.coordinator(try_sta=attempt)
        self.tick(policy)
        self.assertNotEqual(policy.phase, "online")
        self.tick(policy)
        self.assertEqual(policy.phase, "manual_ap_ready")
        self.assertEqual(self.calls[-1], ("ap",))

    def test_target_change_during_retry_ignores_stale_failure(self):
        policy = self.coordinator()
        self.fallback(policy)
        def attempt(config, timeout):
            self.calls.append(("sta", config["ssid"], timeout))
            self.config["ssid"] = "replacement"
            return False
        policy.try_sta = attempt
        self.clock.now = policy._next_at
        ap_count = self.calls.count(("ap",))
        self.tick(policy)
        self.assertEqual(policy.phase, "connecting")
        self.assertEqual(policy.status()["ssid"], "replacement")
        self.assertEqual(self.calls.count(("ap",)), ap_count)
        self.connected = True
        policy.try_sta = self.attempt
        self.tick(policy)
        self.assertEqual(self.calls[-1][1], "replacement")
        self.assertEqual(policy.phase, "online")

    def test_new_credentials_restart_attempts_without_waiting_for_retry(self):
        policy = self.coordinator()
        self.fallback(policy)
        self.config["password"] = "updated-secret"
        self.connected = True
        self.tick(policy)
        self.assertEqual(policy.phase, "online")
        self.assertNotIn("password", policy.status())
        self.assertFalse(any("secret" in message for message in self.messages))

    def test_callbacks_cannot_mutate_saved_or_internal_preference(self):
        before = deepcopy(self.config)
        def attempt(config, timeout):
            config["ssid"] = "mutated"
            return True
        policy = self.coordinator(try_sta=attempt)
        self.tick(policy)
        self.assertEqual(self.config, before)
        self.assertEqual(policy.status()["ssid"], before["ssid"])

    def test_broken_callbacks_are_rate_limited_and_secrets_not_logged(self):
        def broken(config, timeout):
            self.calls.append(("sta",))
            raise RuntimeError("do-not-log")
        policy = self.coordinator(try_sta=broken, startup_budget=4)
        self.fallback(policy)
        self.assertEqual(self.calls, [("sta",), ("sta",), ("ap",)])
        before = len(self.calls)
        for _ in range(20):
            self.tick(policy)
        self.assertEqual(len(self.calls), before)
        self.assertFalse(any("do-not-log" in message for message in self.messages))

    def test_preference_read_error_freezes_network_operations(self):
        def unreadable():
            raise OSError("bad config")
        policy = self.coordinator(read_preference=unreadable)
        for _ in range(10):
            self.tick(policy)
        self.assertEqual(self.calls, [])
        self.assertEqual(self.clock(), 20)

    def test_ap_failure_is_retried_without_busy_loop(self):
        self.config["mode"] = 1
        self.ap_ok = False
        policy = self.coordinator()
        for _ in range(30):
            self.tick(policy)
        self.assertEqual(self.calls, [("ap",)])
        self.ap_ok = True
        self.tick(policy)
        self.assertEqual(self.calls, [("ap",), ("ap",)])
        self.assertEqual(policy.phase, "manual_ap_ready")

    def test_stop_during_attempt_prevents_fallback_mutation(self):
        stopped = False
        def attempt(config, timeout):
            nonlocal stopped
            stopped = True
            return False
        policy = self.coordinator(try_sta=attempt, should_stop=lambda: stopped)
        self.tick(policy)
        self.assertEqual(self.calls, [])
        self.assertIsNone(policy.step())

    def test_run_checks_stop_between_short_waits(self):
        self.config["mode"] = 1
        policy = self.coordinator(should_stop=lambda: self.clock() >= 6)
        policy.run()
        self.assertEqual(self.clock(), 6)
        self.assertEqual(self.calls, [("ap",)])

    def test_timing_arguments_cannot_create_busy_loops(self):
        for name, value in [("retry_interval", 0), ("check_interval", -1),
                            ("attempt_timeout", float("nan")),
                            ("startup_budget", float("inf"))]:
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.coordinator(**{name: value})


if __name__ == "__main__":
    unittest.main()
