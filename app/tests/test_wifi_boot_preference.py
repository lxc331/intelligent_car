"""Read saved Wi-Fi preferences through fresh service startup, without GPIO."""

import ast
from copy import deepcopy
from pathlib import Path
import runpy
import sys
import tempfile
import unittest
from unittest.mock import Mock


_ROOT = Path(__file__).resolve().parents[2]
_NETWORK = _ROOT / "tools" / "vehicle_network"
_SERVICE = _NETWORK / "wifi_service.py"
sys.path.insert(0, str(_NETWORK))
try:
    from wifi_recovery import RecoveryCoordinator
    from wifi_recovery_adapter import WifiRecoveryAdapter
finally:
    sys.path.remove(str(_NETWORK))


def initialize_service(master_path, legacy_path):
    """Execute the real entry's initialization, stopping before its GPIO thread."""
    tree = ast.parse(_SERVICE.read_text(encoding="utf-8"))
    main = next(node for node in tree.body if isinstance(node, ast.If)
                and "__main__" in ast.unparse(node.test))
    initialization = []
    for statement in main.body:
        if (isinstance(statement, ast.Assign)
                and any(isinstance(target, ast.Name) and target.id == "wifi_thread"
                        for target in statement.targets)):
            break
        initialization.append(statement)
    else:
        raise AssertionError("Service thread boundary missing")

    def select_config(master, legacy):
        if Path(master).is_file():
            return Path(master)
        if Path(legacy).is_file():
            return Path(legacy)
        return None

    def load_config(path):
        return {key: value for key, value in runpy.run_path(str(path)).items()
                if key.startswith("WIFI_")}

    namespace = {
        "get_cpu_serial_number": lambda: "10CCA58E12345678",
        "normalize_generated_ap_ssid": lambda saved, default: saved or default,
        "external_config_file_path": str(master_path),
        "internal_config_file_path": str(legacy_path),
        "select_config_path": select_config,
        "load_config_values": load_config,
        "logger": Mock(),
    }
    update = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                  and node.name == "update_globals_from_file")
    code = ast.Module(body=[update, *initialization], type_ignores=[])
    exec(compile(code, str(_SERVICE), "exec"), namespace)
    return namespace


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class WifiBootPreferenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=_ROOT / ".work")
        self.addCleanup(self.temp.cleanup)
        self.master = Path(self.temp.name) / "wifi_conf.py"
        self.legacy = Path(self.temp.name) / "legacy_wifi_conf.py"
        self.legacy.write_text(
            "WIFI_MODE = 1\nWIFI_STA_SSID = 'old-network'\n"
            "WIFI_STA_PASSWORD = 'old-password'\n", encoding="utf-8")
        self.save_target("new-network", "new-password")

    def save_target(self, ssid, password):
        self.master.write_text(
            "WIFI_MODE = 2\nWIFI_AP_SSID = 'HW-10CCA58E'\n"
            "WIFI_STA_SSID = %r\nWIFI_STA_PASSWORD = %r\n"
            "WIFI_STA_STARTUP_TIMEOUT = 120\nWIFI_STA_RETRY_INTERVAL = 60\n"
            "WIFI_LED = False\n" % (ssid, password), encoding="utf-8")

    def test_fresh_service_initialization_prefers_saved_master_over_defaults_and_legacy(self):
        namespace = initialize_service(self.master, self.legacy)
        self.assertEqual(Path(namespace["config_path"]), self.master)
        self.assertEqual(namespace["WIFI_MODE"], 2)
        self.assertEqual(namespace["WIFI_STA_SSID"], "new-network")
        self.assertEqual(namespace["WIFI_STA_PASSWORD"], "new-password")
        self.assertEqual(namespace["WIFI_AP_SSID"], "HW-10CCA58E")
        self.assertFalse(namespace["WIFI_LED"])
        self.assertNotIn("wifi_thread", namespace)

    def test_new_service_instance_rereads_replaced_ssid_and_password(self):
        old_namespace = initialize_service(self.master, self.legacy)
        self.save_target("replacement-network", "replacement-password")
        new_namespace = initialize_service(self.master, self.legacy)
        self.assertEqual(old_namespace["WIFI_STA_SSID"], "new-network")
        self.assertEqual(new_namespace["WIFI_MODE"], 2)
        self.assertEqual(new_namespace["WIFI_STA_SSID"], "replacement-network")
        self.assertEqual(new_namespace["WIFI_STA_PASSWORD"], "replacement-password")
        preference = WifiRecoveryAdapter(new_namespace).read_preference()
        self.assertEqual((preference["mode"], preference["ssid"], preference["password"]),
                         (2, "replacement-network", "replacement-password"))

    def test_restart_after_temporary_ap_still_attempts_persisted_sta_target_first(self):
        before = self.master.read_bytes()
        clock = Clock()
        namespace = initialize_service(self.master, self.legacy)
        adapter = WifiRecoveryAdapter(namespace, clock=clock, sleep=clock.sleep)
        first_attempts, ap_calls = [], []

        def fail_attempt(cfg, timeout):
            first_attempts.append(deepcopy(cfg))
            clock.sleep(timeout)
            return False

        coordinator = RecoveryCoordinator(
            adapter.read_preference, fail_attempt,
            lambda: ap_calls.append(True) or True, lambda cfg: False,
            clock=clock, sleep=clock.sleep, startup_budget=1,
        )
        coordinator.step()
        self.assertEqual(coordinator.phase, "fallback_ap")
        self.assertEqual(coordinator.runtime_mode, "ap")
        self.assertEqual(len(ap_calls), 1)
        self.assertEqual(first_attempts[0]["ssid"], "new-network")
        self.assertEqual(self.master.read_bytes(), before)

        # Discard every runtime object, just as restarting the vehicle service
        # does. Its first request must still target the saved STA preference.
        fresh_namespace = initialize_service(self.master, self.legacy)
        fresh_adapter = WifiRecoveryAdapter(fresh_namespace, clock=clock, sleep=clock.sleep)
        next_attempts, unexpected_ap = [], Mock(return_value=True)
        fresh_coordinator = RecoveryCoordinator(
            fresh_adapter.read_preference,
            lambda cfg, timeout: next_attempts.append(deepcopy(cfg)) or True,
            unexpected_ap, lambda cfg: True, clock=clock, sleep=clock.sleep,
        )
        fresh_coordinator.step()
        self.assertEqual(fresh_coordinator.phase, "online")
        self.assertEqual(next_attempts[0]["mode"], 2)
        self.assertEqual(next_attempts[0]["ssid"], "new-network")
        self.assertEqual(next_attempts[0]["password"], "new-password")
        unexpected_ap.assert_not_called()
        self.assertEqual(self.master.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
