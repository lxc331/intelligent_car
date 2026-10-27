"""Hardware-free checks for bounded vehicle NetworkManager operations."""

from copy import deepcopy
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest


_DIRECTORY = Path(__file__).resolve().parents[2] / "tools" / "vehicle_network"
sys.path.insert(0, str(_DIRECTORY))
try:
    from wifi_recovery_adapter import WifiRecoveryAdapter
finally:
    sys.path.remove(str(_DIRECTORY))


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Network:
    def __init__(self, clock):
        self.clock = clock
        self.active = None
        self.ip = "192.168.137.192"
        self.profiles = {
            "hiwonder-wlan0-client": {"uuid": "client-uuid", "mode": "infrastructure", "ssid": "test-network"},
            "hiwonder-wlan0-ap": {"uuid": "ap-uuid", "mode": "ap", "ssid": "HW-Test"},
        }
        self.calls, self.secret_files = [], []
        self.failure = False
        self.cost = 0.01

    def __call__(self, argv, **kwargs):
        self.calls.append((list(argv), kwargs["timeout"]))
        self.clock.sleep(min(self.cost, kwargs["timeout"]))
        if self.cost > kwargs["timeout"]:
            raise TimeoutError("secret must not reach logs")
        result = types.SimpleNamespace(returncode=0, stdout="", stderr="")
        args = argv[1:]
        if argv[0] == "ip":
            result.stdout = json.dumps([{"addr_info": [{"family": "inet", "local": self.ip}]}])
        elif "GENERAL.CONNECTION" in args:
            result.stdout = self.active or "--"
        elif "NAME,UUID" in args:
            result.stdout = "\n".join(name + ":" + value["uuid"] for name, value in self.profiles.items())
        elif "802-11-wireless.mode,802-11-wireless.ssid" in args:
            profile = self.profiles[args[-1]]
            result.stdout = profile["mode"] + "\n" + profile["ssid"]
        elif args[:2] == ["device", "disconnect"]:
            self.active = None
        elif args[:2] == ["connection", "add"]:
            name = args[args.index("con-name") + 1]
            self.profiles[name] = {"uuid": name + "-uuid", "mode": "infrastructure", "ssid": args[-1]}
        elif args[:2] == ["connection", "modify"]:
            uuid = args[3]
            profile = next(value for value in self.profiles.values() if value["uuid"] == uuid)
            settings = dict(zip(args[4::2], args[5::2]))
            profile["mode"] = settings["802-11-wireless.mode"]
            profile["ssid"] = settings["802-11-wireless.ssid"]
            profile["settings"] = settings
        elif "up" in args:
            if "passwd-file" in args:
                path = Path(args[args.index("passwd-file") + 1])
                self.secret_files.append((str(path), path.read_text(), path.stat().st_mode & 0o777))
            if self.failure:
                result.returncode = 4
                result.stderr = "NM failure with do-not-log-secret"
            else:
                uuid = args[args.index("uuid") + 1]
                self.active = next(name for name, value in self.profiles.items() if value["uuid"] == uuid)
                self.ip = "192.168.149.1" if self.active.endswith("-ap") else "192.168.137.192"
        return result


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.network = Network(self.clock)
        self.temp = tempfile.TemporaryDirectory(dir=_DIRECTORY.parents[1] / ".work")
        self.addCleanup(self.temp.cleanup)
        self.saved = {
            "WIFI_MODE": 2, "WIFI_STA_SSID": "test-network", "WIFI_STA_PASSWORD": "do-not-log-secret",
            "WIFI_AP_SSID": "HW-Test", "WIFI_AP_PASSWORD": "ap-secret",
        }
        self.logs, self.servers = [], []
        owner = self

        class Lock:
            def __init__(self, timeout):
                self.timeout = timeout
            def __enter__(self):
                if owner.on_lock:
                    owner.on_lock()
                return self
            def __exit__(self, *args):
                pass

        self.on_lock = None
        self.api = {
            "select_config_path": lambda master, legacy: master,
            "load_config_values": lambda path: deepcopy(self.saved),
            "external_config_file_path": "/master/wifi_conf.py",
            "internal_config_file_path": "/legacy/wifi_conf.py",
            "WifiOperationLock": Lock,
            "logger": types.SimpleNamespace(info=self.logs.append),
            "set_led": lambda *args: None,
            "start_server": self.start_server,
            "stop_server": lambda old: self.servers.append(("stop", old)),
            "server": None, "ip": None,
        }
        self.adapter = WifiRecoveryAdapter(self.api, self.clock, self.clock.sleep,
                                           self.network, self.temp.name)

    def start_server(self, address, port):
        self.servers.append(("start", address, port))
        return "server-" + address

    def test_reads_master_every_poll_and_updates_preference(self):
        self.assertEqual(self.adapter.read_preference()["mode"], 2)
        self.saved["WIFI_MODE"] = 1
        self.assertEqual(self.adapter.read_preference()["mode"], 1)
        self.assertEqual(self.api["WIFI_MODE"], 1)

    def test_preserves_existing_sta_without_disconnection(self):
        self.network.active = self.adapter.CLIENT
        self.assertTrue(self.adapter.try_sta(self.adapter.read_preference(), 25))
        self.assertFalse(any("disconnect" in argv for argv, _ in self.network.calls))
        self.assertEqual(self.api["current_wifi_mode"], 2)

    def test_sta_secret_file_removed_and_hidden_flag_reset(self):
        self.assertTrue(self.adapter.try_sta(self.adapter.read_preference(), 25))
        path, contents, mode = self.network.secret_files[0]
        self.assertFalse(Path(path).exists())
        self.assertEqual(contents, "802-11-wireless-security.psk:do-not-log-secret\n")
        if os.name != "nt":
            self.assertEqual(mode, 0o600)
        settings = self.network.profiles[self.adapter.CLIENT]["settings"]
        self.assertEqual(settings["802-11-wireless.hidden"], "no")
        self.assertEqual(settings["wifi-sec.psk-flags"], "0")
        self.assertEqual(settings["ipv4.addresses"], "")
        self.assertFalse(any("do-not-log-secret" in line for line in self.logs))

    def test_failure_cleans_secret_and_preserves_saved_sta(self):
        self.network.failure = True
        self.assertFalse(self.adapter.try_sta(self.adapter.read_preference(), 25))
        self.assertFalse(Path(self.network.secret_files[0][0]).exists())
        self.assertEqual(self.saved["WIFI_MODE"], 2)
        self.assertFalse(any("do-not-log-secret" in line for line in self.logs))

    def test_open_network_removes_old_security_and_uses_no_secret_file(self):
        self.saved['WIFI_STA_PASSWORD']=''
        self.assertTrue(self.adapter.try_sta(self.adapter.read_preference(),25))
        settings=self.network.profiles[self.adapter.CLIENT]['settings']
        self.assertEqual(settings['remove'],'wifi-sec')
        self.assertNotIn('wifi-sec.key-mgmt',settings)
        self.assertNotIn('wifi-sec.psk',settings)
        self.assertEqual(self.network.secret_files,[])

    def test_mode_changed_while_waiting_for_lock_prevents_disconnect(self):
        cfg = self.adapter.read_preference()
        self.on_lock = lambda: self.saved.update(WIFI_MODE=1)
        self.assertFalse(self.adapter.try_sta(cfg, 25))
        self.assertEqual(self.network.calls, [])

    def test_password_changed_while_waiting_for_lock_prevents_stale_attempt(self):
        cfg = self.adapter.read_preference()
        self.on_lock = lambda: self.saved.update(WIFI_STA_PASSWORD="new-secret")
        self.assertFalse(self.adapter.try_sta(cfg, 25))
        self.assertEqual(self.network.calls, [])

    def test_ap_cannot_be_mistaken_for_sta(self):
        self.network.active = self.adapter.AP
        self.network.ip = "192.168.149.1"
        self.assertFalse(self.adapter.sta_ready(self.adapter.read_preference()))

    def test_client_profile_with_ap_mode_is_not_ready(self):
        self.network.active = self.adapter.CLIENT
        self.network.profiles[self.adapter.CLIENT]["mode"] = "ap"
        self.assertFalse(self.adapter.sta_ready(self.adapter.read_preference()))

    def test_ip_change_rebinds_server_and_disconnect_clears_it(self):
        self.network.active = self.adapter.CLIENT
        cfg = self.adapter.read_preference()
        self.assertTrue(self.adapter.sta_ready(cfg))
        self.network.ip = "192.168.137.193"
        self.assertTrue(self.adapter.sta_ready(cfg))
        self.assertIn(("stop", "server-192.168.137.192"), self.servers)
        self.network.active = None
        self.assertFalse(self.adapter.sta_ready(cfg))
        self.assertIsNone(self.api["server"])
        self.assertIsNone(self.api["ip"])

    def test_bounded_budget_clamps_every_subprocess_timeout(self):
        self.network.cost = 3
        started = self.clock()
        self.assertFalse(self.adapter.try_sta(self.adapter.read_preference(), 4))
        self.assertLessEqual(self.clock() - started, 4)
        self.assertTrue(all(timeout <= 4 for _, timeout in self.network.calls))

    def test_temporary_ap_does_not_write_sta_preference(self):
        self.assertTrue(self.adapter.start_ap())
        self.assertEqual(self.api["current_wifi_mode"], 1)
        self.assertEqual(self.saved["WIFI_MODE"], 2)

    def test_manual_ap_coordinator_never_attempts_sta(self):
        self.saved["WIFI_MODE"] = 1
        coordinator = self.adapter.coordinator()
        coordinator.step()
        self.assertEqual(coordinator.phase, "manual_ap_ready")
        self.assertEqual(self.network.active, self.adapter.AP)
        self.assertFalse(any("client-uuid" in argv and "up" in argv for argv, _ in self.network.calls))

    def test_coordinator_defaults_and_tunable_parameters(self):
        coordinator = self.adapter.coordinator()
        self.assertEqual((coordinator.startup_budget, coordinator.retry_interval), (120, 60))
        self.saved.update(WIFI_STA_STARTUP_TIMEOUT=15, WIFI_STA_RETRY_INTERVAL=20)
        coordinator = self.adapter.coordinator()
        self.assertEqual((coordinator.startup_budget, coordinator.retry_interval), (15, 20))

    def test_missing_profile_is_created(self):
        del self.network.profiles[self.adapter.CLIENT]
        self.assertTrue(self.adapter.try_sta(self.adapter.read_preference(), 25))


if __name__ == "__main__":
    unittest.main()
