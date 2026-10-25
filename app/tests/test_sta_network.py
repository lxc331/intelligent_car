"""STA network selection and diagnostics; every system/network read is mocked."""
import copy
import subprocess
import unittest
from unittest.mock import patch

from carapp.config import DEFAULTS
from carapp.core.connection import diagnose, search_prefixes


def address(alias, ip, prefix=24):
    return {"InterfaceAlias": alias, "IPAddress": ip, "PrefixLength": prefix}


class STASearchTests(unittest.TestCase):
    def setUp(self):
        self.car = {"transport": "sta", "host": "192.168.137.192"}

    def test_legacy_paths_keep_configured_prefixes_without_reading_network(self):
        for transport in ("wired", "wifi"):
            with self.subTest(transport=transport), \
                    patch("carapp.core.connection.local_addresses") as read:
                car = {"transport": transport,
                       "subnet_prefixes": ["10.12.1", "192.168.149"]}
                self.assertEqual(search_prefixes(car, []), car["subnet_prefixes"])
                self.assertEqual(search_prefixes({"transport": transport}, []),
                                 ["192.168.150", "192.168.149"])
                self.assertEqual(search_prefixes({"transport": transport,
                                                  "subnet_prefixes": "10.12.1"}, []),
                                 ["10.12.1"])
                read.assert_not_called()

    def test_wifi_precedes_saved_target_and_ethernet(self):
        local = [address("Ethernet", "192.168.150.100"),
                 address("WLAN", "10.31.5.16", 16)]
        with patch("carapp.core.connection.local_addresses", return_value=local):
            self.assertEqual(search_prefixes(self.car, []),
                             ["10.31.5", "192.168.137", "192.168.150"])

    def test_virtual_loopback_and_link_local_addresses_are_excluded(self):
        local = [address("VMware Network Adapter VMnet8", "192.168.41.1"),
                 address("vEthernet (WSL)", "172.25.0.1"),
                 address("Docker", "172.17.0.1"),
                 address("本地连接* 2", "192.168.139.1"),
                 address("无线虚拟适配器", "10.2.3.1"),
                 address("Wi-Fi", "169.254.11.3", 16),
                 address("Loopback", "127.0.0.1", 8),
                 address("Ethernet", "::1", 128),
                 address("无线网络连接", "192.168.137.1")]
        with patch("carapp.core.connection.local_addresses", return_value=local):
            self.assertEqual(search_prefixes(self.car, []), ["192.168.137"])

    def test_large_subnet_is_bounded_to_four_unique_24_prefixes(self):
        local = [address("Wi-Fi %d" % i, "10.%d.7.2" % i, 8)
                 for i in range(1, 7)]
        local.insert(1, address("WLAN duplicate", "10.1.7.99", 8))
        with patch("carapp.core.connection.local_addresses", return_value=local):
            self.assertEqual(search_prefixes(self.car, []),
                             ["10.1.7", "10.2.7", "10.3.7", "192.168.137"])

    def test_saved_target_is_fallback_when_system_read_fails(self):
        for error in (OSError("no command"), ValueError("bad JSON"),
                      subprocess.TimeoutExpired("read", 8)):
            with self.subTest(error=error), \
                    patch("carapp.core.connection.local_addresses", side_effect=error):
                self.assertEqual(search_prefixes(self.car, []), ["192.168.137"])

    def test_no_address_or_local_network_returns_no_prefix(self):
        with patch("carapp.core.connection.local_addresses", return_value=[]):
            self.assertEqual(search_prefixes({"transport": "sta", "host": ""}, []), [])
            self.assertEqual(search_prefixes({"transport": "sta", "host": "bad-ip"}, []), [])

    def test_selected_profile_is_only_used_when_current_address_is_empty(self):
        car = {"transport": "sta", "host": "", "profile_id": "car-two"}
        profiles = [{"id": "car-one", "sta_host": "10.1.2.3"},
                    {"id": "car-two", "sta_host": "192.168.137.192"}]
        with patch("carapp.core.connection.local_addresses", return_value=[]):
            self.assertEqual(search_prefixes(car, profiles), ["192.168.137"])


class STADiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(DEFAULTS)
        self.cfg["car"].update(host="192.168.137.192", transport="sta",
                               profile_id="HW-A0857935")
        self.cfg["cars"][1]["sta_ssid"] = "Win11_Meng_plus"

    def report(self, local, ports=(False, False), ssid="Win11_Meng_plus"):
        with patch("carapp.core.connection.local_addresses", return_value=local), \
                patch("carapp.core.connection.wifi_ssid", return_value=ssid), \
                patch("carapp.core.connection.ping_once", side_effect=ports):
            return diagnose(self.cfg["car"], self.cfg["cars"])

    def test_sta_mismatch_explains_same_lan_and_dhcp_not_wired_static_ip(self):
        report = self.report([address("WLAN", "172.27.0.84", 15)])
        for message in ("STA 局域网", "没有目标网段", "同一局域网", "DHCP",
                        "Win11_Meng_plus", "客户端隔离"):
            self.assertIn(message, report)
        self.assertNotIn("192.168.150.100", report)
        self.assertNotIn("两台车热点都是", report)

    def test_ssid_difference_is_information_and_reachable_services_remain_ready(self):
        report = self.report([address("WLAN", "192.168.137.1")],
                             ports=(True, True), ssid="Win11_Meng_plus_5G")
        self.assertIn("Win11_Meng_plus_5G", report)
        self.assertIn("SSID 仅供核对", report)
        self.assertEqual(report.count("端口可达"), 3)
        self.assertNotIn("客户端隔离", report)
        self.assertNotIn("192.168.150.100", report)

    def test_partial_service_failure_mentions_isolation_and_startup_delay(self):
        report = self.report([address("WLAN", "192.168.137.1")], ports=(True, False))
        self.assertIn("图像服务 8080：端口可达", report)
        self.assertIn("ROS 服务 9090：端口不可达", report)
        self.assertIn("客户端隔离", report)
        self.assertIn("40–60", report)

    def test_ap_diagnostics_retains_legacy_ssid_identity_explanation(self):
        self.cfg["car"].update(host="192.168.149.1", transport="wifi")
        report = self.report([address("WLAN", "192.168.149.100")],
                             ssid="HW-A0857935")
        self.assertIn("Wi-Fi 热点", report)
        self.assertIn("应连接：HW-A0857935", report)
        self.assertIn("两台车热点都是 192.168.149.1", report)
        self.assertNotIn("STA 局域网", report)


if __name__ == "__main__":
    unittest.main()
