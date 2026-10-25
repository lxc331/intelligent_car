"""Offline profile migration and STA connection validation; no vehicle access."""

import copy
import json
import unittest

from carapp.core.connection_profiles import (
    TRANSPORT_LABELS, apply_connection, endpoint_for, get_profile,
    normalize_profiles, plan_connection,
)


def config():
    return {"car": {"host": "192.168.150.3"},
            "cars": [{"id": "one", "name": "1 号车", "host": "192.168.150.3",
                      "wifi_host": "192.168.149.1", "ssid": "HW-one"},
                     {"id": "two", "name": "2 号车", "host": "192.168.150.2",
                      "wifi_host": "192.168.149.1", "ssid": "HW-two",
                      "sta_host": "192.168.137.192", "sta_ssid": "Win11_Meng_plus"}],
            "tracking_pick": {"calibrations": {"one": {"x": .4}, "two": {"x": .6}}},
            "servo": {"actions": {"close": 1540}}, "camera": {"default_topic": "/image_raw"}}


class StaProfileTests(unittest.TestCase):
    def test_transport_labels_keep_existing_order(self):
        self.assertEqual(list(TRANSPORT_LABELS), ["wired", "wifi", "sta"])
        self.assertEqual(TRANSPORT_LABELS["sta"], "STA 局域网")

    def test_old_wired_config_infers_vehicle(self):
        cfg = normalize_profiles(config())
        self.assertEqual(cfg["car"]["profile_id"], "one")
        self.assertEqual(cfg["car"]["transport"], "wired")
        self.assertEqual(cfg["cars"][0]["sta_host"], "")
        self.assertEqual(cfg["cars"][0]["sta_ssid"], "")

    def test_missing_transport_infers_existing_sta_endpoint(self):
        cfg = config()
        cfg["car"]["host"] = "192.168.137.192"
        normalize_profiles(cfg)
        self.assertEqual(cfg["car"]["profile_id"], "two")
        self.assertEqual(cfg["car"]["transport"], "sta")
        self.assertEqual(cfg["car"]["sta_ssid"], "Win11_Meng_plus")

    def test_explicit_empty_sta_selection_survives_normalization(self):
        cfg = config()
        cfg["car"].update(host="", profile_id="one", transport="sta")
        normalize_profiles(cfg)
        self.assertEqual(cfg["car"]["profile_id"], "one")
        self.assertEqual(cfg["car"]["transport"], "sta")
        self.assertEqual(cfg["car"]["host"], "")

    def test_shared_ap_address_cannot_infer_identity(self):
        cfg = config()
        cfg["car"]["host"] = "192.168.149.1"
        normalize_profiles(cfg)
        self.assertEqual(cfg["car"]["profile_id"], "")
        self.assertEqual(cfg["car"]["transport"], "wifi")

    def test_explicit_ap_identity_survives_normalization(self):
        cfg = config()
        cfg["car"].update(host="192.168.149.1", profile_id="two")
        normalize_profiles(cfg)
        self.assertEqual(cfg["car"]["profile_id"], "two")

    def test_lookup_has_no_unknown_vehicle_fallback(self):
        cfg = normalize_profiles(config())
        self.assertEqual(get_profile(cfg)["id"], "one")
        self.assertIsNone(get_profile(cfg, "missing"))
        self.assertIsNone(get_profile(cfg, ""))
        self.assertEqual(endpoint_for(get_profile(cfg, "two"), "sta"), "192.168.137.192")
        self.assertEqual(endpoint_for(None, "sta"), "")

    def test_planning_is_pure_and_keeps_independent_profiles(self):
        cfg = normalize_profiles(config())
        before = copy.deepcopy(cfg)
        plan = plan_connection(cfg, " 192.168.137.200 ", "one", "sta", " Shared ")
        self.assertEqual(cfg, before)
        self.assertEqual(plan, {"host": "192.168.137.200", "profile_id": "one",
                                "transport": "sta", "sta_ssid": "Shared"})
        apply_connection(cfg, plan)
        self.assertEqual(cfg["cars"][0]["sta_host"], "192.168.137.200")
        self.assertEqual(cfg["cars"][1], before["cars"][1])
        self.assertEqual(cfg["cars"][0]["host"], before["cars"][0]["host"])
        self.assertEqual(cfg["cars"][0]["wifi_host"], before["cars"][0]["wifi_host"])
        for key in ("tracking_pick", "servo", "camera"):
            self.assertEqual(cfg[key], before[key])

    def test_roundtrip_modes_keep_sta_endpoint_and_vehicle(self):
        cfg = normalize_profiles(config())
        for mode in ("sta", "wired", "wifi", "sta"):
            profile = get_profile(cfg, "two")
            apply_connection(cfg, plan_connection(cfg, endpoint_for(profile, mode), "two", mode))
            cfg = normalize_profiles(json.loads(json.dumps(cfg)))
            self.assertEqual(cfg["car"]["profile_id"], "two")
            self.assertEqual(cfg["car"]["transport"], mode)
            self.assertEqual(cfg["cars"][1]["sta_host"], "192.168.137.192")
            self.assertEqual(cfg["cars"][1]["sta_ssid"], "Win11_Meng_plus")

    def test_sta_invalid_address_never_mutates_config(self):
        cfg = normalize_profiles(config())
        before = copy.deepcopy(cfg)
        for host in ("", " ", None, "http://192.168.137.192", "192.168.137.192:8080",
                     "999.1.2.3", "localhost", "127.0.0.1", "127.99.1.2", "::1",
                     "0.0.0.0", "224.0.0.1", "239.255.255.250", "255.255.255.255"):
            with self.subTest(host=host), self.assertRaises(ValueError):
                plan_connection(cfg, host, "two", "sta")
            self.assertEqual(cfg, before)

    def test_ipv4_validation_does_not_assume_subnet_mask(self):
        cfg = normalize_profiles(config())
        plan = plan_connection(cfg, "10.0.0.255", "two", "sta")
        self.assertEqual(plan["host"], "10.0.0.255")

    def test_explicit_sta_requires_selected_vehicle_argument(self):
        cfg = normalize_profiles(config())
        with self.assertRaises(ValueError):
            plan_connection(cfg, "192.168.137.200", transport="sta")
        with self.assertRaises(ValueError):
            plan_connection(cfg, "192.168.137.200", "missing", "sta")

    def test_sta_cannot_claim_another_cars_known_endpoint(self):
        cfg = normalize_profiles(config())
        for host in ("192.168.137.192", "192.168.150.2"):
            with self.subTest(host=host), self.assertRaises(ValueError):
                plan_connection(cfg, host, "one", "sta")
        plan = plan_connection(cfg, "192.168.150.3", "one", "sta")
        self.assertEqual(plan["profile_id"], "one")

    def test_legacy_host_changes_infer_wired_then_sta(self):
        cfg = normalize_profiles(config())
        for host, profile_id, mode in (("192.168.150.2", "two", "wired"),
                                      ("192.168.137.192", "two", "sta")):
            plan = plan_connection(cfg, host)
            self.assertEqual((plan["profile_id"], plan["transport"]), (profile_id, mode))
        cfg["cars"][0]["sta_host"] = "192.168.150.2"
        self.assertEqual(plan_connection(cfg, "192.168.150.2")["transport"], "wired")

    def test_explicit_wired_address_infers_known_vehicle(self):
        cfg = normalize_profiles(config())
        plan = plan_connection(cfg, "192.168.150.2", transport="wired")
        self.assertEqual(plan["profile_id"], "two")

    def test_explicit_vehicle_conflict_never_changes_identity_or_config(self):
        cfg = normalize_profiles(config())
        before = copy.deepcopy(cfg)
        for host in ("192.168.150.2", " 192.168.137.192 "):
            for mode in (None, "wired", "wifi", "sta"):
                with self.subTest(host=host, mode=mode), self.assertRaises(ValueError):
                    plan_connection(cfg, host, "one", mode)
                self.assertEqual(cfg, before)

    def test_explicit_matching_vehicle_retains_inferred_transport(self):
        cfg = normalize_profiles(config())
        for host, mode in (("192.168.150.2", "wired"), ("192.168.137.192", "sta")):
            plan = plan_connection(cfg, host, "two")
            self.assertEqual((plan["profile_id"], plan["transport"]), ("two", mode))

    def test_legacy_shared_ap_only_reuses_confirmed_ap_selection(self):
        cfg = normalize_profiles(config())
        with self.assertRaises(ValueError):
            plan_connection(cfg, "192.168.149.1")
        cfg["car"].update(profile_id="two", transport="wifi")
        plan = plan_connection(cfg, "192.168.149.1")
        self.assertEqual(plan["profile_id"], "two")

    def test_wired_hostname_validation_and_explicit_ap(self):
        cfg = normalize_profiles(config())
        for host in ("clawpi.local", "clawpi", "127.0.0.1", "car-2.example.com."):
            self.assertEqual(plan_connection(cfg, host, transport="wired")["host"], host)
        for host in ("http://car.local", "car.local:8080", "-car.local", "999.0.0.1", "a b", "::1"):
            with self.subTest(host=host), self.assertRaises(ValueError):
                plan_connection(cfg, host, transport="wired")
        self.assertEqual(plan_connection(cfg, "192.168.149.1", "two", "wifi")["profile_id"], "two")

    def test_unknown_mode_and_invalid_ssid_rejected_without_mutation(self):
        cfg = normalize_profiles(config())
        before = copy.deepcopy(cfg)
        with self.assertRaises(ValueError):
            plan_connection(cfg, "192.168.137.192", "two", "invalid")
        with self.assertRaises(ValueError):
            plan_connection(cfg, "192.168.137.192", "two", "sta", sta_ssid=123)
        self.assertEqual(cfg, before)


if __name__ == "__main__":
    unittest.main()
