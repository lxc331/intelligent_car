"""Vehicle identity probe: serial -> profile mapping, recorded host keys, log wording."""
import base64
import os
import tempfile
import types
import unittest
from unittest.mock import Mock

import paramiko

from carapp.core.vehicle_identity import (_OnlyKnownKeys, _known_keys, identity_from_serial)
from carapp.main import App
from carapp.model.state import CarState


class SerialIdentityTests(unittest.TestCase):
    def test_pi_serial_maps_to_the_profile_id(self):
        # 实测值：2 号车 /proc/device-tree/serial-number
        self.assertEqual(identity_from_serial("e92c53fa0857935d"), "HW-A0857935")

    def test_too_short_or_garbage_serial_is_rejected(self):
        for value in ("", "   ", "1234", "e92c53faZZZZZZZZ"):
            self.assertEqual(identity_from_serial(value), "", value)

    def test_known_hosts_keeps_only_this_address(self):
        key = paramiko.RSAKey.generate(2048)
        blob = base64.b64encode(key.asbytes()).decode()
        text = ("HW-A0857935@192.168.150.2 %s %s\n"
                "HW-10CCA58E@192.168.150.3 %s %s\n") % (key.get_name(), blob, key.get_name(), blob)
        with tempfile.TemporaryDirectory() as folder:
            path = os.path.join(folder, "ssh_known_hosts")
            with open(path, "w", encoding="utf-8") as stream:
                stream.write(text)
            assert _known_keys("192.168.150.2", path) == [(key.get_name(), key.asbytes())]
            self.assertEqual(_known_keys("10.0.0.9", path), [])
            self.assertEqual(_known_keys("192.168.150.2", os.path.join(folder, "missing")), [])

    def test_policy_rejects_an_unrecorded_key(self):
        recorded, stranger = paramiko.RSAKey.generate(2048), paramiko.RSAKey.generate(2048)
        policy = _OnlyKnownKeys([(recorded.get_name(), recorded.asbytes())])
        policy.missing_host_key(None, "192.168.150.2", recorded)
        with self.assertRaises(paramiko.SSHException):
            policy.missing_host_key(None, "192.168.150.2", stranger)


class IdentityLogTests(unittest.TestCase):
    def app(self, generation=7):
        app = App.__new__(App)
        app.cfg = {"terminal": {"username": "pi", "port": 22}}
        app.state = CarState()
        app.log = Mock()
        app._closing = False
        app._connection_generation = generation
        return app

    def test_mismatch_is_reported_loudly(self):
        app = self.app()
        app._identity_checked(7, {"host": "192.168.167.136", "name": "1 号车",
                                  "profile_id": "HW-10CCA58E"},
                              {"ok": True, "identity": "HW-A0857935"})
        message = app.log.call_args.args[0]
        self.assertIn("车辆身份不符", message)
        self.assertIn("HW-A0857935", message)
        self.assertIn("HW-10CCA58E", message)
        self.assertFalse(app.state.get("vehicle_identity_ok"))

    def test_match_is_confirmed(self):
        app = self.app()
        app._identity_checked(7, {"host": "192.168.150.2", "name": "2 号车",
                                  "profile_id": "HW-A0857935"},
                              {"ok": True, "identity": "HW-A0857935"})
        self.assertIn("核对通过", app.log.call_args.args[0])
        self.assertTrue(app.state.get("vehicle_identity_ok"))

    def test_skips_quietly_when_the_car_could_not_be_reached(self):
        app = self.app()
        app._identity_checked(7, {"host": "192.168.150.2", "name": "2 号车",
                                  "profile_id": "HW-A0857935"},
                              {"ok": False, "identity": "", "detail": "没有可用的登录密码"})
        app.log.assert_not_called()

    def test_stale_connection_result_is_ignored(self):
        app = self.app(generation=8)
        app._identity_checked(7, {"host": "192.168.150.2", "name": "2 号车",
                                  "profile_id": "HW-A0857935"},
                              {"ok": True, "identity": "HW-10CCA58E"})
        app.log.assert_not_called()


if __name__ == "__main__":
    unittest.main()
