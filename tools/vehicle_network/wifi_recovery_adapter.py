"""Bounded NetworkManager adapter for the vehicle Wi-Fi recovery policy.

Loaded by the existing vendor ``wifi.py`` with its globals dictionary. This
module never imports GPIO, changes the saved preferred mode, or logs secrets.
"""

import ipaddress
import json
import math
import os
import subprocess
import tempfile
import time

try:
    from .wifi_recovery import RecoveryCoordinator
except ImportError:
    from wifi_recovery import RecoveryCoordinator


class WifiRecoveryAdapter:
    INTERFACE = "wlan0"
    CLIENT = "hiwonder-wlan0-client"
    AP = "hiwonder-wlan0-ap"

    def __init__(self, api, clock=time.monotonic, sleep=time.sleep,
                 run=subprocess.run, runtime_dir="/run"):
        self.api = api
        self.clock, self.sleep, self.run = clock, sleep, run
        self.runtime_dir = runtime_dir

    def _log(self, message):
        logger = self.api.get("logger")
        if logger:
            logger.info(message)

    def read_preference(self):
        path = self.api["select_config_path"](
            self.api["external_config_file_path"],
            self.api["internal_config_file_path"],
        )
        if path is None:
            raise ValueError("Wi-Fi configuration missing")
        values = self.api["load_config_values"](path)
        # Keep existing API globals in sync for vendor reset/status consumers.
        self.api.update({key: value for key, value in values.items()
                         if key.startswith("WIFI_")})
        cfg = {
            "mode": values.get("WIFI_MODE", self.api.get("WIFI_MODE", 1)),
            "ssid": values.get("WIFI_STA_SSID", self.api.get("WIFI_STA_SSID", "")),
            "password": values.get("WIFI_STA_PASSWORD", self.api.get("WIFI_STA_PASSWORD", "")),
            "ap_ssid": values.get("WIFI_AP_SSID", self.api.get("WIFI_AP_SSID", "HW-Rescue")),
            "ap_password": values.get("WIFI_AP_PASSWORD", self.api.get("WIFI_AP_PASSWORD", "hiwonder")),
            "gateway": values.get("WIFI_AP_GATEWAY", self.api.get("WIFI_AP_GATEWAY", "192.168.149.1")),
            "band": values.get("WIFI_FREQ_BAND", self.api.get("WIFI_FREQ_BAND", "a")),
            "channel": values.get("WIFI_CHANNEL", self.api.get("WIFI_CHANNEL", 36)),
        }
        if cfg["mode"] not in (1, 2):
            raise ValueError("Invalid preferred Wi-Fi mode")
        for key in ("ssid", "password", "ap_ssid", "ap_password", "gateway", "band"):
            if not isinstance(cfg[key], str):
                raise ValueError("Invalid Wi-Fi configuration value")
        if cfg["mode"] == 2 and not cfg["ssid"]:
            raise ValueError("STA SSID missing")
        return cfg

    def _command(self, argv, deadline):
        remaining = deadline - self.clock()
        if remaining <= 0:
            raise TimeoutError("Wi-Fi operation deadline reached")
        result = self.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          text=True, timeout=remaining)
        if result.returncode:
            # NM errors can echo command lines; deliberately do not propagate.
            raise RuntimeError("NetworkManager command failed")
        return result.stdout.strip()

    def _nm(self, args, deadline):
        return self._command(["nmcli", *args], deadline)

    def _profile_uuid(self, name, deadline):
        output = self._nm(["-t", "--escape", "yes", "-f", "NAME,UUID",
                           "connection", "show"], deadline)
        matches = []
        # Our fixed connection names contain no escaping characters.
        for line in output.splitlines():
            prefix, _, uuid = line.rpartition(":")
            if prefix == name:
                matches.append(uuid)
        if len(matches) > 1:
            raise RuntimeError("Duplicate managed Wi-Fi profiles")
        return matches[0] if matches else None

    def _ensure_profile(self, cfg, mode, deadline):
        name = self.CLIENT if mode == 2 else self.AP
        uuid = self._profile_uuid(name, deadline)
        ssid = cfg["ssid"] if mode == 2 else cfg["ap_ssid"]
        password = cfg["password"] if mode == 2 else cfg["ap_password"]
        if not uuid:
            self._nm(["connection", "add", "type", "wifi", "ifname", self.INTERFACE,
                      "con-name", name, "autoconnect", "no", "ssid", ssid], deadline)
            uuid = self._profile_uuid(name, deadline)
        if not uuid:
            raise RuntimeError("Managed Wi-Fi profile missing")
        settings = [
            "connection.autoconnect", "no", "connection.interface-name", self.INTERFACE,
            "802-11-wireless.mode", "infrastructure" if mode == 2 else "ap",
            "802-11-wireless.ssid", ssid, "802-11-wireless.hidden", "no",
            "ipv4.method", "auto" if mode == 2 else "shared",
            "ipv4.addresses", "" if mode == 2 else cfg["gateway"] + "/24",
            "ipv4.gateway", "" if mode == 2 else cfg["gateway"],
            "ipv4.never-default", "no" if mode == 2 else "yes",
            "ipv6.method", "auto" if mode == 2 else "disabled",
            "wifi-sec.key-mgmt", "wpa-psk" if password else "",
            "wifi-sec.psk", password, "wifi-sec.psk-flags", "0",
        ]
        if mode == 1:
            settings += ["wifi.band", cfg["band"], "wifi.channel", str(cfg["channel"])]
        self._nm(["connection", "modify", "uuid", uuid, *settings], deadline)
        return uuid

    def _status(self, cfg, mode, deadline):
        name = self.CLIENT if mode == 2 else self.AP
        if self._nm(["-g", "GENERAL.CONNECTION", "device", "show", self.INTERFACE], deadline) != name:
            return None
        info = self._nm(["--escape", "no", "-g", "802-11-wireless.mode,802-11-wireless.ssid",
                         "connection", "show", name], deadline).splitlines()
        expected = ("infrastructure", cfg["ssid"]) if mode == 2 else ("ap", cfg["ap_ssid"])
        if tuple(info[:2]) != expected:
            return None
        addresses = json.loads(self._command(["ip", "-j", "-4", "addr", "show", "dev", self.INTERFACE], deadline))
        for interface in addresses:
            for address in interface.get("addr_info", []):
                if address.get("family") != "inet":
                    continue
                value = address.get("local", "")
                ipv4 = ipaddress.IPv4Address(value)
                if ipv4.is_unspecified or ipv4.is_loopback or ipv4.is_link_local or ipv4.is_multicast:
                    continue
                if (mode == 1 and value == cfg["gateway"]) or (mode == 2 and value != cfg["gateway"]):
                    return value
        return None

    def _stop_server(self):
        old = self.api.get("server")
        self.api["server"] = None
        self.api["ip"] = None
        if old is not None:
            self.api["stop_server"](old)

    def _bind_server(self, address, mode):
        if self.api.get("ip") != address or self.api.get("server") is None:
            self._stop_server()
            self.api["server"] = self.api["start_server"](address, 9028)
            self.api["ip"] = address
        self.api["current_wifi_mode"] = mode
        self.api["set_led"](2, 1 if mode == 2 else 0.5, 0 if mode == 2 else 0.5, 0)

    def sta_ready(self, cfg):
        try:
            address = self._status(cfg, 2, self.clock() + 5)
            if address:
                self._bind_server(address, 2)
                return True
        except Exception:
            self._log("STA status temporarily unavailable; recovery continues")
        self._stop_server()
        return False

    def _activate(self, cfg, mode, deadline):
        # The file is root-only, removed on success, error, and timeout. Supplying
        # passwd-file avoids NM secret-agent requests during unattended boot.
        secret_path = None
        password = cfg["password"] if mode == 2 else cfg["ap_password"]
        try:
            self._stop_server()
            self.api["current_wifi_mode"] = None
            self.api["set_led"](2, 0.05 if mode == 2 else 0.5, 0.05 if mode == 2 else 0.5, 0)
            self._nm(["radio", "wifi", "on"], deadline)
            self._nm(["device", "set", self.INTERFACE, "autoconnect", "no"], deadline)
            # 'down' can legitimately fail when wlan0 is already disconnected.
            try:
                self._nm(["device", "disconnect", self.INTERFACE], deadline)
            except RuntimeError:
                pass
            uuid = self._ensure_profile(cfg, mode, deadline)
            args = ["--wait", str(max(1, math.ceil(deadline - self.clock()))),
                    "connection", "up", "uuid", uuid]
            if password:
                if "\n" in password or "\r" in password:
                    raise ValueError("Invalid Wi-Fi secret")
                fd, secret_path = tempfile.mkstemp(prefix="hiwonder-wifi-", dir=self.runtime_dir)
                try:
                    os.chmod(secret_path, 0o600)
                    with os.fdopen(fd, "w", encoding="utf-8") as stream:
                        fd = -1
                        stream.write("802-11-wireless-security.psk:" + password + "\n")
                finally:
                    if fd >= 0:
                        os.close(fd)
                args += ["passwd-file", secret_path]
            self._nm(args, deadline)
            while self.clock() < deadline:
                address = self._status(cfg, mode, deadline)
                if address:
                    self._bind_server(address, mode)
                    return True
                self.sleep(min(0.5, max(0, deadline - self.clock())))
            return False
        finally:
            if secret_path is not None:
                try:
                    os.unlink(secret_path)
                except FileNotFoundError:
                    pass

    def try_sta(self, cfg, timeout):
        deadline = self.clock() + max(0.0, float(timeout))
        try:
            with self.api["WifiOperationLock"](timeout=max(0, min(2, deadline - self.clock()))):
                # A GUI may have changed mode or credentials while we waited.
                current = self.read_preference()
                if current != cfg or current["mode"] != 2:
                    return False
                address = self._status(cfg, 2, deadline)
                if address:
                    self._bind_server(address, 2)
                    return True
                return self._activate(cfg, 2, deadline)
        except Exception:
            self._log("STA activation failed or timed out; saved preference unchanged")
            return False

    def start_ap(self):
        deadline = self.clock() + 25
        try:
            expected = self.read_preference()
            with self.api["WifiOperationLock"](timeout=2):
                cfg = self.read_preference()
                if cfg != expected:
                    return False
                address = self._status(cfg, 1, deadline)
                if address:
                    self._bind_server(address, 1)
                    return True
                return self._activate(cfg, 1, deadline)
        except Exception:
            self._log("AP activation failed or timed out; saved preference unchanged")
            return False

    def coordinator(self):
        self.read_preference()
        def duration(name, default):
            try:
                value = float(self.api.get(name, default))
                return value if math.isfinite(value) and value > 0 else default
            except (TypeError, ValueError):
                return default
        return RecoveryCoordinator(
            self.read_preference, self.try_sta, self.start_ap, self.sta_ready,
            log=self._log, clock=self.clock, sleep=self.sleep,
            startup_budget=duration("WIFI_STA_STARTUP_TIMEOUT", 120),
            retry_interval=duration("WIFI_STA_RETRY_INTERVAL", 60),
            attempt_timeout=25,
        )
