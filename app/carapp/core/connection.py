# -*- coding: utf-8 -*-
"""两车连接诊断。只读取电脑网络信息、探测端口，不修改网卡或车端设置。"""

import ipaddress
import json
import os
import re
import subprocess

from carapp.core.discovery import ping_once


def _run(args):
    result = subprocess.run(args, capture_output=True, timeout=8,
                            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    encoding = "mbcs" if os.name == "nt" else "utf-8"
    return result.stdout.decode(encoding, errors="replace").strip()


def wifi_ssid():
    if os.name == "nt":
        output = _run(["netsh", "wlan", "show", "interfaces"])
        return next(iter(re.findall(r"^\s*SSID\s*:\s*(.+?)\s*$", output, re.M)), "")
    try:
        return _run(["iwgetid", "-r"])
    except (OSError, subprocess.SubprocessError):
        return ""


def require_wifi(ssid):
    """两车热点 IP 相同；SSID 未确认时禁止将另一台当成已选车辆。"""
    if not ssid:
        raise ConnectionError("请选择具体车辆后再连接热点")
    current = wifi_ssid()
    if current != ssid:
        raise ConnectionError("请先连接热点 %s（当前：%s）" % (ssid, current or "未识别"))


def local_addresses():
    if os.name != "nt":
        return []
    # ConvertTo-Json 的字段名固定，不依赖 Windows 界面语言。
    output = _run(["powershell", "-NoProfile", "-NonInteractive", "-Command",
                   "Get-NetIPAddress -AddressFamily IPv4 | "
                   "Select-Object InterfaceAlias,IPAddress,PrefixLength | ConvertTo-Json -Compress"])
    data = json.loads(output or "[]")
    return [data] if isinstance(data, dict) else data


def diagnose(car, profiles):
    profile = next((p for p in profiles if p["id"] == car.get("profile_id")), {})
    lines = ["目标：%s %s" % (profile.get("name", "手动地址"), profile.get("id", "")),
             "地址：%s（%s）" % (car["host"], "Wi-Fi 热点" if car.get("transport") == "wifi" else "有线 / 手动")]
    try:
        addresses = local_addresses()
        target = ipaddress.ip_address(car["host"])
        same_subnet = any(target in ipaddress.ip_network("%s/%s" %
                          (a["IPAddress"], a["PrefixLength"]), strict=False) for a in addresses)
        lines.append("\n电脑 IPv4：")
        lines.extend("  %s：%s/%s" % (a["InterfaceAlias"], a["IPAddress"], a["PrefixLength"])
                     for a in addresses if not a["IPAddress"].startswith("127."))
        if addresses and not same_subnet:
            lines.append("⚠ 电脑没有目标网段的地址。网线直连时，请接好网线/USB 网卡，"
                         "将该有线网卡设为 192.168.150.100，掩码 255.255.255.0，网关留空。")
    except (OSError, ValueError, subprocess.SubprocessError):
        lines.append("本机网卡信息读取失败，请手动检查是否处于小车网段。")
    if car.get("transport") == "wifi":
        try:
            ssid = wifi_ssid()
        except (OSError, subprocess.SubprocessError):
            ssid = ""
        lines.append("当前热点：%s；应连接：%s" % (ssid or "未识别", profile.get("ssid", "请确认车辆")))
        lines.append("两台车热点都是 192.168.149.1，只能通过所连 SSID 区分。")
    http = ping_once(car["host"], int(car["http_port"]), timeout=1)
    ros = ping_once(car["host"], int(car["ws_port"]), timeout=1)
    lines.extend(["\n图像服务 %s：%s" % (car["http_port"], "端口可达" if http else "端口不可达"),
                  "ROS 服务 %s：%s" % (car["ws_port"], "端口可达" if ros else "端口不可达")])
    if not http or not ros:
        lines.append("刚开机时业务服务可能尚未就绪，通常需要等待约 40–60 秒，App 会持续重试。"
                     "电脑网段正确仍长期不通时，再检查车端服务。")
    else:
        lines.append("端口可达只说明服务正在监听；ROS 已连接和收到新画面才代表 App 就绪。")
    return "\n".join(lines)
