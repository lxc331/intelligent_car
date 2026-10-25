# -*- coding: utf-8 -*-
"""连接诊断与搜索网段选择；只读取网络信息，不修改网卡或车端设置。"""

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


def _ipv4_prefix(value):
    """仅返回单个 /24 前缀，避免按校园网的大掩码展开扫描。"""
    try:
        address = ipaddress.ip_address(str(value))
    except ValueError:
        return None
    if (not isinstance(address, ipaddress.IPv4Address) or address.is_loopback
            or address.is_link_local or address.is_multicast or address.is_unspecified
            or str(address) == "255.255.255.255"):
        return None
    return str(address).rsplit(".", 1)[0]


def search_prefixes(car, profiles):
    """选择搜索网段；STA 优先本机 Wi-Fi，并保留保存地址的 /24。

    local_addresses 会调用系统命令，调用方应在搜索工作线程内使用。
    端口候选仍需用户选择，不能据网段或地址认定车辆身份。
    """
    if car.get("transport") != "sta":
        prefixes = car.get("subnet_prefixes")
        if not prefixes:
            prefixes = [car.get("subnet_prefix", "192.168.150"), "192.168.149"]
        return [prefixes] if isinstance(prefixes, str) else list(prefixes)

    try:
        addresses = local_addresses()
    except (OSError, ValueError, subprocess.SubprocessError):
        addresses = []
    wireless, other = [], []
    virtual = ("vmware", "vmnet", "virtual", "vethernet", "docker", "wsl",
               "loopback", "本地连接", "local area connection*", "hyper-v",
               "虚拟", "bluetooth", "蓝牙", "vpn",
               "tailscale", "zerotier", "tap-", "tun")
    for item in addresses:
        alias = str(item.get("InterfaceAlias", "")).lower()
        if any(name in alias for name in virtual):
            continue
        prefix = _ipv4_prefix(item.get("IPAddress", ""))
        if not prefix:
            continue
        bucket = wireless if any(name in alias for name in
                                 ("wlan", "wi-fi", "wifi", "wireless", "无线")) else other
        bucket.append(prefix)

    profile = next((p for p in profiles if p.get("id") == car.get("profile_id")), {})
    saved = _ipv4_prefix(car.get("host") or profile.get("sta_host", ""))
    # 最多四个 /24，且为已保存目标保留一个位置。
    result = list(dict.fromkeys(wireless))[:4]
    if saved and saved not in result:
        result = result[:3]
    for prefix in ([saved] if saved else []) + other:
        if prefix not in result and len(result) < 4:
            result.append(prefix)
    return result


def diagnose(car, profiles):
    profile = next((p for p in profiles if p["id"] == car.get("profile_id")), {})
    transport = car.get("transport")
    label = {"wifi": "Wi-Fi 热点", "sta": "STA 局域网"}.get(transport, "有线 / 手动")
    lines = ["目标：%s %s" % (profile.get("name", "手动地址"), profile.get("id", "")),
             "地址：%s（%s）" % (car["host"], label)]
    try:
        addresses = local_addresses()
        target = ipaddress.ip_address(car["host"])
        same_subnet = any(target in ipaddress.ip_network("%s/%s" %
                          (a["IPAddress"], a["PrefixLength"]), strict=False) for a in addresses)
        lines.append("\n电脑 IPv4：")
        lines.extend("  %s：%s/%s" % (a["InterfaceAlias"], a["IPAddress"], a["PrefixLength"])
                     for a in addresses if not a["IPAddress"].startswith("127."))
        if addresses and not same_subnet:
            if transport == "sta":
                lines.append("⚠ 电脑没有目标网段的地址。请确认电脑和小车接入同一局域网，"
                             "再核对小车当前获得的无线 IP。")
            else:
                lines.append("⚠ 电脑没有目标网段的地址。网线直连时，请接好网线/USB 网卡，"
                             "将该有线网卡设为 192.168.150.100，掩码 255.255.255.0，网关留空。")
    except (OSError, ValueError, subprocess.SubprocessError):
        lines.append("本机网卡信息读取失败，请手动检查是否处于小车网段。")
    if transport in ("wifi", "sta"):
        try:
            ssid = wifi_ssid()
        except (OSError, subprocess.SubprocessError):
            ssid = ""
        if transport == "sta":
            lines.append("当前电脑 Wi-Fi：%s；保存的 STA Wi-Fi：%s" %
                         (ssid or "未识别", profile.get("sta_ssid") or "未设置"))
            lines.append("电脑和小车应接入同一局域网；SSID 仅供核对，连接以目标地址可达为准。"
                         "DHCP 分配的地址可能随重启或换网变化，可自动搜索或在路由器中查看小车当前 IP。")
        else:
            lines.append("当前热点：%s；应连接：%s" % (ssid or "未识别", profile.get("ssid", "请确认车辆")))
            lines.append("两台车热点都是 192.168.149.1，只能通过所连 SSID 区分。")
    http = ping_once(car["host"], int(car["http_port"]), timeout=1)
    ros = ping_once(car["host"], int(car["ws_port"]), timeout=1)
    lines.extend(["\n图像服务 %s：%s" % (car["http_port"], "端口可达" if http else "端口不可达"),
                  "ROS 服务 %s：%s" % (car["ws_port"], "端口可达" if ros else "端口不可达")])
    if not http or not ros:
        lines.append("刚开机时业务服务可能尚未就绪，通常需要等待约 40–60 秒，App 会持续重试。"
                     "电脑网段正确仍长期不通时，再检查车端服务。")
        if transport == "sta":
            lines.append("STA 仍不通时，检查路由器或热点是否启用了客户端隔离/AP 隔离、访客网络隔离，"
                         "以及电脑与小车之间是否允许访问上述服务端口。")
    else:
        lines.append("端口可达只说明服务正在监听；ROS 已连接和收到新画面才代表 App 就绪。")
    return "\n".join(lines)
