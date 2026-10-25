"""Vehicle endpoints and connection choices, independent of Tk and networking.

The shared AP address identifies a transport, never a physical vehicle.  LAN
addresses belong to individual profiles so changing cars keeps calibration keys
and the wired/AP endpoints intact.
"""

import ipaddress
import re


TRANSPORT_LABELS = {"wired": "有线网口", "wifi": "Wi-Fi 热点", "sta": "STA 局域网"}
_ENDPOINT_KEYS = {"wired": "host", "wifi": "wifi_host", "sta": "sta_host"}
_HOST_LABEL = re.compile(r"[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?\Z")


def get_profile(cfg, profile_id=None):
    """Return an explicitly requested or currently selected vehicle profile."""
    if profile_id is None:
        profile_id = cfg.get("car", {}).get("profile_id")
    return next((p for p in cfg.get("cars", [])
                 if profile_id and p.get("id") == profile_id), None)


def endpoint_for(profile, transport):
    """Return an endpoint, or empty text when the vehicle has no saved endpoint."""
    key = _ENDPOINT_KEYS.get(transport)
    return str((profile or {}).get(key) or "") if key else ""


def _matching_profile(cfg, host, transport):
    if not host:
        return None
    return next((p for p in cfg.get("cars", [])
                 if endpoint_for(p, transport) == host), None)


def normalize_profiles(cfg):
    """Migrate profile fields and infer legacy connection identity in place.

    Config's loader supplies the built-in cars and legacy profile IDs before
    this function runs.  Explicit STA selections remain intact when the address
    is still empty, so a configuration dialog can ask the user to fill it in.
    """
    for profile in cfg.get("cars", []):
        profile.setdefault("sta_host", "")
        profile.setdefault("sta_ssid", "")
    car = cfg.setdefault("car", {})
    host = str(car.get("host") or "").strip()
    selected = get_profile(cfg)
    transport = car.get("transport")
    if transport not in TRANSPORT_LABELS:
        wired = _matching_profile(cfg, host, "wired")
        sta = _matching_profile(cfg, host, "sta")
        if wired:
            selected, transport = wired, "wired"
        elif sta:
            selected, transport = sta, "sta"
        else:
            transport = "wifi" if host == "192.168.149.1" else "wired"
    elif not selected:
        # A shared AP address cannot fill in a missing vehicle ID.
        if transport in ("wired", "sta"):
            selected = _matching_profile(cfg, host, transport)
    car["profile_id"] = selected["id"] if selected else ""
    car["transport"] = transport
    car["sta_ssid"] = str((selected or {}).get("sta_ssid") or "") if transport == "sta" else ""
    return cfg


def _validated_host(host, transport):
    if not isinstance(host, str) or not host.strip():
        raise ValueError("请输入小车地址")
    host = host.strip()
    try:
        address = ipaddress.ip_address(host)
    except ValueError:
        address = None
    if transport == "sta":
        if not isinstance(address, ipaddress.IPv4Address):
            raise ValueError("STA 局域网请输入小车的 IPv4 地址，例如 192.168.137.192")
        if (address.is_unspecified or address.is_loopback or address.is_multicast
                or address == ipaddress.IPv4Address("255.255.255.255")):
            raise ValueError("STA 局域网地址必须是小车可用的单播 IPv4 地址")
        return str(address)
    if isinstance(address, ipaddress.IPv4Address):
        return str(address)
    # This app builds HTTP/WS URLs from a host and port; do not allow URLs,
    # IPv6 literals, embedded ports or malformed dotted numeric addresses.
    name = host[:-1] if host.endswith(".") else host
    if (len(name) > 253 or not name or all(c.isdigit() or c == "." for c in name)
            or not all(_HOST_LABEL.fullmatch(label) for label in name.split("."))):
        raise ValueError("请输入有效的 IPv4 地址或主机名，不要包含协议、端口或路径")
    return host


def plan_connection(cfg, host, profile_id=None, transport=None, sta_ssid=None):
    """Validate a connection change without touching config or current sessions.

    An explicit STA choice requires an explicit vehicle ID; an exact saved STA
    address can infer its vehicle when transport is omitted (legacy callers).
    """
    if transport is not None and transport not in TRANSPORT_LABELS:
        raise ValueError("未知连接方式")
    if profile_id and get_profile(cfg, profile_id) is None:
        raise ValueError("请选择有效的小车档案")
    requested_profile = get_profile(cfg, profile_id) if profile_id else None
    profile = requested_profile
    raw_host = host.strip() if isinstance(host, str) else host
    if requested_profile and raw_host:
        collision = next((p for p in cfg.get("cars", [])
                          if p.get("id") != requested_profile["id"]
                          and raw_host in (endpoint_for(p, "wired"), endpoint_for(p, "sta"))), None)
        if collision:
            raise ValueError("该地址已属于 %s，请核对所选车辆和连接地址" % collision.get("name", collision["id"]))
    if transport is None:
        wired = _matching_profile(cfg, raw_host, "wired")
        sta = _matching_profile(cfg, raw_host, "sta")
        if wired:
            profile, transport = wired, "wired"
        elif sta:
            profile, transport = sta, "sta"
        elif raw_host == "192.168.149.1":
            transport = "wifi"
            if profile is None and cfg.get("car", {}).get("transport") == "wifi":
                profile = get_profile(cfg)
        else:
            transport = "wired"
    elif transport == "wired" and profile is None:
        profile = _matching_profile(cfg, raw_host, "wired")
    elif transport == "wifi" and profile is None and cfg.get("car", {}).get("transport") == "wifi":
        profile = get_profile(cfg)
    if transport == "sta" and profile is None:
        raise ValueError("请先选择车辆，再配置 STA 局域网地址")
    if transport == "wifi" and profile is None:
        raise ValueError("两车热点地址相同，请先选择车辆，再连接 Wi-Fi 热点")
    host = _validated_host(host, transport)
    if transport == "sta":
        collision = next((p for p in cfg.get("cars", [])
                          if p.get("id") != profile["id"]
                          and host in (endpoint_for(p, "wired"), endpoint_for(p, "sta"))), None)
        if collision:
            raise ValueError("该地址已属于 %s，请核对车辆和 STA 地址" % collision.get("name", collision["id"]))
        if sta_ssid is not None and not isinstance(sta_ssid, str):
            raise ValueError("Wi-Fi 名称必须为文本")
        ssid = (sta_ssid if sta_ssid is not None else profile.get("sta_ssid", "")) or ""
    else:
        ssid = ""
    return {"host": host, "profile_id": profile["id"] if profile else "",
            "transport": transport, "sta_ssid": ssid.strip()}


def apply_connection(cfg, plan):
    """Apply a previously validated plan, preserving all unrelated settings."""
    cfg.setdefault("car", {}).update(plan)
    if plan["transport"] == "sta":
        profile = get_profile(cfg, plan["profile_id"])
        profile["sta_host"] = plan["host"]
        profile["sta_ssid"] = plan["sta_ssid"]
    return cfg
