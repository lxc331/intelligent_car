# -*- coding: utf-8 -*-
"""Battery guard shared by the measurement scripts.

The sandbox car raises a low-voltage alarm below 6.8 V (user, 2026-10-29), so any
script that drives the wheels must check the voltage first and refuse to move.
"""
import json
import time

TOPIC = "/ros_robot_controller/battery"
WARN_VOLTS = 6.9


def read_volts(ws, seconds=1.2):
    """Subscribe, read one battery message and return volts (None when silent)."""
    ws.send_text(json.dumps({"op": "subscribe", "topic": TOPIC, "id": "battery"}))
    end = time.monotonic() + seconds
    volts = None
    while time.monotonic() < end:
        try:
            raw = ws.recv_text()
        except Exception:
            continue
        try:
            data = json.loads(raw)
        except ValueError:
            continue
        if data.get("op") != "publish" or data.get("topic") != TOPIC:
            continue
        msg = data.get("msg") or {}
        value = msg.get("voltage")
        if value is None:
            value = msg.get("data")
        if value is None:
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        volts = number / 1000.0 if number > 100 else number
    return volts


def ensure_ok(ws, log=print):
    """Return volts when safe to drive; raise RuntimeError when the pack is low."""
    volts = read_volts(ws)
    if volts is None:
        log("电压未知，按安全策略拒绝驱动车轮")
        raise RuntimeError("battery unknown")
    if volts < WARN_VOLTS:
        log("电压 %.2f V 低于 %.1f V，拒绝驱动车轮" % (volts, WARN_VOLTS))
        raise RuntimeError("battery low")
    log("电压 %.2f V，可以驱动" % volts)
    return volts
