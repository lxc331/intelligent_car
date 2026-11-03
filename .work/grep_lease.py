# -*- coding: utf-8 -*-
"""Print every lease/heartbeat related line in the navigation stack."""
import pathlib

FILES = [
    "app/carapp/core/navigation_runtime.py",
    "app/carapp/core/navigation_session.py",
    "app/carapp/core/navigation_remote_scripts.py",
    "app/carapp/core/navigation_remote.py",
    "app/carapp/features/navigation.py",
]

for name in FILES:
    path = pathlib.Path(name)
    short = path.name
    for index, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        text = line.strip()
        if any(word in text.lower() for word in ("lease", "heartbeat", "renew", "keepalive")):
            print("%s:%d: %s" % (short, index, text))
