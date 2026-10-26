"""Optional existing local credentials; never persisted by features or logged."""
import os
import re

from carapp.config import APP_DIR


def local_password():
    path = os.path.join(os.path.dirname(APP_DIR), "小车账号密码信息.txt")
    try:
        with open(path, encoding="utf-8-sig") as stream:
            match = re.search(r"密码\s*[：:]\s*([^\s（(]+)", stream.read())
        return match.group(1) if match else ""
    except OSError:
        return ""
