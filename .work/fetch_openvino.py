"""下载 OpenVINO Windows cp312 wheel 并装到 .work/pylib（不影响系统 Python）。

用法：python .work/fetch_openvino.py
进度直接打印到 stdout，可由后台任务流式查看；同时写入 .work/openvino_install.log。
"""
import json
import os
import sys
import time
import urllib.request

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WHEELS = os.path.join(ROOT, ".work", "wheels")
TARGET = os.path.join(ROOT, ".work", "pylib")
LOG = os.path.join(ROOT, ".work", "openvino_install.log")
VERSION = "2026.4.1"
PY_TAG = "cp312"

os.makedirs(WHEELS, exist_ok=True)
os.makedirs(TARGET, exist_ok=True)


def say(message):
    stamp = time.strftime("%H:%M:%S")
    line = "[%s] %s" % (stamp, message)
    print(line, flush=True)
    with open(LOG, "a", encoding="utf-8") as handle:
        handle.write(line + "\n")


def wheel_url(package, version, tag=None):
    with urllib.request.urlopen("https://pypi.org/pypi/%s/%s/json" % (package, version), timeout=60) as response:
        data = json.load(response)
    for entry in data["urls"]:
        name = entry["filename"]
        if not name.endswith(".whl"):
            continue
        if tag and ("-%s-%s-win_amd64.whl" % (tag, tag)) not in name:
            continue
        if not tag and "py3-none-any" not in name and "py2.py3-none-any" not in name:
            continue
        return name, entry["url"], entry["size"]
    raise SystemExit("找不到 %s %s 的合适 wheel" % (package, version))


def download(name, url, size):
    path = os.path.join(WHEELS, name)
    if os.path.isfile(path) and os.path.getsize(path) == size:
        say("已存在，跳过下载：%s" % name)
        return path
    say("开始下载 %s（%.1f MB）" % (name, size / 1024.0 / 1024.0))
    started = time.time()
    done = 0
    next_mark = 0.0
    with urllib.request.urlopen(url, timeout=120) as response, open(path + ".part", "wb") as handle:
        while True:
            chunk = response.read(256 * 1024)
            if not chunk:
                break
            handle.write(chunk)
            done += len(chunk)
            percent = done * 100.0 / size
            if percent >= next_mark:
                speed = done / 1024.0 / 1024.0 / max(time.time() - started, .001)
                say("进度 %5.1f%%  %6.1f / %.1f MB  %.1f MB/s"
                    % (percent, done / 1024.0 / 1024.0, size / 1024.0 / 1024.0, speed))
                next_mark = percent + 10.0
    os.replace(path + ".part", path)
    say("下载完成：%s（用时 %.1f s）" % (name, time.time() - started))
    return path


def main():
    say("目标：%s %s → %s" % (VERSION, PY_TAG, TARGET))
    files = [wheel_url("openvino", VERSION, PY_TAG)]
    try:
        with urllib.request.urlopen("https://pypi.org/pypi/openvino-telemetry/json", timeout=60) as response:
            latest = json.load(response)["info"]["version"]
        files.append(wheel_url("openvino-telemetry", latest))
    except SystemExit:
        pass
    paths = []
    for name, url, size in files:
        paths.append(download(name, url, size))
    say("开始安装到 %s" % TARGET)
    import subprocess
    command = [sys.executable, "-m", "pip", "install", "--target", TARGET, "--no-deps", "--no-cache-dir"] + paths
    result = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", errors="replace")
    say("pip 退出码 %d" % result.returncode)
    for line in (result.stdout or "").strip().splitlines()[-8:]:
        say("  " + line)
    for line in (result.stderr or "").strip().splitlines()[-8:]:
        say("  [stderr] " + line)
    if result.returncode:
        raise SystemExit(result.returncode)
    sys.path.insert(0, TARGET)
    import openvino
    say("验证成功：openvino %s" % getattr(openvino, "__version__", "?"))
    say("安装结束。")


if __name__ == "__main__":
    main()
