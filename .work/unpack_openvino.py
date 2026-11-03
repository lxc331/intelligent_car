"""把已下载的 wheel 直接解包到 .work/pylib（绕过 pip 在本机沙箱下的临时目录清理失败）。

用法：python .work/unpack_openvino.py
"""
import os
import sys
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WHEELS = os.path.join(ROOT, ".work", "wheels")
TARGET = os.path.join(ROOT, ".work", "pylib")


def unpack(path):
    print("解包 %s" % os.path.basename(path), flush=True)
    with zipfile.ZipFile(path) as archive:
        names = archive.namelist()
        for name in names:
            relative = name
            for marker in (".data/platlib/", ".data/purelib/"):
                if marker in name:
                    relative = name.split(marker, 1)[1]
                    break
            if not relative:
                continue
            destination = os.path.join(TARGET, relative)
            if name.endswith("/"):
                os.makedirs(destination, exist_ok=True)
                continue
            os.makedirs(os.path.dirname(destination), exist_ok=True)
            with archive.open(name) as source, open(destination, "wb") as output:
                while True:
                    chunk = source.read(1024 * 1024)
                    if not chunk:
                        break
                    output.write(chunk)
        print("  完成 %d 个条目" % len(names), flush=True)


def main():
    if not os.path.isdir(WHEELS):
        raise SystemExit("没有 %s，请先运行 fetch_openvino.py" % WHEELS)
    wheels = [os.path.join(WHEELS, name) for name in os.listdir(WHEELS) if name.endswith(".whl")]
    if not wheels:
        raise SystemExit("没有 wheel 文件")
    for wheel in wheels:
        unpack(wheel)
    sys.path.insert(0, TARGET)
    import openvino
    print("验证成功：openvino %s（%s）" % (openvino.__version__, openvino.__file__), flush=True)


if __name__ == "__main__":
    main()
