"""DSH 沙箱环境兼容补丁（只影响本机测试/安装，不进入项目代码）。

现象：本会话的 DSH Windows 沙箱里，任何用 0o700 权限创建的目录
（tempfile.mkdtemp / TemporaryDirectory 正是如此）在创建后立刻对当前进程
不可写（WinError 5）。导致 pip 解包与依赖临时目录的单元测试成片报错，
与项目代码无关。

本文件作为 sitecustomize 注入：把 mkdtemp 换成不显式指定 0o700 的实现。
用 PYTHONPATH=<repo>\\.work;<repo>\\.work\\pylib 启用。
"""
import os
import random
import string
import tempfile as _tempfile

_ALPHABET = string.ascii_lowercase + string.digits


def _mkdtemp(suffix=None, prefix=None, dir=None, *, _orig=_tempfile.mkdtemp):
    base = dir if dir is not None else _tempfile.gettempdir()
    prefix = prefix if prefix is not None else "tmp"
    suffix = suffix if suffix is not None else ""
    for _ in range(10000):
        name = os.path.join(
            base, prefix + "".join(random.choices(_ALPHABET, k=8)) + suffix
        )
        try:
            os.mkdir(name)
        except FileExistsError:
            continue
        return name
    return _orig(suffix=suffix, prefix=prefix, dir=dir)


_tempfile.mkdtemp = _mkdtemp
