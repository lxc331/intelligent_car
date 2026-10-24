# -*- coding: utf-8 -*-
"""功能注册表。

新增功能只需要在下面 FEATURES 里追加一个类，主程序会自动生成对应选项卡。
顺序即选项卡从左到右的顺序。

注：摄像头不再是独立选项卡，已作为 CameraPanel 嵌进「夹取」页底部
（carapp/ui/camerapanel.py），需要单独放一页时再 new 一个即可。
"""

from carapp.features.gripper import GripperFeature
from carapp.features.remote import RemoteFeature
from carapp.features.cruise import CruiseFeature
from carapp.features.terminal import TerminalFeature

FEATURES = [
    GripperFeature,
    RemoteFeature,
    CruiseFeature,
    TerminalFeature,
]

__all__ = ["FEATURES", "GripperFeature", "RemoteFeature", "CruiseFeature", "TerminalFeature"]
