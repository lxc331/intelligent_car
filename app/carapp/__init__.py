# -*- coding: utf-8 -*-
"""ClawPi 小车控制台（可移植版）。

包结构：
    config.py       配置加载
    core/           与小车通信的底层（WebSocket / rosbridge / 摄像头 / 网络发现）
    model/          状态模型
    features/       功能选项卡（每个功能一个文件，可插拔）
    ui/             主题、状态条、日志面板
    main.py         程序入口
"""

__version__ = "1.0.0"
