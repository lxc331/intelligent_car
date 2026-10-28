# 交通灯识别模型

这里的 XML、BIN 和 metadata.yaml 原样复制自项目随附资料：

`ClawPi智能小车/3 源码资料/src/example/example/yolo_detect/models/26/best_traffic_26_openvino_model/`

原推理入口：`example/example/yolo_detect/yolo_node.py`。类别、置信度过滤和检测框读取流程由 `carapp/core/traffic_light_detector.py` 适配为纯电脑端模块，不启动原自动驾驶或 ROS 节点。

- 模型：YOLO26s，输入 RGB float32 `[1,3,320,320]`，输出 `[1,300,6]`，每行是 xyxy、置信度、类别编号。
- 类别：go、right、park、red、green、crosswalk。类别与框交给 `traffic_scene.py`：三帧近斑马线确认后减速，路口内满足面积条件的 red 触发停车。
- 使用原节点默认置信度 0.5，斑马线距离与红灯面积等条件按原 640×480 参考画面归一化；不接入巡线、自动转向、自动泊车或固定航点控制。
- 在电脑 CPU 上通过 OpenVINO 执行；不需要 Torch、Ultralytics、OpenCV 或本机 ROS。
- 作者、导出信息和 AGPL-3.0 许可声明保留在 metadata.yaml。

原始文件 SHA-256：

| 文件 | SHA-256 |
|---|---|
| best_traffic_26.xml | 5a15ac995fc3de4ebcaa433c56fc56616268e114f975b0503bdd993cf542c694 |
| best_traffic_26.bin | 520e0093442a0fe96660df9fb6aaf71e4980e3f3704e2bac71d329b0a7f6eff5 |
| metadata.yaml | 8ff0cfce8c578dab8b6019582aba3fe6ba1cb2d2738bff00a136abae2ecf3c68 |

实现格式对照：[YOLO26 输出说明](https://docs.ultralytics.com/guides/end2end-detection)、[OpenVINO CPU 推理](https://docs.openvino.ai/2026/openvino-workflow/running-inference/inference-devices-and-modes/cpu-device.html)。
