# 车端 Wi-Fi 恢复服务

本目录保存车端 `/home/pi/hiwonder-toolbox/` 网络服务的部署源文件，与桌面 App 的连接方式选择独立。

| 本地文件 | 车端 `/home/pi/hiwonder-toolbox/` 中的文件 | 作用 |
|---|---|---|
| `wifi_service.py` | **`wifi.py`** | 服务入口，保留厂家 GPIO/TCP 接口与网络工具接口 |
| `wifi_recovery.py` | `wifi_recovery.py` | 不依赖硬件的状态调度器 |
| `wifi_recovery_adapter.py` | `wifi_recovery_adapter.py` | NetworkManager、配置锁、指示灯和状态 TCP 服务适配器 |

以上三个脚本与车辆原有的 `wifi_conf.py` 配置放在同一目录。当前仅部署到 **2 号车 `HW-A0857935`**，1 号车尚未部署；断电冷启动尚未实测，不能把离线测试或服务重启当作冷启动验证。

## 模式与恢复

`wifi_conf.py` 中的 `WIFI_MODE=1` 表示用户明确选择 AP，保持热点、不尝试 STA。
`WIFI_MODE=2` 表示始终优先连接配置的 STA 网络；启动/掉线恢复预算默认 120 秒，失败时临时提供 AP，但不改写用户偏好。临时 AP 下默认每 60 秒尝试一次 STA，单次尝试最长 25 秒，失败后恢复 AP。

单张无线网卡在角色切换时会暂时中断 AP 连接。模式、目标网络或密码变更会在当前有限操作结束后重新读取，正常监控每 2 秒检查配置。`WIFI_STA_STARTUP_TIMEOUT` 和 `WIFI_STA_RETRY_INTERVAL` 可在主配置中覆盖默认等待时间。

STA 成功要求活动配置为客户端、SSID 匹配并获得有效 IPv4，不要求访问互联网。IP 变化后重新绑定厂家状态端口 9028。网络恢复与 LED 是否启用无关。

客户端密钥由 NetworkManager 保存；启动连接时还使用权限为 0600 的 `/run/hiwonder-wifi-*` 临时认证文件，完成/失败后删除，不需要交互输入。认证文件格式依据 [NetworkManager nmcli 手册](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nmcli.html)。密码不写入恢复日志。

## 部署与回滚

先将车辆当前 `wifi.py`、`wifi_conf.py`、相关 NetworkManager 配置及服务单元备份到项目 `.work/`。按上表部署三个 Python 文件，尤其注意将本地 `wifi_service.py` 保存为车端现有的 **`wifi.py`**。部署文件必须使用 **UTF-8、LF 换行**，保留现有入口的执行权限；CRLF 可能导致直接运行脚本时无法识别解释器。

保留原有 `wifi_conf.py` 中的网络凭据、AP 名称、频段和信道。STA 优先与默认等待参数为：

```python
WIFI_MODE = 2
WIFI_STA_STARTUP_TIMEOUT = 120
WIFI_STA_RETRY_INTERVAL = 60
```

`WIFI_STA_SSID`、`WIFI_STA_PASSWORD` 继续使用车辆实际要连接的网络；`WIFI_MODE = 1` 是手动 AP。恢复服务只读取偏好，不会在自动回退时把模式永久改成 1。继续依赖车端已有的 `wifi_config_store.py`、`find_device.py`、GPIO 库及 NetworkManager，无需替换这些文件。仅重启 `wifi.service`，不需要重启 ROS 或机器人。

不要直接从电脑运行 `wifi_service.py`：它包含现有厂家的 GPIO 初始化。离线验证从项目的 `app/` 目录执行：

```bash
python -m unittest discover -s tests -p 'test_wifi*.py' -v
```

出现异常时，可通过保留的有线通道恢复备份的 `wifi.py` 与 `wifi_conf.py`，然后重启 `wifi.service`。影像与验证记录保存在电脑，不在车内存放录像。
