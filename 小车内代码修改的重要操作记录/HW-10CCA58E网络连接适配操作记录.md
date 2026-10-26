# 一号车 STA 自动恢复部署记录

操作日期：2026-10-26（按电脑的 Asia/Shanghai 时间）。车辆 HW-10CCA58E，CPU序列号7b5a8de10cca58e8，有线地址192.168.150.3。已在认证后核对硬件身份，并保存SSH指纹。

最终连接：**Win11_Meng_plus（2.4GHz），192.168.137.149**。电脑仍连接Win11_Meng_plus_5G，已从电脑无线地址192.168.137.179成功访问车的22、8080、9090、9028端口，并通过无线SSH核对硬件身份、获取640×480摄像头录像。DHCP地址今后可能变化，可在App中自动搜索更新。

## 车内文件改动

| 路径 | 改动 |
|---|---|
| `/home/pi/hiwonder-toolbox/wifi.py` | 更新网络服务入口，使用与二号车相同的恢复模块；自动回退不再把持久模式改成AP；联网与LED开关独立；状态TCP服务使用守护处理线程并可重用地址 |
| `/home/pi/hiwonder-toolbox/wifi_recovery.py` | 新增纯逻辑调度器：STA优先、限时启动恢复、临时AP、定时重试、掉线恢复、首选配置变化重新调度 |
| `/home/pi/hiwonder-toolbox/wifi_recovery_adapter.py` | 新增NetworkManager适配器：配置锁、限时联网、无人值守认证、IP检查、状态端口随IP重绑 |
| `/home/pi/hiwonder-toolbox/wifi_conf.py` | WIFI_MODE从1改成2；STA网络名最终从Win11_Meng_Plus_5G改成Win11_Meng_plus；新增WIFI_STA_STARTUP_TIMEOUT=120、WIFI_STA_RETRY_INTERVAL=60。STA/AP密码和AP名称HW-10CCA58E均保留 |
| `/etc/NetworkManager/system-connections/hiwonder-wlan0-client.nmconnection` | 设置客户端SSID为Win11_Meng_plus、hidden=no，并在连接时更新connection.timestamp；与原文件逐字段对比，仅wifi.ssid、wifi.hidden、connection.timestamp变化 |
| `/etc/NetworkManager/system-connections/hiwonder-wlan0-ap.nmconnection` | 切换时重存AP配置，最终与备份相比仅connection.timestamp变化 |
| `/home/pi/hiwonder-toolbox/wifi.log` | 网络服务追加连接、临时回退、重试、恢复记录；新恢复模块不记录密码 |
| `/home/pi/hiwonder-toolbox/__pycache__/wifi_recovery.cpython-311.pyc` | Python自动生成的缓存 |
| `/home/pi/hiwonder-toolbox/__pycache__/wifi_recovery_adapter.cpython-311.pyc` | Python自动生成的缓存 |

车端wifi_config_store.py、find_device.py、wifi.service服务单元和旧配置路径均与备份核对，保持不变。没有修改ROS、底盘或舵机程序，没有发送运动命令，没有重启整车。运行时认证文件/run/hiwonder-wifi-*已清除；没有在小车中保存录像。

## 连接过程与验证

1. 原配置写的是Win11_Meng_Plus_5G；电脑实际SSID为Win11_Meng_plus_5G。修正大小写后，车端仍报ssid-not-found，不能据此确定硬件故障或不支持5GHz；网卡列出了149信道支持，但本次未查明该热点不可发现的原因。
2. 已直接观察正式120秒启动预算结束后自动回退AP，热点仍为HW-10CCA58E、网关192.168.149.1，持久模式保持2。
3. 车端能扫描到之前用户提供的Win11_Meng_plus，改用该网络后自动连上，无线SSH与视频互通。没有改电脑当前5GHz连接。
4. 使用合法且不存在的SSID ClawPi_absent_261025、20秒启动等待和10秒重试参数完成加速验证：连接失败→临时AP→定时STA重试→失败后恢复AP。两次回退都没有永久修改STA偏好。
5. 恢复正确SSID后，未重启服务，运行输出显示约13秒连回STA。之后仅重启wifi.service以重新加载正式120/60秒参数，连接正常。
6. 最终额外观察约12秒，服务PID、活动连接和服务日志大小保持一致，没有反复激活STA。正常状态只检查连接，掉线后才恢复；手动WIFI_MODE=1不自动重试STA。
7. 40项网络恢复测试和40项App STA相关测试全部通过；车端部署的三个脚本与本地源文件SHA256逐字节一致。

当前政策：启动或掉线后先尝试STA120秒，失败临时AP；AP状态每60秒尝试一次STA，单次STA操作最多25秒，失败后重建AP。单无线网卡切换角色会中断AP，重建AP还需额外时间。连接成功后停止连接重试，仅约每2秒检查状态。

整车断电冷启动尚未实测，不能将网络服务重启等同于冷启动。车端系统时钟与电脑日期不一致，原始车内日志可能显示10月19日；本次操作目录和报告采用电脑日期，未改动小车时钟。

## 本机文件改动

- `app/config/settings.json`：只更新一号车档案的sta_host=192.168.137.149和sta_ssid=Win11_Meng_plus；二号车档案、标定和其他用户参数保留。App当前选择仍是原来的“有线”，未自动切换正在使用的连接。
- `app/config/ssh_known_hosts`：核对一号车硬件序列后，新增一号车的有线和STA地址主机密钥；原有二号车条目保留。
- 根目录README.md和tools/vehicle_network/README.md：更新两车部署范围及一号车实测地址说明。
- `.work/`：保存本次备份、操作脚本、验证记录和摄像头影像；目录受Git忽略。

如果App已打开，请重新打开后选择“一号车→STA局域网”，应显示192.168.137.149；也可直接手动填入该地址并应用。

## 备份和回滚

before/files_manifest.json、before/network.json与predeploy_inventory.json含原始文件、权限、SHA256及网络配置；before/另存App配置、SSH已知主机和文档备份。其中含原有凭据，只存本机，未打印密码，不提交Git。

需要回滚时，保留网线，通过192.168.150.3恢复原wifi.py、wifi_conf.py以及需要的NetworkManager配置，按原权限还原；本次新增的两个恢复模块原先不存在。恢复后只重启wifi.service。无需修改二号车。

## 关键帧和录像

| 阶段 | 关键帧 | 录像 |
|---|---|---|
| 操作前 | [查看](before_start.jpg) | [播放](before.avi) |
| 正式120秒等待后的临时AP | [查看](default_timeout_ap_end.jpg) | [播放](default_timeout_ap.avi) |
| 首次STA无线连接 | [查看](after_initial_sta_end.jpg) | [播放](after_initial_sta.avi) |
| 加速回退重试测试 | [查看](temporary_ap_end.jpg) | [播放](temporary_ap.avi) |
| 最终STA连接 | [查看](final_sta_end.jpg) | [播放](final_sta.avi) |

所有录像直接保存到电脑，原始.mjpg保留并将原始JPEG帧封装为MJPEG AVI，没有在小车上创建录像文件。

[结构化记录](repair_report.json) · [配置与凭据保留核对](preservation_verification.json) · [无线端口和稳定连接核对](wireless_idle_verification.json) · [测试结果](test_results.json)
