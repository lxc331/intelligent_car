# -*- coding: utf-8 -*-
"""小车自动发现：扫网段找开着 8080 的主机。

为什么需要：这台 ClawPi 的 IP 会变（手改的地址重启就丢，DHCP 也可能变），
换台电脑接上去以后，让程序自己找比让人去翻 IP 更靠谱。
"""

import concurrent.futures
import socket

HTTP_PORT = 8080
WS_PORT = 9090


def _probe(ip, port, timeout=0.25):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        return s.connect_ex((ip, port)) == 0
    except Exception:
        return False
    finally:
        try:
            s.close()
        except Exception:
            pass


def _looks_like_car(ip, http_port=HTTP_PORT, ws_port=WS_PORT):
    """8080 开 + 9090 开，基本就是这台车（rosbridge + web_video_server）。"""
    # 任一服务先起来时也列为候选，由用户选择；端口不能用于推断车辆身份。
    return _probe(ip, http_port) or _probe(ip, ws_port, timeout=0.4)


def scan(subnet_prefix="192.168.150", hosts=254, on_progress=None, cancel=None,
         http_port=HTTP_PORT, ws_port=WS_PORT):
    """返回找到的 IP 列表（通常只有一个）。

    subnet_prefix 可以是单个前缀字符串，也可以是前缀列表 ——
    因为同一台车有两条接入路径：网线 192.168.150.x、WiFi 热点 192.168.149.x。
    """
    if isinstance(subnet_prefix, str):
        prefixes = [subnet_prefix]
    else:
        prefixes = list(subnet_prefix)
    ips = []
    for p in prefixes:
        ips.extend("%s.%d" % (p, i) for i in range(1, hosts + 1))
    found = []
    done = 0
    total = len(ips)
    # 列表里有多个网段时，先探"当前已连上的网段"更省时间：靠前的 prefix 优先并发
    with concurrent.futures.ThreadPoolExecutor(max_workers=96) as pool:
        futures = {pool.submit(_looks_like_car, ip, http_port, ws_port): ip for ip in ips}
        for fut in concurrent.futures.as_completed(futures):
            done += 1
            if cancel is not None and cancel():
                break
            try:
                if fut.result():
                    found.append(futures[fut])
            except Exception:
                pass
            if on_progress and done % 20 == 0:
                on_progress(done, total)
    return sorted(found)


def ping_once(ip, port=HTTP_PORT, timeout=0.6):
    return _probe(ip, port, timeout)
