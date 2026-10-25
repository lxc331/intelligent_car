#!/usr/bin/env python3
import os
import time
import logging
import netifaces
import threading
import subprocess
import socket
import socketserver
from gpiozero import LED
from find_device import get_cpu_serial_number, normalize_generated_ap_ssid
from wifi_config_store import (
    WifiOperationLock,
    clear_directory_contents,
    load_config_values,
    select_config_path,
    update_config_file,
)

# 获取当前脚本所在路径
path = os.path.split(os.path.realpath(__file__))[0]
log_file_path = os.path.join(path, "wifi.log")

# 创建日志文件（如果不存在）
if not os.path.exists(log_file_path):
    os.system(f'touch {log_file_path}')

# 配置文件路径
config_file_name = "wifi_conf.py"
internal_config_file_dir_path = "/etc/wifi"
external_config_file_dir_path = path
internal_config_file_path = os.path.join(internal_config_file_dir_path, config_file_name)
external_config_file_path = os.path.join(external_config_file_dir_path, config_file_name)

# 全局变量
current_wifi_mode = None
server = None
ip = None
WIFI_INTERFACE = "wlan0"
CLIENT_CONNECTION_NAME = f"hiwonder-{WIFI_INTERFACE}-client"
AP_CONNECTION_NAME = f"hiwonder-{WIFI_INTERFACE}-ap"
NETWORK_MANAGER_CONNECTION_DIR = "/etc/NetworkManager/system-connections"
CLIENT_RECONNECT_DELAY_SECONDS = 5
_last_led_state = None

# 配置日志记录
logger = logging.getLogger("WiFi工具")
logger.setLevel(logging.DEBUG)
log_handler = logging.FileHandler(log_file_path)
log_handler.setLevel(logging.INFO)
log_formatter = logging.Formatter('%(name)s - %(asctime)s - %(levelname)s - %(message)s')
log_handler.setFormatter(log_formatter)
logger.addHandler(log_handler)

# ========== 直接 GPIO 控制 LED ==========
led2 = LED(26)   # 根据参考代码，LED2 对应 GPIO26

def set_led(led_id, on_time, off_time, repeat=0):
    if not globals().get("WIFI_LED", True):
        return
    global _last_led_state

    if led_id != 2:
        logger.warning(f"本程序仅控制 LED2，忽略 LED{led_id}")
        return
    state = (on_time, off_time, repeat)
    if state == _last_led_state:
        return
    try:
        led2._controller.stop()
    except:
        pass
    if on_time == 0:
        led2.off()
    elif off_time == 0:
        led2.on()
    else:
        led2.blink(on_time=on_time, off_time=off_time, n=repeat if repeat > 0 else None)
    _last_led_state = state
    logger.info(f"LED2 状态: on_time={on_time}, off_time={off_time}, repeat={repeat}")

# ========== 保留但未使用的函数（避免其他引用报错） ==========
def set_machine_ros():
    pass

def get_machine_type_from_config(config_path):
    pass

def send_machine_type(board, machine_type):
    pass

def set_machine_serial(board):
    pass

# ========== 从指定路径加载配置，避免同名模块被 sys.path 顺序覆盖 ==========
def update_globals_from_file(config_path):
    globals().update(load_config_values(config_path))

# ========== 网络相关函数 ==========
def get_wifi_ip():
    if WIFI_INTERFACE not in netifaces.interfaces():
        return None
    addresses = netifaces.ifaddresses(WIFI_INTERFACE)
    if netifaces.AF_INET in addresses:
        ip_info = addresses[netifaces.AF_INET][0]
        return ip_info['addr']
    return None

def reset_wifi():
    try:
        with WifiOperationLock(timeout=35):
            update_config_file(external_config_file_path, {"WIFI_MODE": 1})
            clear_directory_contents(internal_config_file_dir_path)
            subprocess.run(
                ["/bin/systemctl", "restart", "wifi.service"],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=30,
            )
        logger.info("WiFi已重置为AP直连模式并重启服务。")
    except Exception as e:
        logger.error(f"重置WiFi失败: {e}")

# ========== TCP 服务器 ==========
class TCPHandler(socketserver.BaseRequestHandler):
    def handle(self):
        self.request.settimeout(2)
        data = b""
        while True:
            try:
                chunk = self.request.recv(1024)
                if not chunk:
                    break
                data += chunk
                if data.strip() == b"resetwifi":
                    logger.info("收到重置WiFi命令。")
                    reset_wifi()
                    break
            except socket.timeout:
                logger.warning('TCP连接超时。')
                break
            except Exception as e:
                logger.error(f"TCP处理时出错: {e}")
                break
        self.request.close()

class PhoneServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True
    def __init__(self, server_address, RequestHandlerClass):
        super().__init__(server_address, RequestHandlerClass)

    def handle_timeout(self):
        logger.warning('TCP服务器超时。')

def start_server(ip, port):
    server = PhoneServer((ip, port), TCPHandler)
    thread = threading.Thread(target=server.serve_forever)
    thread.daemon = True
    thread.start()
    logger.info(f"TCP服务器已启动在 {ip}:{port}")
    return server

def stop_server(server):
    if server:
        server.shutdown()
        server.server_close()
        logger.info("TCP服务器已停止。")
    return None

def get_connect():
    try:
        result = subprocess.run(
            ['nmcli', '-g', 'GENERAL.CONNECTION', 'device', 'show', WIFI_INTERFACE],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=6,
        )
        connection = result.stdout.strip()
        if result.returncode == 0 and connection and connection != '--':
            return connection
    except Exception as e:
        logger.error(f"获取WiFi连接时出错: {e}")
    return None


def get_active_wifi_ssid():
    try:
        result = subprocess.run(
            ['iw', 'dev', WIFI_INTERFACE, 'info'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3,
        )
        for line in result.stdout.splitlines():
            value = line.strip()
            if value.startswith('ssid '):
                return value.split(None, 1)[1].strip()
    except Exception as e:
        logger.error(f"获取当前WiFi SSID时出错: {e}")
    return None


def split_nmcli_line(line):
    fields = []
    current = []
    escaped = False
    for character in line:
        if escaped:
            current.append(character)
            escaped = False
        elif character == '\\':
            escaped = True
        elif character == ':':
            fields.append(''.join(current))
            current = []
        else:
            current.append(character)
    if escaped:
        current.append('\\')
    fields.append(''.join(current))
    return fields


def get_wifi_connection_profiles():
    profiles = []
    try:
        result = subprocess.run(
            [
                'nmcli', '-t', '--escape', 'yes',
                '-f', 'NAME,UUID,TYPE,AUTOCONNECT', 'connection', 'show',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            logger.warning(f"读取NetworkManager连接配置失败: {result.stderr.strip()}")
            return profiles

        for line in result.stdout.splitlines():
            values = split_nmcli_line(line)
            if len(values) < 4:
                continue
            name, connection_uuid, connection_type, autoconnect = values[:4]
            if connection_type not in ('802-11-wireless', 'wifi'):
                continue
            profiles.append({
                'name': name,
                'uuid': connection_uuid,
                'autoconnect': autoconnect.lower() == 'yes',
            })
    except Exception as e:
        logger.error(f"读取WiFi连接配置时出错: {e}")
    return profiles


def delete_connections(connection_uuids):
    deleted = 0
    for connection_uuid in connection_uuids:
        result = subprocess.run(
            ['nmcli', 'connection', 'delete', 'uuid', connection_uuid],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=8,
        )
        if result.returncode == 0:
            deleted += 1
        else:
            logger.warning(f"删除连接 {connection_uuid} 失败: {result.stderr.strip()}")
    return deleted


def delete_connections_named(*connection_names):
    names = {name for name in connection_names if name}
    uuids = [
        profile['uuid']
        for profile in get_wifi_connection_profiles()
        if profile['name'] in names
    ]
    return delete_connections(uuids) if uuids else 0


def cleanup_invalid_connection_files():
    if not os.path.isdir(NETWORK_MANAGER_CONNECTION_DIR):
        return
    removed = 0
    try:
        for entry in os.scandir(NETWORK_MANAGER_CONNECTION_DIR):
            if not entry.name.endswith('.nmconnection') or not entry.is_file(follow_symlinks=False):
                continue
            if entry.stat(follow_symlinks=False).st_size != 0:
                continue
            os.unlink(entry.path)
            removed += 1
    except OSError as exc:
        logger.warning(f"清理损坏NetworkManager配置失败: {exc}")
        return
    if removed:
        subprocess.run(
            ['nmcli', 'connection', 'reload'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
        logger.info(f"已清理损坏NetworkManager配置: {removed}")


def disconnect():
    wifi = get_connect()
    if not wifi:
        return
    try:
        result = subprocess.run(
            ['nmcli', 'device', 'disconnect', WIFI_INTERFACE],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10,
        )
        if result.returncode == 0:
            logger.info(f"已断开WiFi连接: {wifi}")
        else:
            logger.warning(f"断开WiFi连接失败: {result.stderr.strip()}")
    except subprocess.TimeoutExpired:
        logger.warning(f"断开WiFi连接超时: {wifi}")
    except Exception as e:
        logger.error(f"断开WiFi连接时出错: {e}")


def prepare_wifi_role_change():
    cleanup_invalid_connection_files()
    result = subprocess.run(
        ['nmcli', 'device', 'set', WIFI_INTERFACE, 'autoconnect', 'no'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=6,
    )
    if result.returncode != 0:
        logger.warning(f"关闭wlan0自动抢占失败: {result.stderr.strip()}")
    disconnect()


def connection_uuid(connection_name):
    profiles = get_wifi_connection_profiles()
    matches = [profile['uuid'] for profile in profiles if profile['name'] == connection_name]
    if len(matches) == 1:
        return matches[0]
    if matches:
        delete_connections(matches)
    return None


def run_nmcli(command, timeout=15, check=False):
    result = subprocess.run(
        ['nmcli', *command],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=timeout,
    )
    if check and result.returncode != 0:
        raise RuntimeError((result.stderr or result.stdout or 'nmcli failed').strip())
    return result


def ensure_client_connection(hidden):
    connection_id = connection_uuid(CLIENT_CONNECTION_NAME)
    if connection_id is None:
        run_nmcli(
            [
                'connection', 'add', 'type', 'wifi', 'ifname', WIFI_INTERFACE,
                'con-name', CLIENT_CONNECTION_NAME, 'autoconnect', 'no',
                'ssid', WIFI_STA_SSID,
            ],
            timeout=15,
            check=True,
        )
        connection_id = connection_uuid(CLIENT_CONNECTION_NAME)
    if connection_id is None:
        raise RuntimeError('无法创建固定的局域网连接配置')

    run_nmcli(
        [
            'connection', 'modify', 'uuid', connection_id,
            'connection.autoconnect', 'no',
            'connection.interface-name', WIFI_INTERFACE,
            '802-11-wireless.mode', 'infrastructure',
            '802-11-wireless.ssid', WIFI_STA_SSID,
            '802-11-wireless.hidden', 'yes' if hidden else 'no',
            'ipv4.method', 'auto',
            'ipv6.method', 'auto',
        ],
        check=True,
    )
    if WIFI_STA_PASSWORD:
        run_nmcli(
            [
                'connection', 'modify', 'uuid', connection_id,
                'wifi-sec.key-mgmt', 'wpa-psk',
                'wifi-sec.psk', WIFI_STA_PASSWORD,
                'wifi-sec.psk-flags', '0',
            ],
            check=True,
        )
    else:
        run_nmcli(
            [
                'connection', 'modify', 'uuid', connection_id,
                'wifi-sec.key-mgmt', '',
                'wifi-sec.psk', '',
            ],
            check=True,
        )
    return connection_id


def ensure_ap_connection():
    delete_connections_named(WIFI_AP_SSID)
    connection_id = connection_uuid(AP_CONNECTION_NAME)
    if connection_id is None:
        run_nmcli(
            [
                'connection', 'add', 'type', 'wifi', 'ifname', WIFI_INTERFACE,
                'con-name', AP_CONNECTION_NAME, 'autoconnect', 'no',
                'ssid', WIFI_AP_SSID,
            ],
            timeout=15,
            check=True,
        )
        connection_id = connection_uuid(AP_CONNECTION_NAME)
    if connection_id is None:
        raise RuntimeError('无法创建固定的AP连接配置')
    run_nmcli(
        [
            'connection', 'modify', 'uuid', connection_id,
            '802-11-wireless.mode', 'ap',
            '802-11-wireless.ssid', WIFI_AP_SSID,
            'wifi-sec.key-mgmt', 'wpa-psk',
            'wifi-sec.psk', WIFI_AP_PASSWORD,
            'wifi-sec.psk-flags', '0',
            'wifi.band', WIFI_FREQ_BAND,
            'wifi.channel', str(WIFI_CHANNEL),
            'ipv4.method', 'shared',
            'ipv4.addresses', f'{WIFI_AP_GATEWAY}/24',
            'ipv4.gateway', WIFI_AP_GATEWAY,
            'ipv4.never-default', 'yes',
            'ipv6.method', 'disabled',
            'connection.autoconnect', 'no',
        ],
        check=True,
    )
    return connection_id


def scan_for_wifi(ssid):
    try:
        result = subprocess.run(
            [
                'nmcli', '-t', '--escape', 'no', '-f', 'SSID',
                'device', 'wifi', 'list', 'ifname', WIFI_INTERFACE,
                '--rescan', 'yes',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=15,
        )
        if result.returncode != 0:
            logger.warning(f"扫描WiFi失败: {result.stderr.strip()}")
            return False
        return ssid in result.stdout.splitlines()
    except subprocess.TimeoutExpired:
        logger.warning("扫描WiFi超时")
    except Exception as e:
        logger.warning(f"扫描WiFi时出错: {e}")
    return False

# ========== WiFi 模式设置 ==========
def setup_ap_mode():
    global current_wifi_mode, ip, server
    current_wifi_mode = 1
    set_led(2, 0.5, 0.5, 0)
    prepare_wifi_role_change()
    try:
        ap_uuid = ensure_ap_connection()
        run_nmcli(['--wait', '20', 'connection', 'up', 'uuid', ap_uuid], timeout=25, check=True)
        logger.info(f"AP创建成功: {WIFI_AP_SSID}")

        timeout = 0

        while timeout < 20:

            try:
                result = subprocess.check_output(['ip', '-4', 'addr', 'show', 'wlan0']).decode()

                real_ip = None

                for line in result.splitlines():
                    line = line.strip()

                    if line.startswith("inet "):
                        real_ip = line.split()[1].split('/')[0]
                        break

                if real_ip == WIFI_AP_GATEWAY and get_active_wifi_ssid() == WIFI_AP_SSID:

                    ip = real_ip

                    logger.info(f"当前AP IP: {ip}")

                    if server:
                        server = stop_server(server)

                    server = start_server(ip, 9028)

                    return

            except Exception as e:
                logger.error(f"获取AP IP失败: {e}")

            timeout += 1

            time.sleep(1)

        logger.error("AP启动成功但未获取到IP")

    except Exception as e:
        logger.error(f"设置AP模式失败: {e}")


def persist_ap_fallback(reason):
    """Compatibility hook: automatic fallback must never change user preference."""
    logger.info("%s; temporary AP fallback, saved Wi-Fi preference unchanged", reason)

def setup_client_mode():
    """Run preferred-mode recovery; a temporary AP does not overwrite WIFI_MODE."""
    from wifi_recovery_adapter import WifiRecoveryAdapter
    while True:
        try:
            WifiRecoveryAdapter(globals()).coordinator().run()
        except Exception as exc:
            logger.error("Wi-Fi recovery worker failed (%s); retry in 5 seconds", type(exc).__name__)
            time.sleep(5)

def WIFI_MGR():
    # The coordinator handles both explicit AP and preferred STA.
    setup_client_mode()

if __name__ == "__main__":
    # 配置参数
    ap_prefix = 'HW-'
    sn = get_cpu_serial_number()
    default_ap_ssid = ''.join([ap_prefix, sn[0:8]])
    WIFI_MODE = 1
    WIFI_AP_SSID = default_ap_ssid
    WIFI_STA_SSID = "ssid"           # 请替换为目标 WiFi 名称
    WIFI_AP_PASSWORD = "hiwonder"    # AP 模式密码
    WIFI_STA_PASSWORD = "12345678"   # 客户端模式密码
    WIFI_AP_GATEWAY = "192.168.149.1"
    WIFI_CHANNEL = 36
    WIFI_FREQ_BAND = 'a'             # a = 5GHz, g = 2.4GHz
    WIFI_TIMEOUT = 15
    WIFI_LED = True

    # 主配置是长期配置；/etc/wifi 仅在主配置不存在时兼容旧版部署。
    config_path = select_config_path(external_config_file_path, internal_config_file_path)
    if config_path is not None:
        try:
            update_globals_from_file(config_path)
            logger.info(f"已加载WiFi配置: {config_path}")
        except Exception as e:
            WIFI_MODE = 1
            logger.error(f"WiFi配置无效，使用AP直连模式: {e}")

    WIFI_AP_SSID = normalize_generated_ap_ssid(WIFI_AP_SSID, default_ap_ssid)
    # Network recovery must run even when the indicator LED is disabled.
    wifi_thread = threading.Thread(target=WIFI_MGR, daemon=True)
    wifi_thread.start()
    try:
        while wifi_thread.is_alive():
            wifi_thread.join(timeout=1)
    except KeyboardInterrupt:
        logger.info("程序被用户手动终止。")
