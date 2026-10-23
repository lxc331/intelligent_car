# -*- coding: utf-8 -*-
"""摄像头拉流：从车上 web_video_server 抓 MJPEG / PNG 连续流。

两种模式：
    jpeg —— 需要 Pillow 解码（画质好、带宽约 4~6 Mbps、约 10 fps）
    png  —— 用 Tk 自带解码，零依赖（带宽约 20 Mbps、约 7 fps）

线程只负责"拆帧 + 计数 + 可选落盘"，解码交给 UI 主线程，避免 Tk 跨线程崩溃。
"""

import os
import queue
import struct
import threading
import time
import urllib.request

JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
PNG_SIG = b"\x89PNG\r\n\x1a\n"


def parse_size(payload, mode):
    """不依赖 Pillow，直接从文件头读分辨率。"""
    try:
        if mode == "png":
            if len(payload) > 24 and payload[:8] == PNG_SIG:
                w, h = struct.unpack(">II", payload[16:24])
                return int(w), int(h)
            return None
        # JPEG: 逐段找 SOF0/SOF2
        i = 2
        n = len(payload)
        while i < n - 9:
            if payload[i] != 0xFF:
                i += 1
                continue
            marker = payload[i + 1]
            if marker in (0xC0, 0xC1, 0xC2):
                h, w = struct.unpack(">HH", payload[i + 5:i + 9])
                return int(w), int(h)
            if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
                i += 2
                continue
            seg_len = struct.unpack(">H", payload[i + 2:i + 4])[0]
            i += 2 + seg_len
        return None
    except Exception:
        return None


class CameraStream(threading.Thread):
    RETRY_SECONDS = 3
    def __init__(self, host, http_port, topic, mode, frame_q, log=None, connection_guard=None):
        super().__init__(daemon=True, name="camera")
        self.host = host
        self.http_port = http_port
        self.topic = topic
        self.mode = mode
        self.frame_q = frame_q
        self.log = log or (lambda text: None)
        self.connection_guard = connection_guard

        self.url = "http://%s:%d/stream?topic=%s&type=%s" % (
            host, http_port, topic, "mjpeg" if mode == "jpeg" else "png")

        self._halt = threading.Event()
        self._resp = None
        self._rec_fp = None
        self._rec_dir = None
        self._rec_count = 0
        self._lock = threading.Lock()

        self.frames = 0
        self.bytes = 0
        self.t0 = time.time()
        self.size = None
        self.last_error = None
        self.connected = False
        self.last_frame_at = 0.0

    # ---------- 统计 ----------
    def fps(self):
        dt = time.time() - self.t0
        return self.frames / dt if dt > 0 else 0.0

    def mbps(self):
        dt = time.time() - self.t0
        return (self.bytes * 8 / 1e6) / dt if dt > 0 else 0.0

    # ---------- 录制 ----------
    def start_record(self, outdir):
        os.makedirs(outdir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        with self._lock:
            if self._rec_dir is not None:
                return self._rec_dir
            if self.mode == "jpeg":
                path = os.path.join(outdir, "预览录像_%s.mjpg" % stamp)
                self._rec_fp = open(path, "wb")
                self._rec_dir = path
            else:
                self._rec_dir = os.path.join(outdir, "帧序列_%s" % stamp)
                os.makedirs(self._rec_dir, exist_ok=True)
                self._rec_count = 0
        return self._rec_dir

    def stop_record(self):
        with self._lock:
            path = self._rec_dir
            if self._rec_fp:
                try:
                    self._rec_fp.close()
                except Exception:
                    pass
            self._rec_fp = None
            self._rec_dir = None
        return path

    @property
    def recording(self):
        return self._rec_dir is not None

    # ---------- 主循环 ----------
    def run(self):
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while not self._halt.is_set():
            try:
                if self.connection_guard:
                    self.connection_guard()
                req = urllib.request.Request(self.url, headers={"User-Agent": "carapp"})
                response = opener.open(req, timeout=8)
                self._resp = response
                if self._halt.is_set():
                    break
                self.log("摄像头已连接 %s" % self.url)
                self.frames = self.bytes = 0
                self.t0 = time.time()
                self.size = None
                buf = bytearray()
                checked = time.monotonic()
                while not self._halt.is_set():
                    if self.connection_guard and time.monotonic() - checked >= 4:
                        self.connection_guard()
                        checked = time.monotonic()
                    chunk = response.read1(8192)
                    if not chunk:
                        raise ConnectionError("流已结束")
                    buf.extend(chunk)
                    while True:
                        frame = self._cut_frame(buf)
                        if not frame:
                            break
                        self._deliver(frame)
            except Exception as exc:
                self.last_error = "摄像头连接中断：%s" % exc
                if not self._halt.is_set():
                    self.log(self.last_error + "，3 秒后重连")
            finally:
                self.connected = False
                if self._resp:
                    try:
                        self._resp.close()
                    except Exception:
                        pass
                self._resp = None
            self._halt.wait(self.RETRY_SECONDS)
        self.stop_record()

    def _cut_frame(self, buf):
        """从缓冲区切出一帧，切不出返回 None。"""
        if self.mode == "jpeg":
            a = buf.find(JPEG_SOI)
            if a < 0:
                if len(buf) > 4 * 1024 * 1024:
                    del buf[:-2]
                return None
            b = buf.find(JPEG_EOI, a + 2)
            if b < 0:
                if a > 0:
                    del buf[:a]
                return None
            data = bytes(buf[a:b + 2])
            del buf[:b + 2]
            return data

        a = buf.find(PNG_SIG)
        if a < 0:
            if len(buf) > 4 * 1024 * 1024:
                del buf[:-8]
            return None
        b = buf.find(b"IEND", a + 8)
        if b < 0:
            if a > 0:
                del buf[:a]
            return None
        data = bytes(buf[a:b + 8])  # IEND + 4 字节 CRC
        del buf[:b + 8]
        return data

    def _deliver(self, payload):
        if self._halt.is_set():
            return
        self.connected = True
        self.last_error = None
        self.last_frame_at = time.monotonic()
        self.frames += 1
        self.bytes += len(payload)
        if self.size is None:
            self.size = parse_size(payload, self.mode)

        rec = self._rec_dir
        if rec:
            try:
                if self._rec_fp:
                    self._rec_fp.write(payload)
                else:
                    self._rec_count += 1
                    with open(os.path.join(rec, "%05d.%s" % (self._rec_count, self.mode), "wb")) as f:
                        f.write(payload)
            except Exception:
                pass

        # 只保留最新一帧，防止显示端积压导致越来越延迟
        try:
            while True:
                self.frame_q.get_nowait()
        except queue.Empty:
            pass
        try:
            self.frame_q.put_nowait((time.time(), payload))
        except queue.Full:
            pass

    def stop(self):
        self._halt.set()
        self.connected = False
        response = self._resp
        if response:
            # HTTPResponse.close 可能等待阻塞读取结束，不阻塞 Tk 主线程。
            def close_response():
                try:
                    response.close()
                except Exception:
                    pass
            threading.Thread(target=close_response, daemon=True).start()
