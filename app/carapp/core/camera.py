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
    def __init__(self, host, http_port, topic, mode, frame_q, log=None):
        super().__init__(daemon=True, name="camera")
        self.host = host
        self.http_port = http_port
        self.topic = topic
        self.mode = mode
        self.frame_q = frame_q
        self.log = log or (lambda text: None)

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
        try:
            req = urllib.request.Request(self.url, headers={"User-Agent": "carapp"})
            self._resp = opener.open(req, timeout=8)
        except Exception as exc:
            self.last_error = "连不上摄像头：%s" % exc
            self.log(self.last_error)
            return

        self.log("摄像头已连接 %s" % self.url)
        buf = bytearray()
        while not self._halt.is_set():
            try:
                chunk = self._resp.read(8192)
            except Exception as exc:
                self.last_error = "读流中断：%s" % exc
                break
            if not chunk:
                self.last_error = "流已结束"
                break
            buf.extend(chunk)

            while True:
                frame = self._cut_frame(buf)
                if not frame:
                    break
                self._deliver(frame)

        try:
            self._resp.close()
        except Exception:
            pass
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
        try:
            if self._resp:
                self._resp.close()
        except Exception:
            pass
