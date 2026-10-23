# -*- coding: utf-8 -*-
"""
小车摄像头实时预览（MJPEG over HTTP）

原理：车上 web_video_server 已经在 8080 端口把 ROS 图像话题转成
      multipart/x-mixed-replace 的 MJPEG 流，本程序只负责收流、解码、显示。
      不需要在小车上装任何东西，也不占用小车存储。

用法：
    python 小车摄像头实时预览.py
    python 小车摄像头实时预览.py --host 192.168.150.3 --topic /image_raw
    python 小车摄像头实时预览.py --selftest 20      # 无界面自检，取 20 帧算帧率

依赖：Python 3 + Pillow（本机已装）。不需要 OpenCV。
"""

import argparse
import io
import os
import queue
import sys
import threading
import time
import urllib.request

import tkinter as tk
from tkinter import ttk

from PIL import Image, ImageTk

DEFAULT_HOST = "192.168.150.3"
DEFAULT_PORT = 8080

TOPICS = [
    "/image_raw",
    "/color_track_pick/image_result",
    "/line_following/image_result",
    "/object_tracking/image_result",
    "/llm_control/image_result",
]

SOI = b"\xff\xd8"      # JPEG 起始
EOI = b"\xff\xd9"      # JPEG 结束


def build_url(host, port, topic):
    return "http://%s:%d/stream?topic=%s&type=mjpeg" % (host, port, topic)


def open_stream(url, timeout=10):
    """打开 MJPEG 流，强制绕过系统代理（本机 63362 代理会截胡本地网段）。"""
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    req = urllib.request.Request(url, headers={"User-Agent": "wb-car-viewer"})
    return opener.open(req, timeout=timeout)


class StreamReader(threading.Thread):
    """后台读流线程：解析 JPEG 帧，塞进队列；可选旁路写原始 MJPEG 文件。"""

    def __init__(self, url, frame_q):
        super().__init__(daemon=True)
        self.url = url
        self.frame_q = frame_q
        self._stop = threading.Event()
        self.resp = None
        self.rec_fp = None
        self.rec_path = None
        self.frames = 0
        self.bytes = 0
        self.t0 = time.time()
        self.last_error = None
        self.connected = False

    # ---- 录制控制 ----
    def start_record(self, path):
        self.rec_fp = open(path, "wb")
        self.rec_path = path

    def stop_record(self):
        if self.rec_fp:
            try:
                self.rec_fp.close()
            except Exception:
                pass
        self.rec_fp = None
        p, self.rec_path = self.rec_path, None
        return p

    # ---- 线程主循环 ----
    def run(self):
        buf = bytearray()
        try:
            self.resp = open_stream(self.url)
            self.connected = True
        except Exception as exc:
            self.last_error = "连接失败: %s" % exc
            return

        while not self._stop.is_set():
            try:
                chunk = self.resp.read(8192)
            except Exception as exc:
                self.last_error = "读流中断: %s" % exc
                break
            if not chunk:
                self.last_error = "流已结束"
                break
            buf.extend(chunk)

            # 从缓冲区里切出完整 JPEG 帧
            while True:
                a = buf.find(SOI)
                if a < 0:
                    if len(buf) > 4 * 1024 * 1024:
                        del buf[:-2]
                    break
                b = buf.find(EOI, a + 2)
                if b < 0:
                    if a > 0:
                        del buf[:a]
                    break
                jpg = bytes(buf[a:b + 2])
                rawpart = jpg
                del buf[:b + 2]

                self.frames += 1
                self.bytes += len(rawpart)
                if self.rec_fp:
                    try:
                        self.rec_fp.write(rawpart)
                        self.rec_fp.flush()
                    except Exception:
                        pass

                # 队列只保留最新一帧，避免显示端积压导致"越看越延迟"
                try:
                    while True:
                        self.frame_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    self.frame_q.put_nowait((time.time(), jpg))
                except queue.Full:
                    pass

    def fps(self):
        dt = time.time() - self.t0
        return self.frames / dt if dt > 0 else 0.0

    def mbps(self):
        dt = time.time() - self.t0
        return (self.bytes * 8 / 1e6) / dt if dt > 0 else 0.0

    def stop(self):
        self._stop.set()
        if self.rec_fp:
            self.stop_record()
        try:
            if self.resp:
                self.resp.close()
        except Exception:
            pass


def selftest(host, port, topic, n):
    """无界面自检：抓 n 帧，打印帧率与平均帧大小。"""
    url = build_url(host, port, topic)
    print("URL:", url)
    q = queue.Queue()
    r = StreamReader(url, q)
    r.start()
    got = 0
    t0 = time.time()
    last = None
    while got < n and time.time() - t0 < 30:
        try:
            ts, jpg = q.get(timeout=2)
        except queue.Empty:
            if r.last_error:
                print("ERROR:", r.last_error)
                break
            continue
        got += 1
        last = jpg
        if got == 1:
            img = Image.open(io.BytesIO(jpg))
            print("分辨率:", img.size, img.mode)
    dt = time.time() - t0
    print("收到 %d 帧 / %.2f 秒 -> %.2f fps" % (got, dt, got / dt if dt else 0))
    print("平均帧大小: %.1f KB" % (len(last or b"") / 1024))
    if last:
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "selftest_frame.jpg")
        open(out, "wb").write(last)
        print("样例帧已存:", out)
    r.stop()
    return 0 if got else 1


class App:
    def __init__(self, host, port, topic, outdir):
        self.host = host
        self.port = port
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)

        self.q = queue.Queue(maxsize=1)
        self.reader = None
        self.imgtk = None
        self.last_jpg = None
        self.recording = False
        self.max_w = 800

        self.root = tk.Tk()
        self.root.title("小车摄像头实时预览")
        self.root.configure(bg="#1e1e1e")
        self.root.geometry("920x720")
        self.root.minsize(720, 560)

        bar = tk.Frame(self.root, bg="#2b2b2b")
        bar.pack(fill="x")

        tk.Label(bar, text="主机", fg="#ddd", bg="#2b2b2b").pack(side="left", padx=(8, 2))
        self.e_host = tk.Entry(bar, width=15)
        self.e_host.insert(0, host)
        self.e_host.pack(side="left")

        tk.Label(bar, text="话题", fg="#ddd", bg="#2b2b2b").pack(side="left", padx=(8, 2))
        self.cb_topic = ttk.Combobox(bar, values=TOPICS, width=30)
        self.cb_topic.set(topic)
        self.cb_topic.pack(side="left")

        self.btn_start = tk.Button(bar, text="开始", width=6, command=self.start)
        self.btn_start.pack(side="left", padx=6)
        self.btn_stop = tk.Button(bar, text="停止", width=6, command=self.stop, state="disabled")
        self.btn_stop.pack(side="left")

        self.btn_snap = tk.Button(bar, text="抓帧", width=6, command=self.snapshot, state="disabled")
        self.btn_snap.pack(side="left", padx=6)
        self.btn_rec = tk.Button(bar, text="录制", width=6, command=self.toggle_record, state="disabled")
        self.btn_rec.pack(side="left")

        self.lbl = tk.Label(self.root, bg="#111", fg="#888", text="点击「开始」连接小车摄像头")
        self.lbl.pack(fill="both", expand=True, padx=6, pady=6)

        self.status = tk.Label(self.root, text="未连接", anchor="w", fg="#9cdcfe", bg="#1e1e1e")
        self.status.pack(fill="x", padx=8, pady=(0, 6))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(30, self.tick)
        # 启动后自动连一次，省得还要手动点「开始」
        self.root.after(300, self.start)

    # ---- 控制 ----
    def start(self):
        if self.reader:
            return
        host = self.e_host.get().strip() or DEFAULT_HOST
        topic = self.cb_topic.get().strip()
        self.q = queue.Queue(maxsize=1)
        self.reader = StreamReader(build_url(host, self.port, topic), self.q)
        self.reader.start()
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.btn_snap.config(state="normal")
        self.btn_rec.config(state="normal")
        self.status.config(text="连接中… %s" % build_url(host, self.port, topic))

    def stop(self):
        if self.reader:
            if self.recording:
                self.toggle_record()
            self.reader.stop()
            self.reader = None
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self.btn_snap.config(state="disabled")
        self.btn_rec.config(state="disabled")
        self.status.config(text="已停止")

    def snapshot(self):
        if not self.last_jpg:
            return
        name = time.strftime("抓帧_%Y%m%d_%H%M%S.jpg")
        path = os.path.join(self.outdir, name)
        open(path, "wb").write(self.last_jpg)
        self.status.config(text="已保存 %s (%.1f KB)" % (path, len(self.last_jpg) / 1024))

    def toggle_record(self):
        if not self.reader:
            return
        if not self.recording:
            name = time.strftime("预览录像_%Y%m%d_%H%M%S.mjpg")
            self.reader.start_record(os.path.join(self.outdir, name))
            self.recording = True
            self.btn_rec.config(text="停止录制", relief="sunken")
        else:
            p = self.reader.stop_record()
            self.recording = False
            self.btn_rec.config(text="录制", relief="raised")
            size = os.path.getsize(p) / 1024 / 1024 if p and os.path.exists(p) else 0
            self.status.config(text="录像已存 %s (%.1f MB)" % (p, size))

    # ---- 刷新 ----
    def tick(self):
        try:
            ts, jpg = self.q.get_nowait()
            self.last_jpg = jpg
            img = Image.open(io.BytesIO(jpg))
            if img.width > self.max_w:
                h = int(img.height * self.max_w / img.width)
                img = img.resize((self.max_w, h), Image.BILINEAR)
            self.imgtk = ImageTk.PhotoImage(img)
            self.lbl.config(image=self.imgtk, text="")
            w, h = Image.open(io.BytesIO(jpg)).size
            r = self.reader
            if r:
                state = "录制中 " if self.recording else ""
                self.status.config(
                    text="%s分辨率 %dx%d | 累计 %.2f fps | %.2f Mbps | 收帧 %d%s"
                    % (state, w, h, r.fps(), r.mbps(), r.frames, (" | " + r.last_error) if r.last_error else "")
                )
        except queue.Empty:
            pass
        except Exception as exc:
            self.status.config(text="解码异常: %s" % exc)
        self.root.after(15, self.tick)

    def on_close(self):
        self.stop()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--topic", default="/image_raw")
    ap.add_argument("--outdir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "预览抓帧"))
    ap.add_argument("--selftest", type=int, default=0, help="无界面自检，抓 N 帧后退出")
    a = ap.parse_args()

    if a.selftest:
        return selftest(a.host, a.port, a.topic, a.selftest)

    App(a.host, a.port, a.topic, a.outdir).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
