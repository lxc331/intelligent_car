"""只读检查：rosbridge websocket 握手 + web_video_server 头。"""
import base64
import os
import socket

host, port = "192.168.150.2", 9090
key = base64.b64encode(os.urandom(16)).decode()
req = (
    f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
    f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
    "Sec-WebSocket-Protocol: rosbridge\r\n\r\n"
)
s = socket.create_connection((host, port), timeout=6)
s.sendall(req.encode())
head = s.recv(4096).decode(errors="replace")
print(head.split("\r\n\r\n")[0])
s.close()

req2 = f"GET /stream?topic=/image_raw&type=mjpeg HTTP/1.1\r\nHost: {host}:8080\r\nConnection: close\r\n\r\n"
s2 = socket.create_connection((host, 8080), timeout=6)
s2.sendall(req2.encode())
buf = b""
while len(buf) < 2048:
    chunk = s2.recv(2048)
    if not chunk:
        break
    buf += chunk
print(buf.split(b"\r\n\r\n")[0].decode(errors="replace"))
s2.close()
