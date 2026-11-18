# -*- coding: utf-8 -*-
"""红绿灯广播接收的离线验证：不占用真实端口，不依赖 Tk 或实车。"""
import json
import socket
import time
import unittest

from carapp.core.signal_receiver import DEFAULT_PORT, RECV_TIMEOUT, SignalReceiver


def wait_until(predicate, timeout=1.):
    end = time.monotonic() + timeout
    while not predicate() and time.monotonic() < end:
        time.sleep(.005)
    if not predicate():
        raise AssertionError("signal receiver did not reach expected state")


def packet(seq=1, **fields):
    message = dict(type="traffic_signal", seq=seq, h="red", v="green",
                   mask_hex="0x21", phase="直行红灯")
    message.update(fields)
    return json.dumps(message, ensure_ascii=False).encode("utf-8")


class FakeSocket:
    """最小 UDP 替身：按顺序吐出报文，没有报文时像真实套接字一样超时。"""

    def __init__(self, packets=(), bind_error=None):
        self.packets = list(packets)
        self.bind_error = bind_error
        self.bound = None
        self.timeout = None
        self.option = None
        self.closed = False

    def setsockopt(self, level, option, value):
        self.option = (level, option, value)

    def settimeout(self, value):
        self.timeout = value

    def bind(self, address):
        if self.bind_error is not None:
            raise self.bind_error
        self.bound = address

    def recvfrom(self, size):
        if self.closed:
            raise OSError("closed")
        if self.packets:
            item = self.packets.pop(0)
            return item if isinstance(item, tuple) else (item, ("10.0.0.1", 9))
        time.sleep(.005)
        raise socket.timeout()

    def close(self):
        self.closed = True


class SignalReceiverTests(unittest.TestCase):
    def receiver(self, sock, **options):
        self.now = [10.]
        options.setdefault("clock", lambda: self.now[0])
        options.setdefault("socket_factory", lambda *args: sock)
        rx = SignalReceiver(**options)
        self.addCleanup(rx.stop)
        return rx

    def test_start_binds_the_default_port_and_reuses_the_address(self):
        sock = FakeSocket()
        rx = self.receiver(sock)
        self.assertTrue(rx.start())
        self.assertEqual(sock.bound, ("", DEFAULT_PORT))
        self.assertEqual(sock.timeout, RECV_TIMEOUT)
        self.assertEqual(sock.option, (socket.SOL_SOCKET, socket.SO_REUSEADDR, 1))
        self.assertTrue(rx.start())  # 重复启动不会新建线程
        self.assertEqual(rx._thread.name, "signal-receiver")

    def test_valid_packet_becomes_the_latest_state(self):
        sock = FakeSocket([(packet(seq=7, h="red"), ("192.168.150.9", 5000))])
        rx = self.receiver(sock)
        rx.start()
        wait_until(lambda: rx.snapshot()["packets"] == 1)
        state = rx.snapshot()
        self.assertEqual(state["state"]["h"], "red")
        self.assertEqual(state["peer"], "192.168.150.9")
        self.assertEqual(state["seq"], 7)
        self.assertEqual(state["age"], 0.)
        self.assertFalse(state["stale"])
        self.assertEqual(state["bad"], 0)
        self.assertIsNone(state["error"])

    def test_wrong_type_and_malformed_payloads_only_count_as_bad(self):
        sock = FakeSocket([b"not json",
                           json.dumps({"type": "other", "seq": 1}).encode(),
                           b"[1, 2, 3]",
                           b"\xff\xfe\x00",
                           packet(seq=2)])
        rx = self.receiver(sock)
        rx.start()
        wait_until(lambda: rx.snapshot()["packets"] == 1)
        state = rx.snapshot()
        self.assertEqual(state["bad"], 4)
        self.assertEqual(state["seq"], 2)  # 坏包不覆盖已有灯态

    def test_absence_beyond_timeout_is_reported_stale(self):
        sock = FakeSocket([(packet(seq=1), ("10.0.0.2", 1))])
        rx = self.receiver(sock, timeout=3.)
        rx.start()
        wait_until(lambda: rx.snapshot()["packets"] == 1)
        self.now[0] = 13.5
        state = rx.snapshot()
        self.assertEqual(state["age"], 3.5)
        self.assertTrue(state["stale"])

    def test_reset_clears_session_data_but_keeps_the_bind_error(self):
        sock = FakeSocket([(packet(seq=5), ("10.0.0.2", 1))])
        rx = self.receiver(sock)
        rx.start()
        rx.last_error = "绑定端口失败"
        wait_until(lambda: rx.snapshot()["packets"] == 1)
        rx.reset()
        state = rx.snapshot()
        self.assertIsNone(state["state"])
        self.assertIsNone(state["peer"])
        self.assertEqual((state["seq"], state["packets"], state["bad"]), (0, 0, 0))
        self.assertIsNone(state["age"])
        self.assertTrue(state["stale"])
        self.assertEqual(state["error"], "绑定端口失败")

    def test_bind_failure_reports_the_reason_without_a_thread(self):
        sock = FakeSocket(bind_error=OSError("address already in use"))
        rx = self.receiver(sock)
        self.assertFalse(rx.start())
        self.assertIn(str(DEFAULT_PORT), rx.last_error)
        self.assertIn("address already in use", rx.last_error)
        self.assertIsNone(rx._thread)
        self.assertTrue(sock.closed)
        self.assertTrue(rx.snapshot()["stale"])

    def test_stop_closes_the_socket_and_a_later_start_listens_again(self):
        first, second = FakeSocket(), FakeSocket([(packet(seq=3), ("10.0.0.3", 1))])
        sockets = [first, second]
        self.now = [10.]
        rx = SignalReceiver(clock=lambda: self.now[0], socket_factory=lambda *args: sockets.pop(0))
        self.addCleanup(rx.stop)
        rx.start()
        thread = rx._thread
        rx.stop()
        self.assertFalse(thread.is_alive())
        self.assertTrue(first.closed)
        self.assertTrue(rx.start())
        wait_until(lambda: rx.snapshot()["seq"] == 3)
        self.assertEqual(rx.snapshot()["peer"], "10.0.0.3")

    def test_on_update_gets_a_snapshot_and_a_failing_callback_cannot_stop_receiving(self):
        seen = []

        def on_update(state):
            seen.append(state)
            raise RuntimeError("界面回调出错")

        sock = FakeSocket([(packet(seq=1), ("10.0.0.4", 1)), (packet(seq=2), ("10.0.0.4", 1))])
        rx = self.receiver(sock, on_update=on_update)
        rx.start()
        wait_until(lambda: rx.snapshot()["packets"] == 2)
        self.assertEqual([state["seq"] for state in seen], [1, 2])
        self.assertEqual(seen[0]["state"]["h"], "red")
        self.assertTrue(rx._thread.is_alive())

    def test_no_callback_runs_for_rejected_payloads(self):
        seen = []
        sock = FakeSocket([b"broken", (packet(seq=1), ("10.0.0.5", 1))])
        rx = self.receiver(sock, on_update=seen.append)
        rx.start()
        wait_until(lambda: rx.snapshot()["packets"] == 1)
        self.assertEqual(len(seen), 1)
        self.assertEqual(seen[0]["seq"], 1)


if __name__ == "__main__":
    unittest.main()
