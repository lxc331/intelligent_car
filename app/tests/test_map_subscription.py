"""Subscriptions must be established before a delayed SLAM publisher appears."""
import json
import time
import types
import unittest
from unittest.mock import Mock, patch

from carapp.core.rosbridge import RosBridge
from carapp.core.map_data import MapModel
from carapp.main import App
from carapp.model.state import CarState


MESSAGE_TYPE = "nav_msgs/msg/OccupancyGrid"
MAP = dict(header=dict(frame_id="map"),data=[0,100,-1,0],info=dict(width=2,height=2,resolution=.05,
    origin=dict(position=dict(x=0,y=0,z=0),orientation=dict(x=0,y=0,z=0,w=1))))


class MapSubscriptionTests(unittest.TestCase):
    def test_type_is_cached_even_when_initial_send_is_disconnected(self):
        bridge = RosBridge("offline")
        self.assertFalse(bridge.subscribe("/map", 1000, message_type=MESSAGE_TYPE))
        with patch.object(bridge, "_send", return_value=True) as send:
            bridge.subscribe("/map", 1000)
        self.assertEqual(send.call_args.args[0], dict(op="subscribe",topic="/map",
            throttle_rate=1000,type=MESSAGE_TYPE))

    def run_delayed_publisher(self, explicit_type):
        model = MapModel()
        bridge = RosBridge("offline", on_message=lambda topic,msg:model.accept_map(msg,time.monotonic()))
        requests, accepted = [], []
        # Matches the installed rosbridge rule: an unknown topic without a
        # specified type fails once; subsequent publishers do not retry it.
        class Socket:
            last_received = time.monotonic()
            sock = types.SimpleNamespace(settimeout=lambda _:None)
            def send_text(self, raw):
                message = json.loads(raw); requests.append(message)
                accepted.append(message.get("type") == MESSAGE_TYPE)
            def recv_text(self):
                bridge._stop.set()
                if accepted[-1]:
                    return json.dumps(dict(op="publish",topic="/map",msg=MAP))
                return json.dumps(dict(op="status",level="error",msg="TopicNotEstablished"))
            def close(self):
                pass
        bridge.subscribe("/map", 1000, message_type=MESSAGE_TYPE if explicit_type else None)
        with patch("carapp.core.rosbridge.WebSocket", return_value=Socket()):
            bridge._run()
        return requests, model.snapshot()

    def test_initial_subscription_and_reconnect_replay_keep_explicit_type(self):
        requests, snapshot = self.run_delayed_publisher(True)
        self.assertEqual(requests[0]["type"], MESSAGE_TYPE)
        self.assertEqual(snapshot["grid"]["data"], (0,100,-1,0))

    def test_old_typeless_request_misses_delayed_publisher(self):
        requests, snapshot = self.run_delayed_publisher(False)
        self.assertNotIn("type", requests[0])
        self.assertIsNone(snapshot["grid"])

    def test_main_passes_type_metadata_without_breaking_legacy_subscriptions(self):
        app = App.__new__(App)
        app._closing = False
        app.state = CarState()
        app.ctx = types.SimpleNamespace(stop_base=Mock())
        app.bridge = types.SimpleNamespace(subscribe=Mock())
        app._request_nodes = Mock()
        app.features = [types.SimpleNamespace(ros_subscriptions=lambda:{"/map":1000},
            ros_subscription_types=lambda:{"/map":MESSAGE_TYPE}),
            types.SimpleNamespace(ros_subscriptions=lambda:{"/scan_raw":100})]
        app._on_ros_status(True)
        app.bridge.subscribe.assert_any_call("/map",throttle_ms=1000,message_type=MESSAGE_TYPE)
        app.bridge.subscribe.assert_any_call("/scan_raw",throttle_ms=100)


if __name__ == "__main__":
    unittest.main()
