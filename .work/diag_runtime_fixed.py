import glob
import os
import sys
import time
from pathlib import Path

roots = sorted(glob.glob('/tmp/carapp_nav_*'), key=os.path.getmtime)
ROOT = Path(roots[-1])
print('session', ROOT)
sys.path.insert(0, str(ROOT))
import rclpy  # noqa: E402
import runtime as rtmod  # noqa: E402


class FakeLaunch:
    def poll(self):
        return None


def fake_prepare(self, vendor_dir):
    self.launch = FakeLaunch()
    self.launch_log = None


def fixed_lifecycle_check(self):
    from lifecycle_msgs.srv import GetState
    now = time.monotonic()
    for name, client in self.lifecycle.items():
        entry = self.lifecycle_futures.get(name)
        if entry and not entry[0].done() and now - entry[1] < 2.0:
            continue
        if not client.service_is_ready():
            self.lifecycle_state[name] = (0, now)
            continue
        future = client.call_async(GetState.Request())

        def receive(done, name=name):
            try:
                self.lifecycle_state[name] = (done.result().current_state.id, time.monotonic())
            except Exception:
                self.lifecycle_state[name] = (0, time.monotonic())

        future.add_done_callback(receive)
        self.lifecycle_futures[name] = (future, now)


rtmod.NavigationRuntime._prepare = fake_prepare
rtmod.NavigationRuntime._lifecycle_check = fixed_lifecycle_check
rclpy.init(args=['--ros-args', '--disable-external-lib-logs'])
obj = rtmod.NavigationRuntime(ROOT, 'diagfix', .2, '/home/ubuntu/ros2_ws/src/navigation/config')
now = time.monotonic
t0 = now()
while now() - t0 < 12:
    rclpy.spin_once(obj.node, timeout_sec=0.05)
print('ready', obj._ready())
print('localized', obj._localized())
print('lifecycle', {k: (v[0], round(now() - v[1], 2)) for k, v in obj.lifecycle_state.items()})
print('pending', [k for k, v in obj.lifecycle_futures.items() if not v[0].done()])
print('planner', obj.planner.server_is_ready(), 'navigator', obj.navigator.server_is_ready())
obj.quitting = True
