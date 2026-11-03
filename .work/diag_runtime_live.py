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


rtmod.NavigationRuntime._prepare = fake_prepare
rclpy.init(args=['--ros-args', '--disable-external-lib-logs'])
obj = rtmod.NavigationRuntime(ROOT, 'diagnow', .2, '/home/ubuntu/ros2_ws/src/navigation/config')
now = time.monotonic
t0 = now()
while now() - t0 < 8:
    rclpy.spin_once(obj.node, timeout_sec=0.05)
print('ready', obj._ready())
print('localized', obj._localized())
try:
    print('pose', obj._pose())
except Exception as exc:
    print('pose error', exc)
print('scan age %.2f odom age %.2f amcl age %.2f' % (now() - obj.scan_at, now() - obj.odom_at, now() - obj.amcl_at))
print('lifecycle', {k: (v[0], round(now() - v[1], 2)) for k, v in obj.lifecycle_state.items()})
print('planner', obj.planner.server_is_ready(), 'navigator', obj.navigator.server_is_ready())
print('launch ok', bool(obj.launch and obj.launch.poll() is None))
obj.quitting = True
