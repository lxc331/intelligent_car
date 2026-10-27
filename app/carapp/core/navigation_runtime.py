"""Owner-scoped car runtime: Nav2 actions, live pose and a leased velocity relay.

Uploaded as runtime.py into /tmp/carapp_nav_TOKEN, never installed in ROS source.
All ROS imports are local to main/runtime construction so the PC app can import
its navigation adapter even when no ROS Python modules are installed.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def atomic(path, value):
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    with temporary.open('x') as stream:
        json.dump(value, stream, allow_nan=False)
    os.replace(temporary, path)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class NavigationRuntime:
    def __init__(self, root, token, speed, vendor_dir):
        import rclpy
        from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry, Path as RosPath
        from lifecycle_msgs.srv import GetState
        from nav2_msgs.action import ComputePathToPose, NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener
        self.root, self.token, self.speed = root, token, speed
        self.rclpy = rclpy
        self.node = rclpy.create_node('carapp_navigation_owner_' + token)
        self.base = self.node.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.pose_publisher = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node)
        self.node.create_subscription(Twist, '/carapp_nav_' + token + '/smoothed_cmd_vel', self._velocity, 10)
        self.node.create_subscription(LaserScan, '/scan_raw', self._scan, qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/odom', self._odom, 10)
        self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, 10)
        self.node.create_subscription(RosPath, '/plan', self._path, 10)
        self.planner = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        self.navigator = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.lifecycle_names = ('map_server', 'amcl', 'controller_server', 'planner_server',
                                'smoother_server', 'behavior_server', 'bt_navigator', 'velocity_smoother')
        self.lifecycle = {name: self.node.create_client(GetState, '/' + name + '/get_state')
                          for name in self.lifecycle_names}
        self.lifecycle_state, self.lifecycle_futures = {}, {}
        self.scan_at = self.odom_at = self.amcl_at = 0.
        self.laser_frame = ''
        self.pose_requested_at = 0.
        self.pose_covariance_ok = False
        self.goal_state = 'idle'
        self.goal_handle = None
        self.cancel_future = None
        self.goal_epoch = -1
        self.revoked_epoch = -1
        self.intent_epoch = 0
        self.lease = dict(at=0., expires=0., allow=False, epoch=0)
        self.path = []
        self.distance_remaining = None
        self.command = None
        self.command_at = 0.
        self.last_sent = False
        self.detail = '导航服务启动中；请标记小车初始位置与朝向'
        self.pending = {}
        self.quitting = False
        self.launch = None
        self.launch_log = None
        self._prepare(vendor_dir)
        self.node.create_timer(.05, self._maintain)
        self.node.create_timer(.10, self._drive)
        self.node.create_timer(.50, self._lifecycle_check)
        self.node.create_timer(.20, self._write_state)

    def _prepare(self, vendor_dir):
        import yaml
        from config_builder import navigation_parameters
        vendor = yaml.safe_load(Path(vendor_dir, 'nav2_params.yaml').read_text())
        controller = yaml.safe_load(Path(vendor_dir, 'nav2_controller_dwb.yaml').read_text())
        params = navigation_parameters(vendor, controller, str(self.root / 'map.yaml'), self.speed)
        # The dimensions were measured at the laser, while Nav2 expects a base
        # frame footprint. Transform all four corners using the actual car TF.
        until = time.monotonic() + 6.
        transform = None
        while time.monotonic() < until:
            self.rclpy.spin_once(self.node, timeout_sec=.1)
            if self.laser_frame:
                try:
                    transform = self.tf.lookup_transform('base_footprint', self.laser_frame, self.rclpy.time.Time())
                    break
                except Exception:
                    pass
        if transform is None:
            raise RuntimeError('无法读取雷达到车体的 TF，暂不启动导航')
        angle = yaw_of(transform.transform.rotation)
        c, s = math.cos(angle), math.sin(angle)
        translation = transform.transform.translation
        corners = [(translation.x + c * x - s * y, translation.y + s * x + c * y)
                   for x, y in ((.24,.105),(.24,-.105),(-.11,-.105),(-.11,.105))]
        for name in ('local_costmap', 'global_costmap'):
            params[name][name]['ros__parameters']['footprint'] = str([list(p) for p in corners])
        (self.root / 'nav2.yaml').write_text(yaml.safe_dump(params, sort_keys=False))
        self.launch_log = (self.root / 'nav2.log').open('w+b', buffering=0)
        self.launch = subprocess.Popen(['ros2', 'launch', str(self.root / 'navigation.launch.py')],
                                       stdin=subprocess.DEVNULL, stdout=self.launch_log,
                                       stderr=subprocess.STDOUT)

    def _scan(self, message):
        self.scan_at = time.monotonic()
        self.laser_frame = message.header.frame_id.lstrip('/')

    def _odom(self, message):
        self.odom_at = time.monotonic()

    def _amcl(self, message):
        covariance = list(message.pose.covariance)
        self.pose_covariance_ok = (len(covariance) == 36 and
                                   all(math.isfinite(x) for x in covariance) and
                                   0 <= covariance[0] <= .50 and 0 <= covariance[7] <= .50 and
                                   0 <= covariance[35] <= 1.)
        self.amcl_at = time.monotonic()

    def _path(self, message):
        if message.header.frame_id != 'map':
            return
        points = [(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation)) for p in message.poses]
        if len(points) <= 20000 and all(all(math.isfinite(v) for v in point) for point in points):
            self.path = points

    def _velocity(self, message):
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(value) for value in values):
            self._revoke('导航速度包含非法数值')
            return
        self.command = (max(0., min(self.speed, message.linear.x)), 0.,
                        max(-.35, min(.35, message.angular.z)))
        self.command_at = time.monotonic()

    def _pose(self):
        try:
            transform = self.tf.lookup_transform('map', 'base_footprint', self.rclpy.time.Time())
            age = (self.node.get_clock().now().nanoseconds -
                   (transform.header.stamp.sec * 1000000000 + transform.header.stamp.nanosec)) / 1e9
            if not -.2 <= age <= 1.5:
                return None
            position = transform.transform.translation
            pose = (position.x, position.y, yaw_of(transform.transform.rotation))
            return pose if all(math.isfinite(value) for value in pose) else None
        except Exception:
            return None

    def _localized(self):
        # AMCL republishes map->odom while stationary, but does not necessarily
        # emit a new PoseWithCovariance. Keep the post-seed confirmation latch;
        # freshness is established by live TF, sensors and lifecycle instead.
        return bool(self.pose_requested_at and self.amcl_at > self.pose_requested_at and
                    self.pose_covariance_ok and self._pose())

    def _ready(self):
        now = time.monotonic()
        return bool(self.launch and self.launch.poll() is None and self._localized() and
                    now - self.scan_at < .7 and now - self.odom_at < 1. and
                    all(self.lifecycle_state.get(name, (0,0))[0] == 3 and
                        now - self.lifecycle_state[name][1] < 3. for name in self.lifecycle_names) and
                    self.planner.server_is_ready() and self.navigator.server_is_ready())

    def _lifecycle_check(self):
        from lifecycle_msgs.srv import GetState
        for name, client in self.lifecycle.items():
            old = self.lifecycle_futures.get(name)
            if old and not old.done():
                continue
            if not client.service_is_ready():
                self.lifecycle_state[name] = (0, time.monotonic())
                continue
            future = client.call_async(GetState.Request())
            self.lifecycle_futures[name] = future
            def receive(done, name=name):
                try:
                    self.lifecycle_state[name] = (done.result().current_state.id, time.monotonic())
                except Exception:
                    self.lifecycle_state[name] = (0, time.monotonic())
            future.add_done_callback(receive)

    def _write_state(self):
        pose = self._pose()
        ready = self._ready()
        running = bool(self.launch and self.launch.poll() is None and not self.quitting)
        atomic(self.root / 'state.json', dict(connected=True, running=running, owned=running,
               ready=ready, localized=self._localized(), pose=pose, goal_state=self.goal_state,
               motion_stopped=not (self.goal_handle or self.last_sent or
                                   self.goal_state in ('accepted','navigating','canceling')),
               path=self.path, distance_remaining=self.distance_remaining,
               detail=self.detail, temporary_paths=[str(self.root / name) for name in
                 ('owner.json','lease.json','intent.lock','runtime.py','config_builder.py',
                  'navigation.launch.py','map.yaml','map.pgm','nav2.yaml','state.json',
                  'supervisor.log','nav2.log')]))

    def _zero(self):
        from geometry_msgs.msg import Twist
        self.base.publish(Twist())
        self.last_sent = False

    def _revoke(self, reason='导航已取消'):
        self.revoked_epoch = max(self.revoked_epoch, self.goal_epoch)
        if self.goal_handle:
            try:
                if self.cancel_future is None:
                    self.cancel_future = self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        if self.goal_state in ('accepted','navigating','planning'):
            self.goal_state = 'canceling' if self.goal_handle else 'canceled'
        self.command = None
        if self.last_sent or self.goal_handle:
            self._zero()
        self.detail = reason

    def _drive(self):
        from geometry_msgs.msg import Twist
        now = time.monotonic()
        allowed = (self.goal_state == 'navigating' and self.goal_handle and
                   self.goal_epoch > self.revoked_epoch and self.lease.get('allow') and
                   int(self.lease.get('epoch',-1)) == self.goal_epoch and
                   now <= self.lease.get('expires',0.) and self._ready())
        if self.goal_state == 'navigating' and not allowed:
            self._revoke('控制租约、定位或传感器失效，导航已停止；需重新点击开始')
        if allowed and self.command is not None and now - self.command_at < .35:
            message = Twist()
            message.linear.x, message.linear.y, message.angular.z = self.command
            self.base.publish(message)
            self.last_sent = any(self.command)
        elif self.last_sent:
            self._zero()

    def _maintain(self):
        now = time.monotonic()
        try:
            lease = json.loads((self.root / 'lease.json').read_text())
            if int(lease['epoch']) >= int(self.lease['epoch']):
                self.lease = lease
        except (OSError, ValueError, KeyError):
            self._revoke('导航控制租约无效')
        if self.goal_state in ('accepted','navigating') and (
                int(self.lease.get('epoch',-1)) > self.goal_epoch or
                not self.lease.get('allow') or now > self.lease.get('expires',0.)):
            self._revoke()
        if now - self.lease.get('at',0.) > 90.:
            self.quitting = True
        if self.launch and self.launch.poll() is not None:
            self._revoke('Nav2 进程已退出')
            self.quitting = True
        for path in sorted(self.root.glob('*.request.json'))[:8]:
            try:
                request = json.loads(path.read_text())
                path.unlink()
                self._request(request)
            except Exception as exc:
                try:
                    self._reply(dict(request_id=path.name.split('.')[0]), error=str(exc))
                except Exception:
                    pass
        for key, pending in list(self.pending.items()):
            if pending['deadline'] < now:
                if pending.get('handle'):
                    pending['handle'].cancel_goal_async()
                self._reply(pending['request'], error='导航计算超时；请重新发起')
                self.pending.pop(key, None)
        if self.launch_log and self.launch_log.tell() > 2097152:
            self.launch_log.seek(0)
            self.launch_log.truncate()

    def _reply(self, request, result=None, error=None):
        atomic(self.root / (request['request_id'] + '.reply.json'),
               dict(result=result or {}, error=error))

    def _stamped(self, x, y, yaw):
        from geometry_msgs.msg import PoseStamped
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = x, y
        message.pose.orientation.z, message.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return message

    def _request(self, request):
        operation = request['op']
        values = request.get('values', {})
        epoch = int(request['epoch'])
        if operation == 'cancel':
            self._revoke()
            # Relay is disabled immediately; the future result confirms Nav2's
            # cancellation separately. Until then goal_state remains canceling.
            if self.goal_handle:
                future = self.cancel_future or self.goal_handle.cancel_goal_async()
                self.goal_state = 'canceling'
                def canceled(done):
                    try:
                        if done.result().return_code != 0:
                            raise RuntimeError('Nav2尚未确认取消；车轮输出已关闭')
                        self.goal_handle = None
                        self.cancel_future = None
                        self.goal_state = 'canceled'
                        self._reply(request, dict(running=True, canceled=True, goal_state='canceled'))
                    except Exception as exc:
                        self._reply(request, error=str(exc))
                future.add_done_callback(canceled)
            else:
                self._reply(request, dict(running=True, canceled=True, goal_state='canceled'))
            return
        if operation in ('plan','navigate') and self.goal_handle:
            raise RuntimeError('请先等待当前导航结束或取消')
        target = tuple(float(values[name]) for name in ('x','y','yaw'))
        if not all(math.isfinite(value) for value in target):
            raise ValueError('目标坐标必须是有限数值')
        if operation == 'initial_pose':
            from geometry_msgs.msg import PoseWithCovarianceStamped
            if self.goal_handle:
                raise RuntimeError('导航期间不能重设定位')
            if any(self.lifecycle_state.get(name,(0,0))[0] != 3 for name in ('amcl','map_server')):
                raise RuntimeError('地图与定位服务尚未激活，请稍候再标记初始位置')
            message = PoseWithCovarianceStamped()
            stamped = self._stamped(*target)
            message.header = stamped.header
            message.pose.pose = stamped.pose
            message.pose.covariance[0] = message.pose.covariance[7] = .04
            message.pose.covariance[35] = .07
            self.pose_requested_at = time.monotonic()
            self.pose_covariance_ok = False
            self.pose_publisher.publish(message)
            self.path = []
            self.detail = '初始位置已提交，等待 AMCL 确认；可低速遥控辅助定位'
            self._reply(request, dict(running=True, localized=False, ready=False))
            return
        if not self._ready():
            raise RuntimeError('导航尚未就绪：请确认初始位置、定位、雷达与 Nav2 生命周期')
        if epoch < int(self.lease.get('epoch',0)) or epoch <= self.revoked_epoch:
            raise RuntimeError('该导航请求已被停止操作作废')
        if operation == 'plan':
            self._plan(request, target)
        elif operation == 'navigate':
            self._navigate(request, target)
        else:
            raise ValueError('Unknown runtime operation')

    def _plan(self, request, target):
        from nav2_msgs.action import ComputePathToPose
        goal = ComputePathToPose.Goal()
        goal.goal = self._stamped(*target)
        goal.planner_id = 'GridBased'
        goal.use_start = False
        key = request['request_id']
        self.pending[key] = dict(request=request, deadline=time.monotonic()+16., handle=None)
        future = self.planner.send_goal_async(goal)
        def accepted(done):
            handle = done.result()
            if key not in self.pending:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.pending.pop(key, None)
                self._reply(request, error='规划器拒绝目标点')
                return
            self.pending[key]['handle'] = handle
            def complete(result):
                if key not in self.pending:
                    return
                self.pending.pop(key, None)
                try:
                    answer = result.result()
                    if answer.status != 4:
                        raise RuntimeError('没有找到可通行路径，请调整目标或地图')
                    self._path(answer.result.path)
                    if len(self.path) < 2:
                        raise RuntimeError('规划路径为空或过短')
                    length = sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(self.path,self.path[1:]))
                    self._reply(request, dict(path=self.path, length=length, running=True, ready=self._ready()))
                except Exception as exc:
                    self._reply(request, error=str(exc))
            handle.get_result_async().add_done_callback(complete)
        future.add_done_callback(accepted)

    def _navigate(self, request, target):
        from nav2_msgs.action import NavigateToPose
        goal = NavigateToPose.Goal()
        goal.pose = self._stamped(*target)
        epoch = int(request['epoch'])
        self.goal_epoch = epoch
        self.goal_state = 'accepted'
        self.command = None
        future = self.navigator.send_goal_async(goal, feedback_callback=self._feedback)
        def accepted(done):
            try:
                handle = done.result()
                if not handle.accepted:
                    raise RuntimeError('Nav2拒绝导航目标')
                self.goal_handle = handle
                self.cancel_future = None
                if epoch <= self.revoked_epoch or int(self.lease.get('epoch',-1)) > epoch:
                    self._revoke()
                    self._reply(request, error='导航已取消，未开始行驶')
                else:
                    self.goal_state = 'navigating'
                    self.detail = '正在导航；动态障碍出现时 Nav2 将重新规划'
                    self._reply(request, dict(running=True, accepted=True, goal_state='navigating'))
                def finished(result):
                    if self.goal_epoch != epoch or self.goal_handle is not handle:
                        return
                    try:
                        code = result.result().status
                        self.goal_state = {4:'succeeded',5:'canceled',6:'aborted'}.get(code,'error')
                        self.detail = {'succeeded':'已到达目标点','canceled':'导航已取消',
                                       'aborted':'导航失败，请检查通路或定位'}.get(self.goal_state,'导航异常结束')
                    except Exception:
                        self.goal_state, self.detail = 'error', '导航结果读取失败'
                    self.revoked_epoch = max(self.revoked_epoch,epoch)
                    self.goal_handle = None
                    self.cancel_future = None
                    self.command = None
                    if self.last_sent:
                        self._zero()
                handle.get_result_async().add_done_callback(finished)
            except Exception as exc:
                self.goal_state = 'error'
                self._revoke(str(exc))
                self._reply(request,error=str(exc))
        future.add_done_callback(accepted)

    def _feedback(self, message):
        distance = float(message.feedback.distance_remaining)
        if math.isfinite(distance):
            self.distance_remaining = distance

    def close(self):
        self.quitting = True
        self._revoke('导航服务已结束')
        if self.launch:
            # launch shares our owner process group. SIGINT sent by the owner
            # RPC reaches every child; lease expiry explicitly signals it too.
            try:
                self.launch.send_signal(signal.SIGINT)
                self.launch.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    self.launch.terminate()
                    self.launch.wait(timeout=2)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
        self._write_state()
        self.node.destroy_node()
        if self.launch_log:
            self.launch_log.close()


def main():
    import rclpy
    root, token, speed, vendor = Path(sys.argv[1]), sys.argv[2], float(sys.argv[3]), sys.argv[4]
    runtime = None
    rclpy.init(args=['--ros-args','--disable-external-lib-logs'])
    try:
        runtime = NavigationRuntime(root, token, speed, vendor)
        runtime._write_state()
        while rclpy.ok() and not runtime.quitting:
            rclpy.spin_once(runtime.node, timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        atomic(root/'state.json', dict(running=False, ready=False, startup_error=str(exc), detail=str(exc)))
        raise
    finally:
        try:
            if runtime:
                runtime.close()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            # Every launched ROS child stays in this private process group.
            # Collect descendants even if ros2 launch has already exited. This
            # check prevents use outside the specifically owned session process.
            if os.getpgid(os.getpid()) == os.getpid() and os.environ.get('CARAPP_NAV_OWNER') == token:
                signal.signal(signal.SIGTERM,signal.SIG_DFL)
                os.killpg(os.getpid(),signal.SIGTERM)


if __name__ == '__main__':
    main()
