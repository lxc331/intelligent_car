# -*- coding: utf-8 -*-
"""Small closed-loop drive helper that runs inside the car's ROS2 container.

Usage (piped through docker exec):
  python3 - status
  python3 - fwd 0.5 0.05
  python3 - back 0.3 0.05
  python3 - turn 90 0.5
It reads /odom + /scan_raw, publishes /controller/cmd_vel, always stops motors at exit,
and prints one JSON line with the measured result.
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

CMD_TOPIC = "/controller/cmd_vel"
GUARD_STOP = 0.25
GUARD_SLOW = 0.45


class Driver(Node):
    def __init__(self):
        super().__init__("agent_drive")
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(LaserScan, "/scan_raw", self.on_scan, 20)
        self.odom = None
        self.scan = None

    def on_odom(self, msg):
        self.odom = msg

    def on_scan(self, msg):
        self.scan = msg

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_data(self, timeout=3.0):
        return self.wait(lambda: self.odom is not None and self.scan is not None, timeout)

    def wait(self, predicate, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def yaw(self):
        q = self.odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def pose(self):
        p = self.odom.pose.pose.position
        return p.x, p.y, self.yaw()

    def sector_min(self, lo_deg, hi_deg):
        if self.scan is None:
            return None
        best = float("inf")
        s = self.scan
        for i, r in enumerate(s.ranges):
            a = math.degrees(s.angle_min + i * s.angle_increment)
            if lo_deg <= a <= hi_deg and math.isfinite(r) and r > s.range_min:
                best = min(best, r)
        return best if math.isfinite(best) else None

    def stop(self):
        for _ in range(4):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def drive(self, vx=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)


def wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "status"
    arg1 = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    arg2 = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
    arg0_radius = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0

    rclpy.init()
    node = Driver()
    result = {"mode": mode, "ok": False, "reason": ""}
    try:
        if not node.wait_data(3.0):
            result["reason"] = "no odom/scan"
            print(json.dumps(result), flush=True)
            return
        x0, y0, yaw0 = node.pose()
        result.update(start=[round(x0, 4), round(y0, 4)], start_yaw_deg=round(math.degrees(yaw0), 2))
        node.spin(0.3)
        front = node.sector_min(-35, 35)
        result["front_min"] = None if front is None else round(front, 3)

        if mode == "status":
            result.update(
                pose=[round(v, 4) for v in node.pose()],
                left=node.sector_min(35, 90),
                right=node.sector_min(-90, -35),
            )
            result["ok"] = True

        elif mode == "fwd":
            target = arg1
            speed = min(0.06, abs(arg2) or 0.05)
            deadline = time.time() + target / speed * 1.8 + 6.0
            while time.time() < deadline:
                x, y, _ = node.pose()
                travelled = math.hypot(x - x0, y - y0)
                front = node.sector_min(-35, 35)
                result["front_min"] = None if front is None else round(front, 3)
                if front is not None and front < GUARD_STOP:
                    result["reason"] = "obstacle %.3f m" % front
                    break
                if travelled >= target:
                    result["ok"] = True
                    break
                v = speed
                if front is not None and front < GUARD_SLOW:
                    v = speed * 0.5
                yaw_err = wrap(yaw0 - node.pose()[2])
                wz = max(-0.25, min(0.25, 2.5 * yaw_err))
                node.drive(v, wz)
                node.spin(0.02)
            result["travelled"] = round(math.hypot(node.pose()[0] - x0, node.pose()[1] - y0), 4)
            if result["reason"] == "" and result.get("travelled", 0) >= target - 0.01:
                result["ok"] = True

        elif mode == "back":
            target = arg1
            speed = -min(0.05, abs(arg2) or 0.05)
            deadline = time.time() + target / abs(speed) * 1.8 + 6.0
            while time.time() < deadline:
                x, y, _ = node.pose()
                travelled = math.hypot(x - x0, y - y0)
                rear = node.sector_min(145, 180) if node.sector_min(145, 180) is not None else node.sector_min(-180, -145)
                if rear is not None and rear < GUARD_STOP:
                    result["reason"] = "obstacle behind %.3f m" % rear
                    break
                if travelled >= target:
                    result["ok"] = True
                    break
                node.drive(speed, 0.0)
                node.spin(0.02)
            result["travelled"] = round(math.hypot(node.pose()[0] - x0, node.pose()[1] - y0), 4)

        elif mode == "arc":
            # arc <length_m> <speed> <radius_m>; positive radius = left (CCW)
            target = abs(arg1)
            speed = min(0.06, abs(arg2) or 0.05)
            radius = abs(arg0_radius)
            if radius < 0.15:
                result["reason"] = "radius too small"
            else:
                wz_sign = 1.0 if arg1 * arg0_radius >= 0 else -1.0
                wz = wz_sign * speed / radius
                deadline = time.time() + target / speed * 1.8 + 8.0
                while time.time() < deadline:
                    x, y, _ = node.pose()
                    travelled = math.hypot(x - x0, y - y0)
                    front = node.sector_min(-35, 35)
                    result["front_min"] = None if front is None else round(front, 3)
                    if front is not None and front < GUARD_STOP:
                        result["reason"] = "obstacle %.3f m" % front
                        break
                    if travelled >= target:
                        result["ok"] = True
                        break
                    node.drive(speed, wz)
                    node.spin(0.02)
            result["travelled"] = round(math.hypot(node.pose()[0] - x0, node.pose()[1] - y0), 4)
            result["wz"] = round(wz, 3) if radius >= 0.15 else None

        elif mode == "turn":
            target = math.radians(arg1)
            max_w = min(0.6, abs(arg2) or 0.5)
            deadline = time.time() + abs(target) / max_w * 2.0 + 8.0
            while time.time() < deadline:
                err = wrap(yaw0 + target - node.pose()[2])
                if abs(err) < math.radians(2.5):
                    result["ok"] = True
                    result["reason"] = ""
                    break
                w = math.copysign(min(max_w, max(0.08, abs(err))), err)
                nearest = node.sector_min(-180, 180)
                if nearest is not None and nearest < 0.10:
                    result["reason"] = "too close to rotate %.3f m" % nearest
                    break
                node.drive(0.0, w)
                node.spin(0.02)
            result["yaw_deg"] = round(math.degrees(node.pose()[2]), 2)
        else:
            result["reason"] = "unknown mode"

        node.spin(0.1)
        x1, y1, yaw1 = node.pose()
        result.update(end=[round(x1, 4), round(y1, 4)], end_yaw_deg=round(math.degrees(yaw1), 2))
        result["front_min_after"] = None if node.sector_min(-35, 35) is None else round(node.sector_min(-35, 35), 3)
    except Exception as exc:  # noqa: BLE001 - always report and stop
        result["reason"] = "%s: %s" % (type(exc).__name__, exc)
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
