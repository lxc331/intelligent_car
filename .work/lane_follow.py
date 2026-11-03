# -*- coding: utf-8 -*-
"""Lane-keeping drive helper for the sandbox loop (runs inside the car container).

Usage (piped through docker exec):
  python3 - follow 30 0.04      # seconds, speed
It subscribes /image_raw + /scan_raw + /odom, keeps the car on the grey road
between the painted lines, publishes /controller/cmd_vel, stops on obstacle or
when the road is lost, and prints one JSON line with what happened.
"""
import json
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan

CMD_TOPIC = "/controller/cmd_vel"
GUARD_STOP = 0.35
GUARD_SLOW = 0.50
ROI_TOP = 0.45   # fraction of image height where the road sample window starts
ROI_BOTTOM = 0.78


def wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class LaneDriver(Node):
    def __init__(self):
        super().__init__("agent_lane")
        self.pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(LaserScan, "/scan_raw", self.on_scan, 20)
        self.create_subscription(Image, "/image_raw", self.on_image, 2)
        self.odom = None
        self.scan = None
        self.frame = None

    def on_odom(self, msg):
        self.odom = msg

    def on_scan(self, msg):
        self.scan = msg

    def on_image(self, msg):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ("rgb8", "bgr8"):
                img = img.reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                self.frame = img
            elif msg.encoding in ("mono8", "8UC1"):
                self.frame = cv2.cvtColor(img.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
            elif msg.encoding in ("yuv422_yuy2", "yuyv", "yuv422"):
                img = img.reshape(msg.height, msg.width, 2)
                self.frame = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
        except Exception:
            pass

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.03)

    def wait_data(self, timeout=4.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None and self.scan is not None and self.frame is not None:
                return True
        return False

    def pose(self):
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw

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

    def road_offset(self):
        """Return (normalised centre offset, road coverage) or (None, 0)."""
        if self.frame is None:
            return None, 0.0
        img = self.frame
        h, w = img.shape[:2]
        top, bottom = int(h * ROI_TOP), int(h * ROI_BOTTOM)
        roi = img[top:bottom]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        # Road: desaturated grey, not too dark, not the bright paint.
        road = (sat < 55) & (val > 95) & (val < 215)
        # Drop the grassy/bright strips hugging the image border.
        centres = []
        cover = road.mean()
        for row in range(road.shape[0]):
            cols = np.flatnonzero(road[row])
            if cols.size < w * 0.18:
                continue
            # longest run
            splits = np.split(cols, np.flatnonzero(np.diff(cols) > 6) + 1)
            run = max(splits, key=len)
            if len(run) < w * 0.18:
                continue
            centres.append((run[0] + run[-1]) / 2.0 / w)
        if not centres:
            return None, float(cover)
        return float(np.median(centres) - 0.5), float(cover)

    def lane_visible(self):
        """True only when the frame shows real lane paint (yellow centre / white edge)."""
        if self.frame is None:
            return False, 0.0, 0.0
        img = self.frame
        h, w = img.shape[:2]
        roi = img[int(h * ROI_TOP):int(h * ROI_BOTTOM)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)
        yellow = (hue >= 15) & (hue <= 40) & (sat > 80) & (val > 90)
        white = (sat < 45) & (val > 205)
        yellow_frac = float(yellow.mean())
        white_frac = float(white.mean())
        ok = yellow_frac >= 0.004 or white_frac >= 0.020
        return ok, yellow_frac, white_frac

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

    def rotate_by(self, degrees, rate=0.5, timeout=6.0):
        target = wrap(self.pose()[2] + math.radians(degrees))
        end = time.time() + timeout
        while time.time() < end:
            err = wrap(target - self.pose()[2])
            if abs(err) < math.radians(3):
                break
            self.drive(0.0, max(-rate, min(rate, 2.0 * err)))
            rclpy.spin_once(self, timeout_sec=0.04)
            time.sleep(0.02)
        self.stop()

    def escape(self, result):
        """Blocked: turn toward the side with more room, at most 60 degrees."""
        front0 = self.sector_min(-30, 30)
        if front0 is not None and front0 < GUARD_STOP:
            left = self.sector_min(20, 95)
            right = self.sector_min(-95, -20)
            sign = 1.0 if (left or 0) >= (right or 0) else -1.0
            for _ in range(4):
                self.rotate_by(sign * 20.0)
                front = self.sector_min(-30, 30)
                if front is None or front > GUARD_STOP + 0.25:
                    return True
                left = self.sector_min(20, 95)
                right = self.sector_min(-95, -20)
                sign = 1.0 if (left or 0) >= (right or 0) else -1.0
        return False


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    speed = min(0.05, float(sys.argv[2]) if len(sys.argv) > 2 else 0.04)
    if len(sys.argv) > 3:
        speed = float(sys.argv[3])
    rclpy.init()
    node = LaneDriver()
    result = {"mode": "follow", "ok": False, "reason": "", "samples": 0}
    try:
        if not node.wait_data(4.0):
            result["reason"] = "no odom/scan/image"
            print(json.dumps(result), flush=True)
            return
        x0, y0, yaw0 = node.pose()
        result.update(start=[round(x0, 4), round(y0, 4)], start_yaw_deg=round(math.degrees(yaw0), 2))
        deadline = time.time() + seconds
        lost = 0
        while time.time() < deadline:
            front = node.sector_min(-30, 30)
            offset, cover = node.road_offset()
            result["front_min"] = None if front is None else round(front, 3)
            result["offset"] = None if offset is None else round(offset, 3)
            result["cover"] = round(cover, 3)
            if front is not None and front < GUARD_STOP:
                result["escapes"] = result.get("escapes", 0) + 1
                if not node.escape(result):
                    result["reason"] = "obstacle %.3f m, no way around" % front
                    break
                continue
            lane_ok, yfrac, wfrac = node.lane_visible()
            result["lane"] = [round(yfrac, 4), round(wfrac, 4)]
            left = node.sector_min(50, 110)
            right = node.sector_min(-110, -50)
            result["sides"] = [None if left is None else round(left, 2),
                               None if right is None else round(right, 2)]
            # A road lane always has something close on at least one side; open
            # floor (the model playground) does not, and must never be entered.
            if left is not None and right is not None and left > 1.0 and right > 1.0:
                node.drive(0.0, 0.0)
                node.spin(0.1)
                result["reason"] = "open area (sides %.2f/%.2f), not a road" % (left, right)
                break
            if not lane_ok:
                node.drive(0.0, 0.0)
                node.spin(0.1)
                result["reason"] = "lane paint lost (yellow %.3f white %.3f)" % (yfrac, wfrac)
                break
            if offset is None or cover < 0.12:
                lost += 1
                node.drive(0.0, 0.0)
                if lost >= 8:
                    result["reason"] = "road lost"
                    break
                node.spin(0.05)
                continue
            lost = 0
            wz = max(-0.30, min(0.30, -1.8 * offset))
            v = speed
            if front is not None and front < GUARD_SLOW:
                v = speed * 0.55
            node.drive(v, wz)
            result["samples"] += 1
            node.spin(0.05)
        else:
            result["reason"] = "time up"
            result["ok"] = True
        x1, y1, yaw1 = node.pose()
        result.update(end=[round(x1, 4), round(y1, 4)], end_yaw_deg=round(math.degrees(yaw1), 2),
                      travelled=round(math.hypot(x1 - x0, y1 - y0), 3))
    except Exception as exc:  # noqa: BLE001
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

