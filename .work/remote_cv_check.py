# -*- coding: utf-8 -*-
"""Check whether OpenCV and /image_raw are available inside the car container."""
import subprocess


def sh(cmd, timeout=30):
    out = subprocess.run(["bash", "-lc", "source /opt/ros/humble/setup.bash; " + cmd],
                         capture_output=True, text=True, timeout=timeout)
    return (out.stdout + out.stderr).strip()


print("cv2:", sh("python3 -c 'import cv2, numpy; print(cv2.__version__, numpy.__version__)'"))
print("image_raw:", sh("ros2 topic info /image_raw 2>&1 | head -6"))
print("image_raw type:", sh("ros2 topic type /image_raw 2>&1"))
print("compressed:", sh("ros2 topic list | grep -i image | head -10"))
