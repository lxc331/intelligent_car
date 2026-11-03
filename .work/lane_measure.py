# -*- coding: utf-8 -*-
"""Report lane-paint fractions for every frame in /tmp/frames."""
import glob

import cv2

for path in sorted(glob.glob("/tmp/frames/*.jpg")):
    img = cv2.imread(path)
    h, w = img.shape[:2]
    roi = img[int(h * 0.45):int(h * 0.78)]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    yellow = (hue >= 15) & (hue <= 40) & (sat > 80) & (val > 90)
    white = (sat < 45) & (val > 205)
    print(path.split("/")[-1], "yellow=%.4f white=%.4f" % (yellow.mean(), white.mean()))
