#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OpenVINS bridge (ROS1 / ROS2 switchable)

Usage:
  ROS1:
    source /opt/ros/noetic/setup.bash
    python3 openvins_bridge.py --ros 1

  ROS2:
    source /opt/ros/humble/setup.bash
    python3 openvins_bridge.py --ros 2
"""

import argparse
import json
import time
import base64
import threading
import xml.etree.ElementTree as ET
import re

import redis
import cv2
import numpy as np


# ============================================================
# Common utilities (ROS-independent)
# ============================================================

def split_stereo_image(jpeg_bytes):
    """Decode JPEG and split into left/right images"""
    jpg = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    img = cv2.imdecode(jpg, cv2.IMREAD_COLOR)

    h, w = img.shape[:2]
    half = w // 2
    left = img[:, :half].copy()
    right = img[:, half:half * 2].copy()
    return left, right


def parse_xml_timestamp(xml_bytes):
    """Parse timestamp from XML header"""
    xml_data = xml_bytes[4:].decode("utf-8")
    xml_data = re.sub(r'</?\w+:(\w+)', r'<\1', xml_data)
    root = ET.fromstring(xml_data)
    sec, usec = root.attrib["timestamp"].split(",")
    return int(sec), int(usec) * 1000  # nsec


# ============================================================
# ROS1 implementation
# ============================================================

class ROS1Bridge:
    def __init__(self):
        import rospy
        from sensor_msgs.msg import Imu, Image

        self.rospy = rospy
        self.Imu = Imu
        self.Image = Image

        rospy.init_node("openvins_bridge", anonymous=False)

        self.imu_pub = rospy.Publisher("/imu0", Imu, queue_size=200)
        self.cam0_pub = rospy.Publisher("/cam0/image_raw", Image, queue_size=2)
        self.cam1_pub = rospy.Publisher("/cam1/image_raw", Image, queue_size=2)

    def make_stamp(self, sec, nsec):
        return self.rospy.Time(secs=sec, nsecs=nsec)

    def publish_imu(self, stamp, gyro, accel):
        msg = self.Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "imu"

        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = float(gyro["x"])
        msg.angular_velocity.y = float(gyro["y"])
        msg.angular_velocity.z = float(gyro["z"])

        msg.linear_acceleration.x = float(accel["x"])
        msg.linear_acceleration.y = float(accel["y"])
        msg.linear_acceleration.z = float(accel["z"])

        self.imu_pub.publish(msg)

    def publish_image(self, mat, frame_id, stamp):
        msg = self.Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = mat.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = mat.tobytes()
        if frame_id == "cam0":
            self.cam0_pub.publish(msg)
        else:
            self.cam1_pub.publish(msg)


# ============================================================
# ROS2 implementation
# ============================================================

class ROS2Bridge:
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Imu, Image

        rclpy.init()
        self.node = Node("openvins_bridge")

        self.Imu = Imu
        self.Image = Image

        self.imu_pub = self.node.create_publisher(Imu, "/imu0", 200)
        self.cam0_pub = self.node.create_publisher(Image, "/cam0/image_raw", 2)
        self.cam1_pub = self.node.create_publisher(Image, "/cam1/image_raw", 2)

    def make_stamp(self, sec, nsec):
        return {"sec": sec, "nanosec": nsec}

    def publish_imu(self, stamp, gyro, accel):
        msg = self.Imu()
        msg.header.stamp.sec = stamp["sec"]
        msg.header.stamp.nanosec = stamp["nanosec"]
        msg.header.frame_id = "imu"

        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = float(gyro["x"])
        msg.angular_velocity.y = float(gyro["y"])
        msg.angular_velocity.z = float(gyro["z"])

        msg.linear_acceleration.x = float(accel["x"])
        msg.linear_acceleration.y = float(accel["y"])
        msg.linear_acceleration.z = float(accel["z"])

        self.imu_pub.publish(msg)

    def publish_image(self, mat, frame_id, stamp):
        msg = self.Image()
        msg.header.stamp.sec = stamp["sec"]
        msg.header.stamp.nanosec = stamp["nanosec"]
        msg.header.frame_id = frame_id
        msg.height, msg.width = mat.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = mat.tobytes()
        if frame_id == "cam0":
            self.cam0_pub.publish(msg)
        else:
            self.cam1_pub.publish(msg)


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros", type=int, choices=[1, 2], default=2)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=6379)
    args = parser.parse_args()

    ros = ROS1Bridge() if args.ros == 1 else ROS2Bridge()

    r = redis.Redis(host=args.host, port=args.port)
    pubsub = r.pubsub()

    tmp_img = []

    # ===== FPS counters =====
    imu_count = 0
    img_count = 0
    lock = threading.Lock()

    def on_imu(msg):
        nonlocal imu_count
        data = json.loads(msg["data"].decode())
        stamp = ros.make_stamp(
            data["timestamp"]["sec"],
            data["timestamp"]["nanosec"]
        )
        ros.publish_imu(stamp, data["gyro"], data["accel"])

        with lock:
            imu_count += 1

    def on_image(msg):
        nonlocal tmp_img, img_count
        data = msg["data"]

        if len(data) == 0 and tmp_img:
            sec, nsec = parse_xml_timestamp(tmp_img[0])
            stamp = ros.make_stamp(sec, nsec)
            left, right = split_stereo_image(tmp_img[2])
            ros.publish_image(left, "cam0", stamp)
            ros.publish_image(right, "cam1", stamp)

            with lock:
                img_count += 1

            tmp_img = []
        else:
            tmp_img.append(base64.b64decode(data))

    def fps_monitor():
        nonlocal imu_count, img_count
        interval = 5.0
        while True:
            time.sleep(interval)
            with lock:
                imu = imu_count
                img = img_count
                imu_count = 0
                img_count = 0

            print(
                f"[FPS] IMU: {imu / interval:.2f} | "
                f"IMG(stereo): {img / interval:.2f}"
            )

    pubsub.subscribe(**{
        "pserver-imu": on_imu,
        "pserver-forward-pst": on_image
    })

    threading.Thread(target=pubsub.run_in_thread, daemon=True).start()
    threading.Thread(target=fps_monitor, daemon=True).start()

    print("OpenVINS bridge started")
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
