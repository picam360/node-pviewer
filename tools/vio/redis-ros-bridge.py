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
import signal
import sys

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
    def __init__(self, image_color):
        import rospy
        from sensor_msgs.msg import Imu, Image
        from nav_msgs.msg import Odometry

        self.rospy = rospy
        self.Imu = Imu
        self.Image = Image
        self.Odometry = Odometry

        self.image_color = image_color

        rospy.init_node("openvins_bridge", anonymous=False)

        self.imu_pub = rospy.Publisher("/imu0", Imu, queue_size=200)
        self.cam0_pub = rospy.Publisher("/cam0/image_raw", Image, queue_size=2)
        self.cam1_pub = rospy.Publisher("/cam1/image_raw", Image, queue_size=2)
        self.vio_odom_pub = rospy.Publisher("/vio/odom", Odometry, queue_size=50)

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

        if self.image_color == "mono":
            # BGR → GRAY
            if mat.ndim == 3:
                gray = cv2.cvtColor(mat, cv2.COLOR_BGR2GRAY)
            else:
                gray = mat

            msg.encoding = "mono8"
            msg.step = msg.width
            msg.data = gray.tobytes()

        else:  # bgr
            if mat.ndim == 2:
                mat = cv2.cvtColor(mat, cv2.COLOR_GRAY2BGR)

            msg.encoding = "bgr8"
            msg.step = msg.width * 3
            msg.data = mat.tobytes()

        (self.cam0_pub if frame_id == "cam0" else self.cam1_pub).publish(msg)

    def subscribe_vio_odom(self, topic: str, cb):
        # cb(msg) will be called by rospy
        return self.rospy.Subscriber(topic, self.Odometry, cb, queue_size=50)


# ============================================================
# ROS2
# ============================================================

class ROS2Bridge:
    def __init__(self, image_color):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Imu, Image
        from nav_msgs.msg import Odometry

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("openvins_bridge")

        self.Imu = Imu
        self.Image = Image
        self.Odometry = Odometry

        self.image_color = image_color

        self.imu_pub = self.node.create_publisher(Imu, "/imu0", 200)
        self.cam0_pub = self.node.create_publisher(Image, "/cam0/image_raw", 2)
        self.cam1_pub = self.node.create_publisher(Image, "/cam1/image_raw", 2)
        self.vio_odom_pub = self.node.create_publisher(Odometry, "/vio/odom", 50)

        # ROS2 subscriber callbacks require spinning
        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self):
        # spin in small steps so we can stop
        while not self._spin_stop.is_set():
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def shutdown(self):
        try:
            self._spin_stop.set()
            if self._spin_thread.is_alive():
                self._spin_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self.rclpy.shutdown()
        except Exception:
            pass

    def make_stamp(self, sec, nsec):
        return {"sec": sec, "nanosec": nsec}

    def publish_imu(self, stamp, gyro, accel):
        msg = self.Imu()
        msg.header.stamp.sec = int(stamp["sec"])
        msg.header.stamp.nanosec = int(stamp["nanosec"])
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
        msg.header.stamp.sec = int(stamp["sec"])
        msg.header.stamp.nanosec = int(stamp["nanosec"])
        msg.header.frame_id = frame_id
        msg.height, msg.width = mat.shape[:2]

        if self.image_color == "mono":
            if mat.ndim == 3:
                gray = cv2.cvtColor(mat, cv2.COLOR_BGR2GRAY)
            else:
                gray = mat

            msg.encoding = "mono8"
            msg.step = msg.width
            msg.data = gray.tobytes()

        else:  # bgr
            if mat.ndim == 2:
                mat = cv2.cvtColor(mat, cv2.COLOR_GRAY2BGR)

            msg.encoding = "bgr8"
            msg.step = msg.width * 3
            msg.data = mat.tobytes()
        (self.cam0_pub if frame_id == "cam0" else self.cam1_pub).publish(msg)

    def subscribe_vio_odom(self, topic: str, cb):
        # cb(msg) will be called by rclpy when spun
        return self.node.create_subscription(self.Odometry, topic, cb, 50)


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros", type=int, choices=[1, 2], default=2)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=6379)

    # VIO odom -> Redis
    parser.add_argument("--vio-odom-topic", default="/odom",
                        help="nav_msgs/Odometry topic from VIO)")
    parser.add_argument("--vio-redis-channel", default="pserver-vio",
                        help="Redis pub channel for VIO pose/position JSON")

    parser.add_argument(
        "--image-color",
        choices=["mono", "bgr"],
        default="bgr",
        help="sensor_msgs/Image encoding: mono8 or bgr8"
    )

    args = parser.parse_args()

    ros = (ROS1Bridge if args.ros == 1 else ROS2Bridge)(
        image_color=args.image_color
    )

    r = redis.Redis(host=args.host, port=args.port)
    pubsub = r.pubsub()

    tmp_img = []

    # ===== FPS counters =====
    imu_count = 0
    img_count = 0
    vio_count = 0
    lock = threading.Lock()

    stop_event = threading.Event()

    # ----------------------------
    # Redis -> ROS (IMU + Image)
    # ----------------------------
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

    def on_vio_pose(msg):
        data = json.loads(msg["data"].decode())

        # timestamp
        t_ns = int(data["t_ns"])
        sec = t_ns // 1_000_000_000
        nsec = t_ns % 1_000_000_000

        stamp = ros.make_stamp(sec, nsec)

        # position
        px, py, pz = data["p"]

        # quaternion (w,x,y,z -> x,y,z,w)
        qw, qx, qy, qz = data["q"]

        odom = ros.Odometry()
        if args.ros == 1:
            odom.header.stamp = stamp
        else:
            odom.header.stamp.sec = sec
            odom.header.stamp.nanosec = nsec

        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(px)
        odom.pose.pose.position.y = float(py)
        odom.pose.pose.position.z = float(pz)

        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        odom.pose.pose.orientation.w = float(qw)

        ros.vio_odom_pub.publish(odom)

    # ----------------------------
    # ROS (VIO Odom) -> Redis
    # ----------------------------
    def vio_odom_cb(msg):
        """
        nav_msgs/Odometry:
          msg.header.stamp
          msg.pose.pose.position (x,y,z)
          msg.pose.pose.orientation (x,y,z,w)
        """
        nonlocal vio_count

        # stamp
        if args.ros == 1:
            sec = int(msg.header.stamp.secs)
            nsec = int(msg.header.stamp.nsecs)
        else:
            sec = int(msg.header.stamp.sec)
            nsec = int(msg.header.stamp.nanosec)

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        payload = {
            "timestamp": {"sec": sec, "nanosec": nsec},
            "position": {"x": float(p.x), "y": float(p.y), "z": float(p.z)},
            "orientation": {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)},
            "frame_id": getattr(msg.header, "frame_id", ""),
            "child_frame_id": getattr(msg, "child_frame_id", ""),
        }

        # publish to Redis

        s = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        r.publish(args.vio_redis_channel, s)

        with lock:
            vio_count += 1

    # subscribe VIO odom topic
    ros.subscribe_vio_odom(args.vio_odom_topic, vio_odom_cb)

    # FPS monitor
    def fps_monitor():
        nonlocal imu_count, img_count, vio_count
        interval = 5.0
        while not stop_event.wait(interval):
            with lock:
                imu = imu_count
                img = img_count
                vio = vio_count
                imu_count = 0
                img_count = 0
                vio_count = 0

            print(
                f"[FPS] IMU: {imu / interval:.2f} | "
                f"IMG(stereo): {img / interval:.2f} | "
                f"VIO(odom): {vio / interval:.2f}"
            )

    pubsub.subscribe(**{
        "pserver-imu": on_imu,
        "pserver-forward-pst": on_image,
        "vio_pose": on_vio_pose,
    })

    pubsub_thread = pubsub.run_in_thread(sleep_time=0.01)
    threading.Thread(target=fps_monitor, daemon=True).start()

    def shutdown_all(exit_code=0):
        stop_event.set()

        try:
            pubsub_thread.stop()
            pubsub_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            pubsub.close()
        except Exception:
            pass

        try:
            r.close()
        except Exception:
            pass

        # ROS2 cleanup
        if args.ros == 2:
            try:
                ros.shutdown()
            except Exception:
                pass

        sys.exit(exit_code)

    def handle_sigint(sig, frame):
        shutdown_all(0)

    signal.signal(signal.SIGINT, handle_sigint)

    print("OpenVINS bridge started (Ctrl+C to stop)")
    print(f"  ROS: {args.ros}")
    print(f"  Redis: {args.host}:{args.port}")
    print(f"  VIO odom topic: {args.vio_odom_topic}  -> Redis channel: {args.vio_redis_channel}")
    signal.pause()


if __name__ == "__main__":
    main()
