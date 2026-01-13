#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS-VIO bridge (ROS1 / ROS2 switchable) + Differential-drive EKF fusion

Adds:
  - SUBSCRIBE "pserver-encoder"
  - EKF predict from encoder counts
  - EKF update from /odom (VIO odom)
  - Publish fused pose back to Redis (equivalent to C++ redis_publish_pose)

Usage:
  ROS1:
    source /opt/ros/noetic/setup.bash
    python3 ROS-VIO_bridge_ekf.py --ros 1

  ROS2:
    source /opt/ros/humble/setup.bash
    python3 ROS-VIO_bridge_ekf.py --ros 2
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
import math

import redis
import cv2
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


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


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


# ============================================================
# EKF (ported from your C++ code)
# State:
#   x = [px, py, pz, yaw, pitch, roll, toff_ns, kv, kw, lrlatio]
# ============================================================

TICKS_PER_REV = 4096.0
WHEEL_RADIUS_M_NOM = 0.027
WHEEL_BASE_M_NOM = 0.200

class DiffDriveEkf:
    def __init__(self):
        self._mtx = threading.Lock()
        self.reset()

    def reset(self):
        with self._mtx:
            self.x = np.zeros((10,), dtype=np.float64)
            self.P = np.zeros((10, 10), dtype=np.float64)

            # variances
            self.P[0, 0] = 1.0   # px
            self.P[1, 1] = 1.0   # py
            self.P[2, 2] = 4.0   # pz
            self.P[3, 3] = 0.5   # yaw
            self.P[4, 4] = 0.5   # pitch
            self.P[5, 5] = 0.5   # roll
            self.P[6, 6] = 1e16  # toff_ns
            self.P[7, 7] = 0.05 * 0.05  # kv
            self.P[8, 8] = 0.05 * 0.05  # kw
            self.P[9, 9] = 0.02 * 0.02  # lrlatio

            self.x[7] = 1.0  # kv
            self.x[8] = 1.0  # kw
            self.x[9] = 0.0  # lr

            self.last_pred_t_ns = 0
            self.last_v_mps = 0.0
            self.last_w_rps = 0.0

    @staticmethod
    def quat_to_ypr_zyx(qw, qx, qy, qz):
        # rotation matrix
        # yaw = atan2(R10, R00)
        # pitch = asin(-R20)
        # roll = atan2(R21, R22)
        # build R from quaternion
        # normalize
        n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if n <= 0:
            return 0.0, 0.0, 0.0
        qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n

        R00 = 1 - 2*(qy*qy + qz*qz)
        R10 = 2*(qx*qy + qw*qz)
        R20 = 2*(qx*qz - qw*qy)
        R21 = 2*(qy*qz + qw*qx)
        R22 = 1 - 2*(qx*qx + qy*qy)

        yaw = math.atan2(R10, R00)
        pitch = math.asin(clamp(-R20, -1.0, 1.0))
        roll = math.atan2(R21, R22)
        return yaw, pitch, roll

    @staticmethod
    def ypr_zyx_to_quat(yaw, pitch, roll):
        # q = qz(yaw) * qy(pitch) * qx(roll)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy*cp*cr + sy*sp*sr
        qx = cy*cp*sr - sy*sp*cr
        qy = cy*sp*cr + sy*cp*sr
        qz = sy*cp*cr - cy*sp*sr
        # normalize
        n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        return qw/n, qx/n, qy/n, qz/n

    def predict_from_encoder_counts(self, enc_t_ns, dL, dR, dt, zupt=False):
        with self._mtx:
            TOFF = 6

            toff_ns = int(round(self.x[TOFF]))
            t_ns = int(enc_t_ns + toff_ns)

            if self.last_pred_t_ns == 0:
                self.last_pred_t_ns = t_ns
                self.last_v_mps = 0.0
                self.last_w_rps = 0.0
                return

            if not (dt > 0.0 and dt < 0.2):
                self.last_pred_t_ns = t_ns
                return

            PX, PY, PZ, YAW, PITCH, ROLL, TOFF, KV, KW, LR = range(10)

            px = self.x[PX]
            py = self.x[PY]
            yaw = self.x[YAW]

            kv = clamp(self.x[KV], 0.5, 1.5)
            kw = clamp(self.x[KW], 0.5, 1.5)
            lr = clamp(self.x[LR], -0.2, 0.2)

            self.x[KV] = kv
            self.x[KW] = kw
            self.x[LR] = lr

            mpt = (2.0 * math.pi * WHEEL_RADIUS_M_NOM) / TICKS_PER_REV

            sL = kv * (1.0 - lr)
            sR = kv * (1.0 + lr)

            dSl = float(dL) * mpt * sL
            dSr = float(dR) * mpt * sR

            B = WHEEL_BASE_M_NOM * kw

            v = (dSr + dSl) * 0.5 / dt
            w = (dSr - dSl) / (B * dt)

            if zupt:
                v = 0.0
                w = 0.0

            v = clamp(v, -5.0, 5.0)
            w = clamp(w, -10.0, 10.0)

            vdt = v * dt
            dyaw = w * dt

            c = math.cos(yaw)
            s = math.sin(yaw)

            self.x[PX] = px + c * vdt
            self.x[PY] = py + s * vdt
            self.x[YAW] = yaw + dyaw

            # Jacobian F
            F = np.eye(10, dtype=np.float64)
            F[PX, YAW] = -s * vdt
            F[PY, YAW] = c * vdt

            A = (mpt / (2.0 * dt)) * (float(dR) * (1.0 + lr) + float(dL) * (1.0 - lr))
            dv_dkv = A
            dv_dlr = kv * (mpt / (2.0 * dt)) * (float(dR) - float(dL))

            C = (mpt / (WHEEL_BASE_M_NOM * dt)) * (float(dR) * (1.0 + lr) - float(dL) * (1.0 - lr))
            dw_dkv = C / kw
            dw_dkw = -w / kw
            dw_dlr = (kv / kw) * (mpt / (WHEEL_BASE_M_NOM * dt)) * (float(dR) + float(dL))

            F[PX, KV] = c * (dv_dkv * dt)
            F[PY, KV] = s * (dv_dkv * dt)
            F[YAW, KV] = (dw_dkv * dt)

            F[YAW, KW] = (dw_dkw * dt)

            F[PX, LR] = c * (dv_dlr * dt)
            F[PY, LR] = s * (dv_dlr * dt)
            F[YAW, LR] = (dw_dlr * dt)

            # Process noise Q
            Q = np.zeros((10, 10), dtype=np.float64)

            sigma_v = 0.5
            sigma_w = 1.0
            if zupt:
                sigma_v = 0.0
                sigma_w = 0.0

            sigma_z = 0.5
            sigma_pr = 0.2
            sigma_toff = 5e5

            sigma_kv = 2e-4
            sigma_kw = 2e-4
            sigma_lr = 5e-5

            q_pos = (sigma_v * sigma_v) * dt * dt
            Q[PX, PX] = q_pos
            Q[PY, PY] = q_pos
            Q[YAW, YAW] = (sigma_w * sigma_w) * dt * dt

            Q[PZ, PZ] = (sigma_z * sigma_z) * dt
            Q[PITCH, PITCH] = (sigma_pr * sigma_pr) * dt
            Q[ROLL, ROLL] = (sigma_pr * sigma_pr) * dt
            Q[TOFF, TOFF] = (sigma_toff * sigma_toff) * dt

            Q[KV, KV] = (sigma_kv * sigma_kv) * dt
            Q[KW, KW] = (sigma_kw * sigma_kw) * dt
            Q[LR, LR] = (sigma_lr * sigma_lr) * dt

            self.P = F @ self.P @ F.T + Q

            self.last_pred_t_ns = t_ns
            self.last_v_mps = v
            self.last_w_rps = w

    def update_from_odom(self, odom_t_ns, p, q):
        """
        p: (x,y,z)
        q: (x,y,z,w)
        """
        with self._mtx:
            PX, PY, PZ, YAW, PITCH, ROLL, TOFF, KV, KW, LR = range(10)

            if self.last_pred_t_ns == 0:
                self._init_from_meas(odom_t_ns, p, q)
                return

            yaw_meas, pitch_meas, roll_meas = self.quat_to_ypr_zyx(q[3], q[0], q[1], q[2])

            z = np.array([p[0], p[1], p[2], yaw_meas, pitch_meas, roll_meas], dtype=np.float64)
            h = np.array([self.x[PX], self.x[PY], self.x[PZ],
                          self.x[YAW], self.x[PITCH], self.x[ROLL]], dtype=np.float64)

            r = z - h
            r[3] = wrap_pi(r[3])

            H = np.zeros((6, 10), dtype=np.float64)
            H[0, PX] = 1.0
            H[1, PY] = 1.0
            H[2, PZ] = 1.0
            H[3, YAW] = 1.0
            H[4, PITCH] = 1.0
            H[5, ROLL] = 1.0

            # timeoffset approx jacobian
            yaw = self.x[YAW]
            c = math.cos(yaw)
            s = math.sin(yaw)
            dpx_dtoff = self.last_v_mps * c * 1e-9
            dpy_dtoff = self.last_v_mps * s * 1e-9
            dyaw_dtoff = self.last_w_rps * 1e-9
            H[0, TOFF] = dpx_dtoff
            H[1, TOFF] = dpy_dtoff
            H[3, TOFF] = dyaw_dtoff

            # Measurement noise R
            R = np.zeros((6, 6), dtype=np.float64)
            sigma_p_xy = 0.001
            sigma_p_z = 0.010
            sigma_yaw = 0.01
            sigma_pr = 0.08

            R[0, 0] = sigma_p_xy * sigma_p_xy
            R[1, 1] = sigma_p_xy * sigma_p_xy
            R[2, 2] = sigma_p_z * sigma_p_z
            R[3, 3] = sigma_yaw * sigma_yaw
            R[4, 4] = sigma_pr * sigma_pr
            R[5, 5] = sigma_pr * sigma_pr

            S = H @ self.P @ H.T + R
            try:
                Sinv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                return

            K = self.P @ H.T @ Sinv

            self.x = self.x + K @ r
            I = np.eye(10, dtype=np.float64)
            self.P = (I - K @ H) @ self.P

            # clamp params
            self.x[TOFF] = clamp(self.x[TOFF], -5e8, 5e8)
            self.x[KV] = clamp(self.x[KV], 0.5, 1.5)
            self.x[KW] = clamp(self.x[KW], 0.5, 1.5)
            self.x[LR] = clamp(self.x[LR], -0.2, 0.2)

            if abs(int(odom_t_ns - self.last_pred_t_ns)) > int(2e9):
                self._init_from_meas(odom_t_ns, p, q)

    def _init_from_meas(self, t_ns, p, q):
        PX, PY, PZ, YAW, PITCH, ROLL, TOFF, KV, KW, LR = range(10)

        yaw, pitch, roll = self.quat_to_ypr_zyx(q[3], q[0], q[1], q[2])

        self.x[PX] = p[0]
        self.x[PY] = p[1]
        self.x[PZ] = p[2]
        self.x[YAW] = yaw
        self.x[PITCH] = pitch
        self.x[ROLL] = roll

        self.last_pred_t_ns = int(t_ns)

        self.P = np.eye(10, dtype=np.float64)
        self.P[PX, PX] = 0.1
        self.P[PY, PY] = 0.1
        self.P[PZ, PZ] = 0.2
        self.P[YAW, YAW] = 0.1
        self.P[PITCH, PITCH] = 0.2
        self.P[ROLL, ROLL] = 0.2
        self.P[TOFF, TOFF] = 1e16
        self.P[KV, KV] = 0.05 * 0.05
        self.P[KW, KW] = 0.05 * 0.05
        self.P[LR, LR] = 0.02 * 0.02

    def get_fused_pose(self):
        with self._mtx:
            qw, qx, qy, qz = self.ypr_zyx_to_quat(self.x[3], self.x[4], self.x[5])
            p = (float(self.x[0]), float(self.x[1]), float(self.x[2]))
            q = (float(qx), float(qy), float(qz), float(qw))  # x,y,z,w
            return p, q

    def get_timeoffset_ns(self):
        with self._mtx:
            return int(round(self.x[6]))

    def get_calib(self):
        with self._mtx:
            return float(self.x[7]), float(self.x[8]), float(self.x[9])


# ============================================================
# Equivalent of C++ redis_publish_pose
# ============================================================

def redis_publish_pose_equiv(r, T_p, T_q_xyzw, t_ns):
    """
    T_p: (x,y,z)
    T_q_xyzw: (x,y,z,w)
    """
    px, py, pz = T_p
    qx, qy, qz, qw = T_q_xyzw

    # ---- publish vio_pose json ----
    j = {
        "t_ns": int(t_ns),
        "p": [px, py, pz],
        "q": [qw, qx, qy, qz],  # C++ order w,x,y,z
    }
    s = json.dumps(j, ensure_ascii=False, indent=2)
    r.publish("vio_pose", s)

    # yaw from quaternion
    # yaw_ccw = atan2(2(wz+xy), 1-2(yy+zz))
    yaw_ccw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    )
    heading = -yaw_ccw * 180.0 / math.pi

    # ---- publish pserver-odometry-info ----
    j2 = {
        "mode": "odom",
        "state": "UPDATE_ODOMETRY",
        "odom": {
            "x": -py,
            "y": px,
            "z": pz,
            "heading": heading,
        },
        "timestamp": float(t_ns) / 1e9,
    }
    s2 = json.dumps(j2, ensure_ascii=False, indent=2)
    r.publish("pserver-odometry-info", s2)

    # ---- publish vehicle_pos csv ----
    yaw_deg = (-yaw_ccw) * 180.0 / math.pi
    csv = f"{-py},{0.13},{px},{yaw_deg},{float(t_ns)/1e9}"
    r.publish("vehicle_pos", csv)


# ============================================================
# ROS1 implementation
# ============================================================

class ROS1Bridge:
    def __init__(self, image_color):
        import rospy
        import tf.transformations as tf_transformations
        from sensor_msgs.msg import Imu, Image
        from nav_msgs.msg import Odometry

        self.rospy = rospy
        self.tf_transformations = tf_transformations
        self.Imu = Imu
        self.Image = Image
        self.Odometry = Odometry

        self.image_color = image_color

        rospy.init_node("ROS-VIO_bridge", anonymous=False)

        self.imu_pub = rospy.Publisher("/imu0", Imu, queue_size=200)
        self.cam0_pub = rospy.Publisher("/cam0/image_raw", Image, queue_size=2)
        self.cam1_pub = rospy.Publisher("/cam1/image_raw", Image, queue_size=2)
        self.vio_odom_pub = rospy.Publisher("/vio/odom", Odometry, queue_size=50)
        self.cam_vis_pub = rospy.Publisher("/vio/camera_pose_visual", MarkerArray, queue_size=10)

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
        return self.rospy.Subscriber(topic, self.Odometry, cb, queue_size=50)


# ============================================================
# ROS2 implementation
# ============================================================

class ROS2Bridge:
    def __init__(self, image_color):
        import rclpy
        from rclpy.node import Node
        import tf_transformations
        from sensor_msgs.msg import Imu, Image
        from nav_msgs.msg import Odometry

        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("ROS-VIO_bridge")
        self.tf_transformations = tf_transformations

        self.Imu = Imu
        self.Image = Image
        self.Odometry = Odometry

        self.image_color = image_color

        self.imu_pub = self.node.create_publisher(Imu, "/imu0", 200)
        self.cam0_pub = self.node.create_publisher(Image, "/cam0/image_raw", 2)
        self.cam1_pub = self.node.create_publisher(Image, "/cam1/image_raw", 2)
        self.vio_odom_pub = self.node.create_publisher(Odometry, "/vio/odom", 50)
        self.cam_vis_pub = self.node.create_publisher(MarkerArray, "/vio/camera_pose_visual", 10)

        self._spin_stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

    def _spin_loop(self):
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
        return self.node.create_subscription(self.Odometry, topic, cb, 50)


# ============================================================
# Main
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ros", type=int, choices=[1, 2], default=2)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=6379)

    parser.add_argument("--vio-odom-topic", default="/odom",
                        help="nav_msgs/Odometry topic from VIO)")
    parser.add_argument("--vio-redis-channel", default="pserver-vio",
                        help="Redis pub channel for raw VIO pose JSON")

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
    ekf = DiffDriveEkf()

    imu_count = 0
    img_count = 0
    vio_count = 0
    enc_count = 0
    lock = threading.Lock()

    stop_event = threading.Event()

    # ===== Encoder prev state =====
    enc_prev = {"t_ns": 0, "left": 0, "right": 0, "have": False}

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

    def on_encoder(msg):
        """
        Expect JSON:
        {
          "timestamp": {"sec":..., "nanosec":...},
          "left": int64,
          "right": int64
        }
        """
        nonlocal enc_count, enc_prev

        try:
            data = json.loads(msg["data"].decode())
            ts = data["timestamp"]
            t_ns = int(ts["sec"]) * 1000000000 + int(ts["nanosec"])
            left = int(data["left"])
            right = int(data["right"])
        except Exception as e:
            print("[enc] parse error:", e)
            return

        if not enc_prev["have"]:
            enc_prev = {"t_ns": t_ns, "left": left, "right": right, "have": True}
            return

        dt_ns = t_ns - enc_prev["t_ns"]
        dt = dt_ns * 1e-9
        if not (dt > 0.0 and dt < 0.2):
            enc_prev = {"t_ns": t_ns, "left": left, "right": right, "have": True}
            return

        dL = left - enc_prev["left"]
        dR = right - enc_prev["right"]

        if abs(dL) <= 1:
            dL = 0
        if abs(dR) <= 1:
            dR = 0

        zupt = (dL == 0 and dR == 0)

        ekf.predict_from_encoder_counts(t_ns, dL, dR, dt, zupt=zupt)

        enc_prev = {"t_ns": t_ns, "left": left, "right": right, "have": True}

        with lock:
            enc_count += 1

    # ----------------------------
    # ROS (VIO Odom) -> EKF update -> Redis fused publish
    # ----------------------------
    def vio_odom_cb(msg):
        nonlocal vio_count

        if args.ros == 1:
            sec = int(msg.header.stamp.secs)
            nsec = int(msg.header.stamp.nsecs)
        else:
            sec = int(msg.header.stamp.sec)
            nsec = int(msg.header.stamp.nanosec)

        t_ns = sec * 1000000000 + nsec

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # raw VIO publish (optional keep)
        payload = {
            "timestamp": {"sec": sec, "nanosec": nsec},
            "position": {"x": float(p.x), "y": float(p.y), "z": float(p.z)},
            "orientation": {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)},
            "frame_id": getattr(msg.header, "frame_id", ""),
            "child_frame_id": getattr(msg, "child_frame_id", ""),
        }
        s = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        r.publish(args.vio_redis_channel, s)

        # ---- EKF update ----
        ekf.update_from_odom(
            t_ns,
            (float(p.x), float(p.y), float(p.z)),
            (float(q.x), float(q.y), float(q.z), float(q.w))
        )

        # ---- publish fused pose (C++ redis_publish_pose equiv) ----
        fused_p, fused_q = ekf.get_fused_pose()
        redis_publish_pose_equiv(r, fused_p, fused_q, t_ns)

        with lock:
            vio_count += 1

    # subscribe VIO odom
    ros.subscribe_vio_odom(args.vio_odom_topic, vio_odom_cb)

    # FPS monitor
    def fps_monitor():
        nonlocal imu_count, img_count, vio_count, enc_count
        interval = 5.0
        while not stop_event.wait(interval):
            with lock:
                imu = imu_count
                img = img_count
                vio = vio_count
                enc = enc_count
                imu_count = 0
                img_count = 0
                vio_count = 0
                enc_count = 0

            kv, kw, lr = ekf.get_calib()
            toff = ekf.get_timeoffset_ns()
            print(
                f"[FPS] IMU: {imu/interval:.2f} | IMG(stereo): {img/interval:.2f} | "
                f"ENC: {enc/interval:.2f} | VIO(odom): {vio/interval:.2f} || "
                f"[EKF] toff_ns={toff} kv={kv:.6f} kw={kw:.6f} lr={lr:.6f}"
            )

    pubsub.subscribe(**{
        "pserver-imu": on_imu,
        "pserver-forward-pst": on_image,
        "pserver-encoder": on_encoder,
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

        if args.ros == 2:
            try:
                ros.shutdown()
            except Exception:
                pass

        sys.exit(exit_code)

    def handle_sigint(sig, frame):
        shutdown_all(0)

    signal.signal(signal.SIGINT, handle_sigint)

    print("ROS-VIO bridge + EKF started (Ctrl+C to stop)")
    print(f"  ROS: {args.ros}")
    print(f"  Redis: {args.host}:{args.port}")
    print(f"  VIO odom topic: {args.vio_odom_topic} -> Redis channel(raw): {args.vio_redis_channel}")
    signal.pause()


if __name__ == "__main__":
    main()
