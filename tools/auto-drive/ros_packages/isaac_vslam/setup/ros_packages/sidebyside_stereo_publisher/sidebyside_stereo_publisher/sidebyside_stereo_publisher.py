import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_ros
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class SideBySideStereoPublisher(Node):

    def __init__(self):
        super().__init__('sidebyside_stereo_publisher')

        # QoS プロファイルの設定
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Callback groups
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        # params
        self.declare_parameter('video_dir', '')
        self.declare_parameter('baseline', 0.065)
        self.declare_parameter('img_size_x', 512)
        self.declare_parameter('img_size_y', 512)
        self.declare_parameter('fov_h', 90)
        self.declare_parameter('fov_v', 90)
        self.declare_parameter('fps_timer', 15.0)
        self.declare_parameter('fps_timestamp', 30.0)
        self.declare_parameter('preload_count', 2000)
        self.declare_parameter('publish_range_start', '0/1')  # 新しいパラメータ
        self.declare_parameter('publish_range_end', '1/1')    # 新しいパラメータ
        self.video_dir = self.get_parameter('video_dir').value
        self.baseline = self.get_parameter('baseline').value
        self.img_size_x = self.get_parameter('img_size_x').value
        self.img_size_y = self.get_parameter('img_size_y').value
        self.fov_h = self.get_parameter('fov_h').value
        self.fov_v = self.get_parameter('fov_v').value
        self.fps_timer = self.get_parameter('fps_timer').value
        self.fps_timestamp = self.get_parameter('fps_timestamp').value
        self.preload_count = self.get_parameter('preload_count').value
        self.publish_range_start = self.get_parameter('publish_range_start').value
        self.publish_range_end = self.get_parameter('publish_range_end').value
        self.get_logger().info(f'[video_dir]: {self.video_dir}')
        self.get_logger().info(f'[baseline]: {self.baseline}')
        self.get_logger().info(f'[img_size_x]: {self.img_size_x}')
        self.get_logger().info(f'[img_size_y]: {self.img_size_y}')
        self.get_logger().info(f'[fov_h]: {self.fov_h}')
        self.get_logger().info(f'[fov_v]: {self.fov_v}')
        self.get_logger().info(f'[fps_timer]: {self.fps_timer}')
        self.get_logger().info(f'[fps_timestamp]: {self.fps_timestamp}')
        self.get_logger().info(f'[preload_count]: {self.preload_count}')
        self.get_logger().info(f'[publish_range_start]: {self.publish_range_start}')
        self.get_logger().info(f'[publish_range_end]: {self.publish_range_end}')

        # パブリッシャーの作成 (QoS 適用)
        self.left_pub = self.create_publisher(Image, '/stereo_camera/left/image', qos)
        self.right_pub = self.create_publisher(Image, '/stereo_camera/right/image', qos)
        self.left_info_pub = self.create_publisher(CameraInfo, '/camera_info_left', qos)
        self.right_info_pub = self.create_publisher(CameraInfo, '/camera_info_right', qos)
        self.transform_pub = self.create_publisher(TransformStamped, '/stereo_camera/stereo_transform', qos)
        self.timer = self.create_timer(1.0/self.fps_timer, self.timer_callback, callback_group=self.timer_cb_group)

        # TF2ブロードキャスターの設定
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.cv_bridge = CvBridge()
        self.image_dir = self.video_dir
        self.image_dir_left = self.video_dir + "/frames_left"
        self.image_dir_right = self.video_dir + "/frames_right"
        self.image_files_left = sorted([f for f in os.listdir(self.image_dir_left) if f.lower().endswith(('.png', '.jpg', '.jpeg'))])
        self.image_files_right = sorted([f for f in os.listdir(self.image_dir_right) if f.lower().endswith(('.png', '.jpg', '.jpeg'))])

        # publish_rangeの処理
        start_numerator, start_denominator = map(int, self.publish_range_start.split('/'))
        end_numerator, end_denominator = map(int, self.publish_range_end.split('/'))
        total_images = len(self.image_files_left)
        self.start_index = int(total_images * start_numerator / start_denominator)
        self.end_index = int(total_images * end_numerator / end_denominator)
        self.current_index = self.start_index
        self.total_images_to_publish = self.end_index - self.start_index
        self.get_logger().info(f'Publishing images from index {self.start_index} to {self.end_index} (total: {self.total_images_to_publish})')

        # 画像をプリロード
        self.preloaded_images = self.preload_images(min(self.preload_count, self.total_images_to_publish))

        # Create camera info messages
        self.left_info = self.create_camera_info(True)
        self.right_info = self.create_camera_info(False)
        self.stereo_transform = self.create_stereo_transform()
        self.base_to_left_camera_transform = self.create_base_to_camera_transform()

        # Initialize the timestamp
        self.last_timestamp = self.get_clock().now()

    def create_camera_info(self, is_left):
        camera_info = CameraInfo()
        camera_info.header.frame_id = "left_camera_frame" if is_left else "right_camera_frame"

        # Set image size
        camera_info.width = self.img_size_x
        camera_info.height = self.img_size_y

        # Calculate focal length based on FOV
        fov_horizontal = math.radians(self.fov_h)  # Convert to radians
        fov_vertical = math.radians(self.fov_v)  # Convert to radians

        fx = camera_info.width / (2 * math.tan(fov_horizontal / 2))
        fy = camera_info.height / (2 * math.tan(fov_vertical / 2))

        # Principal point (assuming center of image)
        cx = camera_info.width / 2
        cy = camera_info.height / 2

        # Set camera matrix
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        # Set the projection matrix
        if is_left:
            camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        else:
            # For right camera, include baseline in the projection matrix
            tx = -fx * self.baseline
            camera_info.p = [fx, 0.0, cx, tx, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        # Set distortion coefficients
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Set the rectification matrix to identity
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Set distortion model
        camera_info.distortion_model = "plumb_bob"

        return camera_info

    def create_stereo_transform(self):
        transform = TransformStamped()
        transform.header.frame_id = "left_camera_frame"
        transform.child_frame_id = "right_camera_frame"

        # Calculate baseline from the projection matrix
        transform.transform.translation.x = self.baseline
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # No rotation (identity quaternion)
        return transform

    def create_base_to_camera_transform(self):
        transform = TransformStamped()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "left_camera_frame"

        # Set the position of the left camera relative to base_link
        # These values should be adjusted based on your robot's configuration
        transform.transform.translation.x = 0.1  # 10cm forward from base_link
        transform.transform.translation.y = 0.0  # Centered
        transform.transform.translation.z = 0.05  # 5cm up from base_link

        # Set rotation (assuming camera is facing forward and level)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        return transform

    def preload_images(self, num_images):
        preloaded = []
        total_images = min(num_images, self.total_images_to_publish)
        for i in range(total_images):
            if i % 100 == 0 or i == total_images - 1:
                self.get_logger().info(f'Preloading images: {i+1}/{total_images}')
            left_path = os.path.join(self.image_dir_left, self.image_files_left[self.start_index + i])
            right_path = os.path.join(self.image_dir_right, self.image_files_right[self.start_index + i])
            left_img = cv2.imread(left_path)
            right_img = cv2.imread(right_path)
            if left_img is not None and right_img is not None:
                preloaded.append((left_img, right_img))
            else:
                self.get_logger().warn(f'Failed to preload image: {left_path} or {right_path}')
        self.get_logger().info(f'Preloading complete. {len(preloaded)} images loaded.')
        return preloaded

    def timer_callback(self):
        if self.current_index < self.end_index:
            if self.current_index % 2 != 0:
                preload_index = self.current_index - self.start_index
                if preload_index < len(self.preloaded_images):
                    cv_image_left, cv_image_right = self.preloaded_images[preload_index]
                else:
                    image_path_left = os.path.join(self.image_dir_left, self.image_files_left[self.current_index])
                    image_path_right = os.path.join(self.image_dir_right, self.image_files_right[self.current_index])
                    cv_image_left = cv2.imread(image_path_left)
                    cv_image_right = cv2.imread(image_path_right)

                if cv_image_left is not None and cv_image_right is not None:
                    # ROS2メッセージに変換
                    left_ros_image = self.cv_bridge.cv2_to_imgmsg(cv_image_left, "bgr8")
                    right_ros_image = self.cv_bridge.cv2_to_imgmsg(cv_image_right, "bgr8")

                    # タイムスタンプの設定
                    current_time = self.last_timestamp + rclpy.duration.Duration(seconds=1.0/self.fps_timestamp)
                    self.last_timestamp = current_time
                    stamp_msg = current_time.to_msg()

                    left_ros_image.header.stamp = stamp_msg
                    right_ros_image.header.stamp = stamp_msg
                    self.left_info.header.stamp = stamp_msg
                    self.right_info.header.stamp = stamp_msg
                    self.stereo_transform.header.stamp = stamp_msg
                    self.base_to_left_camera_transform.header.stamp = stamp_msg

                    # フレームIDの設定
                    left_ros_image.header.frame_id = "left_camera_frame"
                    right_ros_image.header.frame_id = "right_camera_frame"

                    # Add debug output
                    if self.current_index == self.start_index:
                        self.get_logger().info(f"Publishing images with timestamp: {stamp_msg}")
                        self.get_logger().info(f"Left camera info: K={self.left_info.k}, P={self.left_info.p}")
                        self.get_logger().info(f"Right camera info: K={self.right_info.k}, P={self.right_info.p}")

                    # 同期して発行
                    self.left_pub.publish(left_ros_image)
                    self.right_pub.publish(right_ros_image)
                    self.left_info_pub.publish(self.left_info)
                    self.right_info_pub.publish(self.right_info)
                    self.transform_pub.publish(self.stereo_transform)

                    # TF2を使用して変換をブロードキャスト
                    self.tf_broadcaster.sendTransform(self.base_to_left_camera_transform)
                    self.tf_broadcaster.sendTransform(self.stereo_transform)

                    # self.get_logger().info(f'Publishing: {self.image_files_left[self.current_index]}')
                    self.current_index += 1
                else:
                    self.get_logger().warn(f'Failed to read image: {image_path_left} or {image_path_right}')
            else:
                self.current_index += 1
        else:
            self.get_logger().info(f'Published {self.total_images_to_publish} out of {len(self.image_files_left)} images')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    stereo_image_publisher = SideBySideStereoPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(stereo_image_publisher)
    executor.spin()
    stereo_image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()