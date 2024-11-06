#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path

try:
    from pyproj import Proj, transform
except ImportError:
    rospy.logerr("pyproj module not found. Please install it using 'pip install pyproj'")
    exit(1)

class DummyGPS:
    def __init__(self):
        rospy.init_node('dummy_gps', anonymous=True)
        
        self.gps_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
        self.vel_pub = rospy.Publisher('gps/vel', TwistStamped, queue_size=10)
        self.path_pub = rospy.Publisher('gps/path', Path, queue_size=10)
        
        self.latitude = 35.681236  # 東京の緯度
        self.longitude = 139.767125  # 東京の経度
        self.altitude = 0.0
        
        self.update_rate = 10  # Hz
        
        try:
            # 座標変換用のプロジェクション
            self.proj_wgs84 = Proj(init='epsg:4326')  # WGS84
            self.proj_utm = Proj(init='epsg:32654')  # UTM zone 54N (東京付近)
        except Exception as e:
            rospy.logerr(f"Error initializing projections: {e}")
            exit(1)
        
        # ホイールオドメトリの位置をサブスクライブ
        rospy.Subscriber('wheel/pose', PoseStamped, self.wheel_pose_callback)
        
        self.current_x = 0
        self.current_y = 0
        self.path = Path()
        self.path.header.frame_id = "odom"

        # GPSノイズのパラメータ
        self.gps_noise_std = 0.1  # メートル単位のGPSノイズの標準偏差

    def wheel_pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

    def add_gps_noise(self, x, y):
        noise_x = np.random.normal(0, self.gps_noise_std)
        noise_y = np.random.normal(0, self.gps_noise_std)
        return x + noise_x, y + noise_y

    def update_position(self):
        try:
            # ノイズを加えたUTM座標
            noisy_x, noisy_y = self.add_gps_noise(self.current_x, self.current_y)
            
            # UTM座標からWGS84（緯度経度）に変換
            lon, lat = transform(self.proj_utm, self.proj_wgs84, noisy_x, noisy_y)
            self.longitude = lon
            self.latitude = lat
        except Exception as e:
            rospy.logerr(f"Error transforming coordinates: {e}")

    def publish_gps_data(self):
        current_time = rospy.Time.now()
        
        gps_msg = NavSatFix()
        gps_msg.header.stamp = current_time
        gps_msg.header.frame_id = "gps"
        gps_msg.latitude = self.latitude
        gps_msg.longitude = self.longitude
        gps_msg.altitude = self.altitude
        gps_msg.position_covariance = [self.gps_noise_std**2, 0.0, 0.0, 
                                       0.0, self.gps_noise_std**2, 0.0, 
                                       0.0, 0.0, self.gps_noise_std**2]
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_pub.publish(gps_msg)
        
        vel_msg = TwistStamped()
        vel_msg.header.stamp = current_time
        vel_msg.header.frame_id = "gps"
        vel_msg.twist.linear.x = 0  # 速度情報は省略
        vel_msg.twist.angular.z = 0
        
        self.vel_pub.publish(vel_msg)
        
        # Update and publish path
        pose = PoseStamped()
        pose.header.stamp = current_time
        pose.header.frame_id = "odom"
        noisy_x, noisy_y = self.add_gps_noise(self.current_x, self.current_y)
        pose.pose.position.x = noisy_x
        pose.pose.position.y = noisy_y
        pose.pose.position.z = 0
        
        self.path.header.stamp = current_time
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

    def run(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.update_position()
            self.publish_gps_data()
            rate.sleep()

if __name__ == '__main__':
    try:
        gps = DummyGPS()
        gps.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error in DummyGPS node: {e}")
