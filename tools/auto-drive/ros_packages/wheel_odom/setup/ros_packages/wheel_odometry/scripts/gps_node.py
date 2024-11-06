#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String
import pynmea2
import utm

try:
    from pyproj import Proj, transform
except ImportError:
    rospy.logerr("pyproj module not found. Please install it using 'pip install pyproj'")
    exit(1)

class GpsNode:
    def __init__(self):
        rospy.init_node('gps_node', anonymous=True)
        
        self.gps_pub = rospy.Publisher('gps/fix', NavSatFix, queue_size=10)
        # self.vel_pub = rospy.Publisher('gps/vel', TwistStamped, queue_size=10)
        self.path_pub = rospy.Publisher('gps/path', Path, queue_size=10)
        # 初期位置を保存（UTM基準点用）
        self.initial_lat = None
        self.initial_lon = None
        self.odom_pub = rospy.Publisher('gps/odometry', Odometry, queue_size=10)
        self.subscriber = rospy.Subscriber(
            'gps_nmea',  # rosnodejsで発行しているトピック名
            String,       # メッセージ型
            self.nmea_callback,  # コールバック関数
            queue_size=10
        )
        
        # self.latitude = 35.681236
        # self.longitude = 139.767125
        
        self.cur_nmea = None
        self.current_x = 0
        self.current_y = 0
        self.path = Path()
        self.path.header.frame_id = "odom"


    def nmea_callback(self, msg):
        try:
            nmea_data, timestamp_str = msg.data.split('@')
            nmea = pynmea2.parse(nmea_data)

            if isinstance(nmea, pynmea2.GGA):
                self.cur_nmea = nmea

                rospy.loginfo(
                    f"Received gps data:\n"
                    f"  Timestamp: {timestamp_str}\n"
                    f"  nema: {nmea}"
                )

                if self.initial_lat is None:
                    self.initial_lat = nmea.latitude
                    self.initial_lon = nmea.longitude
                
                # UTM座標に変換
                easting, northing, zone_number, zone_letter = utm.from_latlon(nmea.latitude, nmea.longitude)
                
                if self.initial_lat is not None:

                    timestamp = float(timestamp_str)
                    ros_timestamp = rospy.Time.from_sec(timestamp)

                    # 初期位置からの相対位置を計算
                    initial_easting, initial_northing, _, _ = utm.from_latlon(self.initial_lat, self.initial_lon)
                    
                    # Odometryメッセージの作成
                    odom = Odometry()
                    odom.header.stamp = ros_timestamp
                    odom.header.frame_id = "odom"
                    odom.child_frame_id = "base_link"
                    
                    # 位置の設定
                    odom.pose.pose.position.x = easting - initial_easting
                    odom.pose.pose.position.y = northing - initial_northing
                    odom.pose.pose.position.z = 0.0

                    # 共分散の設定
                    gps_noise_std = 0.0000001  # todo. arrange with fixtype 0.0000001 rad ≈ 0.00000573° ≈ 2cm @ 地球半径
                    odom.pose.covariance = [
                        gps_noise_std**2, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, gps_noise_std**2, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 99999.0
                    ]
                    
                    # 速度は未使用
                    odom.twist.twist.linear.x = 0.0
                    odom.twist.twist.linear.y = 0.0
                    odom.twist.twist.angular.z = 0.0
                    
                    self.odom_pub.publish(odom)

                    pose = PoseStamped()
                    pose.header = odom.header
                    pose.pose = odom.pose.pose
                    self.path.header.stamp = ros_timestamp
                    self.path.poses.append(pose)
                    self.path_pub.publish(self.path)
                
        except pynmea2.ParseError as e:
            rospy.logwarn(f"Parse error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing NMEA message: {e}")

if __name__ == '__main__':
    try:
        gps = GpsNode()
        # gps.run()
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error in GpsNode node: {e}")
