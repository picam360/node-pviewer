#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Quaternion, PoseStamped
from std_msgs.msg import Int32, Float64MultiArray, Header
from tf.transformations import quaternion_from_euler


class WheelOdometry:
    def __init__(self):
        rospy.init_node('wheel_odometry')
        
        self.odom_pub = rospy.Publisher('wheel/odometry', Odometry, queue_size=50)
        self.pose_pub = rospy.Publisher('wheel/pose', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('wheel/path', Path, queue_size=10)
        rospy.Subscriber('wheel_counts', Float64MultiArray, self.wheel_callback)
        
        self.bUpdate = False
        self.ros_timestamp = rospy.Time()
        self.left_counts = 0
        self.right_counts = 0
        self.last_left_counts = 0
        self.last_right_counts = 0
        self.offset_left_counts = None
        self.offset_right_counts = None
        
        self.wheel_separation = 0.15  # meters
        self.wheel_radius = 0.04  # meters
        self.ticks_per_revolution = 360
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_time = rospy.Time.now()
        self.path = Path()
        self.path.header.frame_id = "odom"

    def wheel_callback(self, msg):
        try:
            # データの取り出し
            timestamp = float(msg.data[0])
            self.ros_timestamp = rospy.Time.from_sec(timestamp)
            self.left_counts = int(msg.data[1])
            self.right_counts = int(msg.data[2])
            if self.offset_left_counts is None:
                self.offset_left_counts = self.left_counts
                self.offset_right_counts = self.right_counts
            
            self.left_counts = self.left_counts - self.offset_left_counts
            self.right_counts = self.right_counts - self.offset_right_counts
            
            rospy.loginfo(
                f"Received wheel counts:\n"
                f"  Timestamp: {timestamp}\n"
                f"  Left: {self.left_counts}\n"
                f"  Right: {self.right_counts}"
            )
            print("---------")

            self.update_odometry();

        except Exception as e:
            rospy.logerr(f"Error processing message: {e}")

    def update_odometry(self):
        try:
            # current_time = rospy.Time.now()
            current_time = self.ros_timestamp
            dt = (current_time - self.last_time).to_sec()
            
            if dt == 0 or self.offset_left_counts is None:
                return
            
            delta_left = self.left_counts - self.last_left_counts
            delta_right = self.right_counts - self.last_right_counts
            
            distance_left = 2 * math.pi * self.wheel_radius * delta_left / self.ticks_per_revolution
            distance_right = 2 * math.pi * self.wheel_radius * delta_right / self.ticks_per_revolution
            
            distance = (distance_left + distance_right) / 2
            dtheta = (distance_right - distance_left) / self.wheel_separation
            
            if abs(dtheta) < 1e-6:
                dx = distance * math.cos(self.theta)
                dy = distance * math.sin(self.theta)
            else:
                radius = distance / dtheta
                dx = radius * (math.sin(self.theta + dtheta) - math.sin(self.theta))
                dy = radius * (math.cos(self.theta) - math.cos(self.theta + dtheta))
            
            self.x += dx
            self.y += dy
            self.theta += dtheta
            
            quaternion = quaternion_from_euler(0, 0, self.theta)
            
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = Quaternion(*quaternion)
            
            odom.twist.twist.linear.x = distance / dt
            odom.twist.twist.angular.z = dtheta / dt
            
            # 共分散の調整
            position_covariance = 0.3 * 0.3  # 30cm の標準偏差の二乗
            orientation_covariance = 0.1 * 0.1  # 適当な値（ラジアン）
            velocity_covariance = 0.1 * 0.1  # 適当な値（m/s）
            
            odom.pose.covariance = [
                position_covariance, 0, 0, 0, 0, 0,
                0, position_covariance, 0, 0, 0, 0,
                0, 0, position_covariance, 0, 0, 0,
                0, 0, 0, orientation_covariance, 0, 0,
                0, 0, 0, 0, orientation_covariance, 0,
                0, 0, 0, 0, 0, orientation_covariance
            ]
            
            odom.twist.covariance = [
                velocity_covariance, 0, 0, 0, 0, 0,
                0, velocity_covariance, 0, 0, 0, 0,
                0, 0, velocity_covariance, 0, 0, 0,
                0, 0, 0, velocity_covariance, 0, 0,
                0, 0, 0, 0, velocity_covariance, 0,
                0, 0, 0, 0, 0, velocity_covariance
            ]
            
            self.odom_pub.publish(odom)
            
            # Publish current pose
            pose = PoseStamped()
            pose.header = odom.header
            pose.pose = odom.pose.pose
            self.pose_pub.publish(pose)
            
            # Update and publish path
            self.path.header.stamp = current_time
            self.path.poses.append(pose)
            self.path_pub.publish(self.path)
            
            self.last_left_counts = self.left_counts
            self.last_right_counts = self.right_counts
            self.last_time = current_time
        except Exception as e:
            rospy.logerr(f"Error in update_odometry: {e}")

    # def run(self):
    #     rate = rospy.Rate(33)  # 30Hz
    #     while not rospy.is_shutdown():
    #         # self.update_odometry()
    #         rate.sleep()

if __name__ == '__main__':
    try:
        odometry = WheelOdometry()
        # odometry.run()
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error in WheelOdometry node: {e}")
