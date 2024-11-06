#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class EKFPathNode:
    def __init__(self):
        rospy.init_node('ekf_path_node', anonymous=True)
        
        self.path_pub = rospy.Publisher('odometry/filtered_path', Path, queue_size=10)
        rospy.Subscriber('odometry/filtered', Odometry, self.odometry_callback)
        
        self.path = Path()
        self.path.header.frame_id = "odom"

    def odometry_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(pose)
        
        self.path_pub.publish(self.path)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = EKFPathNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
