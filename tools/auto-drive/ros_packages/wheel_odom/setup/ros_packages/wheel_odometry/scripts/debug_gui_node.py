#!/usr/bin/env python3

import rospy
import tkinter as tk
from std_msgs.msg import Int32

class DebugGUI:
    def __init__(self):
        rospy.init_node('debug_gui', anonymous=True)
        
        self.left_pub = rospy.Publisher('left_wheel_counts', Int32, queue_size=10)
        self.right_pub = rospy.Publisher('right_wheel_counts', Int32, queue_size=10)
        
        self.window = tk.Tk()
        self.window.title("Wheel Odometry Debug")
        
        self.left_count = 0
        self.right_count = 0
        
        self.left_scale = tk.Scale(self.window, from_=-100, to=100, orient=tk.HORIZONTAL, label="Left Wheel Speed", length=300, command=self.update_left)
        self.left_scale.set(0)
        self.left_scale.pack()
        
        self.right_scale = tk.Scale(self.window, from_=-100, to=100, orient=tk.HORIZONTAL, label="Right Wheel Speed", length=300, command=self.update_right)
        self.right_scale.set(0)
        self.right_scale.pack()

        self.left_label = tk.Label(self.window, text="Left Wheel Count: 0")
        self.left_label.pack()

        self.right_label = tk.Label(self.window, text="Right Wheel Count: 0")
        self.right_label.pack()

        self.left_speed = 0
        self.right_speed = 0

        self.update_rate = 33  # 33Hz
        self.update_counts()

    def update_left(self, value):
        self.left_speed = int(value)

    def update_right(self, value):
        self.right_speed = int(value)

    def update_counts(self):
        self.left_count += self.left_speed
        self.right_count += self.right_speed

        self.left_pub.publish(Int32(self.left_count))
        self.right_pub.publish(Int32(self.right_count))

        self.left_label.config(text=f"Left Wheel Count: {self.left_count}")
        self.right_label.config(text=f"Right Wheel Count: {self.right_count}")

        self.window.after(int(1000/self.update_rate), self.update_counts)

    def run(self):
        self.window.mainloop()

if __name__ == '__main__':
    gui = DebugGUI()
    gui.run()

