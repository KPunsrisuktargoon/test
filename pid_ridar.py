#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
import os 
import cv2
from enum import Enum
from std_msgs.msg import UInt8, Float64, String
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
class ridarr():
    def __init__(self):
        self.sub_scan_obstacle = rospy.Subscriber('/scan', LaserScan, self.cb11, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.lastError = 0
    def cb11(self, scan):
        error = scan.ranges[270] - 0.2
        Kp = 0.8

        Kd = 0.03

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)
    def main(self):
        rospy.spin()
if __name__ == '__main__':
    rospy.init_node('ridarr')
    node = ridarr()
    node.main()
