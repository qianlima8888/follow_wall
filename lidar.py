#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float32MultiArray


class Lidar_detect:
    def __init__(self):
        self.ang2idx = 1.0 / 0.5 #0.352
        self.idx2ang = 1.0 / self.ang2idx
        self.ang2rad = math.pi / 180.0
        self.rad2ang = 1.0 / self.ang2rad
        self.theta = 20.0

        self.sub = rospy.Subscriber("scan", LaserScan, self.callback)
        self.pub = rospy.Publisher("scan_detect", Float32MultiArray, queue_size=10)

        # def the data of laser 
    def callback(self,msg):
        
        minRight = min(msg.ranges[0:72])
        maxRight = max(msg.ranges[0:72])

        minRight_ahead = min(msg.ranges[73:80])
        maxRight_ahead = max(msg.ranges[73:80])

        minFront =min (msg.ranges[160:200])
        maxFront =max (msg.ranges[160:200])

        minLeft_ahead = min(msg.ranges[280:289])
        maxLeft_ahead = max(msg.ranges[280:289])

        minLeft = min (msg.ranges[290:359])
        maxLeft = max (msg.ranges[290:359])

        minData=min(msg.ranges)
        maxData=max(msg.ranges)

        dis = Float32MultiArray()

        dis.data = [minRight, maxRight,
                    minRight_ahead, maxRight_ahead,
                    minFront, maxFront, 
                    minLeft_ahead, maxLeft_ahead, 
                    minLeft, maxLeft,
                    minData, maxData]

        self.pub.publish(dis)

if __name__ == '__main__':
  rospy.init_node('Lidar', anonymous=False)
  lidar = Lidar_detect()
  rospy.spin()