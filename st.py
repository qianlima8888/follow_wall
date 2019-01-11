#!/usr/bin/env python
# -*- coding:utf-8 -*-
# import the necessary packages
import rospy
import math
from math import pi as PI
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float32MultiArray,String
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped,PolygonStamped
from nav_msgs.msg import Odometry, OccupancyGrid

import tf 
global x 
global y 
x=0
y=0
class wall_fallow_control():

    def __init__(self,enter_loop=False,align_wall=False):
        self.sub = rospy.Subscriber("scan_detect",Float32MultiArray, self.callback)
        self.pose_sub = rospy.Subscriber("/move_base/global_costmap/footprint" , PolygonStamped,self.Poly_callback)
        self.pose_sub = rospy.Subscriber("Go_Posit",String,self.strcallback)
        self.pub = rospy.Publisher("exploratoin", Twist, queue_size=10)

        self.dis = 0.5
        self.k=0
        self.j=0
        self.l=0
        self.m=0
        self.r=0
        self.dl=0.5
        self.dm=0.5
        self.dr=0.5
        self.fx=0
        self.s=0
        self.stp=0
        self.robot_pose=[[0,0,0]]
        self.x=0
        self.y=0
        self.str=0
        self.count=1
        self.count2=0
        self.st=0
        self.orgx = 0
        self.orgy = 0

    def forward(self):
        vel_msg = Twist()
        vel_msg.linear.x = 1
        vel_msg.angular.z = 0
        cmd_vel_pub.publish(vel_msg)

    def back(self):
        vel_msg = Twist()
        vel_msg.linear.x = -1
        vel_msg.angular.z = 0
        cmd_vel_pub.publish(vel_msg)

    def turnRight(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0.3
        cmd_vel_pub.publish(vel_msg) 

    def turnLeft(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = -0.3
        cmd_vel_pub.publish(vel_msg)

    def stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        cmd_vel_pub.publish(vel_msg)
        
    def calDis(self,x1,y1,x2,y2):
        dis = math.sqrt(pow((x1-x2),2)+pow((y1-y2),2))
        return dis
    
    def roaming(self,minR,maxR,minRa,maxRa,minF,maxF,minLa,maxLa,minL,maxL,minData,maxData):
        self.l=min(minL,minLa)
        self.r=min(minR,minRa)
        self.m=minF
       
        print("dissssssssssssssssssss")
        print(self.calDis(self.orgx,self.orgy,self.x,self.y))
        print("x,yyyyyyyyyyyyyyyyyyyyyyyyyyyyy")
        print(self.orgx,self.orgy)

        if self.calDis(self.orgx,self.orgy,self.x,self.y)<=1.5 and self.count2==1:
            self.st=0
        else:
            if self.l<self.dl and self.m>=self.dm and self.r>=self.dr:
                if minL<0.2:
                   self.turnRight()
                elif self.k==4:
                   self.turnRight()
                else:
                    self.forward()

                self.k=2
                self.j=0
                self.q=0
                self.fx=self.fx+1

                if self.count==1:
                    self.orgx=self.x
                    self.orgy=self.y
                    print('l.......')
                    print(self.orgx,self.orgy)
                    self.count=0

                if self.calDis(self.orgx,self.orgy,self.x,self.y)>2:
                    self.count2=1
                    
            elif self.l>=self.dl and self.m>=self.dm and self.r<self.dr :
                if minR<0.2:
                   self.turnLeft()
                elif self.k==2:
                   self.turnLeft()
                else:            
                    self.forward()

                self.k=4
                self.j=1
                self.q=1
                self.fx=self.fx-1

                if self.count==1:
                    self.orgx=self.x
                    self.orgy=self.y
                    print('r.......')
                    print(self.orgx,self.orgy)
                    self.count=0

                if self.calDis(self.orgx,self.orgy,self.x,self.y)>2:
                    self.count2=1
            elif self.l>=self.dl and self.m<self.dm and self.r<self.dr :           
                self.turnLeft()
                self.k=6
                self.j=1
                self.q=1
            elif self.l<self.dl and self.m<self.dm and self.r>=self.dr :            
                self.turnRight()
                self.k=5
                self.j=0
                self.q=0
            elif self.l<self.dl and self.r<self.dr and self.m>=self.dm :
                if minR>=0.25 and minL>=0.25:
                   self.forward() 
                elif self.k==6 or self.k==4 or (self.k==3 and self.s==2) or (self.k==1 and self.fx<0):
                    self.turnLeft()
                elif self.k==5 or self.k==2 or (self.k==3 and self.s==1) or (self.k==1 and self.fx>=0):
                    self.turnRight()
                    self.k=7   
            elif self.l<self.dl and self.r<self.dr and self.m<self.dm :
                if self.fx>=0:
                    self.turnRight()
                    self.s=1               
                else:
                    self.turnLeft()
                    self.s=2 
                    self.k=8
            elif self.l>=self.dl and self.m<self.dm and self.r>=self.dr :
                if self.fx>=0:
                    self.turnRight()
                    self.s=1               
                else:
                    self.turnLeft()
                    self.s=2    
                    self.k=3
            elif self.l>=self.dl and self.m>=self.dm and self.r>=self.dr:                   
                if self.k==1:
                   self.forward()
                elif self.k==2:
                    self.forward()
                    if minR>=0.2:
                        rospy.sleep(0.012)
                    elif minR<0.2:
                        self.turnRight()
                        self.turnLeft()
                elif self.k==3:
                    self.forward()
                elif self.k==4:
                    self.forward()
                    if minL>=0.2:
                        rospy.sleep(0.012)
                    elif minL<0.2:
                        self.turnLeft()
                        self.turnRight()
                elif self.k==5:
                    self.turnLeft()
                elif self.k==6:
                    self.turnRight()
                elif self.k==7:
                    if self.fx<=0:
                        self.turnRight()
                    elif self.fx>0:
                        self.turnLeft()
                else:
                    self.forward()
                    self.k = 1
            else:
                self.stop()
               
    def callback(self,data):
        minR = data.data[0]
        maxR = data.data[1]
        minRa = data.data[2]
        maxRa = data.data[3]
        minF = data.data[4]
        maxF = data.data[5]
        minLa = data.data[6]
        maxLa = data.data[7]
        minL = data.data[8]
        maxL = data.data[9]
        minData = data.data[10]
        maxData = data.data[11]

        if self.str==10:
            self.st=1
            self.str=100
        elif self.str==11:
            self.st=0
            self.str=100

        if self.st==1:
            self.roaming(minR, maxR, minRa, maxRa, minF, maxF, minLa, maxLa, minL, maxL, minData, maxData)
   
    def Poly_callback(self,poly):
        self.x=(poly.polygon.points[0].x + poly.polygon.points[1].x + poly.polygon.points[2].x + poly.polygon.points[3].x) / 4
        self.y=(poly.polygon.points[0].y + poly.polygon.points[1].y + poly.polygon.points[2].y + poly.polygon.points[3].y) / 4

    def strcallback(self,str):
        self.str=int(str.data)

if __name__ == '__main__':
    rospy.init_node('exploratoin', anonymous=False)
    cmd_vel_pub=rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    while not rospy.is_shutdown():
        wall_fallow_control()
        rospy.spin()