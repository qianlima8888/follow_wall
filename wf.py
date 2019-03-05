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
global x   #全局坐标x
global y   #全局坐标y
x=0
y=0

class wall_fallow_control():    #沿墙算法

    def __init__(self,enter_loop=False,align_wall=False):
        self.sub = rospy.Subscriber("scan_detect",Float32MultiArray, self.callback) #订阅scan_detect 话题，激光雷达的扫描数据
        self.pose_sub = rospy.Subscriber("/move_base/global_costmap/footprint" , PolygonStamped,self.Poly_callback) #订阅/move_base/global_costmap/footprint 话题
        self.pose_sub = rospy.Subscriber("Go_Posit",String,self.strcallback)  #订阅Go_Posit 话题
        self.pub = rospy.Publisher("exploratoin", Twist, queue_size=10) #发布exploratoin 话题

        self.dis = 0.5 #障碍物离机器人距离
        self.k=0       #机器人运动模式标识参数，并赋初值0
        self.j=0       #机器人左转还是右转记录，并赋初值0
        self.l=0       #机器人左边与左前离障碍物最近距离，并赋初值0
        self.m=0       #机器人正前离障碍物最近距离，并赋初值0
        self.r=0       #机器人右边与右前离障碍物最近距离，并赋初值0
        self.dl=0.5    #允许机器人左边与左前离障碍物最近距离为0.5，小于0.5认为机器人左边或左前有障碍物
        self.dm=0.5    #允许机器人正前离障碍物最近距离为0.5，小于0.5认为机器人正前有障碍物
        self.dr=0.5    #允许机器人右边与右前离障碍物最近距离为0.5，小于0.5认为机器人右边或右前有障碍物
        self.fx=0      #机器人顺时针、逆时针标识符；fx大于或等于0机器人沿墙顺时针行走，fx小于0机器人沿墙逆时针行走
        self.s=0        #机器人处于前、左、右三面都有障碍物时机器人旋向记录
        self.stp=0      #这个变量其实后面没用
        self.robot_pose=[[0,0,0]] #机器人位姿矩阵
        self.x=0        #机器人当前位置x坐标
        self.y=0        #机器人当前位置y坐标
        self.str=0      #记录当机器人接收语音信息并转换为数字后的值，self.str==10表示接收到语音信息“开始自动建图”
        self.count=1    #机器人初始位置只采集一次；self.count=1，采集一次后self.count=0。
        self.count2=0   #机器人首次距离初始位置大于2米时使self.count2=1
        self.st=0      #只有当机器人接收语音信息“开始自动建图”后，才使self.st=1，使机器人进入roaming(self,minR,maxR,minRa,maxRa,minF,maxF,minLa,maxLa,minL,maxL,minData,maxData)程序
        self.orgx = 0  #机器人初始位置x坐标
        self.orgy = 0  #机器人初始位置y坐标

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

        if self.calDis(self.orgx,self.orgy,self.x,self.y)<=1.5 and self.count2==1:  #机器人首次距离初始位置大于2米后又运动到距离初始位置小于1.5米的地方，认为机器人已经转了一圈，停止沿墙走
            self.st=0  #机器人停止沿墙走标识
        else:
            print("self.l={}, self.r={}".format(self.l, self.r))
            if self.l<self.dl and self.m>=self.dm and self.r>=self.dr:#2
                print("in_K=2")
                if minL<0.2:
                   self.turnRight()
                elif self.k==4 or self.k == 6:
                   self.turnRight()
                elif self.k == 1 or self.k == 2 or self.k == 3 or self.k ==5:
                    self.forward()
                elif self.fx > 0:
                    if self.k == 8:
                        self.turnRight()
                    else:
                        self.forward()
                elif self.fx < 0:
                    if self.k == 8:
                        self.turnLeft()
                    else:
                        self.turnRight()

                self.k=2
                self.j=0
                self.fx = 1
                print("out_K=2")

                if self.count==1:  #机器人初始位置只采集一次
                    self.orgx=self.x
                    self.orgy=self.y
                    print('l.......')
                    print(self.orgx,self.orgy)
                    self.count=0

                if self.calDis(self.orgx,self.orgy,self.x,self.y)>2:
                    self.count2=1
                    
            elif self.l>=self.dl and self.m>=self.dm and self.r<self.dr :#4
                print("in_K=4")
                if minR<0.2:
                   self.turnLeft()
                elif self.k==2 or self.k==5:
                   self.turnLeft()
                elif self.k == 1 or self.k == 3 or self.k == 4 or self.k ==6:
                    self.forward()
                elif self.fx>0:
                    if self.k == 7:
                        self.turnLeft()
                    else:
                        self.turnRight()
                elif self.fx<0:
                    if self.k==7:
                        self.forward()
                    else:
                        self.turnLeft()

                self.k=4
                print("out_K=4")
                
                if self.count==1:
                    self.orgx=self.x
                    self.orgy=self.y
                    print('r.......')
                    print(self.orgx,self.orgy)
                    self.count=0

                if self.calDis(self.orgx,self.orgy,self.x,self.y)>2:   #机器人首次距离初始位置大于2米时使self.count2=1
                    self.count2=1

            elif self.l>=self.dl and self.m<self.dm and self.r<self.dr :#6 
                print("in_K=6")         
                self.turnLeft()
                self.k=6
                self.j=1
                self.q=1
                print("out_K=6")

            elif self.l<self.dl and self.m<self.dm and self.r>=self.dr :#5
                print("in_K=5")         
                self.turnRight()
                self.k=5
                self.j=0
                self.q=0
                print("out_K=5")

            elif self.l<self.dl and self.r<self.dr and self.m>=self.dm :#7
                print("in_K=7")
                if minR>=0.25 and minL>=0.25:
                   self.forward() 
                elif self.k == 8:
                    if self.fx > 0:
                        self.turnRight()
                    else:
                        self.turnLeft()
                else:
                    self.forward()
                self.k=7
                print("out_K=7")

            elif self.l<self.dl and self.r<self.dr and self.m<self.dm :#8
                print("in_K=8")
                if self.k == 2 or self.k == 5:
                    self.turnRight()
                    self.s=1               
                elif self.k == 4 or self.k == 6:
                    self.turnLeft()
                    self.s=2 
                elif self.fx > 0:
                    self.turnRight()
                else:
                    self.turnLeft()
                self.k=8
                print("out_K=8")

            elif self.l>=self.dl and self.m<self.dm and self.r>=self.dr :#3
                print("in_K=3")
                if self.k == 2 or self.k == 5:
                    self.turnRight()              
                elif self.k == 4 or self.k == 6:
                    self.turnLeft()
                elif self.fx>0:
                    if self.k == 7:
                        self.turnLeft()
                    else:
                        self.turnRight()
                else:
                    if self.k == 7:
                        self.turnRight()
                    else:
                        self.turnLeft()
                self.k=3
                print("out_K=3")

            elif self.l>=self.dl and self.m>=self.dm and self.r>=self.dr: #1
                print("in_K=1")                 
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
                        rospy.sleep(0.012)  #延时
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
                print("out_K=1")
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