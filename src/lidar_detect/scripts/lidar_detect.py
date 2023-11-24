#! /usr/bin/env python3.8
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LidarDetect:
    def __init__(self):

        rospy.init_node("lidar_detect_node")

        self.start_subscrber = rospy.Subscriber(
            "/nav_start", Bool, self.nav_start)
        self.vel_pub = rospy.Publisher(
            "lidar_vel",Twist,queue_size=1000)
        self.lidar_subscrber = rospy.Subscriber(
            "/scan", LaserScan, self.LidarCallback)
        
        self.lidar_vel = Twist()
        self.is_nav_start = False

        self.linear_x = rospy.get_param("~linear_x",0.0)
        self.linear_y = rospy.get_param("~linear_y",0.0)
        self.linear_z = rospy.get_param("~linear_z",0.0)
        self.angular_x = rospy.get_param("~angular_x",0.0)
        self.angular_y = rospy.get_param("~angular_y",0.0)
        self.angular_z = rospy.get_param("~angular_z",0.0)
        self.obstacledist = rospy.get_param("~obstacledist",0.5)

        self.lidar_vel.linear.x = self.linear_x
        self.lidar_vel.linear.y = self.linear_y
        self.lidar_vel.linear.z = self.linear_z
        self.lidar_vel.angular.x = self.angular_x
        self.lidar_vel.angular.y = self.angular_y
        self.lidar_vel.angular.z = self.angular_z


    def LidarCallback(self,msg):
        f_mid_dist = msg.ranges[719]
        # 根据激光雷达数据调整速度
        if f_mid_dist > self.obstacledist:
            self.lidar_vel.linear.x = 0.5  # 如果距离大于1.0米，设置线速度为0.5 m/s
        else:
            self.lidar_vel.linear.x = 0.1  # 否则，设置线速度为0.1 m/s
        rospy.loginfo("Front distance ranges[719] = %f m", f_mid_dist)
        rospy.loginfo("Setting linear velocity to: %f m/s", self.lidar_vel.linear.x)

        # 发布调整后的速度
        self.vel_pub.publish(self.lidar_vel)
    
    def nav_start(self):
        if not self.is_nav_start:
            print("Action Stations!!!")
            self.is_nav_start = True
            return True, "AccumviaTimer started, good luck."
        else:
            print("Starting failed: already running")
            return False, "Already running"
    def flow(self):
        if self.is_nav_start == True:
            self.lidar_subscrber()
    
    def run(self):
        rate = rospy.Rate(20)
        self.flow()
        while not rospy.is_shutdown():
            self.flow()
            rate.sleep()        



def main():
    node = LidarDetect()
    node.run()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)
