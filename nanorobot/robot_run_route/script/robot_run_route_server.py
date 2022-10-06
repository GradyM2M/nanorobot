#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import actionlib
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import robot_run_route.msg
import robot_run_route.msg as rrrmsg
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class robot_run_route(object):
    _feedback = robot_run_route.msg.robot_run_routeFeedback()
    _result = robot_run_route.msg.robot_run_routeResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, rrrmsg.robot_run_routeAction, execute_cb=self.execute_cb, auto_start=False)
        # self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.get_odom)
        self.init_stats = True
        self._as.start()
        rospy.loginfo('Server On')

    def get_odom(self, odom):
        self.position = Point()
        self.position = odom.pose.pose.position
        self.quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
        self.euler = euler_from_quaternion(self.quaternion)
        self.yaw = self.euler[2]

    # def quaternion(self,quaternion):
    #     self.euler = euler_from_quaternion(quaternion)
    #     self.yaw = euler[2]
    #     return self.yaw

    def turn(self, angle):
        if self.init_stats:
            self.init_stats = False
            self.init_angle = self.yaw
            angle = (float(angle)/180)*3.14159
            print("Angle ",angle,self.init_angle)
        while(abs(self.init_angle - self.yaw) < abs(angle)):
            if angle > 0:
                self.twist.angular.z = -0.5
            else:
                self.twist.angular.z = 0.5

            self.cmd_pub.publish(self.twist)
            self.r.sleep()
        while(abs(self.init_angle - self.yaw) > abs(angle)+0.02):
            if angle < 0:
                self.twist.angular.z = -0.25
            else:
                self.twist.angular.z = 0.25

            self.cmd_pub.publish(self.twist)
            self.r.sleep()
        self.init_stats = True
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)
        self.r.sleep()

    def go_front(self, lenght, count):
        print("position",self.position,"count ",count,"yaw ",self.yaw)
        if count == 0:
            while self.position.x  < lenght:
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()
            while (self.position.x  > (lenght+0.02)):
                self.twist.linear.x = -0.1
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()           
        elif count == 1:
            while self.position.y < lenght:
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()
            while (self.position.y > (lenght+0.02)):
                self.twist.linear.x = -0.1
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()
        elif count == 2:
            while self.position.x > lenght:
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()
            while self.position.x < (lenght- 0.02):
                self.twist.linear.x = -0.1
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()                
        else:
            while self.position.y > lenght:
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()
            while self.position.y < (lenght- 0.02):
                self.twist.linear.x = -0.1
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.r.sleep()
        self.twist.linear.x = 0.0
        self.cmd_pub.publish(self.twist)
        self.r.sleep()

    def execute_cb(self, goal):
        position = Point()
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.twist = Twist()
        self.r = rospy.Rate(25)
        self.r1 = rospy.Rate(25)
        success = True
        mode = goal.goal.x
        patrol_count = int(goal.goal.z)
        circle_mode = True
        half_patrol = False
        circle_count = 0

        for i in range(patrol_count):
            if mode == 1:
                area = [0,0,0,0]
                area[0] = goal.goal.y
                area[1] = goal.goal.y
                for i in range(4):
                    self.go_front(area[i], i)
                    self.r1.sleep()
                    self.turn(-90)

            elif mode == 2:
                area = [0, 0, 0]
                area[0] = goal.goal.y
                area[1] = goal.goal.y
                for i in range(3):
                    self.go_front(area[i], i)
                    self.turn(-120)
            elif mode == 3:
                while(circle_mode):
                    if self.position.x < -goal.goal.y / 2:
                        half_patrol = True
                        count_flag = True
                    else:
                        self.twist.linear.x = goal.goal.y / 2
                        self.twist.angular.z = 0.5
                    if half_patrol == True and self.position.x > 0:
                        if count_flag == True:
                            circle_count = circle_count + 1
                            count_flag = False
                        half_patrol = False
                        if circle_count == patrol_count:
                            circle_mode = False
                            self.twist.linear.x = 0
                            self.twist.angular.z = 0
                    self.cmd_pub.publish(self.twist)
                    self.r.sleep()

        if success:
            self._result = 0
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('robot_run_route')
    server = robot_run_route(rospy.get_name())
    rospy.spin()
    