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

from __future__ import print_function
import rospy
import actionlib
import robot_run_route.msg
import sys

msg = """
patrol your Robot!
-----------------------
mode : s - Patrol to Square
       t - Patrol to Triangle
       c - Patrol to Circle

area : Square, Triangle mode - length of side (m)
       Circle mode - radius (m)

count - patrol count

If you want to close, insert 'x'
"""

class Client():
    def __init__(self):
        rospy.loginfo("wait for server")
        self.client()

    def getkey(self):
        mode, area, count = raw_input("| mode | area | count |\n").split()
        mode, area, count = [str(mode), float(area), int(count)]

        if mode == 's':
            mode = 1
        elif mode == 't':
            mode = 2
        elif mode == 'c':
            mode = 3
        elif mode == 'x':
            self.shutdown()
        else:
            rospy.loginfo("you select wrong mode")

        return mode, area, count

    def client(self):
        print("step 1")
        client = actionlib.SimpleActionClient('robot_run_route', robot_run_route.msg.robot_run_routeAction)
        print("step 2")
        mode, area, count = self.getkey()
        print("step 3")
        client.wait_for_server()
        print("step 4")
        goal = robot_run_route.msg.robot_run_routeGoal()
        print("step 5")
        goal.goal.x = mode
        goal.goal.y = area
        goal.goal.z = count
        client.send_goal(goal)
        print("step 6")
        rospy.loginfo("send to goal")
        client.wait_for_result()

        rospy.loginfo(client.get_result())

    def shutdown(self):
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('robot_run_route_client')
    try:
        while not rospy.is_shutdown():
            print (msg)
            result = Client()
            print("step a1")
    except:
        print("program close.", file=sys.stderr)
