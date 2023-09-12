#!/usr/bin/env python3

import rospy
import math
import random

from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import actionlib

odom_topic = '/odom'
frame_destinatin = 'wcias_base_footprint'
goal_topic = '/move_base/goal/'

navigation_name = "move_base"

def odom_callback(data: Odometry):
    global odom_position
    odom_position = data

def goal_callback(data):
    global current_destination
    current_destination = data.goal.target_pose

def setup_listeners():
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.Subscriber(goal_topic, MoveBaseActionGoal, goal_callback)

def init_globals():
    global  current_destination, path, odom_position, state
    state = 0
    odom_position = Odometry()
    path = Path()
    current_destination = PoseStamped()
    current_destination.header.frame_id = "wcias_odom"


def set_controller_state():
    global odom_position, current_destination, movebase_client,  state

    d = math.sqrt( (odom_position.pose.pose.position.x - current_destination.pose.position.x) **2 +\
                   (odom_position.pose.pose.position.y - current_destination.pose.position.y) **2 )

    print(d)


def main():
    rospy.init_node('print_d')

    global movebase_client
    movebase_client = actionlib.SimpleActionClient(navigation_name, MoveBaseAction)
    movebase_client.wait_for_server()

    init_globals()
    setup_listeners()

    # TODO : update these as parameters
    rate = rospy.Rate(16)

    while not rospy.is_shutdown():
        set_controller_state()
        rate.sleep()
    
if __name__ == '__main__':
    main()
