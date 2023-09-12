#!/usr/bin/env python3

import rospy
import math
import random

from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import actionlib

from std_srvs.srv import Empty 

odom_topic = '/odom'
goal_topic = '/move_base/goal/'
delta_distance = 0.5 # [m]
Hz = 16

def odom_callback(data: Odometry):
    global odom_position
    odom_position = data

def goal_callback(data):
    global current_destination
    current_destination = data.goal.target_pose

def setup_listeners():
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.Subscriber(goal_topic, MoveBaseActionGoal, goal_callback)

def setup_parameters():
    odom_topic = rospy.get_param('~odom_topic', odom_topic)
    goal_topic = rospy.get_param('~goal_topic', goal_topic)
    delta_distance = rospy.get_param('~delta_distance', delta_distance)
    Hz = rospy.get_param('~hz', Hz)

def setup_services():
    global request_new_target
    rospy.wait_for_service('/navigation/request_new_target')
    request_new_target = rospy.ServiceProxy('/navigation/request_new_target', Empty)

def init_globals():
    global  current_destination, path, odom_position
    odom_position = Odometry()
    path = Path()
    current_destination = PoseStamped()
    current_destination.header.frame_id = "wcias_odom"


def check_distance_to_current_destination():
    global odom_position, current_destination

    d = math.sqrt( (odom_position.pose.pose.position.x - current_destination.pose.position.x) **2 +\
                   (odom_position.pose.pose.position.y - current_destination.pose.position.y) **2 )

    return d < delta_distance


def main():
    rospy.init_node('print_d')
    
    setup_parameters()
    init_globals()
    setup_listeners()
    setup_services()

    # TODO : update these as parameters
    rate = rospy.Rate(Hz)

    global request_new_target

    while not rospy.is_shutdown():
        if check_distance_to_current_destination():
            request_new_target()
        rate.sleep()
    
if __name__ == '__main__':
    main()
