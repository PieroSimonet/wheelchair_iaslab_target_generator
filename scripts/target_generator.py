#!/usr/bin/env python3

import rospy
import math
import random

from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import actionlib
import tf2_ros
import tf2_geometry_msgs

from std_srvs.srv import Empty


def setup_parameters():
    global odom_topic, frame_base, navigation_frame, goal_topic, navigation_name

    odom_topic = rospy.get_param('~odom_topic', '/odom')
    navigation_name = rospy.get_param('~navigation_name', "move_base")
    goal_topic = rospy.get_param('~goal_topic', '/move_base/goal/')
    navigation_frame = rospy.get_param('~navigation_frame', 'wcias_odom')

def odom_callback(data: Odometry):
    global odom_position
    odom_position = data

def goal_callback(data):
    global current_destination
    current_destination = data.goal.target_pose

def setup_listeners():
    global odom_topic, frame_base, navigation_frame, goal_topic, navigation_name

    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.Subscriber(goal_topic, MoveBaseActionGoal, goal_callback)

def init_globals():
    global tf_buffer, tf_listener, current_destination, path, odom_position, state
    global odom_topic, frame_base, navigation_frame, goal_topic, navigation_name

    state = 0
    odom_position = Odometry()
    path = Path()
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    current_destination = PoseStamped()
    current_destination.header.frame_id = navigation_frame

def callback_bci(data):
    global state
    if (data > 0.7):
        state = 1
    elif(data < 0.3):
        state = -1
    else:
        state = 0

def generate_new_target():
    global state, navigation_frame
    pose = PoseStamped()
    pose.header.frame_id = navigation_frame
    pose.pose.position.x = 1
    pose.pose.position.y = state
    return pose

def request_new_target_callback():
    global movebase_client, tf_buffer, state
    global odom_topic, frame_base, navigation_frame, goal_topic, navigation_name


    pose = generate_new_target()
    transform = tf_buffer.lookup_transform(navigation_frame, frame_base, rospy.Time(0), rospy.Duration(1))
    pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

    goal = MoveBaseGoal()
    goal.target_pose.pose = pose.pose
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = navigation_frame

    movebase_client.send_goal(goal)


def setup_services():
    rospy.Service('/navigation/request_new_target', Empty, request_new_target_callback)

def main():
    rospy.init_node('bci_sender')

    global odom_topic, frame_base, navigation_frame, goal_topic, navigation_name


    setup_parameters()
    init_globals()
    setup_listeners()

    global movebase_client
    movebase_client = actionlib.SimpleActionClient(navigation_name, MoveBaseAction)
    movebase_client.wait_for_server()

    # TODO : update these as parameters
    rate = rospy.Rate(0.5)

    rospy.spin()
    
if __name__ == '__main__':
    main()
