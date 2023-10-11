#!/usr/bin/env python3

import rospy
import math

from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import actionlib

from  std_srvs.srv import Empty 

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

def setup_services():
    global start_srv, stop_srv
    rospy.wait_for_service("/navigation/navigation_start")
    rospy.wait_for_service("/navigation/navigation_stop")
    start_srv = rospy.ServiceProxy("/navigation/navigation_start", Empty)
    stop_srv  = rospy.ServiceProxy("/navigation/navigation_stop",  Empty)

def init_globals():
    global  current_destination, odom_position
    odom_position = Odometry()
    current_destination = PoseStamped()
    current_destination.header.frame_id = "wcias_odom"


def set_controller_state():
    global odom_position, current_destination, movebase_client, running

    d = math.sqrt( (odom_position.pose.pose.position.x - current_destination.pose.position.x) **2 +\
                   (odom_position.pose.pose.position.y - current_destination.pose.position.y) **2 )

    #print("----------------")
    #print(d)

    global start_srv, stop_srv

    if (d > 0.1 and not running):
        start_srv()
        running = True

    
    elif d < 0.4 and running:
        stop_srv()

        # remove the current path
        current_destination.pose = odom_position.pose.pose

        running = False
    


def main():
    rospy.init_node('print_d')

    global movebase_client, running
    #movebase_client = actionlib.SimpleActionClient(navigation_name, MoveBaseAction)
    #movebase_client.wait_for_server()

    init_globals()
    setup_listeners()
    setup_services()

    running = False

    # TODO : update these as parameters
    rate = rospy.Rate(Hz)

    global request_new_target

    while not rospy.is_shutdown():
        set_controller_state()
        #if check_distance_to_current_destination():
        #    request_new_target()
        rate.sleep()
    
if __name__ == '__main__':
    main()
