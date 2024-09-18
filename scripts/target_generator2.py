#!/usr/bin/env python3

import rospy
import math

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from sensor_msgs.msg import Joy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

import random

import actionlib
import tf2_ros
import tf2_geometry_msgs

class TargetGenerator:
    def __init__(self):
        rospy.init_node('targetgen2')

        self.with_yolo   = True 
        self.with_errors = False #true con yolo e no-yolo

        self.prob = 0.2

        self.setup_listeners()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pos_x = 0.0
        self.pos_y = 0.0

        self.last_time_joy = rospy.get_time()
        self.last_time_yolo = rospy.get_time()
        self.last_time = rospy.get_time() 

        self.new_data = False
        self.navigation_frame = 'wcias_odom'

        self.joy_y = 0
        self.joy_x = 3

        self.yolo_y = 0
        self.yolo_x = 3.0

        navigation_name = rospy.get_param('~navigation_name', "move_base")

        # rospy.get_param('~with_errors', True)

        self.movebase_client = actionlib.SimpleActionClient(navigation_name, MoveBaseAction)
        self.movebase_client.wait_for_server()

        self.is_active = False

        self.last_cmd_joy = rospy.get_time()


    def setup_listeners(self):
        if self.with_yolo == True:
            rospy.Subscriber("/yolo_detected_objects_destination", PoseArray, self.callback_yolo)
        rospy.Subscriber("/joy", Joy, self.callback_joy)

    def callback_yolo(self, msg):
        for obj in msg.poses:
            self.last_time_yolo = rospy.get_time()
            # TODO check the distance to the wheelchair and take the nearest, for now take a ranmom one
            self.yolo_x = obj.position.x
            self.yolo_y = obj.position.y
            self.yolo_frame = msg.header.frame_id
            self.new_data = True

    def callback_joy(self, msg):
        self.last_time_joy = rospy.get_time()
        power = 3

        if msg.buttons[3] == 1:
            # Reverse the activation if the y is pressed
            self.is_active = not self.is_active

        if (abs(msg.axes[4]) == 1):
            self.joy_y = msg.axes[4] * power
            if random.random() < self.prob and self.with_errors:
                self.joy_y = - self.joy_y
            self.joy_x = 0
            self.new_data = True
        if (abs(msg.axes[5]) == 1):
            self.joy_x = msg.axes[5] * power
            self.joy_y = 0
            self.new_data = True

    def get_current_target(self):
        current_time = rospy.get_time()

        navigation_frame = 'wcias_base_link'

        if current_time - self.last_time_joy > 2.5 and current_time - self.last_time_yolo > 2.5:
            self.pos_x = 3.0
            self.pos_y = 0.0

        if current_time - self.last_time_joy < 4.0:# and current_time - self.last_cmd_joy < 2.5:
            self.pos_x = self.joy_x
            self.pos_y = self.joy_y
            self.last_cmd_joy = rospy.get_time()

        elif current_time - self.last_time_yolo < 4.0:
            self.pos_x = self.yolo_x
            self.pos_y = self.yolo_y

            navigation_frame = self.navigation_frame

        pose = PoseStamped()
        pose.header.frame_id = navigation_frame
        pose.pose.position.x = self.pos_x
        pose.pose.position.y = self.pos_y 

        return pose

    def request_move_base(self):
        destination = self.get_current_target()

        navigation_frame = 'wcias_odom'
        frame_base = destination.header.frame_id
        
        transform = self.tf_buffer.lookup_transform(navigation_frame, frame_base, rospy.Time(0), rospy.Duration(1))
        goal_pose = tf2_geometry_msgs.do_transform_pose(destination, transform)

        goal = MoveBaseGoal()
        goal.target_pose.pose = goal_pose.pose
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = navigation_frame

        self.movebase_client.send_goal(goal)
        self.last_time = rospy.get_time() 

    def run(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self.is_active:
                current_time = rospy.get_time() 
                if current_time - self.last_time > 4.0:
                    self.new_data = True

                if self.new_data:
                    self.request_move_base()
                    self.new_data = False
            rate.sleep()

def main():
    tg = TargetGenerator()
    tg.run()

if __name__ == '__main__':
    main()

