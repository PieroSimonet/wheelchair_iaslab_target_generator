#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Joy
from geometry_msg.msg import Twist
from std_srvs.srv import Empty

import actionlib

class main_controller():
    def __init__(self):
        self.setup_variables()
        self.setup_listeners()
        self.setup_services()
        self.setup_senders()

    def setup_senders(self):
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def setup_variables(self):
        self.joy_topic = rospy.get_param('~joy_topic', '/joy')
        hz = rospy.get_param('~hz', 50)
        self.rate = rospy.Rate(hz)
        self.main_have_power = False
        self.request_power = False
        self.give_power = False
        self.cmd = Twist()

    def setup_services(self):
        rospy.wait_for_service('/navigation/navigation_disable')
        rospy.wait_for_service('/navigation/navigation_enable')

        self.request_stop_svc = rospy.ServiceProxy('/navigation/navigation_disable', Empty)
        self.give_power_svc = rospy.ServiceProxy('/navigation/navigation_enable', Empty)

    def setup_listeners(self):
        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback)

    def joy_callback(self, data):
        if not self.main_have_power:
            self.request_power = True

        # Now the logic to give power and move the robot
        if data.buttons[0] == 1:
            self.give_power = True

        # TODO: test
        self.cmd.linear.x = data.axes[1]
        self.cmd.angular.z = data.axes[0]

        pass

    def send_cmd(self):
        self.cmd_pub.publish(self.cmd)

        # Now reset the cmd
        self.cmd.linear.x = 0
        self.cmd.angular.z = 0

    def run(self):
        while not rospy.is_shutdown():
            if self.request_power:
                self.request_stop_svc()
                self.request_power = False
                self.main_have_power = True

            if self.give_power:
                self.give_power_svc()
                self.give_power = False
                self.main_have_power = False

            if self.main_have_power:
                self.send_cmd()

            self.rate.sleep()

def main():
    rospy.init_node('main_joy_controller')
    controller = main_controller()
    controller.run()

if __name__ == '__main__':
    main()
