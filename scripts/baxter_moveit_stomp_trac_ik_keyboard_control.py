#!/usr/bin/env python

import rospy

import baxter_interface
import baxter_external_devices
import sys

from std_msgs.msg import (
        String,
)

def map_keyboard():
        pub = rospy.Publisher("moveit_control_command", String, queue_size=10);
        keyboard_binding = {}
        keyboard_binding["p"] = "plan"

        #rospy.loginfo('press ? to print help')
        while not rospy.is_shutdown():
                c = raw_input("command: ")

                if c:
                        #catch Esc or ctrl-c
                        if c in ['\x1b', '\x03']:
                                rospy.signal_shutdown("Finished.Exiting...")
                                return

                        if c == '?':
                                printHelper(keyboard_binding)

                        if(c in keyboard_binding.keys()):
                                rospy.loginfo("sending command: " + keyboard_binding[c])
                                pub.publish(String(keyboard_binding[c]))
                        else:
                                rospy.loginfo("invalid command: " + c)
                                rospy.loginfo('press ? to print help')


def main():
        rospy.init_node("baxter_moveit_trac_ik_keyboard_control", anonymous=True, log_level=rospy.INFO, argv=sys.argv)

        try:
                map_keyboard()
        except():
                pass

def printHelper(keyboard_binding):
        print 'key maping:'
        for c in keyboard_binding.keys():
                print c + ' ---> ' + keyboard_binding[c]

if __name__ == '__main__':
        main()
