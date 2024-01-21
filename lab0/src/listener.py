#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def flip_letters(data):
    midpoint = len(data)//2
    return data[:midpoint]+data[midpoint:][::-1]

def callback(data):
    modified_msg = flip_letters(data.data)
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", modified_msg)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
