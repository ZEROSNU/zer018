#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int64
import time

fuck = 0


def callback(msg):
    global fuck
    fuck = msg.data

def callback2(msg):
    global mode
    mode = msg.data

def shutdown():
    rospy.loginfo("JUNOOOOFLOOO")

def main():
    rospy.init_node('junoflo')
    sub = rospy.Subscriber('/juno', Int64, callback, queue_size=1)
    pub = rospy.Publisher('/flo', Int64, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        pub.publish(fuck)
        rospy.loginfo("fuck published")
        rate.sleep()
    rospy.on_shutdown(shutdown)

if __name__ == "__main__":
    main()
