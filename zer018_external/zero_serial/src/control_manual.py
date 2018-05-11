#!/usr/bin/env python
# license removed for brevity
# input : steer-degree, speed:m/s

import rospy
import time
import serial
import math
import sys, select, termios, tty
from std_msgs.msg import String
from zero_serial.msg import control


def init():
    
    pub = rospy.Publisher('control', control, queue_size=10)
    rospy.init_node('control_sender', anonymous=True)
    rate = rospy.Rate(20)
    msg = control()
    is_auto=1
    estop=0
    gear=0
    speed=0 #m/s
    steer=0 #degree
    brake=1
        
    
    while not rospy.is_shutdown():
            
        key = getKey()
        print("key" + key)
        
        if key == 'q':
            break
        elif key == 'a':
            steer = steer - 1
            if steer <= -15 :
                steer = -15
        elif key == 'd':
            steer = steer + 1
            if steer >= 15 :
                steer = 15
        elif key == 'w':
            speed = speed + 0.1
            if speed >= 3:
                speed = 3
        elif key == 's':
            speed = speed - 0.1
            if speed < 0:
                speed = 0
        elif key == 'e':
            if gear == 0:
                gear = 2
            elif gear == 2:
                gear = 0
        elif key == 'b':
            if brake == 1:
                brake = 50
            elif brake == 50:
                brake = 1
         
        
            
        rospy.loginfo(msg)
        
        msg.is_auto = is_auto
        msg.estop = estop
        msg.gear = gear
        msg.brake = brake
        msg.speed = round(speed,3)
        msg.steer = round(steer,3)
        msg.encoder = 100
        
        pub.publish(msg)
        rate.sleep()
        
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    try:
        settings = termios.tcgetattr(sys.stdin)
        init()
    except rospy.ROSInterruptException:
        pass