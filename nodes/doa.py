#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from pass_butter.tuning import Tuning
import usb.core
import usb.util
import time

def listen():
    pub = rospy.Publisher('/cmd_pos', Point, queue_size = 0)
    rospy.init_node('controller')
    rospy.loginfo("Started listening...")

    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

    ## use the test file to determine this
    origin = 150
 
    if(dev):
        Mic_tuning = Tuning(dev)
        while(True):
            angleDiff = origin - Mic_tuning.direction
            voiceDetected = Mic_tuning.is_voice()
            if(voiceDetected):
                angleCommand = -(angleDiff)*(0.9/90)
                pub.publish(0.0, 0.0, angleCommand)
                rospy.sleep(10.0)

if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
