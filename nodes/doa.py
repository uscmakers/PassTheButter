#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from tuning import Tuning
import usb.core
import usb.util
import time

def listen():

    pub = rospy.Publisher('/cmd_pos', Point, queue_size = 0)
    rospy.init_node('controller')
    rospy.loginfo("Started listening...")

    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

    ## use the test file to determine this
    origin = 0
 
    if dev:
        Mic_tuning = Tuning(dev)
        angleDiff = origin - Mic_tuning.direction
        while True:
            voiceDetected = Mic_tuning.is_voice()
            if voiceDetected:
                try:
                    angleCommand = (angleDiff/1.1)*(0.9/81)
                    pub(0.0, 0.0, angleCommand)
                    break

if __name__ == '__main__':
    try:
        listen()
    except rospy.ROSInterruptException:
        pass
