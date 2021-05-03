#!/usr/bin/env python3
from playsound import playsound
from pass_butter.servo_move import up, down, wave
import rospy
from geometry_msgs.msg import Point


def demo():
    pub = rospy.Publisher('/cmd_pos', Point, queue_size=0)
    rospy.init_node('control')
    text = ''
    while True:
        text = input('cmd: ')
        if text == 'x' or len(text) < 1:
            break
        if text == 'u':
            up()
        elif text == 'd':
            down()
        elif text == 'w':
            wave()
        elif text[0] == 'r':
            print(text)
            pub.publish(0.0, 0.0, float(text[1:]))
        elif text[0] == 's':
            if text[1] == '0':
                playsound('/home/robot/WhatIsMyPurposeCut.wav') #TODO refactor the absolute paths
            if text[1] == '1':
                playsound('/home/robot/OhMyGodCut.wav')
            if text[1] == '2':
                playsound('/home/robot/robotCalculating.wav')
            if text[1] == '3':
                playsound('/home/robot/NotProgammedForFriendshipCut2.wav')
            if text[1] == '4':
                playsound('/home/robot/Theme.wav')
        else:
            pub.publish(float(text), 0.0, 0.0)

if __name__ == '__main__':
    try:
        demo()
        rospy.loginfo('done')
    except rospy.ROSInterruptException:
        pass
