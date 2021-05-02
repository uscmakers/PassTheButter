#!/bin/user/env python3
from playsound import playsound
from servo_move import up, down, wave
import rospy
from geometry_msgs.msg import Point

def setup():
    pub = rospy.Publisher('/cmd_pos', Point, queue_size=0)
    rospy.init_node('control')

def run():
    """The script: playsound, rotate 90, move 3.3, arms down, move back 3.3, 
        lift arms, rotate -90
    """
    wave()
    playsound('VoiceCommands/WhatIsMyPurposeCut.wav')
    pub.publish(0.0, 0.0, -0.9)
    rospy.sleep(1.0)
    pub.publish(-3.3, 0.0, 0.0)
    rospy.sleep(3.0)
    down()
    pub.publish(3.3, 0.0, 0.0)
    rospy.sleep(3.0)
    up()
    rospy.publish(0.0, 0.0, 0.9)
    rospy.sleep(1.0)
    playsound('VoiceCommands/WhatIsMyPurposeCut.wav')
    playsound('OhMyGodCut.wav')

    

if __name__ == '__main__':
    try:
        setup()
        rospy.loginfo('butter-run setup complete, running script...')
        run()
        rospy.loginfo('done')
    except rospy.ROSInterruptException:
        pass