#!/usr/bin/env python3
from playsound import playsound
from pass_butter.servo_move import up, down, wave
import rospy
from geometry_msgs.msg import Point


def run():
    """The script: playsound, rotate 90, move 3.3, arms down, move back 3.3, 
        lift arms, rotate -90
    """
    pub = rospy.Publisher('/cmd_pos', Point, queue_size=0)
    rospy.init_node('control')
    rospy.loginfo('butter-run setup complete, running script...')
    up()
    wave()
    playsound('/home/robot/WhatIsMyPurposeCut.wav') #TODO refactor the absolute paths
    rospy.sleep(1.0)
    pub.publish(0.0, 0.0, -0.9)
    rospy.sleep(1.0)
    pub.publish(-3.3, 0.0, 0.0)
    rospy.sleep(3.0)
    down()
    rospy.sleep(1.0)
    pub.publish(3.3, 0.0, 0.0)
    rospy.sleep(3.0)
    up()
    pub.publish(0.0, 0.0, 0.9)
    rospy.sleep(1.0)
    playsound('/home/robot/WhatIsMyPurposeCut.wav')
    rospy.sleep(1.0)
    playsound('/home/robot/OhMyGodCut.wav')

    

if __name__ == '__main__':
    try:
        run()
        rospy.loginfo('done')
    except rospy.ROSInterruptException:
        pass
