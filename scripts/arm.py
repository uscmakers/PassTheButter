#!/bin/usr/env python3
import Jetson.GPIO as GPIO
from time import sleep
from adafruit_servokit import ServoKit
import math

class Arm:
    #arm segment length in mm
    #isLeft is boolean for if the arm is the left or not
    def __init__(self, bicepLength, forearmLength, shoulderChannel, elbowChannel, isLeft):

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.bicepLength = bicepLength
        self.forearmLength = forearmLength
        kit = ServoKit(channels=16)
        pca.frequency = 50 #sets the PCA9685 for servos I believe
        elbowServo = adafruit_motor.servo.Servo(elbowChannel)
        shoulderServo = adafruit_motor.servo.Servo(shoulderChannel)
        elbowServo.angle = 30
        shoulderServo.angle = 30

    def moveShoulder(self, degree):
        if isLeft:
            shoulderServo.angle = -degree + 50
        else:
            shoulderServo.angle = degree - 50

    def moveElbow(self, degree):
        if isLeft:
            shoulderServo.angle = -degree + 90
        else:
            shoulderServo.angle = degree - 90

    #x and y are in mm as a Cartesian coordiante system where the shoulder servo axis is the origin
    def moveXY(self, x, y):
        #determine the distance of end from shoulder (effective length L)
        L = sqrt(pow(x,2) + pow(y,2))
        #find required elbow angle for the effective length
        elbowAngle = acos((pow(L,2) - pow(bicepLength,2) - pow(forearmLength,2)) / (2*bicepLength*forearmLength))

        #find required shoulder angle for correct end effector placement at x,y
        offsetAngle = (math.pi - elbowAngle) * forearmLength /     \
        (sqrt(pow(bicepLength,2) + pow(forearmLength,2) + 2 * bicepLength * forearmLength * cos(elbowAngle)))
        shoulderAngle = atan(x/y) - offsetAngle

        moveShoulder(shoudlerAngle*180/math.pi)
        moveElbow(elbowAngle*180/math.pi)

