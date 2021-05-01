from adafruit_servokit import ServoKit
import board
import busio
import time
from pynput import keyboard

# On the Jetson Nano
# Bus 0 (pins 28,27) is board SCL_1, SDA_1 in the jetson board definition file
# Bus 1 (pins 5, 3) is board SCL, SDA in the jetson definition file
# Default is to Bus 1; We are using Bus 0, so we need to construct the busio first ...
print("Initializing Servos")
i2c_bus0=(busio.I2C(board.SCL_1, board.SDA_1))
print("Initializing ServoKit")
kit = ServoKit(channels=16, i2c=i2c_bus0)
# kit[0] is the bottom servo
# kit[1] is the top servo
print("Done initializing")

def up():
    kit.servo[0].angle=180
    kit.servo[1].angle=180
    kit.servo[2].angle=0
    kit.servo[3].angle=0

def down():
    kit.servo[0].angle=0
    kit.servo[1].angle=145
    kit.servo[2].angle=180
    kit.servo[3].angle=40

def wave():
    kit.servo[0].angle=180
    kit.servo[1].angle=180
    kit.servo[2].angle=0
    kit.servo[3].angle=0
    time.sleep(.5)
    kit.servo[2].angle=40
    kit.servo[3].angle=120
    time.sleep(.3)
    kit.servo[3].angle=40
    time.sleep(.3)
    kit.servo[3].angle=120
    time.sleep(.3)
    kit.servo[3].angle=40
    time.sleep(.3)
    kit.servo[3].angle=120
    time.sleep(.3)
    kit.servo[3].angle=40
    time.sleep(.3)
    kit.servo[3].angle=120
    time.sleep(.3)

#kit.servo[0].angle=0 #180 #smaller cw, larger ccw
#kit.servo[1].angle=180 #smaller cw larger ccw --down setpoint at 150, straight at 105, back at 180.
#kit.servo[3].angle=0 #100 straight 40 down
#kit.servo[2].angle=180 #0 up 180 up
