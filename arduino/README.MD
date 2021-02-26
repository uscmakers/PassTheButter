# Install Instructions

1. Install Arduino IDE
2. Get ROS Arduino libraries and install for ROS and Arduino:
```bash
sudo apt-get update
sudo apt-get install ros-melodic-rosserial-arduino
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```
3. Flash Arduino with provided code.
4. Start ros and run serial node communication (replace /dev/tty... with path matching your Arduino)
```bash
roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
5. Publish updates to the Arduino using ROS, e.g. `rostopic pub /servo std_msgs/UInt16 "data: 180"`


# References
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

http://wiki.ros.org/rosserial_arduino/Tutorials/Blink