#!/usr/bin/env python
# script to listen to ROS joy messages and publish ROS command vel messages

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from drive_interpreter import interpret_msg
from autonomous_mode import AutoControl

global mode
mode = 'Tank Drive'
modes = ['Tank Drive', 'Arcade Drive', 'Rocket League-style Drive', 'Autonomous']
auto_control = None
curr_angle = 0

def track_angle(msg):
    quat = msg.orientation
    rpy = euler_from_quaternion(quat)
    curr_angle = rpy[2]

def change_mode(msg):
    global mode
    mode = msg.data

def read_message(msg):
    if not mode == 'Autonomous':
        vel_pub.publish(interpret_msg(mode, msg))
        
def obstacle_detect(msg):
    if mode == 'Autonomous':
        vel_pub.publish(auto_control.get_twist(msg, curr_angle))

if __name__ == '__main__':
    try:
        rospy.init_node('drive_node')
	auto_control = AutoControl()
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, read_message, queue_size=10)
        rospy.Subscriber('/drive_style', String, change_mode, queue_size=10)
        rospy.Subscriber('/radar', Range, obstacle_detect, queue_size=10)
        #rospy.Subscriber('/IMU', Imu, track_angle, queue_size=10)
        rospy.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
