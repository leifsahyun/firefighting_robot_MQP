#!/usr/bin/env python
# how the drive_selector should drive the robot for given input

from geometry_msgs.msg import Twist

axes = ['dpad_vert', 'right_horz', 'right_vert', 'right_trig', 'left_vert', 'left_horz', 'dpad_horz', 'left_trig']
buttons = ['a', 'b', 'ls', 'rb', 'start', 'lb', 'rs', 'home', 'x', 'y', 'select']

def interpret_msg(mode, msg):
    left_vert = msg.axes[axes.index('left_vert')]
    left_horz = msg.axes[axes.index('left_horz')]
    right_vert = msg.axes[axes.index('right_vert')]
    right_horz = msg.axes[axes.index('right_horz')]
    left_trig = msg.axes[axes.index('left_trig')]
    right_trig = msg.axes[axes.index('right_trig')]

    twist = Twist()

    if mode == 'tank':
        twist.linear.x = (-right_vert - left_vert) / 2.
        twist.angular.z = (right_vert - left_vert) / 2.
    elif mode == 'arcade':
        twist.linear.x = -left_vert
        twist.angular.z = right_horz
    elif mode == 'rocket':
        twist.linear.x = (right_trig - left_trig) / 2.
        twist.angular.z = left_horz

    return twist
