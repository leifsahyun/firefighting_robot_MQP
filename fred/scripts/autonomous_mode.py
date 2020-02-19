#!/usr/bin/env python
# determines how the robot should move at a specific distance from an obstacle

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import random
import rospy

MAX_TURN = 1.0
MAX_LINEAR = 1.0

class AutoControl:

	def __init__(self):
		self.lock_turn = 0
		self.unlock_angle = 0

	def get_twist(range_msg, curr_angle):
		twist = Twist()
		if not self.lock_turn == 0:
			if curr_angle > self.unlock_angle and self.lock_turn > 0:
				twist.linear.x = MAX_LINEAR
				self.lock_turn = 0
			elif curr_angle < self.unlock_angle and self.lock_turn < 0:
				twist.linear.x = MAX_LINEAR
				self.lock_turn = 0
			else:
				twist.angular.z = self.lock_turn		
		elif range_msg.range < range_msg.min_range+0.05:
			#turn
			self.lock_turn = random.choice([-MAX_TURN, MAX_TURN])
			twist.angular.z = self.lock_turn
			self.unlock_angle = curr_angle + self.lock_turn * random.random()
		else:
			#go straight
			twist.linear.x = MAX_LINEAR
		return twist
