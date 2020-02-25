#!/usr/bin/env python
# determines how the robot should move at a specific distance from an obstacle

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion
import random
import rospy
import tf
import math

MAX_TURN_DIST = 0.5
MAX_TURN_SPEED = 1.0
MAX_LINEAR = 0.5
MIN_RANGE_MARGIN = 0.1
MAX_TURN_MARGIN = 0.01

#helper function to fix -pi/pi range on angles
def fix_angle(angle):
	#rospy.loginfo("fix angle in: %f", angle)
	angle = math.fmod(angle, 2*math.pi)
	if abs(angle) > math.pi:
		angle = angle * (1.0 - 2*math.pi/abs(angle))
	#rospy.loginfo("fix angle out: %f", angle)
	return angle

#helper functions for proportional control
def calc_proportional_linear(curr, minimum, maximum, scale):
	return scale*(curr-minimum)/(maximum-minimum)

def calc_proportional_angular(curr, origin, goal, scale):
	return scale*fix_angle(goal-curr)/abs(fix_angle(origin-goal))

class AutoControl:

	def __init__(self):
		self.unlock_angle = None
		self.origin_angle = None
		self.turning_dir = None
		self.tfListener = tf.TransformListener()

	def get_twist(self, range_msg, curr_angle):
		#declare variable for result
		twist = Twist()
		#get range values from message
		curr_range = range_msg.range
		min_range = range_msg.min_range + MIN_RANGE_MARGIN
		max_range = range_msg.max_range
		#get current angle relative to world
		try:
			(trans, rot) = self.tfListener.lookupTransform('odom', 'chassis', rospy.Time(0))
			rpy = euler_from_quaternion(rot)			
			curr_angle = rpy[2]
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			rospy.logerr("Error getting tranformation")
		#if currently in a turn, continue or stop based on angle
		if not self.unlock_angle is None:
			if ( (fix_angle(self.unlock_angle-self.origin_angle)>0 and curr_angle>self.unlock_angle-MAX_TURN_MARGIN) or
				(fix_angle(self.unlock_angle-self.origin_angle)<0 and curr_angle<self.unlock_angle+MAX_TURN_MARGIN) ):
				#reached the end of the turn, stop if no wall in front, otherwise set a new goal
				if curr_range < min_range:
					rospy.loginfo("CONTINUE TURN - NEW GOAL")
					self.origin_angle = curr_angle
					self.unlock_angle = fix_angle( curr_angle + MAX_TURN_DIST * self.turning_dir * random.random() )
				else:
					rospy.loginfo("END TURN")
					self.unlock_angle = None
					#if we will be moving more than a marginal amount, forget the turning direction
					if curr_range-min_range>MIN_RANGE_MARGIN:
						self.turning_dir = None
			else:
				#continue turn
				speed = calc_proportional_angular(curr_angle, self.origin_angle, self.unlock_angle, MAX_TURN_SPEED)
				rospy.loginfo("AUTO MODE: TURN, %f", speed)
				rospy.loginfo("ANGLE/GOAL: %f / %f", curr_angle, self.unlock_angle)
				twist.linear.x = 0
				twist.angular.z = speed
		#if obstacle in min range, begin turn	
		elif curr_range < min_range:
			#turn
			rospy.loginfo("BEGIN TURN")
			self.origin_angle = curr_angle
			if self.turning_dir is None:
				self.turning_dir = random.choice([1.0, -1.0])
			self.unlock_angle = fix_angle( curr_angle + MAX_TURN_DIST * self.turning_dir * random.random() )
		#go straight if no obstacle and no turn		
		else:
			#go straight
			speed = max(calc_proportional_linear(curr_range, min_range, max_range, MAX_LINEAR), 0)
			rospy.loginfo("AUTO MODE: STRAIGHT, %f", speed)
			rospy.loginfo("RANGE: %f", curr_range)
			#calculate speed proportionally to distance
			twist.linear.x = speed
		#return result
		return twist
