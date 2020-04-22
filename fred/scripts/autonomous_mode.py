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
MIN_TURN_DIST = 0.1
MAX_TURN_SPEED = 0.5
MAX_LINEAR = 0.5
MIN_RANGE_MARGIN = 0.1
MAX_TURN_MARGIN = 0.01
ROBOT_LENGTH = 0.3048/2+2*0.1016 #distance between center of wheel and back edge of the robot
WHEEL_BASE = 0.3048 #distance between wheels
D = WHEEL_BASE/2 #half wheel base

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

#gets the minimum turning radius so robot rear end only swings out by obstacle_dist
def get_min_R(obstacle_dist):
	if obstacle_dist >= ROBOT_LENGTH:
		return 0 #will never swing out to obstacle_dist
	else: #obstacle_dist < L
		return (math.pow(ROBOT_LENGTH,2)-math.pow(obstacle_dist,2))/(2*obstacle_dist)-D #R=(L^2-dist^2)/2dist - D

#calculates the distance the robot travels forward based on the turning radius and angle to turn
def calc_forward_travel(radius, angle):
	return (radius+D)*math.sin(angle)

#calculates turn angle to go a given distance at a given radius
def calc_angle_for_dist(radius, distance):
	if distance<0 or distance>radius:
		return 2*math.pi
	return math.asin(distance/(radius+D))

class AutoControl:

	def __init__(self):
		self.unlock_angle = None
		self.origin_angle = None
		self.turning_dir = None
		self.obstacle_memory = {'left': -1, 'right': -1}
		self.fail_flag = False
		self.tfListener = tf.TransformListener()

	

	def get_twist(self, range_msg, curr_angle):
		#declare variable for result
		twist = Twist()
		if self.fail_flag: #we are stuck
			rospy.loginfo("the robot is stuck")
			return twist
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
					#rospy.loginfo("CONTINUE TURN - NEW GOAL")
					self.origin_angle = curr_angle
					self.unlock_angle = fix_angle( curr_angle + (MAX_TURN_DIST-MIN_TURN_DIST) * self.turning_dir * random.random()+MIN_TURN_DIST )
				else:
					#rospy.loginfo("END TURN")
					self.unlock_angle = None
					#if we will be moving more than a marginal amount, forget the turning direction
					if curr_range-min_range>MIN_RANGE_MARGIN:
						self.turning_dir = None
			else:
				#continue turn
				speed_ratio = (self.min_R-2*D)/(self.min_R+2*D)
				if speed_ratio > 1:
					speed_ratio = 1.0/speed_ratio
				speed = calc_proportional_angular(curr_angle, self.origin_angle, self.unlock_angle, MAX_TURN_SPEED)
				V_1 = speed
				V_2 = speed*speed_ratio
				#rospy.loginfo("AUTO MODE: TURN, %f", speed)
				#rospy.loginfo("ANGLE/GOAL: %f / %f", curr_angle, self.unlock_angle)
				twist.linear.x = (V_1+V_2)/2
				twist.angular.z = (V_1-V_2)/(2*D)
		#if obstacle in min range, begin turn
		elif curr_range < min_range and curr_range>0:
			#turn
			#rospy.loginfo("BEGIN TURN")
			#check if we remember any obstacles
			#if self.obstacle_memory['left'] > 0 and self.obstacle_memory['right'] > 0: #obstacles on both sides
			#	if self.obstacle_memory['left']<self.obstacle_memory['right']:
			#		self.turning_dir = -1.0 #turn right
			#		self.set_min_R(self.obstacle_memory['left'])
			#	else:
			#		self.turning_dir = 1.0 #turn left
			#		self.set_min_R(self.obstacle_memory['right'])
			if self.obstacle_memory['left'] > 0: #obstacle on the left
				self.turning_dir = -1.0 #turn right (these turning dir values should be reversed, but there was a bug)
				self.min_R = get_min_R(self.obstacle_memory['left'])
			elif self.obstacle_memory['right'] > 0: #obstacle on the right
				self.turning_dir = 1.0 #turn left
				self.min_R = get_min_R(self.obstacle_memory['right'])
			else: #no obstacles on the sides
				#turn at random
				self.min_R = 0
				if self.turning_dir is None:
					self.turning_dir = random.choice([1.0, -1.0])
			self.origin_angle = curr_angle
			local_max_turn = MAX_TURN_DIST
			if calc_forward_travel(self.min_R, MIN_TURN_DIST) > curr_range: #if we cannot make the minimum turn
				self.fail_flag = True
			else:
				local_max_turn = min(MAX_TURN_DIST, calc_angle_for_dist(self.min_R, curr_range))
			self.unlock_angle = fix_angle( curr_angle + (local_max_turn-MIN_TURN_DIST) * self.turning_dir * random.random() + MIN_TURN_DIST )
			#add current obstacle to obstacle memory with the distance it will be at when the turn ends
			if self.turning_dir>0: #turning left
				if self.obstacle_memory['right'] > 0: #we remember an obstacle on the right
					self.obstacle_memory['right'] = self.obstacle_memory['right'] - calc_forward_travel(self.min_R, self.unlock_angle)
				else:
					self.obstacle_memory['right'] = curr_range - calc_forward_travel(self.min_R, self.unlock_angle)
			else: #turning right
				if self.obstacle_memory['left'] > 0: #we remember an obstacle on the left
					self.obstacle_memory['left'] = self.obstacle_memory['left'] - calc_forward_travel(self.min_R, self.unlock_angle)
				else:
					self.obstacle_memory['left'] = curr_range - calc_forward_travel(self.min_R, self.unlock_angle)
		#go straight if no obstacle and no turn
		else:
			#go straight
			speed = max(calc_proportional_linear(curr_range, min_range, max_range, MAX_LINEAR), 0)
			#rospy.loginfo("AUTO MODE: STRAIGHT, %f", speed)
			#rospy.loginfo("RANGE: %f", curr_range)
			#calculate speed proportionally to distance
			twist.linear.x = speed
			#if there will be enough distance between us and the previous obstacle, forget it
			if curr_range > ROBOT_LENGTH+min_range:
				#forget obstacles
				self.obstacle_memory['left'] = -1
				self.obstacle_memory['right'] = -1
				self.min_R = 0
		#return result
		return twist
