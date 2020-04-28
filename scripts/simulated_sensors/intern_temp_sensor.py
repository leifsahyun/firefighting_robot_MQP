#!/usr/bin/env python
import rospy
import random
from numpy.random import normal
from math import sqrt
from sensor_msgs.msg import Temperature
from std_msgs.msg import Header

lastTime = None
intern_variance = 1
intern_temp = 25
temp_resist = 3000 #degrees per (delta seconds delta degrees)

def temp_sensors():
	rospy.init_node('temp_sensors')
	global lastTime
	lastTime = rospy.Time.now()
	rate = rospy.Rate(10)
	seq = 0
	intern_sensor = rospy.Publisher('intern_temp_sensor', Temperature)
	extern_sensor = rospy.Subscriber('extern_temp_sensor', Temperature, calc_intern)
	while not rospy.is_shutdown():
		h = Header(seq, rospy.Time.now(), '')
		intern_curr_temp = normal(intern_temp, sqrt(intern_variance))
		intern_sensor.publish(Temperature(h,intern_curr_temp,intern_variance))
		seq = seq+1
		rate.sleep()

def calc_intern(message):
	global intern_temp
	global lastTime
	curr_time = rospy.Time.now()
	intern_temp = intern_temp + (message.temperature-intern_temp)*(curr_time-lastTime).to_sec()/temp_resist
	lastTime = curr_time

if __name__ == '__main__':
	temp_sensors()
