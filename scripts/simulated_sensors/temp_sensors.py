#!/usr/bin/env python
import rospy
import random
from numpy.random import normal
from math import sqrt
from sensor_msgs.msg import Temperature
from std_msgs.msg import Header

def temp_sensors():
	intern_base_temp = 50
	intern_variance = 1
	intermediate_base_temp = 100
	intermediate_variance = 2
	extern_base_temp = 160
	extern_variance = 4
	intern_sensor = rospy.Publisher('intern_temp_sensor', Temperature)
	intermediate_sensor = rospy.Publisher('intermediate_temp_sensor', Temperature)
	extern_sensor = rospy.Publisher('extern_temp_sensor', Temperature)
	rospy.init_node('temp_sensors')
	rate = rospy.Rate(10)
	seq = 0
	while not rospy.is_shutdown():
		h = Header(seq, rospy.Time.now(), '')
		intern_curr_temp = normal(intern_base_temp, sqrt(intern_variance))
		intermediate_curr_temp = normal(intermediate_base_temp, sqrt(intermediate_variance))
		extern_curr_temp = normal(extern_base_temp, sqrt(extern_variance))
		intern_sensor.publish(Temperature(h,intern_curr_temp,intern_variance))
		intermediate_sensor.publish(Temperature(h,intermediate_curr_temp,intermediate_variance))
		extern_sensor.publish(Temperature(h,extern_curr_temp,extern_variance))
		seq = seq+1
		rate.sleep()

if __name__ == '__main__':
	temp_sensors()
