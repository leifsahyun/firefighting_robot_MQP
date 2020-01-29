#!/usr/bin/env python
import rospy
import random
from numpy.random import normal
from math import sqrt
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header

def gas_sensor():
	base_humid = 0.5 #percent water vapor
	humid_variance = 0.2 #percent water vapor
	base_pressure = 101325 #in pascals
	pressure_variance = 1000 #in pascals
	base_temp = 50
	temp_variance = 1
	humid_sensor = rospy.Publisher('BME680-humidity', RelativeHumidity)
	pressure_sensor = rospy.Publisher('BME680-pressure', FluidPressure)
	temp_sensor = rospy.Publisher('BME680-temperature', Temperature)
	rospy.init_node('BME680')
	rate = rospy.Rate(10)
	seq = 0
	while not rospy.is_shutdown():
		h = Header(seq, rospy.Time.now(), '')
		curr_humid = normal(base_humid, sqrt(humid_variance))
		curr_pressure = normal(base_pressure, sqrt(pressure_variance))
		curr_temp = normal(base_temp, sqrt(temp_variance))
		humid_sensor.publish(RelativeHumidity(h,curr_humid,humid_variance))
		pressure_sensor.publish(FluidPressure(h,curr_pressure,pressure_variance))
		temp_sensor.publish(Temperature(h,curr_temp,temp_variance))
		seq = seq+1
		rate.sleep()

if __name__ == '__main__':
	gas_sensor()
