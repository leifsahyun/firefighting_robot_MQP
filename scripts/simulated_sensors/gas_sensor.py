#!/usr/bin/env python
import rospy
import random
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header

def gas_sensor():
	base_humid = 0.5 #percent water vapor
	humid_variance = 0.2 #percent water vapor
	base_pressure = 101325 #in pascals
	pressure_variance = 1000 #in pascals
	base_temp = 10
	temp_variance = 1
	humid_sensor = rospy.Publisher('BME680-humidity', RelativeHumidity)
	pressure_sensor = rospy.Publisher('BME680-pressure', FluidPressure)
	temp_sensor = rospy.Publisher('BME680-temperature', Temperature)
	rospy.init_node('BME680')
	rate = rospy.Rate(10)
	seq = 0
	while not rospy.is_shutdown():
		h = Header(seq, rospy.Time.now(), '')
		humid_sensor.publish(RelativeHumidity(h,base_humid-0.5*humid_variance+random.random()*humid_variance,humid_variance))
		pressure_sensor.publish(FluidPressure(h,base_pressure-0.5*pressure_variance+random.random()*pressure_variance,pressure_variance))
		temp_sensor.publish(Temperature(h,base_temp-0.5*temp_variance+random.random()*temp_variance,temp_variance))
		seq = seq+1
		rate.sleep()

if __name__ == '__main__':
	gas_sensor()
