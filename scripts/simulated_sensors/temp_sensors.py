#!/usr/bin/env python
import rospy
import random
from sensor_msgs.msg import Temperature
from std_msgs.msg import Header

def temp_sensors():
	base_temp = 10
	intern_variance = 1
	intermediate_variance = 2
	extern_variance = 4
	intern_sensor = rospy.Publisher('intern_temp_sensor', Temperature)
	intermediate_sensor = rospy.Publisher('intermediate_temp_sensor', Temperature)
	extern_sensor = rospy.Publisher('extern_temp_sensor', Temperature)
	rospy.init_node('temp_sensors')
	rate = rospy.Rate(10)
	seq = 0
	while not rospy.is_shutdown():
		h = Header(seq, rospy.Time.now(), '')
		intern_sensor.publish(Temperature(h,base_temp+random.random()*intern_variance,intern_variance))
		intermediate_sensor.publish(Temperature(h,base_temp+random.random()*intermediate_variance,intermediate_variance))
		extern_sensor.publish(Temperature(h,base_temp+random.random()*extern_variance,extern_variance))
		seq = seq+1
		rate.sleep()

if __name__ == '__main__':
	temp_sensors()
