#!/usr/bin/env python
import rospy
import random
from numpy.random import normal
from math import sqrt
from sensor_msgs.msg import BatteryState

def bms():
	rospy.init_node('BMS')
	rate = rospy.Rate(10)
	start = rospy.Time.now()
	mAh_cap = 3000 #battery charge max capacity
	variance = 50 #mAh variance in battery reading
	pub = rospy.Publisher('battery', BatteryState, queue_size=10)
	while not rospy.is_shutdown():
		diff = rospy.Time.now() - start
		fifteen = rospy.Duration.from_sec(900)
		charge_remaining = mAh_cap*(1-diff/fifteen)
		reading = normal(charge_remaining, sqrt(variance))
		message = BatteryState()
		message.charge = reading
		message.voltage = reading
		pub.publish(message)
		rate.sleep()

if __name__ == '__main__':
	bms()
