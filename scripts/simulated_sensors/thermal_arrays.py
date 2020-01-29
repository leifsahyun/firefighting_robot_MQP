#!/usr/bin/env python
import rospy

from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import MultiArrayLayout
import numpy as np
from math import sqrt

def thermal_arrays():
	rospy.init_node('thermal_arrays')
	array1 = rospy.Publisher('/thermal_0u', UInt8MultiArray, queue_size=10)
	array2 = rospy.Publisher('/thermal_1u', UInt8MultiArray, queue_size=10)
	array3 = rospy.Publisher('/thermal_2u', UInt8MultiArray, queue_size=10)
	temp_base = 160
	variance = 100
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		reading = np.random.normal(temp_base, sqrt(variance), (24,32))
		reading_uint = reading.astype(np.uint8)
		array1.publish(UInt8MultiArray(MultiArrayLayout(),list(reading_uint)))
		array2.publish(UInt8MultiArray(MultiArrayLayout(),list(reading_uint)))
		array3.publish(UInt8MultiArray(MultiArrayLayout(),list(reading_uint)))
		rate.sleep()


if __name__ == "__main__":
	thermal_arrays()
