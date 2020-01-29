#!/usr/bin/env python
import rospy

from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import Image
import numpy as np
from math import sqrt
from cv_bridge import CvBridge, CvBridgeError
import cv2

def thermal_arrays():
	#there is currently an error in this code related to the unsigned integer arrays
	rospy.init_node('thermal_arrays')
	array1 = rospy.Publisher('/thermal_0u', UInt8MultiArray, queue_size=10)
	array2 = rospy.Publisher('/thermal_1u', UInt8MultiArray, queue_size=10)
	array3 = rospy.Publisher('/thermal_2u', UInt8MultiArray, queue_size=10)
	temp_base = 160
	variance = 10
	rate = rospy.Rate(10)
	layout = MultiArrayLayout([
		MultiArrayDimension("height", 24, 24*32),
		MultiArrayDimension("width", 32, 32)
	], 0)
	while not rospy.is_shutdown():
		reading = np.random.normal(temp_base, sqrt(variance), (24,32))
		reading_uint = reading.astype(np.uint8)
		print(type(reading_uint[0][0]))
		print(reading_uint)
		array1.publish(UInt8MultiArray(layout,list(reading_uint)))
		array2.publish(UInt8MultiArray(layout,list(reading_uint)))
		array3.publish(UInt8MultiArray(layout,list(reading_uint)))
		rate.sleep()

def alternate_thermal_arrays():
	rospy.init_node('thermal_arrays')
	test_img = cv2.imread("resources/test_image.png")
	array1 = rospy.Publisher('/thermal_img_0', Image, queue_size=10)
	array2 = rospy.Publisher('/thermal_img_1', Image, queue_size=10)
	array3 = rospy.Publisher('/thermal_img_2', Image, queue_size=10)
	rate = rospy.Rate(10)
	bridge = CvBridge()
	while not rospy.is_shutdown():
		msg_out = bridge.cv2_to_imgmsg(test_img, "bgr8")
		array1.publish(msg_out)
		array2.publish(msg_out)
		array3.publish(msg_out)
		rate.sleep()


if __name__ == "__main__":
	alternate_thermal_arrays()
