#!/usr/bin/env python
import rospy
import cv2

import struct
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image
import numpy as np
from math import sqrt
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class thermal_array_converter:
	def __init__(self):
		rospy.init_node('thermal_array_converter')
		rospy.Subscriber('/thermal_0u',  UInt8MultiArray, self.convert_data, callback_args=(0), queue_size=10)
		rospy.Subscriber('/thermal_1u',  UInt8MultiArray, self.convert_data, callback_args=(1), queue_size=10)
		rospy.Subscriber('/thermal_2u',  UInt8MultiArray, self.convert_data, callback_args=(2), queue_size=10)
		self.pub0 = rospy.Publisher('/thermal_img_0', Image)
		self.pub1 = rospy.Publisher('/thermal_img_1', Image)
		self.pub2 = rospy.Publisher('/thermal_img_2', Image)
		self.bridge = CvBridge()
	
	def convert_data(self, msg, args):
		# which sensor sent the message
		sensor = args
		# array of temp ints coming from ROS
		data = np.zeros(768, dtype=np.uint8)
		# copy data from message packet
		for i in range(len(msg.data)):
			data[i] = struct.unpack("B", msg.data[i])[0]
		small_pixels = np.reshape(data, (24, 32))
		scaling_matrix = np.ones((30,40))
		result = np.kron(small_pixels, scaling_matrix)
		result = np.uint8(result)
		bgrResult = cv2.merge((result, result, result))
		
		msg_out = None
		try:
			msg_out = self.bridge.cv2_to_imgmsg(bgrResult, "rgb8")
		except CvBridgeError as e:
			rospy.logerr(e)
		if not msg_out == None:
			if args==0:
				self.pub0.publish(msg_out)
			elif args==1:
				self.pub1.publish(msg_out)
			else:
				self.pub2.publish(msg_out)


if __name__ == "__main__":
	thermal_array_converter()
	rospy.spin()
