#!/usr/bin/env python
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def video_receiver():
	pub = rospy.Publisher('camera/decompressed', Image, queue_size=1)
	rospy.init_node('camera')
	bridge = CvBridge()
	#Ideally, the lines below would connect this node directly to the video being transmitted over UDP. Unfortunately, they don't work.
	#inStream = cv2.VideoCapture('udp://localhost:1234?overrun_nonfatal=1&fifo_size=50000000')
	#inStream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H','2','6','4'))
	inStream = cv2.VideoCapture("test_video.mp4")
	r = rospy.Rate(30)
	r.sleep()
	while(inStream.isOpened()):
		ret, frame = inStream.read()
		ros_frame = None
		if ret:
			try:
				ros_frame = bridge.cv2_to_imgmsg(frame, "bgr8")
			except CvBridgeError as e:
				rospy.logerr(e)
				break
			if not ros_frame == None:
				pub.publish(ros_frame)
		r.sleep()

if __name__ == '__main__':
	video_receiver()
