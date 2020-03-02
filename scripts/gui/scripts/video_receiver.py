#!/usr/bin/env python
import rospy
import cv2
import subprocess as sp
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#This ffmpeg command definition from https://stackoverflow.com/questions/35166111/opencv-python-reading-video-from-named-pipe
FFMPEG_BIN = "ffmpeg"
command = [ FFMPEG_BIN,
        '-i', '/home/leif/video_streaming_ws/video_pipe.h264',             # fifo is the named pipe
        '-pix_fmt', 'bgr24',      # opencv requires bgr24 pixel format.
        '-vcodec', 'rawvideo',
        '-an','-sn',              # we want to disable audio processing (there is no audio)
        '-f', 'image2pipe', '-']

#frame size and speed definitions
frame_width = 640
frame_height = 480
frame_per_sec = 30

def video_receiver():
	pub = rospy.Publisher('camera/decompressed', Image, queue_size=10)
	rospy.init_node('camera')
	bridge = CvBridge()
	#Start the ffmpeg subprocess defined above
	pipe = sp.Popen(command, stdout = sp.PIPE, bufsize=10**8)
	print("ffmpeg subprocess begun")
	r = rospy.Rate(frame_per_sec)
	#r.sleep()
	while not rospy.is_shutdown():
		if pipe:
			# Capture frame-by-frame
			raw_image = pipe.stdout.read(frame_width*frame_height*3)
			pipe.stdout.flush()
			if raw_image:
				# transform the byte read into a numpy array
				frame =  np.fromstring(raw_image, dtype='uint8')
				frame = frame.reshape((frame_height,frame_width,3))
				ros_frame = None
				if frame is not None:
					try:
						ros_frame = bridge.cv2_to_imgmsg(frame, "bgr8")
					except CvBridgeError as e:
						#rospy.logerr(e)
						break
					if ros_frame is not None:
						pub.publish(ros_frame)
				#r.sleep()
		else:
			print("stream ended, starting again")
			inStream.open(pipe.name)

if __name__ == '__main__':
	video_receiver()
