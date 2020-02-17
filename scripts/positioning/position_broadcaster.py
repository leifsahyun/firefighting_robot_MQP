#!/usr/bin/env python
import rospy

import tf
import tf2_ros
import geometry_msgs.msg
import math
from sensor_msgs.msg import Imu

class PositionBroadcaster:
	def __init__(self):
		#setup broadcaster
		rospy.init_node('position_broadcaster')
		self.broadcaster = tf2_ros.TransformBroadcaster()
		
		#initial position broadcast
		self.transform = geometry_msgs.msg.TransformStamped()
		
		self.transform.header.stamp = rospy.Time.now()
		self.transform.header.frame_id = "world"
		self.transform.child_frame_id = "base_link"

		self.transform.transform.translation.x = 0
		self.transform.transform.translation.y = 0
		self.transform.transform.translation.z = 0

		quat = tf.transformations.quaternion_from_euler(0, 0, 0)
		self.transform.transform.rotation.x = quat[0]
		self.transform.transform.rotation.y = quat[1]
		self.transform.transform.rotation.z = quat[2]
		self.transform.transform.rotation.w = quat[3]

		self.broadcaster.sendTransform(self.transform)
		
		#setup callback on IMU messages
		rospy.Subscriber("IMU", Imu, self.update_position)
		
	def update_position(self, msg):
		self.transform.header.stamp = rospy.Time.now()
		self.transform.transform.rotation = msg.orientation
		self.broadcaster.sendTransform(self.transform)


if __name__ == '__main__':
	pb = PositionBroadcaster()
	rospy.spin()
