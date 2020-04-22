#!/usr/bin/env python
import rospy

import tf
import tf2_ros
import geometry_msgs.msg
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

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

		#wheel position updates:
		self.wheel1 = geometry_msgs.msg.TransformStamped()
		
		self.wheel1.header.stamp = rospy.Time.now()
		self.wheel1.header.frame_id = "base_link"
		self.wheel1.child_frame_id = "left_wheel"

		self.wheel1.transform.translation.x = 0
		self.wheel1.transform.translation.y = 0
		self.wheel1.transform.translation.z = 0

		self.wheel1.transform.rotation.x = quat[0]
		self.wheel1.transform.rotation.y = quat[1]
		self.wheel1.transform.rotation.z = quat[2]
		self.wheel1.transform.rotation.w = quat[3]

		self.broadcaster.sendTransform(self.wheel1)

		self.wheel2 = geometry_msgs.msg.TransformStamped()
		
		self.wheel2.header.stamp = rospy.Time.now()
		self.wheel2.header.frame_id = "base_link"
		self.wheel2.child_frame_id = "right_wheel"

		self.wheel2.transform.translation.x = 0
		self.wheel2.transform.translation.y = 0
		self.wheel2.transform.translation.z = 0

		self.wheel2.transform.rotation.x = quat[0]
		self.wheel2.transform.rotation.y = quat[1]
		self.wheel2.transform.rotation.z = quat[2]
		self.wheel2.transform.rotation.w = quat[3]

		self.broadcaster.sendTransform(self.wheel2)
		
		#setup callback on IMU messages
		rospy.Subscriber("imu", Imu, self.update_position)
		
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.periodic()
			r.sleep()
		
	def update_position(self, msg):
		self.transform.transform.rotation = Quaternion(*tf.transformations.unit_vector([0, 0, msg.orientation.z, msg.orientation.w]))

	def periodic(self):
		stamp = rospy.Time.now()
		self.transform.header.stamp = stamp
		self.wheel1.header.stamp = stamp
		self.wheel2.header.stamp = stamp
		self.broadcaster.sendTransform(self.transform)
		self.broadcaster.sendTransform(self.wheel1)
		self.broadcaster.sendTransform(self.wheel2)


if __name__ == '__main__':
	pb = PositionBroadcaster()
	rospy.spin()
