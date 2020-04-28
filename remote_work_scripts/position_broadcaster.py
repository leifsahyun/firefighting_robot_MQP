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
		self.start = rospy.Time.now()

		self.broadcaster = tf2_ros.TransformBroadcaster()
		
		#initial position broadcast
		self.transform = geometry_msgs.msg.TransformStamped()
		
		self.transform.header.stamp = rospy.Time.now()
		self.transform.header.frame_id = "map"
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

		self.radar = geometry_msgs.msg.TransformStamped()
		
		self.radar.header.stamp = rospy.Time.now()
		self.radar.header.frame_id = "base_link"
		self.radar.child_frame_id = "radar"

		self.radar.transform.translation.x = 0
		self.radar.transform.translation.y = 0.125
		self.radar.transform.translation.z = 0

		quat = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
		self.radar.transform.rotation.x = quat[0]
		self.radar.transform.rotation.y = quat[1]
		self.radar.transform.rotation.z = quat[2]
		self.radar.transform.rotation.w = quat[3]

		self.broadcaster.sendTransform(self.radar)


		self.camera = geometry_msgs.msg.TransformStamped()
		
		self.camera.header.stamp = rospy.Time.now()
		self.camera.header.frame_id = "base_link"
		self.camera.child_frame_id = "camera"

		self.camera.transform.translation.x = 0
		self.camera.transform.translation.y = 0.125
		self.camera.transform.translation.z = 0

		quat = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
		self.camera.transform.rotation.x = quat[0]
		self.camera.transform.rotation.y = quat[1]
		self.camera.transform.rotation.z = quat[2]
		self.camera.transform.rotation.w = quat[3]

		self.broadcaster.sendTransform(self.camera)


		self.therm1 = geometry_msgs.msg.TransformStamped()
		
		self.therm1.header.stamp = rospy.Time.now()
		self.therm1.header.frame_id = "base_link"
		self.therm1.child_frame_id = "therm1"

		self.therm1.transform.translation.x = 0
		self.therm1.transform.translation.y = 0.125
		self.therm1.transform.translation.z = 0

		quat = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
		self.therm1.transform.rotation.x = quat[0]
		self.therm1.transform.rotation.y = quat[1]
		self.therm1.transform.rotation.z = quat[2]
		self.therm1.transform.rotation.w = quat[3]

		self.broadcaster.sendTransform(self.therm1)


		self.therm2 = geometry_msgs.msg.TransformStamped()
		
		self.therm2.header.stamp = rospy.Time.now()
		self.therm2.header.frame_id = "base_link"
		self.therm2.child_frame_id = "therm2"

		self.therm2.transform.translation.x = 0.125
		self.therm2.transform.translation.y = 0
		self.therm2.transform.translation.z = 0

		quat = tf.transformations.quaternion_from_euler(0, 0, 0)
		self.therm2.transform.rotation.x = quat[0]
		self.therm2.transform.rotation.y = quat[1]
		self.therm2.transform.rotation.z = quat[2]
		self.therm2.transform.rotation.w = quat[3]

		self.broadcaster.sendTransform(self.therm2)


		self.therm3 = geometry_msgs.msg.TransformStamped()
		
		self.therm3.header.stamp = rospy.Time.now()
		self.therm3.header.frame_id = "base_link"
		self.therm3.child_frame_id = "therm3"

		self.therm3.transform.translation.x = -0.125
		self.therm3.transform.translation.y = 0
		self.therm3.transform.translation.z = 0

		quat = tf.transformations.quaternion_from_euler(0, 0, math.pi)
		self.therm3.transform.rotation.x = quat[0]
		self.therm3.transform.rotation.y = quat[1]
		self.therm3.transform.rotation.z = quat[2]
		self.therm3.transform.rotation.w = quat[3]

		self.broadcaster.sendTransform(self.therm3)
		
		#setup callback on IMU messages
		#rospy.Subscriber("imu", Imu, self.update_position)
		
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.periodic()
			r.sleep()
		
	def update_position(self, msg):
		self.transform.transform.rotation = Quaternion(*tf.transformations.unit_vector([0, 0, msg.orientation.z, msg.orientation.w]))

	def periodic(self):
		stamp = rospy.Time.now()
		speed = 2*math.pi/20
		quat = tf.transformations.quaternion_from_euler(0, 0, speed*(stamp-self.start).to_sec())
		self.transform.transform.rotation.x = quat[0]
		self.transform.transform.rotation.y = quat[1]
		self.transform.transform.rotation.z = quat[2]
		self.transform.transform.rotation.w = quat[3]

		self.transform.header.stamp = stamp
		self.wheel1.header.stamp = stamp
		self.wheel2.header.stamp = stamp
		self.radar.header.stamp = stamp
		self.camera.header.stamp = stamp
		self.therm1.header.stamp = stamp
		self.therm2.header.stamp = stamp
		self.therm3.header.stamp = stamp
		self.broadcaster.sendTransform(self.transform)
		self.broadcaster.sendTransform(self.wheel1)
		self.broadcaster.sendTransform(self.wheel2)
		self.broadcaster.sendTransform(self.radar)
		self.broadcaster.sendTransform(self.camera)
		self.broadcaster.sendTransform(self.therm1)
		self.broadcaster.sendTransform(self.therm2)
		self.broadcaster.sendTransform(self.therm3)


if __name__ == '__main__':
	pb = PositionBroadcaster()
