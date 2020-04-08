#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker

class ObstaclePub:
	
	def __init__(self):
		#setup node
		rospy.init_node('obstacle_marker_pub')
		self.pub = rospy.Publisher('rviz_obstacle', Marker)
		#setup message
		self.obstacle = Marker()
		self.obstacle.header.frame_id = "base_link"
		self.obstacle.header.stamp = rospy.Time.now()
		self.obstacle.type = Marker.CUBE
		self.obstacle.action = Marker.ADD
		self.obstacle.ns = "fred_markers"
		self.obstacle.id = 0
		self.obstacle.pose.position.x = 0
		self.obstacle.pose.position.y = 1
		self.obstacle.pose.position.z = 0
		self.obstacle.pose.orientation.x = 0.0
		self.obstacle.pose.orientation.y = 0.0
		self.obstacle.pose.orientation.z = 0.0
		self.obstacle.pose.orientation.w = 1.0
		self.obstacle.scale.x = 1.0
		self.obstacle.scale.y = 0.1
		self.obstacle.scale.z = 1.0
		self.obstacle.color.a = 1.0
		self.obstacle.color.r = 0.0
		self.obstacle.color.g = 0.0
		self.obstacle.color.b = 1.0
		#initial publish
		self.pub.publish(self.obstacle)
		#setup subscriber for radar messages
		rospy.Subscriber('radar', Range, self.update)

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.periodic()
			rate.sleep()

	def periodic(self):
		self.obstacle.header.stamp = rospy.Time.now()
		self.pub.publish(self.obstacle)

	def update(self, msg):
		self.obstacle.pose.position.y = msg.range

if __name__ == '__main__':
	ObstaclePub()
