#!/usr/bin/env python
import rospy
import copy
from sensor_msgs.msg import Range

def radar_broadcaster():
	seq = 0
	radar = rospy.Publisher('radar', Range, queue_size=1)
	camera = rospy.Publisher('camera', Range, queue_size=1)
	therm1 = rospy.Publisher('therm1', Range, queue_size=1)
	therm2 = rospy.Publisher('therm2', Range, queue_size=1)
	therm3 = rospy.Publisher('therm3', Range, queue_size=1)
	rospy.init_node('RadarBroadcastNode')
	rad_message = Range()
	rad_message.header.frame_id = 'radar'
	rad_message.field_of_view = 1.3962634
	rad_message.min_range = 0.2
	rad_message.max_range = 1.0
	rad_message.range = 1.0
	cam_message = Range()
	cam_message.header.frame_id = 'camera'
	cam_message.field_of_view = 1.085595
	cam_message.min_range = 0.02
	cam_message.max_range = 10
	cam_message.range = 10
	therm1_message = Range()
	therm1_message.header.frame_id = 'therm1'
	therm1_message.field_of_view = 1.91986
	therm1_message.min_range = 0.02
	therm1_message.max_range = 8
	therm1_message.range = 8
	therm2_message = copy.deepcopy(therm1_message)
	therm2_message.header.frame_id = 'therm2'
	therm3_message = copy.deepcopy(therm1_message)
	therm3_message.header.frame_id = 'therm3'
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		seq = seq+1
		stamp = rospy.Time.now()
		rad_message.header.seq = seq
		rad_message.header.stamp = stamp
		cam_message.header.seq = seq
		cam_message.header.stamp = stamp
		therm1_message.header.seq = seq
		therm1_message.header.stamp = stamp
		radar.publish(rad_message)
		camera.publish(cam_message)
		therm1.publish(therm1_message)
		therm2_message.header.seq = seq
		therm2_message.header.stamp = rospy.Time.now()
		therm2.publish(therm2_message)
		therm3_message.header.seq = seq
		therm3_message.header.stamp = rospy.Time.now()
		therm3.publish(therm3_message)
		rate.sleep()


if __name__ == '__main__':
	radar_broadcaster()
