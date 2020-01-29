#!/usr/bin/env python
import rospy
import random
from numpy.random import normal
from math import pi, sqrt
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler

def imu_sensor():
	euler_orient = (0, 0.2*pi, 0)
	variance = 0.1
	orientation_cov = [
	variance, 0, 0,
	0, variance, 0,
	0, 0, variance]
	base_angular_vel = Vector3(0, 0, 0)
	angular_vel_cov = [
	-1, -1, -1,
	-1, -1, -1,
	-1, -1, -1]
	base_accel = Vector3(0, 0, 0)
	accel_cov = [
	-1, -1, -1,
	-1, -1, -1,
	-1, -1, -1]
	imu_pub = rospy.Publisher('IMU', Imu)
	rospy.init_node('IMU')
	rate = rospy.Rate(10)
	seq = 0
	while not rospy.is_shutdown():
		h = Header(seq, rospy.Time.now(), '')
		temp_orient = quaternion_from_euler(*normal(euler_orient,sqrt(variance)))
		quat_orient = Quaternion(temp_orient[0],temp_orient[1],temp_orient[2],temp_orient[3])
		imu_pub.publish(Imu(h,quat_orient,orientation_cov,base_angular_vel,angular_vel_cov,base_accel,accel_cov))
		seq = seq+1
		rate.sleep()

if __name__ == '__main__':
	imu_sensor()
