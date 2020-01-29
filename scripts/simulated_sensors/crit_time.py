#!/usr/bin/env python
import rospy
from std_msgs.msg import Time

def crit_time():
	pub = rospy.Publisher('critical_time', Time, queue_size=1)
	rospy.init_node('CriticalTimeNode')
	rate = rospy.Rate(10)
	start = rospy.Time.now()
	while not rospy.is_shutdown():
		diff = rospy.Time.now() - start
		fifteen = rospy.Duration.from_sec(900)
		result = fifteen - diff
		pub.publish(result)
		rate.sleep()


if __name__ == '__main__':
	crit_time()
