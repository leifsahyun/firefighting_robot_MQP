#!/usr/bin/env python
import rospy

start_time = None
def comsol_function(intern_temp, extern_temp):
	if start_time is not None:
		return (start_time-rospy.Time.now()+rospy.Duration.from_sec(900)).to_sec()
	else:
		return 900
	#Demi, put your comsol function here. The code at present just counts down from 15 minutes. Intern_temp and extern_temp are the internal and external sensed temperatures.
