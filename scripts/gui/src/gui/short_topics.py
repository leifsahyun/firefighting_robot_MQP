#!/usr/bin/env python

from rqt_gui_py.plugin import Plugin
#from rosgraph_msgs.msg import Log
from rqt_topic.topic import Topic

class ShortTopic(Topic):
	def __init__(self, context):
		super(ShortTopic, self).__init__(context)
		self._widget.set_selected_topics([
			("/extern_temp","sensor_msgs/Temperature"),
			("/extern_temp/temperature","std_msgs/Float64"),
			("/intern_temp","sensor_msgs/Temperature"),
			("/intern_temp/temperature","std_msgs/Float64"),
			("/intern_pressure","sensor_msgs/FluidPressure"),
			("/intern_pressure/fluid_pressure","std_msgs/Float32"),
			("/battery","sensor_msgs/BatteryState"),
			("/battery/voltage","std_msgs/String"),
			("/radar","sensor_msgs/Range")
		])

