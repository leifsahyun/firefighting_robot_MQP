#!/usr/bin/env python

from rqt_gui_py.plugin import Plugin
#from rosgraph_msgs.msg import Log
from rqt_topic.topic import Topic

class ShortTopic(Topic):
	def __init__(self, context):
		super(ShortTopic, self).__init__(context)
		self._widget.set_selected_topics([
			("/extern_temp_sensor","sensor_msgs/Temperature"),
			("/extern_temp_sensor/temperature","std_msgs/Float64"),
			("/intern_temp_sensor","sensor_msgs/Temperature"),
			("/intern_temp_sensor/temperature","std_msgs/Float64"),
			("/BME680-pressure","sensor_msgs/FluidPressure"),
			("/BME680-pressure/fluid_pressure","std_msgs/Float64"),
			("/battery","sensor_msgs/BatteryState"),
			("/battery/voltage","std_msgs/String")
		])

