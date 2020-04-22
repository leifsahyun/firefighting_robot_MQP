#!/usr/bin/env python

from rqt_gui_py.plugin import Plugin
#from rosgraph_msgs.msg import Log
from rqt_topic.topic import Topic
from short_topic_widget import ShortTopicWidget

class ShortTopic(Plugin):
	def __init__(self, context):
		super(ShortTopic, self).__init__(context)
		self.setObjectName('Topic')

		self._widget = ShortTopicWidget(self)

		self._widget.start()
		if context.serial_number() > 1:
			self._widget.setWindowTitle(
				self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		context.add_widget(self._widget)

		#self._widget.set_selected_topics([
		#	("/extern_temp","sensor_msgs/Temperature"),
		#	("/extern_temp/temperature","std_msgs/Float64"),
		#	("/intern_temp","sensor_msgs/Temperature"),
		#	("/intern_temp/temperature","std_msgs/Float64"),
		#	("/intern_pressure","sensor_msgs/FluidPressure"),
		#	("/intern_pressure/fluid_pressure","std_msgs/Float32"),
		#	("/battery","sensor_msgs/BatteryState"),
		#	("/battery/voltage","std_msgs/String"),
		#	("/radar","sensor_msgs/Range")
		#])

	def shutdown_plugin(self):
		self._widget.shutdown_plugin()

	def save_settings(self, plugin_settings, instance_settings):
		self._widget.save_settings(plugin_settings, instance_settings)

	def restore_settings(self, plugin_settings, instance_settings):
		self._widget.restore_settings(plugin_settings, instance_settings)

