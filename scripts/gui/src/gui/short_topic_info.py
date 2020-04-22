#!/usr/bin/env python

from rqt_gui_py.plugin import Plugin
#from rosgraph_msgs.msg import Log
from rqt_topic.topic_info import TopicInfo

class ShortTopicInfo(TopicInfo):

	def __init__(self, topic_name, topic_type):
		super(ShortTopicInfo, self).__init__(topic_name, topic_type)
