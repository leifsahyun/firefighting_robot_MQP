#!/usr/bin/env python
# -*- coding: latin-1 -*-

from rqt_gui_py.plugin import Plugin
#from rosgraph_msgs.msg import Log
from python_qt_binding import loadUi
from rqt_topic.topic_widget import TopicWidget
from rqt_topic.topic_info import TopicInfo
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot

import os
import rospy
import roslib
import rospkg

sensor_defs = {
    "/extern_temp_sensor/temperature": {'topic_type':"sensor_msgs/Temperature", 'name':"Temperature (external)", 'units': '°C', 'yellow_range':[60, 200]},
    "/intern_temp_sensor/temperature": {'topic_type':"sensor_msgs/Temperature", 'name':"Temperature (internal)", 'units': '°C', 'yellow_range':[40, 60]},
    "/intern_pressure/fluid_pressure": {'topic_type':"sensor_msgs/FluidPressure", 'name':"Pressure", 'units': "mbars", 'yellow_range':[1100, 1200]},
    "/battery/charge": {'topic_type':"sensor_msgs/BatteryState", 'name':"Battery Charge", 'units': "mAh", 'yellow_range':[3000, 1000]},
    "/radar/range": {'topic_type':"sensor_msgs/Range", 'name':"Obstacle Range", 'units': "m", 'yellow_range':[0.5, 0.2]},
    "/critical_time/data/secs": {'topic_type':"std_msgs/Time", 'name':"Time Remaining (Estimate)", 'units': "mm:ss", 'yellow_range':[600, 300]}
}

class ShortTopicWidget(QWidget):

    SELECT_BY_NAME = 0
    SELECT_BY_MSGTYPE = 1

    _column_names = ['topic', 'type', 'bandwidth', 'rate', 'value']

    def __init__(self, plugin=None, selected_topics=None, select_topic_type=SELECT_BY_NAME):
        """
        @type selected_topics: list of tuples.
        @param selected_topics: [($NAME_TOPIC$, $TYPE_TOPIC$), ...]
        @type select_topic_type: int
        @param select_topic_type: Can specify either the name of topics or by
                                  the type of topic, to filter the topics to
                                  show. If 'select_topic_type' argument is
                                  None, this arg shouldn't be meaningful.
        """
        super(ShortTopicWidget, self).__init__()

        self._select_topic_type = select_topic_type

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('gui'), 'resource', 'ShortTopicWidget.ui')
        loadUi(ui_file, self)
        self._plugin = plugin
        
        self._current_topic_list = []
        self._topics = {}
        self._tree_items = {}
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)

        # init and start update timer
        self._timer_refresh_topics = QTimer(self)
        self._timer_refresh_topics.timeout.connect(self.refresh_topics)

        #rospy.loginfo("super initialization complete")
        self._selected_topics = []
        #rospy.loginfo("selected topic keys")
        if selected_topics is not None:
            self._selected_topics.extend(selected_topics)
        for topic_name in sensor_defs.keys():
            #rospy.loginfo(topic_name.split('/'))
            #rospy.loginfo(topic_name.split('/')[1])
            self._selected_topics.append(("/"+topic_name.split('/')[1], sensor_defs[topic_name]['topic_type']))
        #rospy.loginfo("main initialization complete")
        
        

    def start(self):
        """
        This method needs to be called to start updating topic pane.
        """
        self._timer_refresh_topics.start(1000)
        
        
        
    @Slot()
    def refresh_topics(self):
        """
        refresh tree view items
        """
        try:
            if self._selected_topics is None:
                topic_list = rospy.get_published_topics()
                if topic_list is None:
                    rospy.logerr(
                        'Not even a single published topic found. Check network configuration')
                    return
            else:  # Topics to show are specified.
                topic_list = self._selected_topics
                topic_specifiers_server_all = None
                topic_specifiers_required = None

                rospy.logdebug('refresh_topics) self._selected_topics=%s' % (topic_list,))

                if self._select_topic_type == self.SELECT_BY_NAME:
                    topic_specifiers_server_all = \
                        [name for name, type in rospy.get_published_topics()]
                    topic_specifiers_required = [name for name, type in topic_list]
                    # The required topics that match with published topics.
                    topics_match = [(name, type) for name, type in rospy.get_published_topics()
                                    if name in topic_specifiers_required]
                    topic_list = topics_match
                elif self._select_topic_type == self.SELECT_BY_MSGTYPE:
                    # The topics that are required (by whoever uses this class).
                    topic_specifiers_required = [type for name, type in topic_list]

                    # The required topics that match with published topics.
                    topics_match = [(name, type) for name, type in rospy.get_published_topics()
                                    if type in topic_specifiers_required]
                    topic_list = topics_match
                    rospy.logdebug('selected & published topic types=%s' % (topic_list,))

                rospy.logdebug('server_all=%s\nrequired=%s\ntlist=%s' %
                               (topic_specifiers_server_all, topic_specifiers_required, topic_list))
                if len(topic_list) == 0:
                    rospy.logerr(
                        'None of the following required topics are found.\n(NAME, TYPE): %s' %
                        (self._selected_topics,))
                    return
        except IOError as e:
            rospy.logerr("Communication with rosmaster failed: {0}".format(e.strerror))
            return

        if self._current_topic_list != topic_list:
            self._current_topic_list = topic_list

            # start new topic dict
            new_topics = {}

            for topic_name, topic_type in topic_list:
                # if topic is new or has changed its type
                if topic_name not in self._topics or \
                   self._topics[topic_name]['type'] != topic_type:
                    # create new TopicInfo
                    topic_info = TopicInfo(topic_name, topic_type)
                    topic_info.start_monitoring()
                    message_instance = None
                    if topic_info.message_class is not None:
                        message_instance = topic_info.message_class()
                    # add it to the dict and tree view
                    topic_item = self._recursive_create_widget_items(
                        self.layout, topic_name, topic_type, message_instance)
                    new_topics[topic_name] = {
                        'item': topic_item,
                        'info': topic_info,
                        'type': topic_type,
                    }
                else:
                    # if topic has been seen before, copy it to new dict and
                    # remove it from the old one
                    new_topics[topic_name] = self._topics[topic_name]
                    del self._topics[topic_name]

            # clean up old topics
            #for topic_name in self._topics.keys():
            #    self._topics[topic_name]['info'].stop_monitoring()
            #    index = self.topics_tree_widget.indexOfTopLevelItem(
            #        self._topics[topic_name]['item'])
            #    self.topics_tree_widget.takeTopLevelItem(index)

            # switch to new topic dict
            self._topics = new_topics

        self._update_topics_data()
        


    def _update_topics_data(self):
        #rospy.loginfo("updating topics")
        for topic in self._topics.values():
            topic_info = topic['info']

            # update values
            value_text = ''
            self.update_value(topic_info._topic_name, topic_info.last_message)

            #self._tree_items[topic_info._topic_name].setText(self._column_index['rate'], rate_text)
            #self._tree_items[topic_info._topic_name].setData(
            #    self._column_index['bandwidth'], Qt.UserRole, bytes_per_s)
            #self._tree_items[topic_info._topic_name].setText(
            #    self._column_index['bandwidth'], bandwidth_text)
            #self._tree_items[topic_info._topic_name].setText(
            #    self._column_index['value'], value_text)

    def update_value(self, topic_name, message):
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name in message.__slots__:
                self.update_value(topic_name + '/' + slot_name, getattr(message, slot_name))

        elif type(message) in (list, tuple) and \
                (len(message) > 0) and \
                hasattr(message[0], '__slots__'):

            for index, slot in enumerate(message):
                if topic_name + '[%d]' % index in self._tree_items:
                    self.update_value(topic_name + '[%d]' % index, slot)
                #else:
                #    base_type_str, _ = self._extract_array_info(
                #        self._tree_items[topic_name].text(self._column_index['type']))
                #    self._recursive_create_widget_items(
                #        self._tree_items[topic_name],
                #        topic_name + '[%d]' % index, base_type_str, slot)
            # remove obsolete children
            if len(message) < self._tree_items[topic_name].childCount():
                for i in range(len(message), self._tree_items[topic_name].childCount()):
                    item_topic_name = topic_name + '[%d]' % i
                    self._recursive_delete_widget_items(self._tree_items[item_topic_name])
        else:
            if topic_name in self._tree_items:
                self._tree_items[topic_name].setText(self._column_index['value'], repr(message))
                

    def _recursive_create_widget_items(self, parent, topic_name, type_name, message):
        #rospy.loginfo("creating widgets")
        if topic_name in sensor_defs:
            item = SensorDisplayItem(topic_name, parent)
            self._tree_items[topic_name] = item
        else:
            item=parent
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            for slot_name, type_name in zip(message.__slots__, message._slot_types):
                self._recursive_create_widget_items(
                    item, topic_name + '/' + slot_name, type_name, getattr(message, slot_name))

        #else:
        #    base_type_str, array_size = self._extract_array_info(type_name)
        #    try:
        #        base_instance = roslib.message.get_message_class(base_type_str)()
        #    except (ValueError, TypeError):
        #        base_instance = None
        #    if array_size is not None and hasattr(base_instance, '__slots__'):
        #        for index in range(array_size):
        #            self._recursive_create_widget_items(
        #                item, topic_name + '[%d]' % index, base_type_str, base_instance)
        return item
        
    def shutdown_plugin(self):
        for topic in self._topics.values():
            topic['info'].stop_monitoring()
        self._timer_refresh_topics.stop()
        
        
class SensorDisplayItem(QLabel):
    
    def __init__(self, topic_name, parent=None):
        #rospy.loginfo("creating a single display item")
        super(SensorDisplayItem, self).__init__()
        self._topic_name = topic_name
        parent.addWidget(self)

    def setData(self, column, role, value):
        #super(TreeWidgetItem, self).setData(column, role, value)
        #set data in QLabel
        if value is None or value == "None":
            return
        value = float(value)
        color = 'orange'
        if sensor_defs[self._topic_name]['yellow_range'][0] < sensor_defs[self._topic_name]['yellow_range'][1]:
            if value < sensor_defs[self._topic_name]['yellow_range'][0]:
                color = 'green'
            if value > sensor_defs[self._topic_name]['yellow_range'][1]:
                color = 'red'
        else:
            if value < sensor_defs[self._topic_name]['yellow_range'][0]:
                color = 'red'
            if value > sensor_defs[self._topic_name]['yellow_range'][1]:
                color = 'green'
        if sensor_defs[self._topic_name]['topic_type'] == "std_msgs/Time":
            super(SensorDisplayItem, self).setText('<b style="color:{textcolor};font-size:20pt;">{name}: {mins:02d}:{secs:02d} {units}</b>'.format(textcolor=color, name=sensor_defs[self._topic_name]['name'], mins=int(value//60), secs=int(value%60), units=sensor_defs[self._topic_name]['units']))
        else:
            super(SensorDisplayItem, self).setText('<b style="color:{textcolor};font-size:20pt;">{name}: {value:.2f} {units}</b>'.format(textcolor=color, name=sensor_defs[self._topic_name]['name'], value=float(value), units=sensor_defs[self._topic_name]['units']))
        
    def setText(self, column, value):
        self.setData(column, None, value)
    
    def __lt__(self, other_item):
        #column = self.treeWidget().sortColumn()
        #if column == TopicWidget._column_names.index('bandwidth'):
        #    return self.data(column, Qt.UserRole) < other_item.data(column, Qt.UserRole)
        #return super(TreeWidgetItem, self).__lt__(other_item)
        return 0;
