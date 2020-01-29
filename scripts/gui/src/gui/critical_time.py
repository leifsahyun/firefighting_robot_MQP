import os
import rospy
import rospkg

from std_msgs.msg import String
from std_msgs.msg import Time
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QLabel

class CriticalTime(Plugin):

    def __init__(self, context):
        ### General rqt plugin setup ###
        super(CriticalTime, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('critical_time')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        ### Accessing the UI definition and loading the UI ###
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('gui'), 'resource', 'CriticalTimeUI.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('critical_time.Ui')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        ### Non-template code begins here ###
        # Get a reference to the time display in the GUI
        self.display = self._widget.findChild(QLabel, "TimeDisplay")
        # Subscribe to the critical time topic with the update_text function
        rospy.Subscriber("/critical_time", Time, self.update_text)
            
    # Called when a new critical time estimate is published
    # Updates the time displayed with the time in msg, which will be a ROS Duration
    def update_text(self, msg):
        rospy.loginfo("received message")
        secs = msg.data.to_sec()
        if secs>10*60:
            self.display.setText(
            '<b style="color:green;font-size:20pt;">'+str(int(secs//60))+":"+str(int(secs%60))+"</b>"
            )
        elif secs>5*60:
            self.display.setText(
            '<b style="color:orange;font-size:20pt;">'+str(int(secs//60))+":"+str(int(secs%60))+"</b>"
            )
        else:
            self.display.setText(
            '<b style="color:red;font-size:20pt;">'+str(int(secs//60))+":"+str(int(secs%60))+"</b>"
            )

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
