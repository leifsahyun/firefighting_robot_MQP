import os
import rospy
import rospkg

from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QRadioButton

class DriveSelector(Plugin):

    def __init__(self, context):
        ### General rqt plugin setup ###
        super(DriveSelector, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('drive_selector')

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
        ui_file = os.path.join(rospkg.RosPack().get_path('gui'), 'resource', 'DriveSelectorUI.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('drive_selector.Ui')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        ### Declare ROS publisher to be used by callbacks ###
        self.pub = rospy.Publisher('drive_style', String, queue_size=1)
        
        ### Setup button callbacks ###
        self.buttons = self._widget.findChildren(QRadioButton)
        for button in self.buttons:
            button.clicked.connect(self.handleSelection)
        
    # Button selection handler, publishes the name of whatever button has been selected
    def handleSelection(self):
        for button in self.buttons:
            if button.isChecked():
                self.pub.publish(button.text())            

    def shutdown_plugin(self):
        # unregister all publishers here
        self.pub.unregister()
        pass


