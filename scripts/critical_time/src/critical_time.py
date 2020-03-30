#!/usr/bin/env python
import rospy
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Temperature
from std_msgs.msg import Time
#Change the following line to import whatever file contains the comsol estimation function
import comsol_function_template

class crit_time_node:
    batt_time_remaining = None
    heat_time_remaining = None
    temp_in = None
    temp_out = None
    batt_charge = None
    batt_deriv = None
    last_batt_measurement = None
    pub = None
    rate = None

    def __init__(self):
        rospy.init_node('critical_time_node', anonymous=True)
        #need to check what topic the gui expects to find the critical time under
        rospy.Subscriber("temp_intern", Temperature, self.temp_update, "intern")
        rospy.Subscriber("temp_extern", Temperature, self.temp_update, "extern")
        rospy.Subscriber("battery", BatteryState, self.batt_update)
        self.pub = rospy.Publisher('critical_time', Time, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
        self.loop()

    def temp_update(self, temp_msg, source):
        if source == "intern":
	        self.temp_in = temp_msg.val
        else:
	        self.temp_out = temp_msg.val
        if self.temp_in is not None and self.temp_out is not None:
	    #Change the following line to the name of the comsol estimation function
	        heat_time_remaining = comsol_function_template.comsol_function(self.temp_in, self.temp_out)

    def batt_update(self, batt_msg):
        current_time = rospy.Time.now()
        if self.batt_charge is not None and self.last_batt_measurement is not None:
	        if self.batt_deriv is not None:
	            self.batt_deriv = (self.batt_deriv-(batt_msg.charge-self.batt_charge)/(current_time-self.last_batt_measurement).to_sec())/2
	        else:
	            self.batt_deriv = -(batt_msg.charge-self.batt_charge)/(current_time-self.last_batt_measurement).to_sec()
	        if self.batt_deriv is not None and self.batt_deriv>0:
	            self.batt_time_remaining = batt_msg.charge/self.batt_deriv
	        else:
	            self.batt_time_remaining = None
        self.batt_charge = batt_msg.charge
        self.last_batt_measurement = current_time

    def loop(self):
        while not rospy.is_shutdown():
	    if self.batt_time_remaining < self.heat_time_remaining and self.batt_time_remaining is not None:
	        self.pub.publish(rospy.Duration(batt_time_remaining))
	    elif self.heat_time_remaining is not None:
	        self.pub.publish(rospy.Duration(self.heat_time_remaining))
	    self.rate.sleep()

#need to make a callback function for each of the topics this node will subscribe to.

if __name__ == '__main__':
    try:
        crit_time_node()
    except rospy.ROSInterruptException:
        pass
