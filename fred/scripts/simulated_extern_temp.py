#!/usr/bin/env python
import rospy
import random
from numpy.random import normal
from math import sqrt
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from sensor_msgs.msg import Temperature
import numpy as np
from math import sqrt
from cv_bridge import CvBridge, CvBridgeError
import cv2
#from math import abs

class ColorToTemp:

    def __init__(self):
        self.extern_temp = 0
        self.subscriber = rospy.Subscriber("/fred/camera/camera", Image , self.front_callback, queue_size = 10)

    def front_callback(self, ros_img):
        bridge = CvBridge()
        try:
            img = bridge.imgmsg_to_cv2(ros_img, "bgr8")
            sum = 0

            for i in range(10):
                for j in range(10):
                    sum = sum + img[150 + i][235 + j][2]
            temp = (sum/100)

            self.extern_temp = temp*2 + 20 

        #    print (self.extern_temp)
        except CvBridgeError as e:
            print (e)


def simulated_extern_temp():
    seq = 0
    rospy.init_node('simulated_extern_temp')
    #rospy.Subscriber("/fred/camera/camera", Image , callback)
    extern_sensor = rospy.Publisher('extern_temp', Temperature, queue_size = 10)
    rate = rospy.Rate(10)
    ctt = ColorToTemp()
    #print(ctt.extern_temp);
    while not rospy.is_shutdown():
        h = Header(seq, rospy.Time.now(), '')
        extern_sensor.publish(Temperature(h, ctt.extern_temp, 0))
        seq = seq+1;
        rate.sleep



if __name__ == '__main__':
    simulated_extern_temp()
