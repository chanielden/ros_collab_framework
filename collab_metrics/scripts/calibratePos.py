#!/usr/bin/env python
import rospy
import numpy as np
import math

class calibratePosition():
    def __init__(self, print_output, start_delay, calibrate_time):
        self.print_output = print_output
        self.start_delay = start_delay
        self.calibrate_time = calibrate_time

        # define a subscriber to retrive tracked bodies
        rospy.Subscriber(frame_topic, Frame, self.frame_callback)

    def spin():
        '''
        We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
        '''
        rospy.spin()

    def calculate():


        

if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)

    # read the parameters from ROS parameter server
    print_output = rospy.get_param('~print_output')
    start_delay = rospy.get_param('~start_delay')
    calibrate_time = rospy.get_param('~calibrate_time')
    calibrate = calibratePosition(print_output,start_delay,calibrate_time)
    

print("\nRUNNING\n")
print("\n%d\n" %start_delay)
print("\n%d\n" %calibrate_time)