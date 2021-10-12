#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int8
import math
from ros_openpose.msg import Frame 
from ros_openpose.msg import Pixel
from collab_decision.msg import TeleopMode as tm

class Body:
    def __init__(self,bodyPts):
        self.head          = bodyPts[0]
        self.chest         = bodyPts[1]
        self.shoulderR     = bodyPts[2]
        self.elbowR        = bodyPts[3]
        self.wristR        = bodyPts[4]
        self.shoulderL     = bodyPts[5]
        self.elbowL        = bodyPts[6]
        self.wristL        = bodyPts[7]
        self.waist         = bodyPts[8]
        # Lower body excluded as it is not used [for now]

class MovementDecision():
    def __init__(self):

        # define a subscriber to retrive tracked bodies
        rospy.Subscriber('frame', Frame, self.frame_callback)

    def spin(self):
        '''
        We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
        '''
        rospy.spin()

    def AngleCalc(self, pointA, pointB):
        
        # Back angle - vertical up angle is 0, increases both forward and back
        if (pointA.x-pointB.x) == 0:
            myAngle = 0
        elif pointA.x > pointB.x: # leaning forwards
            myAngle = 90+math.degrees(math.atan((pointA.y-pointB.y)/(pointA.x-pointB.x)))
        elif pointA.x < pointB.x: # leaning back
            myAngle = -90+math.degrees(math.atan((pointA.y-pointB.y)/(pointA.x-pointB.x)))
            
        return myAngle

    def frame_callback(self, data):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''

        # Extracting limb point data from topic 'frame'
        bodyPts = [bodyPart.pixel for person in data.persons for bodyPart in person.bodyParts]

        # Assigning variables
        if len(bodyPts) == 25: # Check for valid input length
            body = Body(bodyPts)
        else:
            #print("\nNot all body points detected during this frame\n")
            return

        ##### Calculating Angle using function
        # Upper Arm angle - vertical down angle is 0, increases forward, decreases back
        self.armURAngle = -self.AngleCalc(body.shoulderR,body.elbowR)
        
        # Lower Arm angle - vertical down angle is 0, increases forward, decreases back
        self.armLRAngle = -self.AngleCalc(body.elbowR,body.wristR)
        
        #Back angle - vertical up angle is 0, increases forward, decreases back
        self.backAngle  = self.AngleCalc(body.chest, body.waist) 

        # define a publisher for the direction of travel
        self.dir_pub = rospy.Publisher("movedir", tm, queue_size=1)

        ### Printing for troubleshooting
        #print('ArmU Angle: %d\n' %self.armURAngle)
        #print('ArmL Angle: %d\n' %self.armLRAngle)
        #print('Back Angle: %d\n' %self.backAngle)

        ### Arm angle comparison
        perfection_score = 0
        
        tm_local = tm()
        # Back
        if self.backAngle > (testBack+leeway):
            print('lean back')
            tm_local.mode = 'back'
            tm_local.dir = 1
        elif self.backAngle < (testBack-leeway):
            print('lean forward')
            tm_local.mode = 'back'
            tm_local.dir = -1
        else:
            perfection_score += 1
            tm_local.mode = 'back'
            tm_local.dir = 0

        # Upper Arm
        if perfection_score == 1:
            if self.armURAngle > (testArmUR+leeway):
                print('lower your arm')
                tm_local.mode = 'arm_upper'
                tm_local.dir = 1
            elif self.armURAngle < (testArmUR-leeway):
                print('raise your arm')
                tm_local.mode = 'arm_upper'
                tm_local.dir = -1
            else: 
                perfection_score += 1
                tm_local.mode = 'arm_upper'
                tm_local.dir = 0

        # Lower Arm
        if perfection_score == 2:
            if self.armLRAngle > (testArmLR+leeway):
                print('lower your hand')
                tm_local.mode = 'arm_lower'
                tm_local.dir = -1
            elif self.armLRAngle < (testArmLR-leeway):
                print('raise your hand')
                tm_local.mode = 'arm_lower'
                tm_local.dir = 1
            else:
                perfection_score += 1
                tm_local.mode = 'arm_lower'
                tm_local.dir = 0

        # Full tests
        if perfection_score == 3:
            print('Perfect posture! Hold in place')

        # Publish direction results
        self.dir_pub.publish(tm_local)

if __name__ == '__main__':

    # initialize ros node
    rospy.init_node('posture_node', anonymous=False)
    
    # read the parameters from ROS parameter server
    testArmUR = rospy.get_param('~armURAngle')
    testArmLR = rospy.get_param('~armLRAngle')
    testBack = rospy.get_param('~backAngle')
    leeway = rospy.get_param('~leeway')


    # instantize the RealTimePosture class
    movementDecision = MovementDecision()
    movementDecision.spin()