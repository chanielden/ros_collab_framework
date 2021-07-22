#!/usr/bin/env python
import rospy
import numpy as np
import math
from ros_openpose.msg import Frame 
from ros_openpose.msg import Pixel

i = 0 # Setting value for first run only
firstRun = 1 # Setting value to check for first run
armScoreList = [] # Creating empty list
backScoreList = [] # Creating empty list

def callback(msg):
    # Collecting data from topic 'frame'
    bodyPts = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]

    # Collection global values
    global i
    global firstRun
    global armScoreList
    global backScoreList

    # Assigning variables
    chestPt = bodyPts[1]
    waistPt = bodyPts[8]
    shoulderPt = bodyPts[2]
    elbowPt = bodyPts[3]
    
    ### Calculating variables

    # Back angle - vertical up angle is 0, increases both forward and back
    if chestPt.x >= waistPt.x: # leaning forwards
        backAngle = 90-math.degrees(math.atan(-(chestPt.y-waistPt.y)/(chestPt.x-waistPt.x)))
    elif chestPt.x < waistPt.x: # leaning back
        backAngle = (90-math.degrees(math.atan((chestPt.y-waistPt.y)/(chestPt.x-waistPt.x))))

    # Arm angle - vertical down angle is 0, increases both forward and back
    if shoulderPt.x >= elbowPt.x: # arm back
        armAngle = 90-math.degrees(math.atan(-(shoulderPt.y-elbowPt.y)/(shoulderPt.x-elbowPt.x)))
    elif shoulderPt.x < elbowPt.x: # arm forward
        armAngle = (90-math.degrees(math.atan((shoulderPt.y-elbowPt.y)/(shoulderPt.x-elbowPt.x))))

    # Displaying formatted results in console
    #print('\nBack: %d \nArm: %d' %(backAngle, armAngle))

    ### Calculating scores in floating average

    # Assigning variables
    fps = 20 # Set this variable MANUALLY
    sampleFreq = 2 # Set this variable MANUALLY (times per second)
    scoreLength = 5 # Set this variable MANUALLY (seconds)

    sampleVal = fps/sampleFreq
    listRange = fps*scoreLength

    # Iterative loop
    if i % sampleVal == 0: # Taking a sample at preset interval
        # Calculating scores at intervals for efficiency
        ## Back scores
        if backAngle < 5:
            backScore = 1
        elif backAngle < 20:
            backScore = 3
        elif backAngle < 60:
            backScore = 5
        else:
            backScore = 7
        ## Arm scores
        if armAngle < 20:
            armScore = 1
        elif armAngle < 45:
            armScore = 3
        elif armAngle < 90:
            armScore = 5
        else:
            armScore = 7

        # Appending scores in dynamic size list
        if firstRun == 1:   # On first run we grow list size
            armScoreList.append(armScore)
            backScoreList.append(backScore)
        else:               # On subsequent runs we replace values
            armScoreList[i/sampleVal] = armScore
            backScoreList[i/sampleVal] = backScore
        
        # Calculating the moving average only when values are updated
        avgArmScore = np.convolve(armScoreList,np.ones(len(armScoreList))/len(armScoreList),'valid')
        avgBackScore = np.convolve(backScoreList,np.ones(len(backScoreList))/len(backScoreList),'valid')
        # Printing
        print('\nAVG Back: %d \nAVG Arm: %d' %(avgBackScore, avgArmScore))
    
    # Iterate 'i' last
    i += 1
    if i > listRange:
        i = 0
        firstRun = 0

    #legacy debugging
    #rospy.loginfo('%s\n' % text)
    #rospy.loginfo(backAngle)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # read the parameter from ROS parameter server
    #frame_topic = rospy.get_param('~pub_topic')

    rospy.Subscriber("frame", Frame, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()