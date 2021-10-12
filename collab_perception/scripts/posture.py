#!/usr/bin/env python
import rospy
import numpy as np
import math
from ros_openpose.msg import Frame 
from ros_openpose.msg import Pixel

i = 0 # Setting value for first run only
firstRun = 1 # Setting value to check for first run
armScoreUList = [] # Creating empty list
armScoreLList = [] # Creating empty list
backScoreList = [] # Creating empty list
ergoCheckArmL = [] # Creating empty list
ergoCheckArmU = [] # Creating empty list
ergoCheckBack = [] # Creating empty list
ergoFloatArmU = [] # Creating empty list
ergoFloatArmL = [] # Creating empty list
ergoFloatBack = [] # Creating empty list
ergoSecsArmU = 0 # Start at 0
ergoSecsArmL = 0 # Start at 0
ergoSecsBack = 0 # Start at 0

def callback(msg):
    # Collecting data from topic 'frame'
    bodyPts = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]

    # Collection global values
    global i
    global firstRun
    global armScoreUList
    global armScoreLList
    global backScoreList
    global ergoCheckArmU
    global ergoCheckArmL
    global ergoCheckBack
    global ergoFloatArmU
    global ergoFloatArmL
    global ergoFloatBack
    global ergoSecsArmU
    global ergoSecsArmL
    global ergoSecsBack

    # Assigning variables
    if len(bodyPts) == 25:
        chestPt = bodyPts[1]
        waistPt = bodyPts[8]
        shoulderPt = bodyPts[2]
        elbowPt = bodyPts[3]
        wristPt = bodyPts[4]
    
        ### Calculating variables

        # Back angle - vertical up angle is 0, increases both forward and back
        if (chestPt.x-waistPt.x) == 0:
            backAngle = 0
        elif chestPt.x >= waistPt.x: # leaning forwards
            backAngle = 90-math.degrees(math.atan(-(chestPt.y-waistPt.y)/(chestPt.x-waistPt.x)))
        elif chestPt.x < waistPt.x: # leaning back
            backAngle = -(90-math.degrees(math.atan((chestPt.y-waistPt.y)/(chestPt.x-waistPt.x))))

        # Upper Arm angle - vertical down angle is 0, increases both forward and back
        if (shoulderPt.x-elbowPt.x) == 0:
            armAngleU = 0
        elif shoulderPt.x >= elbowPt.x: # arm back
            armAngleU = -90+math.degrees(math.atan(-(shoulderPt.y-elbowPt.y)/(shoulderPt.x-elbowPt.x)))
        elif shoulderPt.x < elbowPt.x: # arm forward
            armAngleU = (90-math.degrees(math.atan((shoulderPt.y-elbowPt.y)/(shoulderPt.x-elbowPt.x))))

        # Lower Arm angle - vertical down angle is 0, increases both forward and back
        if (elbowPt.x-wristPt.x) == 0:
            armAngleL = 0
        elif elbowPt.x >= wristPt.x: # arm back
            armAngleL = -90+math.degrees(math.atan(-(elbowPt.y-wristPt.y)/(elbowPt.x-wristPt.x)))
        elif elbowPt.x < wristPt.x: # arm forward
            armAngleL = (90-math.degrees(math.atan((elbowPt.y-wristPt.y)/(elbowPt.x-wristPt.x))))

        # Displaying formatted results in console
        #print('\nBack: %d \nUpper Arm: %d \nLower Arm: %d' %(backAngle, armAngleU, armAngleL))
        #print('\nChestX: %d WaistX: %d\nChestY: %d WaistY: %d\n' %(chestPt.x, waistPt.x, chestPt.y, waistPt.y))

        ### Calculating scores in floating average

        ### Assigning variables
        fps = 20 # Set this variable MANUALLY (Openpose stable FPS)
        sampleFreq = 2 # Set this variable MANUALLY (times per second)
        scoreLength = 5 # Set this variable MANUALLY (seconds) Total Floating Average duration
        ergoTrigger = 5 # Set this variable MANUALLY (seconds) Time it takes for ergo penalty to kick in
        ergoPenalty = 15 # Set this variable MANUALLY (pts) Added penalty pts for extended unergonomic pose

        sampleVal = fps/sampleFreq
        listRange = sampleFreq*scoreLength

        # Iterative loop
        if i % sampleVal == 0: # Taking a sample at preset interval
            # Calculating scores at intervals for efficiency
            ## Back scores
            if abs(backAngle) < 5:
                backScore = 1
            elif abs(backAngle) < 20:
                backScore = 2
            elif abs(backAngle) < 60:
                backScore = 3
            else:
                backScore = 4
            ## Upper Arm scores
            if abs(armAngleU) < 5:
                armScoreU = 1
            elif abs(armAngleU) < 45:
                armScoreU = 2
            elif abs(armAngleU) < 90:
                armScoreU = 3
            else:
                armScoreU = 4

            ## Lower Arm scores
            if abs(armAngleL) < 60:
                armScoreL = 2
            elif abs(armAngleL) < 100:
                armScoreL = 1
            else:
                armScoreL = 2

            # Appending scores in dynamic size list
            if firstRun == 1:   # On first run we grow list size
                armScoreUList.append(armScoreU)
                armScoreLList.append(armScoreL)
                backScoreList.append(backScore)
            else:               # On subsequent runs we replace values
                armScoreUList[i/sampleVal] = armScoreU
                armScoreLList[i/sampleVal] = armScoreL
                backScoreList[i/sampleVal] = backScore
        
           # Calculating the moving average only when values are updated
            avgArmScoreU = np.convolve(armScoreUList,np.ones(len(armScoreUList))/len(armScoreUList),'valid')
            avgArmScoreL = np.convolve(armScoreLList,np.ones(len(armScoreLList))/len(armScoreLList),'valid')
            avgBackScore = np.convolve(backScoreList,np.ones(len(backScoreList))/len(backScoreList),'valid')

            ### Detecting extended unergonomic posture duration
            # Upper Arm Float Calculation and Check
            if len(ergoCheckArmU) == 0:      # First run
                ergoCheckArmU.append(avgArmScoreU)
            elif len(ergoCheckArmU) == 1:    # Subsequent runs
                if avgArmScoreU < 3:         # Score resets if arm is lowered past certain threshold
                    ergoCheckArmU = []
                    ergoFloatArmU = []
                    ergoSecsArmU = 0
                else:                       # Score added to float
                    ergoSecsArmU += 1
                    ergoFloatArmU.append(armScoreU)
                    #print("\nTRIGGERED ARM %d\n" %ergoSecsArm)
                    if ergoSecsArmU >= ergoTrigger*sampleFreq:       # Duration exceeded, add penalty score
                        avgArmScoreU += ergoSecsArmU/10*np.convolve(ergoFloatArmU,np.ones(len(ergoFloatArmU))/len(ergoFloatArmU),'valid')
            # Lower Arm Float Calculation and Check
            if len(ergoCheckArmL) == 0:      # First run
                ergoCheckArmL.append(avgArmScoreL)
            elif len(ergoCheckArmL) == 1:    # Subsequent runs
                if avgArmScoreL < 3:         # Score resets if arm is lowered past certain threshold
                    ergoCheckArmL = []
                    ergoFloatArmL = []
                    ergoSecsArmL = 0
                else:                       # Score added to float
                    ergoSecsArmL += 1
                    ergoFloatArmL.append(armScoreL)
                    #print("\nTRIGGERED ARM %d\n" %ergoSecsArm)
                    if ergoSecsArmL >= ergoTrigger*sampleFreq:       # Duration exceeded, add penalty score
                        avgArmScoreL += ergoSecsArmL/10*np.convolve(ergoFloatArmL,np.ones(len(ergoFloatArmL))/len(ergoFloatArmL),'valid')
            # Back Float Calculation and Check
            if len(ergoCheckBack) == 0:     # First run
                ergoCheckBack.append(avgBackScore)
            elif len(ergoCheckBack) == 1:   # Subsequent runs
                if avgBackScore < 3:        # Score resets if arm is lowered past certain threshold
                    ergoCheckBack = []
                    ergoFloatBack = []
                    ergoSecsBack = 0
                else:                       # Score added to float 
                    ergoSecsBack += 1
                    ergoFloatBack.append(backScore)
                    #print("\nTRIGGERED BACK %d\n" %ergoSecsBack)
                    if ergoSecsBack >= ergoTrigger*sampleFreq:      # Duration exceeded, add penalty score
                        avgBackScore += ergoSecsBack/10*np.convolve(ergoFloatBack,np.ones(len(ergoFloatBack))/len(ergoFloatBack),'valid')


            # Printing
            print('\nBack: %d \nUpper Arm: %d \nLower Arm: %d' %(backAngle, armAngleU, armAngleL))
            print('\nBack Score: %d \nUpper Arm Score: %d \nLower Arm Score: %d' %(backScore, armScoreU, armScoreL))
            print('\nAVG Back Score: %d \nAVG Upper Arm Score: %d \nAVG Lower Arm Score: %d' %(avgBackScore, avgArmScoreU, avgArmScoreL))
    
        # Iterate 'i' last
        i += 1
        if i > listRange:
            i = 0
            firstRun = 0

        ## legacy debugging
        #rospy.loginfo('%s\n' % text)
        #rospy.loginfo(backAngle)
    
    #else:
        #print("\nNot all body points detected during this frame\n")

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