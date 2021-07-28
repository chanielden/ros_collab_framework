#!/usr/bin/env python

# import modules
import rospy
import numpy as np
import math
from ros_openpose.msg import Frame 
from ros_openpose.msg import Pixel

i = 0 # Setting value for first run only
firstRun = 1 # First run trigger

class Body:
    def __init__(self,bodyPts):
        self.head          = bodyPts[0]
        self.chest         = bodyPts[1]
        self.shoulderR     = bodyPts[2]
        self.elbowR        = bodyPts[3]
        self.wristR        = bodyPts[4]
        self.shoulderR     = bodyPts[5]
        self.elbowR        = bodyPts[6]
        self.wristR        = bodyPts[7]
        self.waist         = bodyPts[8]
        # Lower body excluded as it is not used [for now]

class ScoreInfo:
    def __init__(self, backlvl, armRlvl, backpt, armRpt):
        self.backlvl = backlvl
        self.armRlvl = armRlvl
        self.backpt = backpt
        self.armRpt = armRpt
        # All values are imported

class ScoreList:
    def __init__(self,inputList,score,listRange,init):
        if init == 1:
            self.list = []
        else:
            self.list = inputList
            if len(self.list) < listRange:
                self.list.append(score)
            else:
                self.list[i/sampleVal] = score
            # Makes decison on appending score vs replacing score

class RealTimePosture():
    def __init__(self,fps,sampleFreq,scoreLength,ergoTrigger,ergoPenalty):
        self.fps = fps
        self.sampleFreq = sampleFreq
        scoreLength = scoreLength
        self.ergoTrigger = ergoTrigger
        self.ergoPenalty = ergoPenalty
        
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
            myAngle = 90-math.degrees(math.atan(-(pointA.y-pointB.y)/(pointA.x-pointB.x)))
        elif pointA.x < pointB.x: # leaning back
            myAngle = (90-math.degrees(math.atan((pointA.y-pointB.y)/(pointA.x-pointB.x))))
            return myAngle

    def frame_callback(self, data):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''

        # Extracting limb point data from topic 'frame'
        bodyPts = [bodyPart.pixel for person in data.persons for bodyPart in person.bodyParts]

        # Assigning variables
        if len(bodyPts) == 25: # Check for valid input
            body = Body(bodyPts)
            scoreinfo = ScoreInfo(backlvl,armRlvl,backpt,armRpt)
        else:
            print("\nNot all body points detected during this frame\n")
            return

        ##### Calculating Angle using function
        # Arm angle - vertical down angle is 0, increases both forward and back
        self.armURAngle = self.AngleCalc(body.shoulderR, body.elbowR)
        # Back angle - vertical up angle is 0, increases both forward and back
        self.backAngle  = self.AngleCalc(body.chest, body.waist) 

        # Displaying formatted results in console
        #print('\nBack: %d \nArm: %d' %(self.armURAngle, self.backAngle))
        #print(self.armURAngle)

        # Collection global values
        global i
        global firstRun

        ### Calculating scores in floating average

        sampleVal = fps/sampleFreq
        listRange = sampleFreq*scoreLength

        # Iterative loop
        if i % sampleVal == 0: # Taking a sample at preset interval
            # Calculating scores at intervals for efficiency
            ## Back scores
            if self.backAngle < scoreinfo.backlvl[0]:
                backScore = scoreinfo.backpt[0]
            elif self.backAngle < scoreinfo.backlvl[1]:
                backScore = scoreinfo.backpt[1]
            elif self.backAngle < scoreinfo.backlvl[2]:
                backScore = scoreinfo.backpt[2]
            else:
                backScore = scoreinfo.backpt[3]
            ## Arm scores
            if self.armURAngle < scoreinfo.armRlvl[0]:
                armScore = scoreinfo.armRpt[0]
            elif self.armURAngle < scoreinfo.armRlvl[1]:
                armScore = scoreinfo.armRpt[1]
            elif self.armURAngle < [2]:
                armScore = scoreinfo.armRpt[2]
            else:
                armScore = scoreinfo.armRpt[3]

            # Calling the Score List function
            if firstRun == 1:
                slArmUR = ScoreList(0,0,0,1)
                slBack = ScoreList(0,0,0,1)
                firstRun = 0
            else:
                scorelistArmUR = ScoreList(slArmUR.list,armScore,listRange,0)
                scorelistBack = ScoreList(slBack.list,backScore,listRange,0)
                slArmUR = scorelistArmUR
                slBack = scorelistBack
                print(scorelistArmUR.list)
                print('working')
            
            return
        # Calculating the moving average only when values are updated
            avgArmScore = np.convolve(armScoreList,np.ones(len(armScoreList))/len(armScoreList),'valid')
            avgBackScore = np.convolve(backScoreList,np.ones(len(backScoreList))/len(backScoreList),'valid')

            ### Detecting extended unergonomic posture duration
            # Arm Float Calculation and Check
            if len(ergoCheckArm) == 0:      # First run
                ergoCheckArm.append(avgArmScore)
            elif len(ergoCheckArm) == 1:    # Subsequent runs
                if avgArmScore < 3:         # Score resets if arm is lowered past certain threshold
                    ergoCheckArm = []
                    ergoFloatArm = []
                    ergoSecsArm = 0
                else:                       # Score added to float
                    ergoSecsArm += 1
                    ergoFloatArm.append(armScore)
                    print("\nTRIGGERED ARM %d\n" %ergoSecsArm)
                    if ergoSecsArm >= ergoTrigger*sampleFreq:       # Duration exceeded, add penalty score
                        avgArmScore += ergoSecsArm/10*np.convolve(ergoFloatArm,np.ones(len(ergoFloatArm))/len(ergoFloatArm),'valid')
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
                    print("\nTRIGGERED BACK %d\n" %ergoSecsBack)
                    if ergoSecsBack >= ergoTrigger*sampleFreq:      # Duration exceeded, add penalty score
                        avgBackScore += ergoSecsBack/10*np.convolve(ergoFloatBack,np.ones(len(ergoFloatBack))/len(ergoFloatBack),'valid')


            # Printing
            print('\nAVG Back: %d \nAVG Arm: %d' %(avgBackScore, avgArmScore))
    
        # Iterate 'i' last
        i += 1
        if i > listRange:
            i = 0
            #firstRun = 0

        ## legacy debugging
        #rospy.loginfo('%s\n' % text)
        #rospy.loginfo(backAngle)


if __name__ == '__main__':

    # initialize ros node
    rospy.init_node('posture_node', anonymous=False)
    
    # read the parameters from ROS parameter server
    fps = rospy.get_param('~fps')
    sampleFreq = rospy.get_param('~sampleFreq')
    scoreLength = rospy.get_param('~scoreLength')
    ergoTrigger = rospy.get_param('~ergoTrigger')
    ergoPenalty = rospy.get_param('~ergoPenalty')
    backlvl = rospy.get_param('~backlvl')
    armRlvl = rospy.get_param('~armRlvl')
    backpt = rospy.get_param('~backpt')
    armRpt = rospy.get_param('~armRpt')

    # instantize the RealTimePosture class
    scoreinfo = ScoreInfo(backlvl,armRlvl,backpt,armRpt)
    posture = RealTimePosture(fps,sampleFreq,scoreLength,ergoTrigger,ergoPenalty)
    posture.spin()