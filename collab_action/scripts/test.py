#! /usr/bin/env python
import rospy
from hrca_action.arm import *
from hrca_action.head import *
from geometry_msgs.msg import Pose, Point, Quaternion
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import (JointTrajectory, JointTrajectoryPoint)
import actionlib
from rail_manipulation_msgs.srv import SegmentObjects
from rail_manipulation_msgs.msg import SegmentedObject
import moveit_commander
from moveit_msgs.msg import PlanningScene, Grasp, PlaceLocation
import tf
import tf_conversions
from tf import transformations
import numpy as np
from copy import deepcopy
from tf.transformations import *

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

def callback(data):
    self.abcde = data

if __name__ == '__main__':
    print("started")
    rospy.init_node("test")

    rospy.Subscriber("/rail_segmentation/segmented_table", SegmentedObject, callback)
    
    # Move fetch to Ready pose
    arm = Arm()
    head = Head()

    #torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    # Raise the torso using just a controller
    #rospy.loginfo("Raising torso...")
    #torso_action.move_to([0.4, ])
    #print("torso done\n")
    #head.look_at(0.9, 0.0, 0.0, "base_link")
    #print("Head done\n")

    rospy.loginfo(arm.get_ee_state())
    ready_joints = [1.3721947308776856, 
    -0.08095153547973633, 
    -1.5693933449962159, 
    1.4752402759094239, 
    1.496383236571045, 
    1.5178792071166993, 
    0.038806350997829434]
    # ready_joints = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
    #q_orig = quaternion_from_euler(0, 0, 0)
    #q_rot = quaternion_from_euler(0, pi/2, 0)
    #q_new = quaternion_multiply(q_rot, q_orig)
    

    # p = Pose(Point( 0.127514142373,0.370387555636,1.4042300346), Quaternion(0.493266059671,-0.635660595277,0.364082578561,0.469114141737))
    # p = Pose(Point( 0.759, 0.07,0.918), Quaternion(q_rot[0],q_rot[1],q_rot[2],q_rot[3]))
    # arm.move_to_pose(p)

    arm.move_to_joints_plan(ready_joints)
    # arm.replay_plan()

    # Finding and converting cartesian point
    currentPose = arm.get_ee_state()
    myPose = Pose(Point(x,y,z),currentPose.pose.orientation)
    #myPose = Pose(Point(x,y,z),Quaternion(currentPose.x,y,z,w))

    try:
        
        print("Premove\n")
        arm.move_cartesian_path(0.6,0.6,0.6)
        arm.move_to_pose(myPose)
        print("Moved\n")
        #arm.close_gripper()
        #print("Gripper\n")


    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)