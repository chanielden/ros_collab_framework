#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import argparse
import os
import subprocess
import sys
from threading import Thread

import rospkg
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from hrca_msgs.msg import RobotTaskAction, RobotTaskFeedback, RobotTaskResult, RobotTaskGoal
import actionlib

def is_moveit_running():
    try:
        output = subprocess.check_output(["rosnode", "info", "move_group"], stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as e:
        output = e.output
    if output.find("unknown node") >= 0:
        return False
    if output.find("Communication with node") >= 0:
        return False
    return True

class ReadyArmThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.client = None
        self.start()

    def run(self):
        move_thread = None
        if not is_moveit_running():
            rospy.loginfo("starting moveit")
            move_thread = MoveItThread()
        else:
            rospy.loginfo("moveit already started")

        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")
        keepout_pose = Pose()
        keepout_pose.position.z = 0.375
        keepout_pose.orientation.w = 1.0
        ground_pose = Pose()
        ground_pose.position.z = -0.03
        ground_pose.orientation.w = 1.0
        rospack = rospkg.RosPack()
        mesh_dir = os.path.join(rospack.get_path('fetch_teleop'), 'mesh')
        scene.addMesh(
            'keepout', keepout_pose, os.path.join(mesh_dir, 'keepout.stl'))
        scene.addMesh(
            'ground', ground_pose, os.path.join(mesh_dir, 'ground.stl'))

        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.33793866634368896, 0.9327091809509277, -0.27269920088500976, -1.661432214854004, 1.0449586368103028, 1.3537226187426759, 1.7119276118103028, 0.001991184524440765]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     0.0,
                                                     max_velocity_scaling_factor=0.5)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                scene.removeCollisionObject("ground")
                if move_thread:
                    move_thread.stop()

                # On success quit
                # Stopping the MoveIt thread works, however, the action client
                # does not get shut down, and future tucks will not work.
                # As a work-around, we die and roslaunch will respawn us.
                rospy.signal_shutdown("done")
                sys.exit(0)
                return

    def stop(self):
        if self.client:
            self.client.get_move_action().cancel_goal()
        # Stopping the MoveIt thread works, however, the action client
        # does not get shut down, and future tucks will not work.
        # As a work-around, we die and roslaunch will respawn us.
        rospy.signal_shutdown("failed")
        sys.exit(0)

class ReadyArmTeleop:

    def __init__(self):
        self.ready_button = rospy.get_param("~ready_button", 5)  # default button is the right button
        self.deadman = rospy.get_param("~deadman_button", 10)
        self.executing = False

        self.pressed = False

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    
    def joy_callback(self, msg):
        if self.executing:
            # Only run once
            if msg.buttons[self.deadman] <= 0:
                # Deadman has been released, don't tuck
                rospy.loginfo("Stopping Ready Arm thread")
                self.ready_arm_thread.stop()
            return
        try:
            if msg.buttons[self.ready_button] > 0 and msg.buttons[self.deadman] > 0:
                if not self.pressed:
                    self.pressed_last = rospy.Time.now()
                    self.pressed = True
                elif self.pressed_last and rospy.Time.now() > self.pressed_last + rospy.Duration(1.0):
                    # Ready Arm the arm
                    self.executing = True
                    rospy.loginfo("Starting Ready Arm thread")
                    self.ready_arm_thread = ReadyArmThread()
            else:
                self.pressed = False
        except KeyError:
            rospy.logwarn("ready_button is out of range")

if __name__ == "__main__":
    rospy.init_node("ready_arm")
    rospy.loginfo("New ready arm script running")

    t = ReadyArmTeleop()
    rospy.spin()
