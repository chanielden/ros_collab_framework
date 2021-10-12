/*
 * Copyright (c) 2015, Fetch Robotics Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fetch Robotics Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
// Author: Michael Ferguson, Hanjun Song
/*
 * This is still a work in progress
 * In the future, each teleop component would probably be a plugin
 */
#include <algorithm>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <topic_tools/MuxSelect.h>
#include <cmath>
#include <collab_decision/TeleopMode.h>
double integrate(double desired, double present, double max_rate, double dt)
{
  if (desired > present)
    return std::min(desired, present + max_rate * dt);
  else
    return std::max(desired, present - max_rate * dt);
}

class TeleopComponent
{
public:
  TeleopComponent() : active_(false)
  {

  }

  virtual ~TeleopComponent() {}

  // This gets called whenever a new direction message comes in
  // Returns whether lower priority teleop components should be stopped
  virtual bool update(collab_decision::TeleopMode tm, 
                      const sensor_msgs::JointState::ConstPtr& state) = 0;

  // This gets called at set frequency
  virtual void publish(const ros::Duration& dt) = 0;

  // Start the component. Must be idempotent.
  virtual bool start()
  {
    active_ = true;
    return active_;
  }

  // Stop the component. Must be idempotent.
  virtual bool stop()
  {
    active_ = false;
    return active_;
  }

protected:
  bool active_;
};

class FollowTeleop : public TeleopComponent
{
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client_t;

public:
  FollowTeleop(const std::string& name, ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh(nh, name);


    // Joint Limits
    pnh.param("min_position", min_position_, 0.0);
    pnh.param("max_position", max_position_, 0.4);
    pnh.param("max_velocity", max_velocity_, 0.075);
    pnh.param("max_accel", max_acceleration_, 0.25);

    // Should we inhibit lower priority components if running?
    pnh.param("inhibit", inhibit_, false);

    // Load topic/joint info
    pnh.param<std::string>("joint_name", joint_name_, "torso_lift_joint");
    std::string action_name;
    pnh.param<std::string>("action_name", action_name, "torso_controller/follow_joint_trajectory");

    client_.reset(new client_t(action_name, true));
    if (!client_->waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("%s may not be connected.", action_name.c_str());
    }
  }

    virtual bool update(collab_decision::TeleopMode tm, 
                        const sensor_msgs::JointState::ConstPtr& state)
    {

      stop();
      // Update joint position
      for (size_t i = 0; i < state->name.size(); i++)
      {
        if (state->name[i] == joint_name_)
        {
          actual_position_ = state->position[i];
          break;
        }
      }
      desired_direction_ = 0;
      if (tm.mode == "back"){
        desired_direction_ = tm.dir;
        if (desired_direction_ > 0){
        ROS_INFO("Moving Back Upwards\n");
      }else{ROS_INFO("Moving Back Downwards\n");}
      }
      
      if (desired_direction_ == 1)
      {
        desired_velocity_ = max_velocity_;
      }
      else if (desired_direction_ == -1)
      {
        desired_velocity_ = -max_velocity_;
      }
      else
      {
        desired_velocity_ = 0.0;
      }

      return inhibit_;
    }

    virtual void publish(const ros::Duration&dt)
    {


      // Fill in a message (future dated at fixed time step)
      double step = 0.25;
      double vel = integrate(desired_velocity_, last_velocity_, max_acceleration_, step);
      double travel = step * (vel + last_velocity_) / 2.0;
      double pos = std::max(min_position_, std::min(max_position_, actual_position_ + travel));
      // Send message
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory.joint_names.push_back(joint_name_);
      trajectory_msgs::JointTrajectoryPoint p;
      p.positions.push_back(pos);
      p.velocities.push_back(vel);
      p.time_from_start = ros::Duration(step);
      goal.trajectory.points.push_back(p);
      goal.goal_time_tolerance = ros::Duration(0.0);
      client_->sendGoal(goal);
      // Update based on actual timestep
      vel = integrate(desired_velocity_, last_velocity_, max_acceleration_, dt.toSec());
      travel = dt.toSec() * (vel + last_velocity_) / 2.0;
      actual_position_ = std::max(min_position_, std::min(max_position_, actual_position_ + travel));
      last_velocity_ = vel;
      
    }

    virtual bool stop()
    {
      active_ = false;
      last_velocity_ = 0.0;
      return active_;
    }

private:
  double min_position_, max_position_, max_velocity_, max_acceleration_;
  bool inhibit_;
  std::string joint_name_;
  double actual_position_;
  double desired_velocity_, desired_direction_, last_velocity_;
  boost::shared_ptr<client_t> client_;
};

class ArmTeleop : public TeleopComponent
{
public:
  ArmTeleop(const std::string& name, ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh(nh, name);

    // Twist limits
    pnh.param("max_vel_x", max_vel_x_, 1.0);
    pnh.param("max_vel_y", max_vel_y_, 1.0);
    pnh.param("max_vel_z", max_vel_z_, 1.0);
    pnh.param("max_acc_x", max_acc_x_, 10.0);
    pnh.param("max_acc_y", max_acc_y_, 10.0);
    pnh.param("max_acc_z", max_acc_z_, 10.0);

    pnh.param("max_vel_roll", max_vel_roll_, 2.0);
    pnh.param("max_vel_pitch", max_vel_pitch_, 2.0);
    pnh.param("max_vel_yaw", max_vel_yaw_, 2.0);
    pnh.param("max_acc_roll", max_acc_roll_, 10.0);
    pnh.param("max_acc_pitch", max_acc_pitch_, 10.0);
    pnh.param("max_acc_yaw", max_acc_yaw_, 10.0);

    cmd_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/arm_controller/cartesian_twist/command", 10);
  }

  virtual bool update(collab_decision::TeleopMode tm,
                      const sensor_msgs::JointState::ConstPtr& state)
  {
    start();


    int desired_direction_ = 0;
    if (tm.mode == "arm_upper"){
      desired_direction_ = tm.dir;
      desired_.twist.linear.x = 0.1*desired_direction_* max_vel_x_; // Forward back
      desired_.twist.linear.y = 0;
      // Messaging
      if (desired_direction_ > 0){
        ROS_INFO("Moving Arm Forwards\n");
      }else if(desired_direction_ == 0){
        ROS_INFO("Arm Stopped");
        desired_.twist.linear.x = 0;
        desired_.twist.linear.y = 0;
        desired_.twist.linear.z = 0;
      }else{
        ROS_INFO("Moving Arm Backwards\n");
      }

    }else if (tm.mode == "arm_lower"){
      desired_direction_ = tm.dir;
      desired_.twist.linear.z = 0.1*desired_direction_* max_vel_z_; // up down
      desired_.twist.linear.y = 0;
      // Messaging
      if (desired_direction_ > 0){
        ROS_INFO("Moving Arm Up\n");
      }else if(desired_direction_ == 0){
        ROS_INFO("Arm Velocity Stopped");
        desired_.twist.linear.x = 0;
        desired_.twist.linear.y = 0;
        desired_.twist.linear.z = 0;
      }else{
        ROS_INFO("Moving Arm Down\n");
      }
    }
    // Linear mode
    
    desired_.twist.linear.y = 0*desired_direction_* max_vel_y_; // side to side
    
    desired_.twist.angular.x = 0.0;
    desired_.twist.angular.y = 0.0;
    desired_.twist.angular.z = 0.0;
    last_update_ = ros::Time::now();
    //ROS_INFO("desired.x = %f", desired_.twist.linear.z);
    return true;
  }

  virtual void publish(const ros::Duration& dt)
  {
    
      // Ramp commands based on acceleration limits
      last_.twist.linear.x = integrate(desired_.twist.linear.x, last_.twist.linear.x, max_acc_x_, dt.toSec());
      last_.twist.linear.y = integrate(desired_.twist.linear.y, last_.twist.linear.y, max_acc_y_, dt.toSec());
      last_.twist.linear.z = integrate(desired_.twist.linear.z, last_.twist.linear.z, max_acc_z_, dt.toSec());

      //last_.twist.angular.x = integrate(desired_.twist.angular.x, last_.twist.angular.x, max_acc_roll_, dt.toSec());
      //last_.twist.angular.y = integrate(desired_.twist.angular.y, last_.twist.angular.y, max_acc_pitch_, dt.toSec());
      //last_.twist.angular.z = integrate(desired_.twist.angular.z, last_.twist.angular.z, max_acc_yaw_, dt.toSec());
      
      last_.header.frame_id = "base_link";
      cmd_pub_.publish(last_);
    
  }

  virtual bool stop()
  {
    // Publish stop message
    if (active_)
    {
      last_ = desired_ = geometry_msgs::TwistStamped();
      cmd_pub_.publish(last_);
    }

    active_ = false;
    return active_;
  }

private:

  // Limits from params
  double max_vel_x_, max_vel_y_, max_vel_z_;
  double max_vel_roll_, max_vel_pitch_, max_vel_yaw_;
  double max_acc_x_, max_acc_y_, max_acc_z_;
  double max_acc_roll_, max_acc_pitch_, max_acc_yaw_;

  // Twist output
  ros::Publisher cmd_pub_;

  geometry_msgs::TwistStamped desired_;
  geometry_msgs::TwistStamped last_;
  ros::Time last_update_;
};

class Teleop
{
  typedef boost::shared_ptr<TeleopComponent> TeleopComponentPtr;

public:
  void init(ros::NodeHandle& nh)
  {
    bool is_fetch;
    bool use_torso;
    bool use_arm;

    nh.param("is_fetch", is_fetch, true);
    nh.param("use_torso", use_torso, true);
    nh.param("use_arm", use_arm, true);

    TeleopComponentPtr c;
    if (is_fetch)
    {
      if (use_torso)
      {
        // Torso does not override
        c.reset(new FollowTeleop("torso", nh));
        components_.push_back(c);
      }

      if(use_arm)
      {
        c.reset(new ArmTeleop("arm", nh));
        components_.push_back(c);
      }
    }

    state_msg_.reset(new sensor_msgs::JointState());

    dir_sub_ = nh.subscribe("/movedir", 10, &Teleop::dirCallback, this);

    state_sub_ = nh.subscribe("/joint_states", 10, &Teleop::stateCallback ,this);


    mode_pub_ = nh.advertise<std_msgs::UInt8>("teleop_mode", 10);

  }

  void publish(const ros::Duration& dt)
  {
    if (ros::Time::now() - last_update_ > ros::Duration(0.25))
    {
      // Timed out
      for (size_t c = 0; c < components_.size(); c++)
      {
        components_[c]->stop();
      }
    }
    else
    {
      for (size_t c = 0; c < components_.size(); c++)
      {
        components_[c]->publish(dt);
      }
    }
  }

private:
  void dirCallback(collab_decision::TeleopMode tm)
  {
    bool ok = true;
    for (size_t c = 0; c < components_.size(); c++)
    {
      if (ok)
      {
        ok &= !components_[c]->update(tm, state_msg_);
      }
      else
      {
        // supressed by a higher priority component
        components_[c]->stop();
      }
    }
    last_update_ = ros::Time::now();
  }

  void stateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    // Lock mutex on state message
    boost::mutex::scoped_lock lock(state_mutex_);

    // Update each joint based on message
    for (size_t msg_j = 0; msg_j < msg->name.size(); msg_j++)
    {
      size_t state_j;
      for (state_j = 0; state_j < state_msg_->name.size(); state_j++)
      {
        if (state_msg_->name[state_j] == msg->name[msg_j])
        {
          state_msg_->position[state_j] = msg->position[msg_j];
          state_msg_->velocity[state_j] = msg->velocity[msg_j];
          break;
        }
      }
      if (state_j == state_msg_->name.size())
      {
        // New joint
        state_msg_->name.push_back(msg->name[msg_j]);
        state_msg_->position.push_back(msg->position[msg_j]);
        state_msg_->velocity.push_back(msg->velocity[msg_j]);
      }
    }
  }

  std::vector<TeleopComponentPtr> components_;
  ros::Time last_update_;
  boost::mutex state_mutex_;
  sensor_msgs::JointStatePtr state_msg_;
  ros::Subscriber dir_sub_, state_sub_;
  ros::Publisher mode_pub_;
};

int main(int argc, char** argv)
{
  system("rosnode kill teleop");

  ros::init(argc, argv, "teleop2");
  ros::NodeHandle n("~");
  
  Teleop teleop;
  teleop.init(n);

  ros::Rate r(30.0);
  while (ros::ok())
  {
    ros::spinOnce();
    teleop.publish(ros::Duration(1/30.0));


    r.sleep();
  }
  return 0;
}