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
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <topic_tools/MuxSelect.h>

double integrate(double desired, double present, double max_rate, double dt)
{
  if (desired > present)
    return std::min(desired, present + max_rate * dt);
  else
    return std::max(desired, present - max_rate * dt);
}

enum mode {mode_nav, mode_linear, mode_angular};
std_msgs::UInt8 mode;

class TeleopComponent
{
public:
  TeleopComponent() : active_(false)
  {

  }

  virtual ~TeleopComponent() {}

  // This gets called whenever new joy message comes in
  // returns whether lower priority teleop components should be stopped
  virtual bool update(const sensor_msgs::Joy::ConstPtr& joy,
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

// Base Teleop

// This controls a single joint through a follow controller (for instance, torso)
class FollowTeleop : public TeleopComponent
{
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> client_t;

public:
  FollowTeleop(const std::string& name, ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh(nh, name);

    // Button mapping
    pnh.param("button_deadman", deadman_, 10);
    pnh.param("button_increase", inc_button_, 12);
    pnh.param("button_decrease", dec_button_, 14);

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

  // This gets called whenever new joy message comes in
  virtual bool update(const sensor_msgs::Joy::ConstPtr& joy,
                      const sensor_msgs::JointState::ConstPtr& state)
  {
    bool deadman_pressed = joy->buttons[deadman_];

    if (!deadman_pressed)
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
      return false;
    }

    if (joy->buttons[inc_button_])
    {
      desired_velocity_ = max_velocity_;
      start();
    }
    else if (joy->buttons[dec_button_])
    {
      desired_velocity_ = -max_velocity_;
      start();
    }
    else
    {
      desired_velocity_ = 0.0;
    }

    return inhibit_;
  }

  // This gets called at set frequency
  virtual void publish(const ros::Duration& dt)
  {
    if (active_)
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
  }

  virtual bool stop()
  {
    active_ = false;
    last_velocity_ = 0.0;
    return active_;
  }

private:
  int deadman_, inc_button_, dec_button_;
  double min_position_, max_position_, max_velocity_, max_acceleration_;
  bool inhibit_;
  std::string joint_name_;
  double actual_position_;
  double desired_velocity_, last_velocity_;
  boost::shared_ptr<client_t> client_;
};

// Gripper Teleop

// Head Teleop

class ArmTeleop : public TeleopComponent
{
public:
  ArmTeleop(const std::string& name, ros::NodeHandle& nh)
  {
    ros::NodeHandle pnh(nh, name);

    pnh.param("axis_x", axis_x_, 3);
    pnh.param("axis_y", axis_y_, 2);
    pnh.param("axis_z", axis_z_, 1);
    pnh.param("axis_roll", axis_roll_, 2);
    pnh.param("axis_pitch", axis_pitch_, 3);
    pnh.param("axis_yaw", axis_yaw_, 0);

    pnh.param("button_deadman", deadman_, 10);
    pnh.param("button_arm_linear", button_linear_, 9);
    pnh.param("button_arm_angular", button_angular_, 11);

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

  virtual bool update(const sensor_msgs::Joy::ConstPtr& joy,
                      const sensor_msgs::JointState::ConstPtr& state)
  {
    bool deadman_pressed = joy->buttons[deadman_];
    // bool button_linear_pressed = joy->buttons[button_linear_];
    // bool button_angular_pressed = joy->buttons[button_angular_];
    // bool button_linear_pressed = true;
    // bool button_angular_pressed = false;
    

    if ( (mode.data == mode_nav || !deadman_pressed) && (ros::Time::now() - last_update_ > ros::Duration(0.5)))
    {
      stop();
      return false;
    }

    start();

    if (deadman_pressed && mode.data == mode_linear)
    {
      desired_.twist.linear.x = joy->axes[axis_x_] * max_vel_x_;
      desired_.twist.linear.y = joy->axes[axis_y_] * max_vel_y_;
      desired_.twist.linear.z = joy->axes[axis_z_] * max_vel_z_;
      desired_.twist.angular.x = 0.0;
      desired_.twist.angular.y = 0.0;
      desired_.twist.angular.z = 0.0;
      last_update_ = ros::Time::now();
    }
    else if (deadman_pressed && mode.data == mode_angular)
    {
      desired_.twist.linear.x = 0.0;
      desired_.twist.linear.y = 0.0;
      desired_.twist.linear.z = 0.0;
      desired_.twist.angular.x = joy->axes[axis_roll_] * max_vel_roll_;
      desired_.twist.angular.y = joy->axes[axis_pitch_] * max_vel_pitch_;
      desired_.twist.angular.z = joy->axes[axis_yaw_] * max_vel_yaw_;
      last_update_ = ros::Time::now();
    }
    else
    {
      desired_.twist.linear.x = 0.0;
      desired_.twist.linear.y = 0.0;
      desired_.twist.linear.z = 0.0;
      desired_.twist.angular.x = 0.0;
      desired_.twist.angular.y = 0.0;
      desired_.twist.angular.z = 0.0;
      
    }

    return true;
  }

  virtual void publish(const ros::Duration& dt)
  {
    if (active_)
    {
      // Ramp commands based on acceleration limits
      last_.twist.linear.x = integrate(desired_.twist.linear.x, last_.twist.linear.x, max_acc_x_, dt.toSec());
      last_.twist.linear.y = integrate(desired_.twist.linear.y, last_.twist.linear.y, max_acc_y_, dt.toSec());
      last_.twist.linear.z = integrate(desired_.twist.linear.z, last_.twist.linear.z, max_acc_z_, dt.toSec());

      last_.twist.angular.x = integrate(desired_.twist.angular.x, last_.twist.angular.x, max_acc_roll_, dt.toSec());
      last_.twist.angular.y = integrate(desired_.twist.angular.y, last_.twist.angular.y, max_acc_pitch_, dt.toSec());
      last_.twist.angular.z = integrate(desired_.twist.angular.z, last_.twist.angular.z, max_acc_yaw_, dt.toSec());

      last_.header.frame_id = "base_link";

      cmd_pub_.publish(last_);
    }
  }

  virtual bool start()
  {
    active_ = true;
    return active_;
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

  // Buttons from params
  int deadman_;
  int axis_x_, axis_y_, axis_z_, axis_roll_, axis_pitch_, axis_yaw_;
  int button_linear_, button_angular_;

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

// This pulls all the components together
class Teleop
{
  typedef boost::shared_ptr<TeleopComponent> TeleopComponentPtr;

public:
  void init(ros::NodeHandle& nh)
  {
    bool is_fetch;
    bool use_arm;

    nh.param("is_fetch", is_fetch, true);
    nh.param("use_arm", use_arm, true);

    nh.param("button_switch_mode", button_switch_mode_, 11);  // R1

    change_mode_pressed = false;
    // TODO: load these from YAML

    TeleopComponentPtr c;
    if (is_fetch)
    {
      if (use_torso)
      {
        // Torso does not override
        c.reset(new FollowTeleop("torso", nh));
        components_.push_back(c);
      }

      if (use_gripper)
      {
        // Gripper does not override
        c.reset(new GripperTeleop("gripper", nh));
        components_.push_back(c);
      }

      if (use_head)
      {
        // Head overrides base
        c.reset(new HeadTeleop("head", nh));
        components_.push_back(c);
      }

      if (use_arm)
      {
        c.reset(new ArmTeleop("arm", nh));
        components_.push_back(c);
      }
    }

    if (use_base)
    {
      // BaseTeleop goes last
      c.reset(new BaseTeleop("base", nh));
      components_.push_back(c);
    }

    state_msg_.reset(new sensor_msgs::JointState());
    joy_sub_ = nh.subscribe("/joy", 1, &Teleop::joyCallback, this);
    state_sub_ = nh.subscribe("/joint_states", 10, &Teleop::stateCallback, this);
    
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
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
  {
    // Lock mutex on state message
    boost::mutex::scoped_lock lock(state_mutex_);

    bool ok = true;
    for (size_t c = 0; c < components_.size(); c++)
    {
      if (ok)
      {
        ok &= !components_[c]->update(msg, state_msg_);
      }
      else
      {
        // supressed by a higher priority component
        components_[c]->stop();
      }
    }

    // Update the mode.
    bool oldVal = change_mode_pressed;
    change_mode_pressed = msg->buttons[button_switch_mode_];

    if (!oldVal && change_mode_pressed)
    {
      // change mode
      mode.data++;
      mode.data = mode.data % 3;

      mode_pub_.publish(mode);
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
  ros::Subscriber joy_sub_, state_sub_;

  ros::Publisher mode_pub_;
  int button_switch_mode_;
  bool change_mode_pressed;

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
