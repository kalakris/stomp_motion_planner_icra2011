/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Peter Pastor */

// system includes

// ros includes
#include <pluginlib/class_list_macros.h>

// local includes
#include <dmp_motion_controller/joint_velocity_filtered_controller.h>

// PLUGINLIB_DECLARE_CLASS(dmp_motion_controller, JointVelocityFilteredController, dmp_controller::JointVelocityFilteredController, pr2_controller_interface::Controller)

namespace dmp_controller
{

JointVelocityFilteredController::JointVelocityFilteredController() :
    joint_state_(NULL), command_(0), robot_(NULL), last_time_(0)
{
}

JointVelocityFilteredController::~JointVelocityFilteredController()
{
}

bool JointVelocityFilteredController::init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle& node_handle)
{
    assert(robot);
    robot_ = robot;
    node_handle_ = node_handle;

    publisher_rate_ = 10;
    publisher_buffer_size_ = 1000;
    publisher_counter_ = 0;

    std::string joint_name;
    if (!node_handle_.getParam("joint", joint_name))
    {
        ROS_ERROR("Could not retrive parameter >>joint<< from param server in the namespace %s.", node_handle_.getNamespace().c_str());
        return false;
    }
    if (!(joint_state_ = robot->getJointState(joint_name)))
    {
        ROS_ERROR("Could not find joint named \"%s\"", joint_name.c_str());
        return false;
    }

    if (!pid_controller_.init(ros::NodeHandle(node_handle_, "pid")))
        return false;

    pr2_controllers_msgs::JointControllerState joint_controller_state_msg;
    controller_state_publisher_.reset(new rosrt::Publisher<pr2_controllers_msgs::JointControllerState>(node_handle_.advertise<
            pr2_controllers_msgs::JointControllerState> (std::string("joint_position_desired"), 1), 100, joint_controller_state_msg));

    velocity_.resize(1);
    if (!((filters::MultiChannelFilterBase<double>&)filter_velocity_).configure(1, node_handle.getNamespace() + std::string("/filter_velocity"), node_handle))
    {
        ROS_ERROR("Could not create velocity filter.");
        return false;
    }

    return true;
}

void JointVelocityFilteredController::setGains(const double& p, const double& i, const double& d, const double& i_max, const double& i_min)
{
    pid_controller_.setGains(p, i, d, i_max, i_min);
}

void JointVelocityFilteredController::getGains(double& p, double& i, double& d, double& i_max, double& i_min)
{
    pid_controller_.getGains(p, i, d, i_max, i_min);
}

std::string JointVelocityFilteredController::getJointName()
{
    return joint_state_->joint_->name;
}

// Set the joint velocity command
void JointVelocityFilteredController::setCommand(const double cmd)
{
    command_ = cmd;
}

// Return the current velocity command
double JointVelocityFilteredController::getCommand() const
{
    return command_;
}

void JointVelocityFilteredController::starting()
{
    command_ = 0.0;
    last_time_ = robot_->getTime();
    pid_controller_.reset();
}

void JointVelocityFilteredController::update()
{
    ros::Time time = robot_->getTime();
    dt_ = time - last_time_;
    last_time_ = time;

    // filter the joint state velocity
    velocity_[0] = joint_state_->velocity_;
    filter_velocity_.update(velocity_, velocity_);
    filtered_velocity_ = velocity_[0];

    // compute velocity error
    error_ = velocity_[0] - command_;
    joint_state_->commanded_effort_ += pid_controller_.updatePid(error_, dt_);

    publish();
}

void JointVelocityFilteredController::stopping()
{
}

void JointVelocityFilteredController::publish()
{
    publisher_counter_++;
    if (publisher_counter_ % publisher_rate_ == 0)
    {
        pr2_controllers_msgs::JointControllerStatePtr joint_controller_state_msg = controller_state_publisher_->allocate();
        if(joint_controller_state_msg)
        {
            joint_controller_state_msg->set_point = command_;
            joint_controller_state_msg->process_value = velocity_[0];
            joint_controller_state_msg->error = error_;
            joint_controller_state_msg->time_step = dt_.toSec();
            double dummy;
            getGains(joint_controller_state_msg->p, joint_controller_state_msg->i, joint_controller_state_msg->d, joint_controller_state_msg->i_clamp, dummy);
            controller_state_publisher_->publish(joint_controller_state_msg);
        }
    }
}

} // namespace
