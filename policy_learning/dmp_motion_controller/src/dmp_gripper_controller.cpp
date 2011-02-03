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
#include <sstream>

// ros includes

// local includes
#include <dmp_motion_controller/dmp_gripper_controller.h>

namespace dmp_controller
{

DMPGripperController::DMPGripperController() :
    initialized_(false), command_published_(0)
{

}

DMPGripperController::~DMPGripperController()
{

}

bool DMPGripperController::initialize(ros::NodeHandle& node_handle)
{
    command_published_ = 0;

    // r_gripper_effort_publisher_ = new realtime_tools::RealtimePublisher<std_msgs::Float64>(node_handle, std::string("/r_gripper_effort_controller/command"), 1);

    //	double param_value;
    //	std::string param_string;
    //
    //	ros::NodeHandle gripper_control_handle(std::string("/dmp_motion_controller/gripper_control"));
    //
    //	param_string.assign("open_effort");
    //	if (!gripper_control_handle.getParam(param_string, param_value))
    //	{
    //		ROS_ERROR("DMPGripperController::initialize>> could not get %s from param server.", param_string.c_str());
    //		initialized_ = false;
    //		return initialized_;
    //	}
    //	gripper_open_msg_.data = param_value;
    //	param_string.assign("close_effort");
    //	if (!gripper_control_handle.getParam(param_string, param_value))
    //	{
    //		ROS_ERROR("DMPGripperController::initialize>> could not get %s from param server.", param_string.c_str());
    //		initialized_ = false;
    //		return initialized_;
    //	}
    //	gripper_close_msg_.data = param_value;
    //
    //	ros::NodeHandle controller_node_handle(std::string("/dmp_motion_controller"));
    //	std::string movement_names;
    //	if(!controller_node_handle.getParam(std::string("movement_names"), movement_names))
    //	{
    //		ROS_ERROR("DMPGripperController::initialize>> could not get movement names from param server.");
    //		initialized_ = false;
    //		return initialized_;
    //	}
    //
    //	// split the group names based on whitespace
    //	std::stringstream ss_movement_names(movement_names);
    //
    //	std::string movement_name;
    //	int movement_type_index = dmp::Parameters::eGRASP;
    //	while (ss_movement_names >> movement_name)
    //	{
    //		ros::NodeHandle movement_type_handle(gripper_control_handle, movement_name);
    //		param_string.assign("open");
    //		if (!movement_type_handle.getParam(param_string, param_value))
    //		{
    //			ROS_ERROR("DMPGripperController::initialize>> could not get %s of %s from param server.", param_string.c_str(), movement_name.c_str());
    //			initialized_ = false;
    //			return initialized_;
    //		}
    //		gripper_open_thresholds_[movement_type_index] = param_value;
    //		param_string.assign("close");
    //		if (!movement_type_handle.getParam(param_string, param_value))
    //		{
    //			ROS_ERROR("DMPGripperController::initialize>> could not get %s of %s from param server.", param_string.c_str(), movement_name.c_str());
    //			initialized_ = false;
    //			return initialized_;
    //		}
    //		gripper_close_thresholds_[movement_type_index] = param_value;
    //		movement_type_index++;
    //	}

    initialized_ = true;
    return initialized_;
}

// REAL-TIME REQUIREMENT
void DMPGripperController::command(/*dmp::Parameters::eTaskType dmp_task_type,*/double progress)
{
    //	int task_type = static_cast<int>(dmp_task_type);
    //
    //	if (progress > gripper_open_thresholds_[task_type] && command_published_ == 0)
    //	{
    //		r_gripper_effort_publisher_->msg_ = gripper_open_msg_;
    //		r_gripper_effort_publisher_->unlockAndPublish();
    //		command_published_ = 1;
    //	}
    //	else if (progress > gripper_close_thresholds_[task_type] && command_published_ == 1)
    //	{
    //		r_gripper_effort_publisher_->msg_ = gripper_close_msg_;
    //		r_gripper_effort_publisher_->unlockAndPublish();
    //		command_published_ = 2;
    //	}
}

void DMPGripperController::reset()
{
    command_published_ = 0;
}

}
