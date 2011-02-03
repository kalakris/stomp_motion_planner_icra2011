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
#include <ros/ros.h>
#include <manipulation_msgs/JointTraj.h>

// #include <experimental_controllers/TrajectoryStart.h>

// local includes
#include <dmp_motion_learner/dmp_trajectory_executer.h>

namespace dmp {

DMPTrajectoryExecuter::DMPTrajectoryExecuter()
{

}

DMPTrajectoryExecuter::~DMPTrajectoryExecuter()
{

}

bool DMPTrajectoryExecuter::initialize()
{
	if(!initialized_)
	{
		execute_mrd_trajectory_service_server_ = node_.advertiseService("execute_mrd_trajectory", &DMPTrajectoryExecuter::executeCLMCTrajectory, this);
	}

	initialized_ = true;
	return initialized_;
}

int DMPTrajectoryExecuter::run()
{
	ros::spin();
	return 0;
}

bool DMPTrajectoryExecuter::executeCLMCTrajectory(dmp_motion_learner::ExecuteCLMCTrajectory::Request &request, dmp_motion_learner::ExecuteCLMCTrajectory::Response &response)
{

	int trajectory_length;
	int trajectory_dimension;
	double sampling_frequency;
	int trajectory_type;

	Trajectory::getHeader(request.filename.c_str(), &trajectory_dimension, &trajectory_length, &sampling_frequency, &trajectory_type);

	if(trajectory_type == Trajectory::eUNINITIALIZED)
	{
		ROS_ERROR("Trajectory type is uninitialized, but should be eR_ARM_JOINT_SPACE.");
		response.success.assign(std::string("trajectory type is uninitialized, but should be eR_ARM_JOINT_SPACE."));
		return true;
	}

	Trajectory dmp_trajectory;

	dmp_trajectory.initialize(trajectory_dimension, sampling_frequency);
	if(!dmp_trajectory.readFromCLMCFile(request.filename.c_str()))
	{
		ROS_ERROR("Could not read file named %s.", request.filename.c_str());
		response.success.assign(std::string("could not read file named ") + request.filename + std::string("."));
		return true;
	}

	dmp_trajectory.writeToCLMCFile(std::string("/u/pastor/dmp/data/test.traj"));

	double dt = static_cast<double>(1.0) / sampling_frequency;

	if((trajectory_dimension % POS_VEL_ACC) != 0)
	{
		ROS_ERROR("Trajectory dimension %i is not a multiple of three (pos, vel, acc).", trajectory_dimension);
		response.success.assign(std::string("trajectory dimension is not a multiple of three."));
		return true;
	}
	int trajectory_trace = trajectory_dimension / POS_VEL_ACC;

	//	trajectory::Trajectory controller_trajectory(trajectory_trace);
	//	trajectory::Trajectory::TPoint trajectory_point;
	//	trajectory_point.setDimension(trajectory_trace);

	double timestamp = 0.0;

	ROS_INFO(">> trajectory_trace = %i",trajectory_trace);
	ROS_INFO(">> trajectory_length = %i",trajectory_length);

	// trajectory which we read from file contains a right arm joint space trajectory
	if(trajectory_type == Trajectory::eR_ARM_JOINT_SPACE)
	{
	  manipulation_msgs::JointTraj joint_trajectory_msg;
	  for(int i=0; i<trajectory_length; i++)
	    {
			manipulation_msgs::JointTrajPoint joint_trajectory_point_msg;
			joint_trajectory_point_msg.positions.resize(trajectory_trace);

			joint_trajectory_point_msg.time = timestamp;
			timestamp += dt;

			for(int j=0; j<trajectory_trace; j++)
			{
				joint_trajectory_point_msg.positions[j] = dmp_trajectory.getTrajectoryPosition(i, j);
			}

			joint_trajectory_msg.points.push_back(joint_trajectory_point_msg);
		}
		joint_trajectory_msg.names = dmp_trajectory.getPositionVariableNames();

		// debugging... TODO: remove this
  	for(std::vector<std::string>::iterator vsi = joint_trajectory_msg.names.begin(); vsi != joint_trajectory_msg.names.end(); vsi++)
  	{
  		ROS_INFO("joint_trajectory_msg.names >> %s",vsi->c_str());
  	}

	ROS_INFO(">> joint_trajectory_msg.points.size() = %i",(int)joint_trajectory_msg.points.size());
	ROS_INFO(">> joint_trajectory_msg.points[0].positions.size() = %i",(int)joint_trajectory_msg.points[0].positions.size());


		experimental_controllers::TrajectoryStart::Request send_trajectory_start_request;
  	experimental_controllers::TrajectoryStart::Response send_trajectory_start_response;
	  send_trajectory_start_request.traj = joint_trajectory_msg;
	  send_trajectory_start_request.hastiming = 1;
	  send_trajectory_start_request.requesttiming = 0;

	  bool call_successful = false;
	  call_successful = ros::service::call("/r_arm_joint_waypoint_controller/TrajectoryStart", send_trajectory_start_request, send_trajectory_start_response);

	  if(!call_successful)
	  {
	  	response.success.assign(std::string("service call was not successful."));
	  	return true;
	  }

	}

	response.success.assign(std::string("trajectory successfully executed."));
	return true;
}


}
