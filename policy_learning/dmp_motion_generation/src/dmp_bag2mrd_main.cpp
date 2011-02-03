/*
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
 */

/** \author Peter Pastor */

// system includes
#include <string>

// ros includes
#include <ros/ros.h>
#include <rosrecord/Player.h>
#include <sensor_msgs/JointState.h>

// local includes
#include <dmp_motion_generation/dmp_motion_unit.h>
#include <dmp_motion_generation/dmp_library.h>

// r_gripper_float_joint
// r_gripper_joint
// r_gripper_l_finger_joint
// r_gripper_l_finger_tip_joint
// r_gripper_r_finger_joint
// r_gripper_r_finger_tip_joint

// r_shoulder_pan_joint
// r_shoulder_lift_joint
// r_upper_arm_roll_joint
// r_elbow_flex_joint
// r_forearm_roll_joint
// r_wrist_flex_joint
// r_wrist_roll_joint

#define SEC2NANO 1000000000

#define NUM_JOINTS 7

using namespace dmp;

void bag2mrd_callback(std::string name, sensor_msgs::JointState* joint_state, ros::Time t, ros::Time t_no_use, void* trajectory)
{

    // FILE* file = (FILE*) f;
	Trajectory *dmp_trajectory = (Trajectory*) trajectory;

	int num_joints_found = 0;

	std::vector<double> joint_positions;
	joint_positions.resize(NUM_JOINTS);

	std::vector<std::string> joint_names;
	joint_names.resize(NUM_JOINTS);

	joint_names[0].assign(std::string("r_shoulder_pan_joint"));
	joint_names[1].assign(std::string("r_shoulder_lift_joint"));
	joint_names[2].assign(std::string("r_upper_arm_roll_joint"));
	joint_names[3].assign(std::string("r_elbow_flex_joint"));
	joint_names[4].assign(std::string("r_forearm_roll_joint"));
	joint_names[5].assign(std::string("r_wrist_flex_joint"));
	joint_names[6].assign(std::string("r_wrist_roll_joint"));

	int index = 0;
	for (std::vector<std::string>::iterator vsi = joint_state->name.begin(); vsi != joint_state->name.end(); vsi++)
	{
		for (int i = 0; i < NUM_JOINTS; i++)
		{
			if (vsi->compare(joint_names[i]) == 0)
			{
				joint_positions[i] = joint_state->position[index];
				num_joints_found++;
			}
		}
		index++;
	}

	if (NUM_JOINTS != num_joints_found)
	{
		ROS_ERROR("bag2mrd_callback>> could not find all joints. Found only %i out of %i.",num_joints_found, NUM_JOINTS);
	}
	else
	{
		double trajectory_point[NUM_JOINTS * POS_VEL_ACC];
		for (int i = 0; i < NUM_JOINTS; i++)
		{
			trajectory_point[i * POS_VEL_ACC] = joint_positions[i];
		}
		dmp_trajectory->add(trajectory_point);
		//		unsigned long long nanosec = t.nsec;
		//		unsigned long long sec = t.sec;
		//		fprintf(file, "%llu ",sec*SEC2NANO + nanosec);
		//		for(int i=0; i<NUM_JOINTS; i++)
		//		{
		//			fprintf(file, "%f ",joint_positions[i]);
		//		}
		//		fprintf(file, "\n");
	}

}

int main(int argc, char **argv)
{

//	if (argc != 2)
//	{
//		ROS_INFO("usage: dmp_bag2mrd <file.bag>");
//		return 1;
//	}

	ros::init(argc, argv, "dmp_bag2mrd");

	ros::record::Player player;

	std::string library_directory("/u/pastor/dmp/library");
	std::string data_directory("/u/pastor/dmp/data/");

//	player.open(std::string(argv[1]), ros::Time());
	player.open(std::string(data_directory + "sim_robot.bag"), ros::Time());

	// FILE* file = fopen("/u/pastor/data/joint_states.txt", "w");

	double sampling_frequency = 92.1246;
	int trajectory_dimension = NUM_JOINTS * POS_VEL_ACC;
	Trajectory *dmp_trajectory = new Trajectory();
	if (!dmp_trajectory->initialize(std::string("pr2_r_arm_joint_names"), trajectory_dimension, sampling_frequency))
	{
		ROS_ERROR("could not initialize dmp_trajectory");
		return -1;
	}

	// player.addHandler<pr2_mechanism_msgs::JointState> (std::string(argv[2]), &bag2mrd_callback, file);
	player.addHandler<sensor_msgs::JointState> (std::string("*"), &bag2mrd_callback, dmp_trajectory);

	while (player.nextMsg())
	{
	}

	if(!dmp_trajectory->writeToCLMCFile(std::string(data_directory + "d00001")))
	{
		ROS_ERROR("could not save trajectory to file");
		return -1;
	}
	// fclose(file);

	int num_transformation_systems = NUM_JOINTS;
	// int num_rescale_trajectory_points = 1000;
	sampling_frequency = 1000.0;
	// double new_movement_duration = 2.0;
	// double new_sampling_frequency = 1000.0;
	double trajectory_point[trajectory_dimension];

	// dmp_trajectory->rescalePositionAndComputeDerivativesAndCutout(num_rescale_trajectory_points, 10);
	//	if (!dmp_trajectory->writeToMRDFile(std::string(data_directory + "d00002")))
	//	{
	//		ROS_ERROR("could not save trajectory to file");
	//		return -1;
	//	}

	DynamicMovementPrimitive *dmp_unit = new DynamicMovementPrimitive();
	int dmp_id = 1;
	if (!dmp_unit->initialize(num_transformation_systems, dmp_id))
	{
		ROS_ERROR("cannot initialize dmp unit");
		return -1;
	}
	if (!dmp_unit->writeToDisc())
	{
		ROS_ERROR("could not write dmp to disc");
		return -1;
	}
	delete dmp_unit;

	double start[num_transformation_systems];
	double goal[num_transformation_systems];
	Trajectory *reproduced_trajectory;

	// create dmp unit
	dmp_unit = new DynamicMovementPrimitive();
	dmp_unit->readFromDisc(library_directory, dmp_id);

	ROS_INFO("start learning dmp from trajectory...");
	if (!dmp_unit->learnFromTrajectory(dmp_trajectory))
	{
		ROS_ERROR("learning dmp from trajectory was not successful");
		return -1;
	}

	// write dmp to file
	if (!dmp_unit->writeToDisc())
	{
		ROS_ERROR("could not write dmp to disc");
		return -1;
	}
	delete dmp_unit;

	// create new dmp unit and initialize it from the dmp unit which was just stored to disc.
	dmp_unit = new DynamicMovementPrimitive();
	dmp_unit->readFromDisc(library_directory, dmp_id);

	// obtain start and goal position from the demonstrated trajectory
	if (!dmp_trajectory->getStartPosition(start))
	{
		ROS_ERROR("could not get start of the trajectory");
		return -1;
	}
	if (!dmp_trajectory->getEndPosition(goal))
	{
		ROS_ERROR("could not get end of the trajectory");
		return -1;
	}

//	// change the goal and setup the dmp
//	for (int j = 0; j < num_transformation_systems; j++)
//	{
//		goal[j] += i * 0.1;
//	}
	dmp_unit->setup(start, goal);
	// dmp_unit->setup(start, goal, new_movement_duration, new_sampling_frequency);

	// create a new trajectory to store the result of the dmp propagation
	reproduced_trajectory = new Trajectory();
	if (!reproduced_trajectory->initialize(std::string("pr2_r_arm_joint_names"), trajectory_dimension, sampling_frequency))
	{
		ROS_ERROR("cannot initialize trajectory");
		return -1;
	}

	// propagate the dmp and add obtained trajectory points to the trajectory
	while (dmp_unit->propagateStep(trajectory_point))
	{
		reproduced_trajectory->add(trajectory_point);
	}
#if (DEBUG_MODE==1)
	dmp_unit->writeDebugTrajectory();
#endif

	// write trajectory to file
	if (!reproduced_trajectory->writeToCLMCFile(std::string(data_directory + "d00003")))
	{
		ROS_ERROR("could not write reproduction trajectory to file");
		return -1;
	}
	delete reproduced_trajectory;

	if (!dmp_unit->writeToDisc())
	{
		ROS_ERROR("could not write dmp to disc");
		return -1;
	}
	delete dmp_unit;

	return 0;
}

