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
#include <math.h>

// ros includes
#include <ros/ros.h>

// local includes
#include <dmp_motion_generation/dmp_trajectory.h>

#include <dmp_motion_generation/dmp_parameters.h>
#include <dmp_motion_generation/lwpr_parameters.h>
#include <dmp_motion_generation/dmp_constants.h>

#include <dmp_motion_generation/dmp_motion_unit.h>

using namespace dmp;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dmp_motion_generation_node");

	int num_transformation_systems = 3;
	int trajectory_dimension = num_transformation_systems * POS_VEL_ACC;

	std::string directory("/u/pastor/library");

	int num_rescale_trajectory_points = 1000;
	int sampling_frequency = 1000;

	int num_trajectory_points = 1000;

	//	int tmp_trajectory_dimension = 2;
	//	DMPTrajectory *tmp_trajectory = new DMPTrajectory(3);
	//	tmp_trajectory->readFromMRDFile(directory + std::string("d00001"));
	//
	//	int trajectory_length = tmp_trajectory->getLength();
	//	double tmp_trajectory_point[tmp_trajectory_dimension*POS_VEL_ACC];
	//
	//	DMPTrajectory *dmp_trajectory = new DMPTrajectory(trajectory_dimension);
	//	for(int i=0; i<trajectory_length; i++)
	//	{
	//		for(int j=0; j<tmp_trajectory_dimension*POS_VEL_ACC; j++)
	//		{
	//			tmp_trajectory_point[j] = 0.0;
	//		}
	//		for(int j=0; j<tmp_trajectory_dimension; j++)
	//		{
	//			tmp_trajectory_point[j*POS_VEL_ACC] = tmp_trajectory->getTrajectoryValue(i,j);
	//		}
	//		dmp_trajectory->add(tmp_trajectory_point);
	//	}

	// initialize random seed:
	srand(time(NULL));
	double noise;
	double point[trajectory_dimension];

	Trajectory *dmp_trajectory = new Trajectory();
	dmp_trajectory->initialize(std::string("3d_variable_names"), trajectory_dimension, sampling_frequency);

	for (int i = 0; i < num_trajectory_points; i++)
	{

		// set all values to zero
		for (int j = 0; j < trajectory_dimension; j++)
		{
			point[j] = 0.0;
		}

		for (int j = 0; j < num_transformation_systems; j++)
		{
			noise = static_cast<double> (rand() % 1000) / 2000.0f;
			point[(_POS_ + (j * POS_VEL_ACC)) - 1] = sin(2.0 * M_PI * (static_cast<double> (i) / static_cast<double> (num_trajectory_points))) + noise;
			//			double mean = 0;
			//			double sigma = 1.5;
			//			double scale = 10;
			//			double x = scale*(static_cast<double> (i) / static_cast<double> (num_trajectory_points)) - (scale/2.0);
			//			point[(_POS_ + (j * POS_VEL_ACC)) - 1] = exp( -pow(x-mean,2) / sigma ) + noise;
		}
		dmp_trajectory->add(point);
	}
	// dmp_trajectory->writeToMRDFile(directory + std::string("d00001"));

	// dmp_trajectory->setSamplingFrequency(tmp_trajectory->getSamplingFrequency());
	// dmp_trajectory->setSamplingFrequency(sampling_frequency);

	dmp_trajectory->rescalePositionAndComputeDerivativesAndCutout(num_rescale_trajectory_points, 100);
	// dmp_trajectory->computeDerivatives();
	// dmp_trajectory->writeToMRDFile(directory + std::string("d00002"));

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
	double trajectory_point[trajectory_dimension];

	for (int i = 0; i < 10; i++)
	{

		// create dmp unit
		dmp_unit = new DynamicMovementPrimitive();
		dmp_unit->readFromDisc(directory, dmp_id);

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
		dmp_unit->readFromDisc(directory, dmp_id);

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

		// change the goal and setup the dmp
		for (int j = 0; j < num_transformation_systems; j++)
		{
			goal[j] += i * 0.1;
		}
		dmp_unit->setup(start, goal);

		// create a new trajectory to store the result of the dmp propagation
		reproduced_trajectory = new Trajectory();
		if (!reproduced_trajectory->initialize(std::string("3d_variable_names"), trajectory_dimension, sampling_frequency))
		{
			ROS_ERROR("cannot initialize trajectory");
			return -1;
		}

		// propagate the dmp and add obtained trajectory points to the trajectory
		while (dmp_unit->propagateStep(trajectory_point))
		{
			reproduced_trajectory->add(trajectory_point);
		}
		//#if (DEBUG_MODE==1)
		//		dmp_unit->writeDebugTrajectory();
		//#endif

		// write trajectory to file
		if (!reproduced_trajectory->writeToCLMCFile(std::string(directory + "d00003")))
		{
			ROS_ERROR("could not reproduction trajectory to file");
			return -1;
		}
		delete reproduced_trajectory;

		if (!dmp_unit->writeToDisc())
		{
			ROS_ERROR("could not write dmp to disc");
			return -1;
		}
		delete dmp_unit;

	}

	//	// generate a faster movement
	//	DMPTrajectory *faster_trajectory = new DMPTrajectory(trajectory_dimension);
	//	faster_trajectory->setSamplingFrequency(sampling_frequency);
	//	dmp_unit->setup(start, goal);
	//	while (dmp_unit->run(trajectory_point, 2.0))
	//	{
	//		faster_trajectory->add(trajectory_point);
	//	}
	//#if (DEBUG_MODE==1)
	//	dmp_unit->writeDebugTrajectory();
	//#endif
	//	faster_trajectory->writeToMRDFile(directory + std::string("d00005"));
	//
	//	// generate a slower movement
	//	DMPTrajectory *slower_trajectory = new DMPTrajectory(trajectory_dimension);
	//	slower_trajectory->setSamplingFrequency(sampling_frequency);
	//	dmp_unit->setup(start, goal);
	//	while (dmp_unit->run(trajectory_point, 0.5))
	//	{
	//		slower_trajectory->add(trajectory_point);
	//	}
	//#if (DEBUG_MODE==1)
	//	dmp_unit->writeDebugTrajectory();
	//#endif
	//	slower_trajectory->writeToMRDFile(directory + std::string("d00006"));

	// delete dmp_unit;
	// delete dmp_trajectory;

	return 0;

}
