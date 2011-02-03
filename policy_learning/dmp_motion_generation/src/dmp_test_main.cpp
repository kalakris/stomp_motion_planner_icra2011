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
#include <dmp_motion_generation/dmp_motion_unit.h>

using namespace dmp;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dmp_motion_generation_node");

	int num_transformation_systems = 3;
	int trajectory_dimension = num_transformation_systems * POS_VEL_ACC;

	std::string library_directory("/u/pastor/dmp/library/");
	std::string data_directory("/u/pastor/dmp/debug/");

	bool doSin = false;

	int num_trajectory_points = 1000;

	double sampling_frequency = 1000.0;

	// double new_movement_duration = 1.0;
	// double new_sampling_frequency = 1000.0;

	double trajectory_point[trajectory_dimension];

	// initialize random seed:
	srand(time(NULL));
	double noise;

	Trajectory *dmp_trajectory = new Trajectory();
	if(!dmp_trajectory->initialize(std::string("3d_variable_names"), trajectory_dimension, sampling_frequency, NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES, NUM_DEBUG_CANONICAL_SYSTEM_VALUES))
	{
		ROS_ERROR("could not initialize trajectory");
		return -1;
	}

	for (int i = 0; i < num_trajectory_points; i++)
	{
		// set all values to zero
		for (int j = 0; j < trajectory_dimension; j++)
		{
			trajectory_point[j] = 0.0;
		}
		for (int j = 0; j < num_transformation_systems; j++)
		{
			// noise = static_cast<double> (rand() % 1000) / 2000.0f;
			noise = 0.0;
			if(doSin)
			{
				double frequency = 4.0;
				trajectory_point[(_POS_ + (j * POS_VEL_ACC)) - 1] = sin(frequency * M_PI * (static_cast<double> (i) / static_cast<double> (num_trajectory_points))) + noise;
			}
			else
			{
				double mean = 0;
				double sigma = 1.5;
				double scale = 10;
				double x = scale*(static_cast<double> (i) / static_cast<double> (num_trajectory_points)) - (scale/2.0);
				trajectory_point[(_POS_ + (j * POS_VEL_ACC)) - 1] = exp( -pow(x-mean,2) / sigma ) + noise;
			}
		}
		dmp_trajectory->add(trajectory_point);
	}

	dmp_trajectory->computeDerivatives();

	if (!dmp_trajectory->writeToCLMCFile(std::string(data_directory + "d00001")))
	{
		ROS_ERROR("could not save trajectory to file");
		return -1;
	}

	// dmp_trajectory->rescalePositionAndComputeDerivativesAndCutout(num_rescale_trajectory_points, 100);
//	int num_rescale_trajectory_points = 1000;
//
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

	for (int i = 0; i < 1; i++)
	{

		// create dmp unit
		dmp_unit = new DynamicMovementPrimitive();
		dmp_unit->readFromDisc(library_directory, dmp_id);

		ROS_INFO("start learning dmp from trajectory...");
		if (!dmp_unit->learnFromTrajectory(dmp_trajectory))
		{
			ROS_ERROR("learning dmp from trajectory was not successful");
			return -1;
		}
//		// write dmp to file
//		if (!dmp_unit->writeToDisc())
//		{
//			ROS_ERROR("could not write dmp to disc");
//			return -1;
//		}
//		delete dmp_unit;
//
//		// create new dmp unit and initialize it from the dmp unit which was just stored to disc.
//		dmp_unit = new DMPMotionUnit();
//		dmp_unit->readFromDisc(library_directory, dmp_id);

		// test copy constructor
		DynamicMovementPrimitive *dmp_unit_test = new DynamicMovementPrimitive(*dmp_unit);
		dmp_unit_test->print();

		// write copied dmp to file
		if (!dmp_unit_test->writeToDisc())
		{
			ROS_ERROR("could not write copied dmp to disc");
			return -1;
		}

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
//		for (int j = 0; j < num_transformation_systems; j++)
//		{
//			goal[j] += i * 0.1;
//		}
		dmp_unit->setup(start, goal); //, new_movement_duration, new_sampling_frequency);

		// create a new trajectory to store the result of the dmp propagation
		reproduced_trajectory = new Trajectory();
		if (!reproduced_trajectory->initialize(std::string("3d_variable_names"), trajectory_dimension, sampling_frequency, NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES, NUM_DEBUG_CANONICAL_SYSTEM_VALUES))
		{
			ROS_ERROR("cannot initialize trajectory");
			return -1;
		}

		int hack = 0;
		double actual[num_transformation_systems];
		for(int i=0; i<num_transformation_systems; i++)
		{
			actual[i] = start[i];
		}

		// propagate the dmp and add obtained trajectory points to the trajectory
		while (dmp_unit->propagateStep(trajectory_point, actual))
		{
			if(hack > 400 && hack < 800)
			{
			  //				ROS_INFO("freeeeeeze... %i",hack);
				// freeze actual position
			}
			else
			{
				for(int i=0; i<num_transformation_systems; i++)
				{
					actual[i] = trajectory_point[POS_VEL_ACC * i];
				}
			}

//			if(hack < 2000)
//			{
				reproduced_trajectory->add(trajectory_point);
				hack++;
//			}
		}
// #if (DEBUG_MODE==1)
		ROS_INFO("writeDebugTrajectory");

		dmp_unit->writeDebugTrajectory();
// #endif

//		std::vector<float> mse_vector = dmp_trajectory->computeMSE(*reproduced_trajectory);
//		int dim = 1;
//		for(std::vector<float>::iterator vfi = mse_vector.begin(); vfi != mse_vector.end(); vfi++)
//		{
//			ROS_INFO_STREAM("MSE " << dim << " dimension>> " << *vfi);
//			dim++;
//		}

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
	//	faster_trajectory->writeToMRDFile(data_directory + std::string("d00005"));
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
	//	slower_trajectory->writeToMRDFile(data_directory + std::string("d00006"));

	// delete dmp_unit;
	// delete dmp_trajectory;

	return 0;

}
