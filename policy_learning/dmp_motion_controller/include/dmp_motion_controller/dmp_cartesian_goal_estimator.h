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


#ifndef DMP_CARTESIAN_GOAL_ESTIMATOR_H_
#define DMP_CARTESIAN_GOAL_ESTIMATOR_H_

// system includes

// ros includes
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <dmp_motion_generation/dynamic_movement_primitive.h>
#include <dmp_motion_generation/parameters.h>

// #include <filters/transfer_function.h>

// local includes
#include <dmp_motion_controller/Goals.h>

namespace dmp_controller {

const int MAX_NUM_BLOB_IDS = 5;

class DMPCartesianGoalEstimator {

public:

	/*!
	 * @return
	 */
	DMPCartesianGoalEstimator();

	/*!
	 * @return
	 */
	~DMPCartesianGoalEstimator();

	/*!
	 * @return
	 */
	bool initialize(ros::NodeHandle node_handle);

	/*! REAL-TIME REQUIREMENT
	 * @return
	 */
	bool updateGoal(dmp::DynamicMovementPrimitive *dmp);

	/*! REAL-TIME REQUIREMENT
	 * @return
	 */
	bool updateGoalOrientation(dmp::DynamicMovementPrimitive *dmp, double *linear_twist);

private:

	/*!
	 */
	bool initialized_;

	/*!
	 * @param goals
	 */
	void goalsCB(const dmp_motion_controller::GoalsConstPtr &goals);
	ros::Subscriber goals_subscriber_;

	/*!
	 */
	dmp::DynamicMovementPrimitive *dmp_;

	/*! goal updates
	 */
	pthread_mutex_t goals_lock_;
	float goals_center_[dmp::N_CART][MAX_NUM_BLOB_IDS];
	float goals_corner_[dmp::N_CART][MAX_NUM_BLOB_IDS];
	bool goal_blob_valid_[MAX_NUM_BLOB_IDS];

	/*!
	 */
//	filters::TransferFunctionFilter<double> blob_position_corner_x_filter_;
//	filters::TransferFunctionFilter<double> blob_position_corner_y_filter_;
//	filters::TransferFunctionFilter<double> blob_position_corner_z_filter_;
//	filters::TransferFunctionFilter<double> blob_position_center_x_filter_;
//	filters::TransferFunctionFilter<double> blob_position_center_y_filter_;
//	filters::TransferFunctionFilter<double> blob_position_center_z_filter_;
//	std::vector<double> blob_position_filter_corner_x_values_;
//	std::vector<double> blob_position_filter_corner_y_values_;
//	std::vector<double> blob_position_filter_corner_z_values_;
//	std::vector<double> blob_position_filter_center_x_values_;
//	std::vector<double> blob_position_filter_center_y_values_;
//	std::vector<double> blob_position_filter_center_z_values_;

	/*!
	 */
	double round_object_offsets_[MAX_NUM_BLOB_IDS];

	/*!
 	 */
	geometry_msgs::Point goal_offsets_[dmp::Parameters::eNUM_TASK_TYPES];

	/*!
	 */
	geometry_msgs::Point rotation_reference_points_[dmp::Parameters::eNUM_TASK_TYPES];

};

}

#endif /* DMP_CARTESIAN_GOAL_ESTIMATOR_H_ */
