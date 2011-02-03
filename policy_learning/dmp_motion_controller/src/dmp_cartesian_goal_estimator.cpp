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
#include <kdl/frames.hpp>
#include <dmp_motion_generation/math_helper.h>
#include <angles/angles.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

// local includes
#include <dmp_motion_controller/dmp_cartesian_goal_estimator.h>

namespace dmp_controller
{

	DMPCartesianGoalEstimator::DMPCartesianGoalEstimator() :
		initialized_(false)
	{

	}

	DMPCartesianGoalEstimator::~DMPCartesianGoalEstimator()
	{

	}

	bool DMPCartesianGoalEstimator::initialize(ros::NodeHandle node_handle)
	{

		double param_value;
		std::string param_string;

		ros::NodeHandle controller_handle(std::string("/dmp_motion_controller"));
		std::string movement_names;
		if (!controller_handle.getParam(std::string("movement_names"), movement_names))
			{
				ROS_ERROR_STREAM("DMPCartesianGoalEstimator::initialize>> could not get parameter movement_names (namespace is " << controller_handle.getNamespace() << ").");
				return false;
			}

		ros::NodeHandle goal_offset_handle(std::string("/dmp_motion_controller/goal_offsets"));

		// split the group names based on whitespace
		std::stringstream ss_movement_names(movement_names);

		std::string movement_name;
		int movement_type_index = dmp::Parameters::eGRASP;
		while (ss_movement_names >> movement_name)
			{
				ros::NodeHandle movement_type_handle(goal_offset_handle, movement_name);

				param_string.assign("offset_x");
				if (!movement_type_handle.getParam(param_string, param_value))
					{
						ROS_ERROR("DMPCartesianGoalEstimator::initialize>> could not get goal offset %s of %s from param server  (namespace is %s).", param_string.c_str(), movement_name.c_str(), movement_type_handle.getNamespace().c_str());
						initialized_ = false;
						return initialized_;
					}
				goal_offsets_[movement_type_index].x = param_value;

				param_string.assign("offset_y");
				if (!movement_type_handle.getParam(param_string, param_value))
					{
						ROS_ERROR("DMPCartesianGoalEstimator::initialize>> could not get goal offset %s of %s from param server  (namespace is %s).", param_string.c_str(), movement_name.c_str(), movement_type_handle.getNamespace().c_str());
						initialized_ = false;
						return initialized_;
					}
				goal_offsets_[movement_type_index].y = param_value;

				param_string.assign("offset_z");
				if (!movement_type_handle.getParam(param_string, param_value))
					{
						ROS_ERROR("DMPCartesianGoalEstimator::initialize>> could not get goal offset %s of %s from param server  (namespace is %s).", param_string.c_str(), movement_name.c_str(), movement_type_handle.getNamespace().c_str());
						initialized_ = false;
						return initialized_;
					}
				goal_offsets_[movement_type_index].z = param_value;

				movement_type_index++;
			}

		ros::NodeHandle round_objects_offset_handle(std::string("/dmp_motion_controller"));
		std::string object_colors;
		if (!round_objects_offset_handle.getParam(std::string("blobs"), object_colors))
			{
				ROS_ERROR("DMPCartesianGoalEstimator::initialize>> could not get round object names from param server (namespace: %s)", round_objects_offset_handle.getNamespace().c_str());
				initialized_ = false;
				return initialized_;
			}

		// split the group names based on whitespace
		std::stringstream ss_object_colors(object_colors);

		std::string object_color;
		int num_blob_id = 0;
		while (ss_object_colors >> object_color)
			{
				if (num_blob_id > MAX_NUM_BLOB_IDS)
					{
						ROS_ERROR("DMPCartesianGoalEstimator::initialize>> maximum number of blobs reached (%i).", num_blob_id);
						initialized_ = false;
						return initialized_;
					}

				ros::NodeHandle color_handle(round_objects_offset_handle, object_color);

				param_string.assign("offset");
				if (!color_handle.getParam(param_string, param_value))
					{
						ROS_ERROR("DMPCartesianGoalEstimator::initialize>> could not get radius %s of %s object from param server (namespace: %s).", param_string.c_str(), movement_name.c_str(), color_handle.getNamespace().c_str());
						initialized_ = false;
						return initialized_;
					}
				round_object_offsets_[num_blob_id] = param_value;
				num_blob_id++;
			}

		// get gripper rotation reference point
		ros::NodeHandle rotation_reference_points_handle(std::string("/dmp_motion_controller/rotation_reference_points"));

		// split the group names based on whitespace
		ss_movement_names.str("");
		ss_movement_names.clear();
		ss_movement_names.str(movement_names);

		movement_type_index = dmp::Parameters::eGRASP;
		while (ss_movement_names >> movement_name)
			{
				ros::NodeHandle movement_type_handle(rotation_reference_points_handle, movement_name);

				param_string.assign("x");
				if (!movement_type_handle.getParam(param_string, param_value))
					{
						ROS_ERROR("DMPCartesianGoalEstimator::initialize>> could not get rotation reference point offset %s of movement type \"%s\" from param server (namespace: %s).", param_string.c_str(), movement_name.c_str(), movement_type_handle.getNamespace().c_str());
						initialized_ = false;
						return initialized_;
					}
				rotation_reference_points_[movement_type_index].x = param_value;

				param_string.assign("y");
				if (!movement_type_handle.getParam(param_string, param_value))
					{
						ROS_ERROR("DMPCartesianGoalEstimator::initialize>> could not get rotation reference point offset %s of movement type \"%s\" from param server (namespace: %s).", param_string.c_str(), movement_name.c_str(), movement_type_handle.getNamespace().c_str());
						initialized_ = false;
						return initialized_;
					}
				rotation_reference_points_[movement_type_index].y = param_value;

				param_string.assign("z");
				if (!movement_type_handle.getParam(param_string, param_value))
					{
						ROS_ERROR("DMPCartesianGoalEstimator::initialize>> could not get rotation reference point offset %s of movement type \"%s\" from param server (namespace: %s).", param_string.c_str(), movement_name.c_str(), movement_type_handle.getNamespace().c_str());
						initialized_ = false;
						return initialized_;
					}
				rotation_reference_points_[movement_type_index].z = param_value;

				ROS_INFO("DMPCartesianGoalEstimator::initialize>> %f %f %f", rotation_reference_points_[movement_type_index].x, rotation_reference_points_[movement_type_index].y, rotation_reference_points_[movement_type_index].z);

				movement_type_index++;
			}

		for (int i = 0; i < MAX_NUM_BLOB_IDS; i++)
			{
				goal_blob_valid_[i] = false;
			}

		goals_subscriber_ = node_handle.subscribe(std::string("/dmp_motion_controller/goals"), 1, &DMPCartesianGoalEstimator::goalsCB, this);
		int mutex_return_code = pthread_mutex_init(&goals_lock_, NULL);
		if (mutex_return_code != 0)
			{
				ROS_ERROR("DMPCartesianGoalEstimator::init>> could not initialize mutex (%i : %s).", mutex_return_code, strerror(mutex_return_code));
				initialized_ = false;
				return initialized_;
			}
		mutex_return_code = pthread_mutex_unlock(&goals_lock_);
		if (mutex_return_code != 0)
			{
				ROS_ERROR("DMPCartesianGoalEstimator::init>> could not unlock mutex (%i : %s).", mutex_return_code, strerror(mutex_return_code));
				initialized_ = false;
				return initialized_;
			}

		//	blob_position_filter_corner_x_values_.resize(MAX_NUM_BLOB_IDS);
		//	if (!((filters::MultiChannelFilterBase<double>&) blob_position_corner_x_filter_).configure(MAX_NUM_BLOB_IDS, node_handle.getNamespace() + std::string(
		//			"/filter_blob_position"), node_handle))
		//	{
		//		ROS_ERROR("DMPCartesianGoalEstimator::init>> could not blob (x) position filter.");
		//		initialized_ = false;
		//		return initialized_;
		//	}
		//	blob_position_filter_corner_y_values_.resize(MAX_NUM_BLOB_IDS);
		//	if (!((filters::MultiChannelFilterBase<double>&) blob_position_corner_y_filter_).configure(MAX_NUM_BLOB_IDS, node_handle.getNamespace() + std::string(
		//			"/filter_blob_position"), node_handle))
		//	{
		//		ROS_ERROR("DMPCartesianGoalEstimator::init>> could not blob (y) position filter.");
		//		initialized_ = false;
		//		return initialized_;
		//	}
		//	blob_position_filter_corner_z_values_.resize(MAX_NUM_BLOB_IDS);
		//	if (!((filters::MultiChannelFilterBase<double>&) blob_position_corner_z_filter_).configure(MAX_NUM_BLOB_IDS, node_handle.getNamespace() + std::string(
		//			"/filter_blob_position"), node_handle))
		//	{
		//		ROS_ERROR("DMPCartesianGoalEstimator::init>> could not blob (z) position filter.");
		//		initialized_ = false;
		//		return initialized_;
		//	}
		//
		//	blob_position_filter_center_x_values_.resize(MAX_NUM_BLOB_IDS);
		//	if (!((filters::MultiChannelFilterBase<double>&) blob_position_center_x_filter_).configure(MAX_NUM_BLOB_IDS, node_handle.getNamespace() + std::string(
		//			"/filter_blob_position"), node_handle))
		//	{
		//		ROS_ERROR("DMPCartesianGoalEstimator::init>> could not blob (x) position filter.");
		//		initialized_ = false;
		//		return initialized_;
		//	}
		//	blob_position_filter_center_y_values_.resize(MAX_NUM_BLOB_IDS);
		//	if (!((filters::MultiChannelFilterBase<double>&) blob_position_center_y_filter_).configure(MAX_NUM_BLOB_IDS, node_handle.getNamespace() + std::string(
		//			"/filter_blob_position"), node_handle))
		//	{
		//		ROS_ERROR("DMPCartesianGoalEstimator::init>> could not blob (y) position filter.");
		//		initialized_ = false;
		//		return initialized_;
		//	}
		//	blob_position_filter_center_z_values_.resize(MAX_NUM_BLOB_IDS);
		//	if (!((filters::MultiChannelFilterBase<double>&) blob_position_center_z_filter_).configure(MAX_NUM_BLOB_IDS, node_handle.getNamespace() + std::string(
		//			"/filter_blob_position"), node_handle))
		//	{
		//		ROS_ERROR("DMPCartesianGoalEstimator::init>> could not blob (z) position filter.");
		//		initialized_ = false;
		//		return initialized_;
		//	}
		//
		//	for (int i = 0; i < MAX_NUM_BLOB_IDS; i++)
		//	{
		//		blob_position_filter_center_x_values_[i] = 0.0;
		//		blob_position_filter_center_y_values_[i] = 0.0;
		//		blob_position_filter_center_z_values_[i] = 0.0;
		//		blob_position_filter_corner_x_values_[i] = 0.0;
		//		blob_position_filter_corner_y_values_[i] = 0.0;
		//		blob_position_filter_corner_z_values_[i] = 0.0;
		//	}

		initialized_ = true;
		return initialized_;
	}

	// REAL-TIME REQUIREMENT
	bool DMPCartesianGoalEstimator::updateGoalOrientation(dmp::DynamicMovementPrimitive *dmp, double *linear_twist)
	{

		return true;

		double desired_goal_position[dmp::N_CART];
		if(!dmp->getGoal(desired_goal_position, 0 , dmp::N_CART))
			return false;

		double desired_current_position[dmp::N_CART];
		if(!dmp->getCurrentPosition(desired_current_position, 0, dmp::N_CART))
			return false;

		Eigen::Matrix<double, 3, 1> velocity_vec;
		Eigen::Matrix<double, 3, 1> world_vec;
		for (int i = 0; i < dmp::N_CART; i++)
			{
				// velocity_vec(i) = linear_twist[i];
				velocity_vec(i) = desired_goal_position[i] - desired_current_position[i];
				world_vec(i) = 0.0;
			}
		world_vec(0) = 1.0;
		Eigen::Quaternion<double> eigen_quat;
		eigen_quat.setFromTwoVectors(world_vec, velocity_vec);

		//	ROS_ERROR("desired_goal_position %f %f %f", desired_goal_position[0], desired_goal_position[1], desired_goal_position[2]);
		//	ROS_ERROR("desired_current_position %f %f %f", desired_current_position[0], desired_current_position[1], desired_current_position[2]);
		//	ROS_ERROR("vec %f %f %f", velocity_vec(0), velocity_vec(1), velocity_vec(2));

		float goal[dmp::N_QUAT];
		goal[dmp::_QX_] = static_cast<float>(eigen_quat.x());
		goal[dmp::_QY_] = static_cast<float>(eigen_quat.y());
		goal[dmp::_QZ_] = static_cast<float>(eigen_quat.z());
		goal[dmp::_QW_] = static_cast<float>(eigen_quat.w());

		return dmp->changeGoal(goal, dmp::N_CART, dmp::N_CART + dmp::N_QUAT);
	}

	// REAL-TIME REQUIREMENT
	bool DMPCartesianGoalEstimator::updateGoal(dmp::DynamicMovementPrimitive *dmp)
	{

		int blob_id = dmp->getGoalBlobID();
		dmp::Parameters::eTaskType task_type = dmp->getTaskType();

		// horrible hack for the demo... TODO: remove me after the demo.
		if(blob_id == -2)
			{
				double goal[dmp::N_CART];
				dmp->getGoal(goal, 0, dmp::N_CART + dmp::N_QUAT);

				// add an offset in the direction of the line of sight (torso_lift_link -> goal) to get to the center of the round object.
				goal[dmp::_XX_] += goal_offsets_[task_type].x;
				goal[dmp::_YY_] += goal_offsets_[task_type].y;
				goal[dmp::_ZZ_] += goal_offsets_[task_type].z;
				
				double initial_goal[dmp::N_CART + dmp::N_QUAT];
				if (!dmp->getInitialGoal(initial_goal, 0, dmp::N_CART + dmp::N_QUAT))
					{
						ROS_ERROR("DMPCartesianGoalEstimator::updateGoal>> could not get initial goal !!! (TODO remove me... REAL-TIME violation)");
						return false;
					}

				KDL::Rotation initial_goal_quat = KDL::Rotation::Quaternion(initial_goal[dmp::N_CART + dmp::_QX_], initial_goal[dmp::N_CART + dmp::_QY_],
																																		initial_goal[dmp::N_CART + dmp::_QZ_], initial_goal[dmp::N_CART + dmp::_QW_]);
				double initial_goal_roll, initial_goal_pitch, initial_goal_yaw;
				initial_goal_quat.GetRPY(initial_goal_roll, initial_goal_pitch, initial_goal_yaw);
				
				double approach_angle = dmp->getApproachAngle();
				double root_frame_to_gripper_frame_yaw = atan2(goal[dmp::_YY_], goal[dmp::_XX_]);
				double new_goal_yaw = root_frame_to_gripper_frame_yaw + approach_angle;

				double qx, qy, qz, qw;
				getQuaternionFromRPY(initial_goal_roll, initial_goal_pitch, new_goal_yaw, qx, qy, qz, qw);
				
				goal[dmp::N_CART + dmp::_QX_] = qx;
				goal[dmp::N_CART + dmp::_QY_] = qy;
				goal[dmp::N_CART + dmp::_QZ_] = qz;
				goal[dmp::N_CART + dmp::_QW_] = qw;
				
			}
	
		// check whether the dmp is setup to a goal/target color blob
		if (blob_id >= 0 && blob_id < MAX_NUM_BLOB_IDS)
			{
				if (goal_blob_valid_[blob_id])
					{
						// update the goal
						if (pthread_mutex_trylock(&goals_lock_) == 0)
							{

								float goal[dmp::N_CART];

								if (task_type == dmp::Parameters::eGRASP)
									{
										for (int i = 0; i < dmp::N_CART; i++)
											{
												goal[i] = goals_center_[i][blob_id];
											}
						
										// add an offset in the direction of the line of sight (torso_lift_link -> goal) to get to the center of the round object.
										float length = sqrt(goal[dmp::_XX_] * goal[dmp::_XX_] + goal[dmp::_YY_] * goal[dmp::_YY_] + goal[dmp::_ZZ_] * goal[dmp::_ZZ_]);
										float goal_direction[dmp::N_CART];
										goal_direction[dmp::_XX_] = goal[dmp::_XX_] / length;
										goal_direction[dmp::_YY_] = goal[dmp::_YY_] / length;
										goal_direction[dmp::_ZZ_] = goal[dmp::_ZZ_] / length;
										for (int i = 0; i < dmp::N_CART; i++)
											{
												goal[i] += round_object_offsets_[blob_id] * goal_direction[i];
											}
						
										goal[dmp::_XX_] += goal_offsets_[task_type].x;
										goal[dmp::_YY_] += goal_offsets_[task_type].y;
										goal[dmp::_ZZ_] += goal_offsets_[task_type].z;

										double initial_goal[dmp::N_CART + dmp::N_QUAT];
										if (!dmp->getInitialGoal(initial_goal, 0, dmp::N_CART + dmp::N_QUAT))
											{
												ROS_ERROR("DMPCartesianGoalEstimator::updateGoal>> could not get initial goal !!! (TODO remove me... REAL-TIME violation)");
												return false;
											}

										KDL::Rotation initial_goal_quat = KDL::Rotation::Quaternion(initial_goal[dmp::N_CART + dmp::_QX_], initial_goal[dmp::N_CART + dmp::_QY_],
																																								initial_goal[dmp::N_CART + dmp::_QZ_], initial_goal[dmp::N_CART + dmp::_QW_]);
										double initial_goal_roll, initial_goal_pitch, initial_goal_yaw;
										initial_goal_quat.GetRPY(initial_goal_roll, initial_goal_pitch, initial_goal_yaw);

										double approach_angle = dmp->getApproachAngle();
										double root_frame_to_gripper_frame_yaw = atan2(goal[dmp::_YY_], goal[dmp::_XX_]);
										double new_goal_yaw = root_frame_to_gripper_frame_yaw + approach_angle;

										double qx, qy, qz, qw;
										getQuaternionFromRPY(initial_goal_roll, initial_goal_pitch, new_goal_yaw, qx, qy, qz, qw);

										goal[dmp::N_CART + dmp::_QX_] = qx;
										goal[dmp::N_CART + dmp::_QY_] = qy;
										goal[dmp::N_CART + dmp::_QZ_] = qz;
										goal[dmp::N_CART + dmp::_QW_] = qw;

									}

								else if (task_type == dmp::Parameters::ePOURE)
									{
										for (int i = 0; i < dmp::N_CART; i++)
											{
												goal[i] = goals_corner_[i][blob_id];
											}

										// add an offset in the direction of the line of sight (torso_lift_link -> goal) to get to the center of the round object.
										float length = sqrt(goal[dmp::_XX_] * goal[dmp::_XX_] + goal[dmp::_YY_] * goal[dmp::_YY_] + goal[dmp::_ZZ_] * goal[dmp::_ZZ_]);
										float goal_direction[dmp::N_CART];
										goal_direction[dmp::_XX_] = goal[dmp::_XX_] / length;
										goal_direction[dmp::_YY_] = goal[dmp::_YY_] / length;
										goal_direction[dmp::_ZZ_] = goal[dmp::_ZZ_] / length;
										for (int i = 0; i < dmp::N_CART; i++)
											{
												goal[i] += round_object_offsets_[blob_id] * goal_direction[i];
											}

										goal[dmp::_XX_] += goal_offsets_[task_type].x;
										goal[dmp::_YY_] += goal_offsets_[task_type].y;
										goal[dmp::_ZZ_] += goal_offsets_[task_type].z;

										double initial_goal[dmp::N_CART + dmp::N_QUAT];
										if (!dmp->getInitialGoal(initial_goal, 0, dmp::N_CART + dmp::N_QUAT))
											{
												ROS_ERROR("DMPCartesianGoalEstimator::updateGoal>> could not get initial goal !!! (TODO remove me... REAL-TIME violation)");
												return false;
											}

										double new_x, new_y, initial_x, initial_y;
										new_x = goal[dmp::_XX_] - rotation_reference_points_[task_type].x;
										new_y = goal[dmp::_YY_] - rotation_reference_points_[task_type].y;
										initial_x = initial_goal[dmp::_XX_] - rotation_reference_points_[task_type].x;
										initial_y = initial_goal[dmp::_YY_] - rotation_reference_points_[task_type].y;

										// angle of v2 relative to v1 = atan2(v2.y,v2.x) - atan2(v1.y,v1.x)
										double yaw = atan2(new_y, new_x) - atan2(initial_y, initial_x);
										KDL::Rotation initial_goal_to_new_goal_rotation = KDL::Rotation::RotZ(yaw);
										KDL::Rotation initial_goal_rotation = KDL::Rotation::Quaternion(initial_goal[dmp::N_CART + dmp::_QX_], initial_goal[dmp::N_CART + dmp::_QY_],
																																										initial_goal[dmp::N_CART + dmp::_QZ_], initial_goal[dmp::N_CART + dmp::_QW_]);

										KDL::Rotation new_goal_rotation = initial_goal_rotation * initial_goal_to_new_goal_rotation;

										double qx, qy, qz, qw;
										new_goal_rotation.GetQuaternion(qx, qy, qz, qw);

										goal[dmp::N_CART + dmp::_QX_] = qx;
										goal[dmp::N_CART + dmp::_QY_] = qy;
										goal[dmp::N_CART + dmp::_QZ_] = qz;
										goal[dmp::N_CART + dmp::_QW_] = qw;

									}

								else if (task_type == dmp::Parameters::ePLACE)
									{
										for (int i = 0; i < dmp::N_CART; i++)
											{
												goal[i] = goals_center_[i][blob_id];
											}

										// add an offset in the direction of the line of sight (torso_lift_link -> goal) to get to the center of the round object.
										float length = sqrt(goal[dmp::_XX_] * goal[dmp::_XX_] + goal[dmp::_YY_] * goal[dmp::_YY_] + goal[dmp::_ZZ_] * goal[dmp::_ZZ_]);
										float goal_direction[dmp::N_CART];
										goal_direction[dmp::_XX_] = goal[dmp::_XX_] / length;
										goal_direction[dmp::_YY_] = goal[dmp::_YY_] / length;
										goal_direction[dmp::_ZZ_] = goal[dmp::_ZZ_] / length;
										for (int i = 0; i < dmp::N_CART; i++)
											{
												goal[i] += round_object_offsets_[blob_id] * goal_direction[i];
											}

										goal[dmp::_XX_] += goal_offsets_[task_type].x;
										goal[dmp::_YY_] += goal_offsets_[task_type].y;
										goal[dmp::_ZZ_] += goal_offsets_[task_type].z;

										double initial_goal[dmp::N_CART + dmp::N_QUAT];
										if (!dmp->getInitialGoal(initial_goal, 0, dmp::N_CART + dmp::N_QUAT))
											{
												ROS_ERROR("DMPCartesianGoalEstimator::updateGoal>> could not get initial goal !!! (TODO remove me... REAL-TIME violation)");
												return false;
											}

										KDL::Rotation initial_goal_quat = KDL::Rotation::Quaternion(initial_goal[dmp::N_CART + dmp::_QX_], initial_goal[dmp::N_CART + dmp::_QY_],
																																								initial_goal[dmp::N_CART + dmp::_QZ_], initial_goal[dmp::N_CART + dmp::_QW_]);
										double initial_goal_roll, initial_goal_pitch, initial_goal_yaw;
										initial_goal_quat.GetRPY(initial_goal_roll, initial_goal_pitch, initial_goal_yaw);

										double approach_angle = dmp->getApproachAngle();
										double root_frame_to_gripper_frame_yaw = atan2(goal[dmp::_YY_], goal[dmp::_XX_]);
										double new_goal_yaw = root_frame_to_gripper_frame_yaw + approach_angle;

										double qx, qy, qz, qw;
										getQuaternionFromRPY(initial_goal_roll, initial_goal_pitch, new_goal_yaw, qx, qy, qz, qw);

										goal[dmp::N_CART + dmp::_QX_] = qx;
										goal[dmp::N_CART + dmp::_QY_] = qy;
										goal[dmp::N_CART + dmp::_QZ_] = qz;
										goal[dmp::N_CART + dmp::_QW_] = qw;

									}

								else if(task_type == dmp::Parameters::eRELEASE)
									{
										for (int i = 0; i < dmp::N_CART; i++)
											{
												goal[i] = goals_center_[i][blob_id];
											}

										goal[dmp::_XX_] += goal_offsets_[task_type].x;
										goal[dmp::_YY_] += goal_offsets_[task_type].y;
										goal[dmp::_ZZ_] += goal_offsets_[task_type].z;

									}

								dmp->changeGoal(goal, 0, dmp::N_CART + dmp::N_QUAT);
								pthread_mutex_unlock(&goals_lock_);
							}
					}
			}
		
		return true;
	}
	
	void DMPCartesianGoalEstimator::goalsCB(const dmp_motion_controller::GoalsConstPtr &goals)
	{
		
		if (pthread_mutex_lock(&goals_lock_) == 0)
			{
				for (int i = 0; i < MAX_NUM_BLOB_IDS; i++)
					{
						goal_blob_valid_[i] = false;
					}
				for (unsigned int i = 0; i < goals->blob_id.size(); i++)
					{
						if (goals->blob_id[i] >= 0 && goals->blob_id[i] < MAX_NUM_BLOB_IDS)
							{
								goals_center_[dmp::_XX_][goals->blob_id[i]] = goals->center_position[goals->blob_id[i]].x;
								goals_center_[dmp::_YY_][goals->blob_id[i]] = goals->center_position[goals->blob_id[i]].y;
								goals_center_[dmp::_ZZ_][goals->blob_id[i]] = goals->center_position[goals->blob_id[i]].z;
								goals_corner_[dmp::_XX_][goals->blob_id[i]] = goals->top_position[goals->blob_id[i]].x;
								goals_corner_[dmp::_YY_][goals->blob_id[i]] = goals->top_position[goals->blob_id[i]].y;
								goals_corner_[dmp::_ZZ_][goals->blob_id[i]] = goals->top_position[goals->blob_id[i]].z;
								goal_blob_valid_[goals->blob_id[i]] = true;
							}
					}
				
				pthread_mutex_unlock(&goals_lock_);
			}
		
	}
	
}
