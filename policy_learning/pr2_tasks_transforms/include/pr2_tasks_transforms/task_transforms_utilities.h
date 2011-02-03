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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

  \file    task_transforms_utilities.h

  \author  Peter Pastor
  \date    Aug 9, 2010

**********************************************************************/

#ifndef TASK_TRANSFORMS_UTILITIES_H_
#define TASK_TRANSFORMS_UTILITIES_H_

// system includes
#include <vector>
#include <string>

// ros includes
#include <ros/ros.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>

#include <dmp_motion_generation/constants.h>

// local includes

namespace pr2_tasks_transforms
{

inline bool getVariableNamesFromParamServer(ros::NodeHandle& node_handle, const std::string& variable_names_key_word, std::vector<std::string>& variable_names)
{
    variable_names.clear();

    std::string all_names;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, variable_names_key_word, all_names));

    // split the variable names based on whitespace
    std::stringstream ss(all_names);
    std::string key_name;
    while (ss >> key_name)
    {
        ros::NodeHandle variable_name_node_handle(node_handle, key_name);
        std::string variable_name;
        ROS_ASSERT_FUNC(policy_improvement_utilities::read(variable_name_node_handle, std::string("name"), variable_name));
        variable_names.push_back(variable_name);
    }
    return true;
}

inline bool setEigenVectorFromKDLFrame(const KDL::Frame& frame, Eigen::VectorXd& task_pos_vel_acc_values)
{
    ROS_ASSERT(task_pos_vel_acc_values.size() == ((dmp::N_CART + dmp::N_QUAT) * dmp::POS_VEL_ACC));

    task_pos_vel_acc_values(0 * dmp::POS_VEL_ACC) = frame.p.x();
    task_pos_vel_acc_values(1 * dmp::POS_VEL_ACC) = frame.p.y();
    task_pos_vel_acc_values(2 * dmp::POS_VEL_ACC) = frame.p.z();

    // task_pos_vel_acc_values(0 * dmp::POS_VEL_ACC) = frame_vel.p.p.x();
    // task_pos_vel_acc_values((0 * dmp::POS_VEL_ACC) + 1) = frame_vel.p.v.x();
    // task_pos_vel_acc_values(1 * dmp::POS_VEL_ACC) = frame_vel.p.p.y();
    // task_pos_vel_acc_values((1 * dmp::POS_VEL_ACC) + 1) = frame_vel.p.v.y();
    // task_pos_vel_acc_values(2 * dmp::POS_VEL_ACC) = frame_vel.p.p.z();
    // task_pos_vel_acc_values((2 * dmp::POS_VEL_ACC) + 1) = frame_vel.p.v.z();

    double qx, qy, qz, qw;
    frame.M.GetQuaternion(qx, qy, qz, qw);
    // frame_vel.M.R.GetQuaternion(qx, qy, qz, qw);
    task_pos_vel_acc_values(3 * dmp::POS_VEL_ACC) = qx;
    task_pos_vel_acc_values(4 * dmp::POS_VEL_ACC) = qy;
    task_pos_vel_acc_values(5 * dmp::POS_VEL_ACC) = qz;
    task_pos_vel_acc_values(6 * dmp::POS_VEL_ACC) = qw;
    return true;
}

inline void computeAngleOffsets(const Eigen::VectorXd& joint_trajectory_point, double& forearm_roll_angle_offset, double& wrist_roll_angle_offset)
{
    double joint_angle;

    // double normalize_joint_angle = angles::normalize_angle(joint_angle);
    // TODO: change this back...
    // forearm_roll_angle_offset_ = joint_angle - normalize_joint_angle;

    joint_angle = joint_trajectory_point(dmp::CONTINOUS_FOREARM_ROLL_JOINT * dmp::POS_VEL_ACC);
    // ROS_INFO("Joint angle is %f.", joint_angle);
    while (joint_angle >= M_PI)
    {
        forearm_roll_angle_offset--;
        joint_angle -= 2 * M_PI;
    }
    while (joint_angle < -M_PI)
    {
        forearm_roll_angle_offset++;
        joint_angle += 2 * M_PI;
    }
    // ROS_INFO("Angle offset of forearm roll joint is %i.", forearm_roll_angle_offset_);

    joint_angle = joint_trajectory_point(dmp::CONTINOUS_WRIST_ROLL_JOINT * dmp::POS_VEL_ACC);
    // ROS_INFO("Joint angle is %f.", joint_angle);
    while (joint_angle >= M_PI)
    {
        wrist_roll_angle_offset--;
        joint_angle -= 2 * M_PI;
    }
    while (joint_angle < -M_PI)
    {
        wrist_roll_angle_offset++;
        joint_angle += 2 * M_PI;
    }
    // ROS_INFO("Angle offset of wrist roll joint is %i.", wrist_roll_angle_offset_);
}

}

#endif /* TASK_TRANSFORMS_UTILITIES_H_ */
