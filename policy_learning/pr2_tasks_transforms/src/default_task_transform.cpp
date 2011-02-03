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

  \file    default_task_transform.cpp

  \author  Peter Pastor
  \date    Aug 9, 2010

**********************************************************************/

// system includes

// ros includes
#include <policy_improvement_utilities/assert.h>

#include <policy_improvement_utilities/kdl_chain_wrapper.h>

// local includes
#include <pr2_tasks_transforms/default_task_transform.h>
#include <pr2_tasks_transforms/task_transforms_utilities.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

namespace pr2_tasks_transforms
{

DefaultTaskTransform::DefaultTaskTransform()
    : initialized_(false)
{
}

DefaultTaskTransform::~DefaultTaskTransform()
{
}

bool DefaultTaskTransform::initialize(ros::NodeHandle& node_handle)
{

    return (initialized_ = true);
}

bool DefaultTaskTransform::getTaskTransform(const WhichArm arm, bool first_transform,
                                            const VectorXd& joint_pos_vel_acc_trajectory_point,
                                            const VectorXd& cartesian_pos_vel_acc_trajectory_point,
                                            VectorXd& cartesian_and_joint_pos_vel_acc_trajectory_point)
{
    ROS_ASSERT(initialized_);
    ROS_ASSERT(cartesian_and_joint_pos_vel_acc_trajectory_point.size() == (dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS) * dmp::POS_VEL_ACC);

    double forearm_roll_angle_offset = 0;
    double wrist_roll_angle_offset = 0;

    if (first_transform)
    {
        computeAngleOffsets(joint_pos_vel_acc_trajectory_point, forearm_roll_angle_offset, wrist_roll_angle_offset);
    }

    // set cartesian trajectory point
    for (int i = 0; i < dmp::N_CART + dmp::N_QUAT; ++i)
    {
        cartesian_and_joint_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC) = cartesian_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC);
    }

    // account for continous joint and set joint trajectory point
    for (int i = dmp::N_CART + dmp::N_QUAT; i < dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS; ++i)
    {
        if (i == dmp::N_CART + dmp::N_QUAT + dmp::CONTINOUS_FOREARM_ROLL_JOINT)
        {
            cartesian_and_joint_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC) = joint_pos_vel_acc_trajectory_point((i - (dmp::N_CART + dmp::N_QUAT)) * dmp::POS_VEL_ACC) + (forearm_roll_angle_offset * 2.0 * M_PI);
        }
        else if (i == dmp::N_CART + dmp::N_QUAT + dmp::CONTINOUS_WRIST_ROLL_JOINT)
        {
            cartesian_and_joint_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC) = joint_pos_vel_acc_trajectory_point((i - (dmp::N_CART + dmp::N_QUAT)) * dmp::POS_VEL_ACC) + (wrist_roll_angle_offset * 2.0 * M_PI);
        }
        else
        {
            cartesian_and_joint_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC) = joint_pos_vel_acc_trajectory_point((i - (dmp::N_CART + dmp::N_QUAT)) * dmp::POS_VEL_ACC);
        }
    }

    return true;
}

// REAL-TIME REQUIREMENTS
bool DefaultTaskTransform::getTaskTransformFromCurrent(const KDL::Frame& frame,
                                                       const Eigen::VectorXd& joint_positions,
                                                       VectorXd& task_pos_vel_acc_values)
{
    ROS_ASSERT(task_pos_vel_acc_values.size() == dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS);
    for (int i = 0; i < dmp::N_CART; i++)
    {
        task_pos_vel_acc_values(i) = frame.p[i];
    }
    double qx, qy, qz, qw;
    frame.M.GetQuaternion(qx, qy, qz, qw);
    task_pos_vel_acc_values(dmp::N_CART + 0) = qx;
    task_pos_vel_acc_values(dmp::N_CART + 1) = qy;
    task_pos_vel_acc_values(dmp::N_CART + 2) = qz;
    task_pos_vel_acc_values(dmp::N_CART + 3) = qw;
    for (int i = 0; i < dmp::N_JOINTS; i++)
    {
        task_pos_vel_acc_values(dmp::N_CART + dmp::N_QUAT + i) = joint_positions(i);
    }
    return true;
}


void DefaultTaskTransform::getQuaternionIndices(const WhichArm arm, std::vector<int>& quaternion_indices)
{
    quaternion_indices.clear();
    quaternion_indices.push_back(dmp::N_CART + dmp::_QX_);
    quaternion_indices.push_back(dmp::N_CART + dmp::_QY_);
    quaternion_indices.push_back(dmp::N_CART + dmp::_QZ_);
    quaternion_indices.push_back(dmp::N_CART + dmp::_QW_);
}

}
