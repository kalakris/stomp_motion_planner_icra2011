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

  \file    task_transforms.h

  \author  Peter Pastor
  \date    Aug 7, 2010

**********************************************************************/

#ifndef TASK_TRANSFORMS_H_
#define TASK_TRANSFORMS_H_

// system includes

// ros includes
#include <ros/ros.h>

#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>

#include <Eigen/Eigen>

#include <pr2_mechanism_model/chain.h>

#include <policy_improvement_utilities/kdl_chain_wrapper.h>

// local includes
#include <pr2_tasks_transforms/default_task_transform.h>
#include <pr2_tasks_transforms/pool_task_transform.h>
#include <pr2_tasks_transforms/chop_stick_task_transform.h>

namespace pr2_tasks_transforms
{

class TaskTransforms
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum TransformType
    {
        LEFT_ARM_NO_TRANSFORM = 0,
        RIGHT_ARM_NO_TRANSFORM,
        LEFT_ARM_POOL_TRANSFORM,
        RIGHT_ARM_POOL_TRANSFORM,
        LEFT_ARM_CHOP_STICK_TRANSFORM,
        RIGHT_ARM_CHOP_STICK_TRANSFORM
    };

    TaskTransforms();
    virtual ~TaskTransforms();

    /*!
     * This function needs to be called from the controller
     * @return
     */
    bool initialize(ros::NodeHandle& node_handle, pr2_mechanism_model::RobotState *robot_state);

    /*!
     * This function needs to be called from the dmp learner
     * @return
     */
    bool initialize(ros::NodeHandle& node_handle);

    /*!
    *
    * @param transform_type
    * @param first_transform
    * @param left_arm_joint_pos_vel_acc_values
    * @param right_arm_joint_pos_vel_acc_values
    * @param task_pos_vel_acc_values
    * @return
    */
    bool getTaskTransform(const TransformType transform_type,
                          bool first_transform,
                          const Eigen::VectorXd& left_arm_joint_pos_vel_acc_values,
                          const Eigen::VectorXd& right_arm_joint_pos_vel_acc_values,
                          Eigen::VectorXd& task_pos_vel_acc_values);

    /*!
     *
     * @param transform_type
     * @param frame
     * @param joint_positions
     * @param task_pos_vel_acc_values
     * @return
     * REAL-TIME REQUIREMENT
     */
    bool getTaskTransformFromCurrent(const TransformType transform_type,
                                     const KDL::Frame& frame,
                                     const Eigen::VectorXd& joint_positions,
                                     Eigen::VectorXd& task_pos_vel_acc_values);

    /*!
     * @param transform_type
     * @param variable_names
     * @return
     */
    bool getTaskVariableDescriptions(const TransformType transform_type, std::vector<std::string>& variable_names);

    /*!
     * @param transform_type
     * @param quaternion_indices
     * @return
     */
    bool getQuaternionIndices(const TransformType transform_type, std::vector<int>& quaternion_indices);

    /*!
     *
     * @param num_dimension
     * @return
     */
    bool getNumDimensions(std::vector<int>& num_dimension);

    /*!
     * @param transform_type
     * @param task_pos_vel_acc_trajectory_point
     * @param robot_pos_vel_acc_trajectory_point
     * @return
     * REAL-TIME REQUIREMENT
     */
    bool getRobotTransform(const TransformType transform_type,
                           const Eigen::VectorXd& task_pos_vel_acc_trajectory_point,
                           Eigen::VectorXd& robot_pos_vel_acc_trajectory_point);

private:

    bool initialized_;

    bool is_get_robot_transform_available_;

    ros::NodeHandle node_handle_;

    pr2_mechanism_model::Chain left_arm_mechanism_chain_;
    pr2_mechanism_model::Chain right_arm_mechanism_chain_;

    KDL::Chain kdl_chain_;

    KDL::JntArray kdl_left_arm_joint_array_;
    KDL::JntArray kdl_right_arm_joint_array_;

    KDL::Frame left_gripper_tool_frame_;
    KDL::Frame right_gripper_tool_frame_;

    Eigen::VectorXd left_arm_gripper_pose_vel_acc_;
    Eigen::VectorXd right_arm_gripper_pose_vel_acc_;

    policy_improvement_utilities::KDLChainWrapper right_gripper_kdl_chain_;
    policy_improvement_utilities::KDLChainWrapper left_gripper_kdl_chain_;

    DefaultTaskTransform default_task_transform_;
    PoolTaskTransform pool_task_transform_;
    ChopStickTaskTransform chop_stick_task_transform_;

    void normalizeQuaternion(Eigen::VectorXd& pos_vel_acc_trajectory_point);

};

}

#endif /* TASK_TRANSFORMS_H_ */
