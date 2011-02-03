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

 \file    task_transforms.cpp

 \author  Peter Pastor
 \date    Aug 7, 2010

 **********************************************************************/

// system includes

// ros includes
#include <policy_improvement_utilities/assert.h>

#include <dmp_motion_generation/constants.h>

#include <kdl_parser/kdl_parser.hpp>

// local includes
#include <pr2_tasks_transforms/task_transforms.h>
#include <pr2_tasks_transforms/task_transforms_utilities.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

namespace pr2_tasks_transforms
{

const std::string root_frame = std::string("torso_lift_link");
const std::string left_arm_tip_frame = std::string("l_gripper_tool_frame");
const std::string right_arm_tip_frame = std::string("r_gripper_tool_frame");

TaskTransforms::TaskTransforms() :
    initialized_(false), is_get_robot_transform_available_(false)
{
}

TaskTransforms::~TaskTransforms()
{
}

bool TaskTransforms::initialize(ros::NodeHandle& node_handle, pr2_mechanism_model::RobotState *robot_state)
{
    ROS_ASSERT_FUNC(left_arm_mechanism_chain_.init(robot_state, root_frame, left_arm_tip_frame));
    ROS_ASSERT_FUNC(right_arm_mechanism_chain_.init(robot_state, root_frame, right_arm_tip_frame));

    is_get_robot_transform_available_ = true;
    return initialize(node_handle);
}

bool TaskTransforms::initialize(ros::NodeHandle& node_handle)
{
    node_handle_ = node_handle;

    std::string robot_description;
    ROS_ASSERT_FUNC(node_handle.getParam("/robot_description", robot_description));

    // create kdl tree
    KDL::Tree left_arm_kdl_tree;
    ROS_ASSERT_FUNC(kdl_parser::treeFromString(robot_description, left_arm_kdl_tree));
    KDL::Tree right_arm_kdl_tree;
    ROS_ASSERT_FUNC(kdl_parser::treeFromString(robot_description, right_arm_kdl_tree));

    // create the chain given the tree
    KDL::Chain left_arm_kdl_chain;
    ROS_ASSERT_FUNC(left_arm_kdl_tree.getChain(root_frame, left_arm_tip_frame, left_arm_kdl_chain));
    KDL::Chain right_arm_kdl_chain;
    ROS_ASSERT_FUNC(right_arm_kdl_tree.getChain(root_frame, right_arm_tip_frame, right_arm_kdl_chain));

    kdl_left_arm_joint_array_.resize(left_arm_kdl_chain.getNrOfJoints());
    kdl_right_arm_joint_array_.resize(right_arm_kdl_chain.getNrOfJoints());

    left_arm_gripper_pose_vel_acc_ = VectorXd::Zero((dmp::N_CART + dmp::N_QUAT) * dmp::POS_VEL_ACC);
    right_arm_gripper_pose_vel_acc_ = VectorXd::Zero((dmp::N_CART + dmp::N_QUAT) * dmp::POS_VEL_ACC);

    left_gripper_kdl_chain_.initialize(node_handle_, root_frame, left_arm_tip_frame);
    right_gripper_kdl_chain_.initialize(node_handle_, root_frame, right_arm_tip_frame);

    ROS_ASSERT_FUNC(default_task_transform_.initialize(node_handle_));
    ROS_ASSERT_FUNC(pool_task_transform_.initialize(node_handle_));
    ROS_ASSERT_FUNC(chop_stick_task_transform_.initialize(node_handle_));

    return (initialized_ = true);
}

bool TaskTransforms::getTaskVariableDescriptions(const TransformType transform_type, std::vector<std::string>& variable_names)
{
    ROS_ASSERT(initialized_);
    ros::NodeHandle trajectory_node_handle(node_handle_, std::string("trajectory"));
    switch(transform_type)
    {
        case TaskTransforms::LEFT_ARM_NO_TRANSFORM:
        {
            ROS_ASSERT_FUNC(getVariableNamesFromParamServer(trajectory_node_handle, std::string("cart_and_joint_left_arm"), variable_names));
            break;
        }
        case TaskTransforms::RIGHT_ARM_NO_TRANSFORM:
        {
            ROS_ASSERT_FUNC(getVariableNamesFromParamServer(trajectory_node_handle, std::string("cart_and_joint_right_arm"), variable_names));
            break;
        }
        case TaskTransforms::LEFT_ARM_POOL_TRANSFORM:
        {
            ROS_ASSERT_FUNC(getVariableNamesFromParamServer(trajectory_node_handle, std::string("cart_and_joint_left_arm"), variable_names));
            break;
        }
        case TaskTransforms::RIGHT_ARM_POOL_TRANSFORM:
        {
            ROS_ASSERT_FUNC(getVariableNamesFromParamServer(trajectory_node_handle, std::string("pool_and_joint_right_arm"), variable_names));
            break;
        }
        case TaskTransforms::LEFT_ARM_CHOP_STICK_TRANSFORM:
        {
            ROS_ASSERT_FUNC(getVariableNamesFromParamServer(trajectory_node_handle, std::string("chop_stick_and_joint_left_arm"), variable_names));
            break;
        }
        case TaskTransforms::RIGHT_ARM_CHOP_STICK_TRANSFORM:
        {
            ROS_ASSERT_FUNC(getVariableNamesFromParamServer(trajectory_node_handle, std::string("chop_stick_and_joint_right_arm"), variable_names));
            break;
        }
    }

    return true;
}

bool TaskTransforms::getQuaternionIndices(const TransformType transform_type, std::vector<int>& quaternion_indices)
{
    ROS_ASSERT(initialized_);
    switch(transform_type)
    {
        case TaskTransforms::LEFT_ARM_NO_TRANSFORM:
        {
            default_task_transform_.getQuaternionIndices(DefaultTaskTransform::LEFT_ARM, quaternion_indices);
            break;
        }
        case TaskTransforms::RIGHT_ARM_NO_TRANSFORM:
        {
            default_task_transform_.getQuaternionIndices(DefaultTaskTransform::RIGHT_ARM, quaternion_indices);
            break;
        }
        case TaskTransforms::LEFT_ARM_POOL_TRANSFORM:
        {
            pool_task_transform_.getQuaternionIndices(PoolTaskTransform::LEFT_ARM, quaternion_indices);
            break;
        }
        case TaskTransforms::RIGHT_ARM_POOL_TRANSFORM:
        {
            pool_task_transform_.getQuaternionIndices(PoolTaskTransform::RIGHT_ARM, quaternion_indices);
            break;
        }
        case TaskTransforms::LEFT_ARM_CHOP_STICK_TRANSFORM:
        {
            chop_stick_task_transform_.getQuaternionIndices(ChopStickTaskTransform::LEFT_ARM, quaternion_indices);
            break;
        }
        case TaskTransforms::RIGHT_ARM_CHOP_STICK_TRANSFORM:
        {
            chop_stick_task_transform_.getQuaternionIndices(ChopStickTaskTransform::RIGHT_ARM, quaternion_indices);
            break;
        }
    }

    return true;
}

bool TaskTransforms::getTaskTransform(const TransformType transform_type, bool first_transform,
                                      const VectorXd& left_arm_joint_pos_vel_acc_values,
                                      const VectorXd& right_arm_joint_pos_vel_acc_values,
                                      Eigen::VectorXd& task_pos_vel_acc_values)
{
    ROS_ASSERT(initialized_);
    // ROS_INFO("TaskTransforms::getTaskTransform %i", (int)transform_type);

    ROS_ASSERT(left_arm_joint_pos_vel_acc_values.size() == right_arm_joint_pos_vel_acc_values.size());
    int num_joint_pos_vel_acc_values = left_arm_joint_pos_vel_acc_values.size();
    ROS_ASSERT(num_joint_pos_vel_acc_values % dmp::POS_VEL_ACC == 0);
    for (int i = 0; i < (num_joint_pos_vel_acc_values / dmp::POS_VEL_ACC); ++i)
    {
        kdl_left_arm_joint_array_.data(i) = left_arm_joint_pos_vel_acc_values(i * dmp::POS_VEL_ACC);
        kdl_right_arm_joint_array_.data(i) = right_arm_joint_pos_vel_acc_values(i * dmp::POS_VEL_ACC);
    }

    ROS_ASSERT_FUNC(left_gripper_kdl_chain_.forwardKinematics(kdl_left_arm_joint_array_, left_gripper_tool_frame_));
    ROS_ASSERT_FUNC(right_gripper_kdl_chain_.forwardKinematics(kdl_right_arm_joint_array_, right_gripper_tool_frame_));

    ROS_ASSERT_FUNC(setEigenVectorFromKDLFrame(left_gripper_tool_frame_, left_arm_gripper_pose_vel_acc_));
    ROS_ASSERT_FUNC(setEigenVectorFromKDLFrame(right_gripper_tool_frame_, right_arm_gripper_pose_vel_acc_));

    switch(transform_type)
    {
        case TaskTransforms::LEFT_ARM_NO_TRANSFORM:
        {
            ROS_ASSERT_FUNC(default_task_transform_.getTaskTransform(DefaultTaskTransform::LEFT_ARM, first_transform,
                                                                     left_arm_joint_pos_vel_acc_values,
                                                                     left_arm_gripper_pose_vel_acc_,
                                                                     task_pos_vel_acc_values));
            break;
        }
        case TaskTransforms::RIGHT_ARM_NO_TRANSFORM:
        {
            ROS_ASSERT_FUNC(default_task_transform_.getTaskTransform(DefaultTaskTransform::RIGHT_ARM, first_transform,
                                                                     right_arm_joint_pos_vel_acc_values,
                                                                     right_arm_gripper_pose_vel_acc_,
                                                                     task_pos_vel_acc_values));
            break;
        }
        case TaskTransforms::LEFT_ARM_POOL_TRANSFORM:
        {
            ROS_ASSERT_FUNC(pool_task_transform_.getTaskTransform(PoolTaskTransform::LEFT_ARM, first_transform,
                                                                  right_gripper_tool_frame_,
                                                                  left_gripper_tool_frame_,
                                                                  left_arm_joint_pos_vel_acc_values,
                                                                  task_pos_vel_acc_values));
            break;
        }
        case TaskTransforms::RIGHT_ARM_POOL_TRANSFORM:
        {
            ROS_ASSERT_FUNC(pool_task_transform_.getTaskTransform(PoolTaskTransform::RIGHT_ARM, first_transform,
                                                                  right_gripper_tool_frame_,
                                                                  left_gripper_tool_frame_,
                                                                  right_arm_joint_pos_vel_acc_values,
                                                                  task_pos_vel_acc_values));
            break;
        }
        case TaskTransforms::LEFT_ARM_CHOP_STICK_TRANSFORM:
        {
            ROS_ASSERT_FUNC(chop_stick_task_transform_.getTaskTransform(ChopStickTaskTransform::LEFT_ARM, first_transform,
                                                                     left_arm_joint_pos_vel_acc_values,
                                                                     left_arm_gripper_pose_vel_acc_,
                                                                     task_pos_vel_acc_values));
            break;
        }
        case TaskTransforms::RIGHT_ARM_CHOP_STICK_TRANSFORM:
        {
            ROS_ASSERT_FUNC(chop_stick_task_transform_.getTaskTransform(ChopStickTaskTransform::RIGHT_ARM, first_transform,
                                                                     right_arm_joint_pos_vel_acc_values,
                                                                     right_arm_gripper_pose_vel_acc_,
                                                                     task_pos_vel_acc_values));
            break;
        }
    }
    return true;
}

bool TaskTransforms::getTaskTransformFromCurrent(const TransformType transform_type, const KDL::Frame& gripper_frame,
                                                 const Eigen::VectorXd& joint_positions, VectorXd& task_pos_vel_acc_trajectory_point)
{

    // ROS_INFO("TaskTransforms::getTaskTransformFromCurrent %i", (int)transform_type);

    switch(transform_type)
    {
        case TaskTransforms::LEFT_ARM_NO_TRANSFORM:
        {
            ROS_ASSERT_FUNC(default_task_transform_.getTaskTransformFromCurrent(gripper_frame, joint_positions, task_pos_vel_acc_trajectory_point));
            break;
        }
        case TaskTransforms::RIGHT_ARM_NO_TRANSFORM:
        {
            ROS_ASSERT_FUNC(default_task_transform_.getTaskTransformFromCurrent(gripper_frame, joint_positions, task_pos_vel_acc_trajectory_point));
            break;
        }
        case TaskTransforms::LEFT_ARM_POOL_TRANSFORM:
        {
            KDL::Frame unsused_frame;
            ROS_ASSERT_FUNC(pool_task_transform_.getTaskTransformFromCurrent(PoolTaskTransform::LEFT_ARM, unsused_frame,
                                                                             gripper_frame, joint_positions,
                                                                             task_pos_vel_acc_trajectory_point));
            break;
        }
        case TaskTransforms::RIGHT_ARM_POOL_TRANSFORM:
        {
            ROS_ASSERT(is_get_robot_transform_available_);
            left_arm_mechanism_chain_.getPositions(kdl_left_arm_joint_array_);

            ROS_ASSERT_FUNC(left_gripper_kdl_chain_.forwardKinematics(kdl_left_arm_joint_array_, left_gripper_tool_frame_));
            ROS_ASSERT_FUNC(pool_task_transform_.getTaskTransformFromCurrent(PoolTaskTransform::RIGHT_ARM,
                                                                             gripper_frame,
                                                                             left_gripper_tool_frame_,
                                                                             joint_positions,
                                                                             task_pos_vel_acc_trajectory_point));
            break;
        }
        case TaskTransforms::LEFT_ARM_CHOP_STICK_TRANSFORM:
        {
            ROS_ASSERT_FUNC(chop_stick_task_transform_.getTaskTransformFromCurrent(gripper_frame, joint_positions, task_pos_vel_acc_trajectory_point));
            break;
        }
        case TaskTransforms::RIGHT_ARM_CHOP_STICK_TRANSFORM:
        {
            ROS_ASSERT_FUNC(chop_stick_task_transform_.getTaskTransformFromCurrent(gripper_frame, joint_positions, task_pos_vel_acc_trajectory_point));
            break;
        }
    }
    return true;
}

// REAL-TIME REQUIREMENT
bool TaskTransforms::getRobotTransform(const TransformType transform_type,
                                       const Eigen::VectorXd& task_pos_vel_acc_trajectory_point,
                                       Eigen::VectorXd& robot_pos_vel_acc_trajectory_point)
{
    ROS_ASSERT(is_get_robot_transform_available_);

    switch(transform_type)
    {
        case TaskTransforms::LEFT_ARM_NO_TRANSFORM:
        {
            ROS_ASSERT(robot_pos_vel_acc_trajectory_point.size() == task_pos_vel_acc_trajectory_point.size());
            robot_pos_vel_acc_trajectory_point = task_pos_vel_acc_trajectory_point;
            normalizeQuaternion(robot_pos_vel_acc_trajectory_point);
            break;
        }
        case TaskTransforms::RIGHT_ARM_NO_TRANSFORM:
        {
            ROS_ASSERT(robot_pos_vel_acc_trajectory_point.size() == task_pos_vel_acc_trajectory_point.size());
            robot_pos_vel_acc_trajectory_point = task_pos_vel_acc_trajectory_point;
            normalizeQuaternion(robot_pos_vel_acc_trajectory_point);
            break;
        }
        case TaskTransforms::LEFT_ARM_POOL_TRANSFORM:
        {
            ROS_ASSERT(robot_pos_vel_acc_trajectory_point.size() == task_pos_vel_acc_trajectory_point.size());
            robot_pos_vel_acc_trajectory_point = task_pos_vel_acc_trajectory_point;
            normalizeQuaternion(robot_pos_vel_acc_trajectory_point);
            break;
        }
        case TaskTransforms::RIGHT_ARM_POOL_TRANSFORM:
        {
            left_arm_mechanism_chain_.getPositions(kdl_left_arm_joint_array_);
            ROS_ASSERT_FUNC(left_gripper_kdl_chain_.forwardKinematics(kdl_left_arm_joint_array_, left_gripper_tool_frame_));

            ROS_ASSERT_FUNC(pool_task_transform_.getRobotTransform(left_gripper_tool_frame_, task_pos_vel_acc_trajectory_point, robot_pos_vel_acc_trajectory_point));
            break;
        }
        case TaskTransforms::LEFT_ARM_CHOP_STICK_TRANSFORM:
        {
            ROS_ASSERT_FUNC(chop_stick_task_transform_.getRobotTransform(ChopStickTaskTransform::LEFT_ARM, task_pos_vel_acc_trajectory_point, robot_pos_vel_acc_trajectory_point));
            break;
        }
        case TaskTransforms::RIGHT_ARM_CHOP_STICK_TRANSFORM:
        {
            ROS_ASSERT_FUNC(chop_stick_task_transform_.getRobotTransform(ChopStickTaskTransform::RIGHT_ARM, task_pos_vel_acc_trajectory_point, robot_pos_vel_acc_trajectory_point));
            break;
        }
    }
    return true;
}

bool TaskTransforms::getNumDimensions(std::vector<int>& num_dimension)
{
    num_dimension.clear();
    std::vector<int> default_transform_vector = default_task_transform_.getNumDimensions();
    std::vector<int> pool_transform_vector = pool_task_transform_.getNumDimensions();
    std::vector<int> chop_stick_transform_vector = chop_stick_task_transform_.getNumDimensions();
    num_dimension.insert(num_dimension.end(), default_transform_vector.begin(), default_transform_vector.end());
    num_dimension.insert(num_dimension.end(), pool_transform_vector.begin(), pool_transform_vector.end());
    num_dimension.insert(num_dimension.end(), chop_stick_transform_vector.begin(), chop_stick_transform_vector.end());
    return true;
}

// REAL-TIME REQUIREMENT
void TaskTransforms::normalizeQuaternion(VectorXd& pos_vel_acc_trajectory_point)
{
    Vector4d quat;
    for (int i = 0; i < dmp::N_QUAT; i++)
    {
        quat(i) = pos_vel_acc_trajectory_point(dmp::N_CART * dmp::POS_VEL_ACC + i * dmp::POS_VEL_ACC);
    }
    quat.normalize();
    for (int i = 0; i < dmp::N_QUAT; i++)
    {
        pos_vel_acc_trajectory_point(dmp::N_CART * dmp::POS_VEL_ACC + i * dmp::POS_VEL_ACC) = quat(i);
    }
}

}
