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

  \file    pool_task_transforms.h

  \author  Peter Pastor
  \date    Aug 7, 2010

**********************************************************************/

// system includes
#include <math.h>

// ros includes
#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/assert.h>

#include <dmp_motion_generation/constants.h>

// local includes
#include <pr2_tasks_transforms/pool_task_transform.h>

USING_PART_OF_NAMESPACE_EIGEN

namespace pr2_tasks_transforms
{

PoolTaskTransform::PoolTaskTransform()
    : initialized_(false)
{

}

PoolTaskTransform::~PoolTaskTransform()
{
}

bool PoolTaskTransform::initialize(ros::NodeHandle& node_handle)
{
    node_handle_ = node_handle;
    publishing_counter_ = 0;

    ROS_ASSERT_FUNC(readParams());
    ROS_ASSERT_FUNC(initRTPublisher());

    left_task_pos_vel_acc_trajectory_point_ = VectorXd::Zero((dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS) * dmp::POS_VEL_ACC);
    right_task_pos_vel_acc_trajectory_point_ = VectorXd::Zero((N_POOL_DOF + dmp::N_JOINTS) * dmp::POS_VEL_ACC);

    return (initialized_ = true);
}

bool PoolTaskTransform::initRTPublisher()
{
    rosrt::init();

    geometry_msgs::PoseStamped debug_frame_visualization_marker;
    vis_marker_debug_frame_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("debug_frame"), 1), 100, debug_frame_visualization_marker));

    geometry_msgs::PoseStamped cue_frame_visualization_marker;
    vis_marker_cue_frame_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("cue_frame"), 1), 100, cue_frame_visualization_marker));


    geometry_msgs::PoseStamped right_gripper_frame_visualization_marker;
    vis_marker_right_gripper_frame_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("right_gripper_frame"), 1), 100, right_gripper_frame_visualization_marker));

    visualization_msgs::Marker cue_visualization_marker;
    cue_visualization_marker.points.resize(2); // root and tip of the cue stick
    vis_marker_cue_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise
            <visualization_msgs::Marker> (std::string("cue"), 1), 100, cue_visualization_marker));

    return true;
}

bool PoolTaskTransform::readParams()
{

    ros::NodeHandle node_handle("/pr2_tasks_transform");
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("publishing_rate"), publishing_rate_));

    std::vector<double> right_gripper_to_cue_offset;
    ROS_ASSERT_FUNC(policy_improvement_utilities::readDoubleArray(node_handle, std::string("right_gripper_to_cue_offset"), right_gripper_to_cue_offset));

    std::vector<double> left_gripper_to_bridge_offset;
    ROS_ASSERT_FUNC(policy_improvement_utilities::readDoubleArray(node_handle, std::string("left_gripper_to_bridge_offset"), left_gripper_to_bridge_offset));

    // set fixed offset between r_gripper_tool_frame and cue
    right_gripper_to_cue_offset_.M.Identity();
    right_gripper_to_cue_offset_.p.x(right_gripper_to_cue_offset[0]);
    right_gripper_to_cue_offset_.p.y(right_gripper_to_cue_offset[1]);
    right_gripper_to_cue_offset_.p.z(right_gripper_to_cue_offset[2]);

    // set fixed offset between l_gripper_tool_frame and bridge
    left_gripper_to_bridge_offset_.M.Identity();
    left_gripper_to_bridge_offset_.p.x(left_gripper_to_bridge_offset[0]);
    left_gripper_to_bridge_offset_.p.y(left_gripper_to_bridge_offset[1]);
    left_gripper_to_bridge_offset_.p.z(left_gripper_to_bridge_offset[2]);

    std::vector<double> cue_offset;
    ROS_ASSERT_FUNC(policy_improvement_utilities::readDoubleArray(node_handle, std::string("cue_offset"), cue_offset));
    cue_offset_ = cue_offset[2];

    return true;
}

bool PoolTaskTransform::getTaskTransform(const WhichArm arm, bool first_transform,
                                         const KDL::Frame& right_gripper,
                                         const KDL::Frame& left_gripper,
                                         const Eigen::VectorXd& arm_joint_positions,
                                         Eigen::VectorXd& task_pos_vel_acc_trajectory_point)
{
    ROS_ASSERT(initialized_);

    switch (arm)
    {
        case PoolTaskTransform::LEFT_ARM:
        {
            ROS_ASSERT_FUNC(getLeftArmTaskTransform(right_gripper, left_gripper, arm_joint_positions, task_pos_vel_acc_trajectory_point));
            break;
        }
        case PoolTaskTransform::RIGHT_ARM:
        {
            ROS_ASSERT_FUNC(getRightArmTaskTransform(right_gripper, left_gripper, arm_joint_positions, task_pos_vel_acc_trajectory_point));
            break;
        }
    }
    return true;
}

// REAL-TIME REQUIREMENTS
bool PoolTaskTransform::getTaskTransformFromCurrent(const WhichArm arm,
                                                    const KDL::Frame& right_gripper,
                                                    const KDL::Frame& left_gripper,
                                                    const Eigen::VectorXd& arm_joint_positions,
                                                    Eigen::VectorXd& task_pos_trajectory_point)
{
    ROS_ASSERT(initialized_);
    switch(arm)
    {
        case PoolTaskTransform::LEFT_ARM:
        {
            ROS_ASSERT_FUNC(getLeftArmTaskTransform(right_gripper, left_gripper, arm_joint_positions, left_task_pos_vel_acc_trajectory_point_));
            for (int i = 0; i < N_POOL_DOF + dmp::N_JOINTS; ++i)
            {
                task_pos_trajectory_point(i) = left_task_pos_vel_acc_trajectory_point_(i * dmp::POS_VEL_ACC);
            }
            break;
        }
        case PoolTaskTransform::RIGHT_ARM:
        {
            ROS_ASSERT_FUNC(getRightArmTaskTransform(right_gripper, left_gripper, arm_joint_positions, right_task_pos_vel_acc_trajectory_point_));
            for (int i = 0; i < N_POOL_DOF + dmp::N_JOINTS; ++i)
            {
                task_pos_trajectory_point(i) = right_task_pos_vel_acc_trajectory_point_(i * dmp::POS_VEL_ACC);
            }
            break;
        }
    }

    return true;
}

bool PoolTaskTransform::getLeftArmTaskTransform(const KDL::Frame& right_gripper,
                                                const KDL::Frame& left_gripper,
                                                const Eigen::VectorXd& arm_joint_positions,
                                                Eigen::VectorXd& task_pos_vel_acc_trajectory_point)
{
    ROS_ASSERT(task_pos_vel_acc_trajectory_point.size() == ((dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS) * dmp::POS_VEL_ACC));
    ROS_ASSERT(arm_joint_positions.size() == (dmp::N_JOINTS * dmp::POS_VEL_ACC));
    for (int i = 0; i < dmp::N_CART; i++)
    {
        task_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC) = left_gripper.p[i];
    }
    double qx, qy, qz, qw;
    left_gripper.M.GetQuaternion(qx, qy, qz, qw);
    task_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (0 * dmp::POS_VEL_ACC)) = qx;
    task_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (1 * dmp::POS_VEL_ACC)) = qy;
    task_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (2 * dmp::POS_VEL_ACC)) = qz;
    task_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (3 * dmp::POS_VEL_ACC)) = qw;
    for (int i = 0; i < dmp::N_JOINTS; i++)
    {
        task_pos_vel_acc_trajectory_point(((dmp::N_CART + dmp::N_QUAT) * dmp::POS_VEL_ACC) + (i * dmp::POS_VEL_ACC)) = arm_joint_positions(i * dmp::POS_VEL_ACC);
    }
    return true;
}

bool PoolTaskTransform::getRightArmTaskTransform(const KDL::Frame& right_gripper,
                                                 const KDL::Frame& left_gripper,
                                                 const Eigen::VectorXd& arm_joint_positions,
                                                 Eigen::VectorXd& task_pos_vel_acc_trajectory_point)
{

    ROS_ASSERT(arm_joint_positions.size() == (dmp::N_JOINTS * dmp::POS_VEL_ACC));

    KDL::Rotation right_gripper_rotation = right_gripper.M;
    double right_gripper_roll, right_gripper_pitch, right_gripper_yaw;
    right_gripper_rotation.GetRPY(right_gripper_roll, right_gripper_pitch, right_gripper_yaw);

    KDL::Frame left_bridge_frame = left_gripper * left_gripper_to_bridge_offset_;
    KDL::Frame right_cue_frame = right_gripper * right_gripper_to_cue_offset_;


    KDL::Frame transform_frame = right_cue_frame.Inverse() * left_bridge_frame;

    double pitch = atan2(-transform_frame.p.x(), transform_frame.p.z());
    double roll = atan2(-transform_frame.p.y(), sin(pitch)*transform_frame.p.x() - cos(pitch)*transform_frame.p.z());
    KDL::Frame cue_align_frame(KDL::Rotation::RotX(roll) * KDL::Rotation::RotY(pitch));

    KDL::Frame corrected_transform_frame = cue_align_frame * right_cue_frame.Inverse() * left_bridge_frame;

    // correct rotational offset
    KDL::Frame fixed_rotation_frame(KDL::Rotation::RotX(M_PI));
    corrected_transform_frame = fixed_rotation_frame * corrected_transform_frame;

    // debug information
    cue_frame_ = right_cue_frame;
    debug_frame_ = left_bridge_frame;
    right_gripper_frame_ = left_bridge_frame * corrected_transform_frame.Inverse(); // * right_gripper_to_cue_offset_.Inverse();

    // assign values
    task_pos_vel_acc_trajectory_point(0 * dmp::POS_VEL_ACC) = corrected_transform_frame.p.Norm();
    // task_pos_vel_acc_trajectory_point(0 * dmp::POS_VEL_ACC) = corrected_transform_frame.p.z();
    double cue_frame_roll, cue_frame_pitch, cue_frame_yaw;
    corrected_transform_frame.M.GetRPY(cue_frame_roll, cue_frame_pitch, cue_frame_yaw);
    task_pos_vel_acc_trajectory_point(1 * dmp::POS_VEL_ACC) = cue_frame_roll;
    task_pos_vel_acc_trajectory_point(2 * dmp::POS_VEL_ACC) = cue_frame_pitch;
    task_pos_vel_acc_trajectory_point(3 * dmp::POS_VEL_ACC) = cue_frame_yaw;
    for (int i = 0; i < dmp::N_JOINTS; i++)
    {
        task_pos_vel_acc_trajectory_point((N_POOL_DOF * dmp::POS_VEL_ACC) + (i * dmp::POS_VEL_ACC)) = arm_joint_positions(i * dmp::POS_VEL_ACC);
    }

    publishMarker();
    return true;
}

// REAL-TIME REQUIREMENTS
bool PoolTaskTransform::getRobotTransform(const KDL::Frame& left_gripper,
                       const Eigen::VectorXd& task_pos_vel_acc_trajectory_point,
                       Eigen::VectorXd& robot_pos_vel_acc_trajectory_point)
{

    KDL::Frame left_bridge_frame = left_gripper * left_gripper_to_bridge_offset_;

    debug_frame_ = left_bridge_frame;

    double offset = task_pos_vel_acc_trajectory_point(0 * dmp::POS_VEL_ACC);
    double roll = task_pos_vel_acc_trajectory_point(1 * dmp::POS_VEL_ACC);
    double pitch = task_pos_vel_acc_trajectory_point(2 * dmp::POS_VEL_ACC);
    double yaw = task_pos_vel_acc_trajectory_point(3 * dmp::POS_VEL_ACC);

    KDL::Frame corrected_transform_frame(KDL::Rotation::RPY(roll, pitch, yaw));
    corrected_transform_frame.p.z(offset);

    right_gripper_frame_ = left_bridge_frame * corrected_transform_frame.Inverse();

    right_gripper_frame_ = right_gripper_frame_ * right_gripper_to_cue_offset_.Inverse();

    robot_pos_vel_acc_trajectory_point(0 * dmp::POS_VEL_ACC) = right_gripper_frame_.p.x();
    robot_pos_vel_acc_trajectory_point(1 * dmp::POS_VEL_ACC) = right_gripper_frame_.p.y();
    robot_pos_vel_acc_trajectory_point(2 * dmp::POS_VEL_ACC) = right_gripper_frame_.p.z();

    double qx, qy, qz, qw;
    right_gripper_frame_.M.GetQuaternion(qx, qy, qz, qw);
    robot_pos_vel_acc_trajectory_point(3 * dmp::POS_VEL_ACC) = qx;
    robot_pos_vel_acc_trajectory_point(4 * dmp::POS_VEL_ACC) = qy;
    robot_pos_vel_acc_trajectory_point(5 * dmp::POS_VEL_ACC) = qz;
    robot_pos_vel_acc_trajectory_point(6 * dmp::POS_VEL_ACC) = qw;

    cue_frame_ = right_gripper_frame_ * right_gripper_to_cue_offset_;

    publishMarker();
    return true;
}

//// REAL-TIME REQUIREMENTS
//bool PoolTaskTransform::convertToRightGripperFrame(const Eigen::VectorXd& task_pos_vel_acc_trajectory_point,
//                                                   const KDL::Frame& left_bridge_gripper,
//                                                   Eigen::VectorXd& robot_pos_vel_acc_trajectory_point)
//{
//
//    return true;
//}

//bool PoolTaskTransform::computeCueTipLocation(const KDL::Frame& right_cue_frame, const KDL::Frame& left_bridge_frame, KDL::Vector& cue_tip_position)
//{
//    Vector3d right_gripper_to_left_gripper;
//    right_gripper_to_left_gripper(0) = left_bridge_frame.p.x() - right_cue_frame.p.x();
//    right_gripper_to_left_gripper(1) = left_bridge_frame.p.y() - right_cue_frame.p.y();
//    right_gripper_to_left_gripper(2) = left_bridge_frame.p.z() - right_cue_frame.p.z();
//    right_gripper_to_left_gripper.normalize();
//    for (int i = 0; i < 3; ++i)
//    {
//        cue_tip_position[i] = right_gripper_to_left_gripper * cue_offset_;
//    }
//    return true;
//}

void PoolTaskTransform::getQuaternionIndices(const WhichArm arm, std::vector<int>& quaternion_indices)
{
    quaternion_indices.clear();
    switch(arm)
    {
        case PoolTaskTransform::LEFT_ARM:
        {
            quaternion_indices.push_back(dmp::N_CART + dmp::_QX_);
            quaternion_indices.push_back(dmp::N_CART + dmp::_QY_);
            quaternion_indices.push_back(dmp::N_CART + dmp::_QZ_);
            quaternion_indices.push_back(dmp::N_CART + dmp::_QW_);
            break;
        }
        case PoolTaskTransform::RIGHT_ARM:
        {
            // there are no quaternions contained
            break;
        }
    }
}

void PoolTaskTransform::publishMarker()
{
    publishing_counter_++;
    if (publishing_counter_ % publishing_rate_ == 0)
    {
        publishing_counter_ = 0;
        double qx, qy, qz, qw;

        visualization_msgs::MarkerPtr cue_msg = vis_marker_cue_publisher_->allocate();
        if (cue_msg)
        {
            cue_msg->header.frame_id = std::string("/torso_lift_link");
            cue_msg->header.stamp = ros::Time::now();
            cue_msg->ns = "pr2_tasks_transform";
            cue_msg->type = visualization_msgs::Marker::ARROW;
            cue_msg->action = visualization_msgs::Marker::ADD;
            cue_msg->id = 9;
            cue_msg->scale.x = 0.025;
            cue_msg->scale.y = 0.025;
            double offset = 0.12;
            KDL::Frame cue_root_offset_frame;
            cue_root_offset_frame.p.z(-offset);
            KDL::Frame cue_tip_offset_frame;
            cue_tip_offset_frame.p.z(cue_offset_-offset);
            KDL::Frame cue_root_frame = right_gripper_frame_ * right_gripper_to_cue_offset_ * cue_root_offset_frame;
            KDL::Frame cue_tip_frame = right_gripper_frame_ * right_gripper_to_cue_offset_ * cue_tip_offset_frame;
            cue_msg->points[0].x = cue_root_frame.p.x();
            cue_msg->points[0].y = cue_root_frame.p.y();
            cue_msg->points[0].z = cue_root_frame.p.z();
            cue_msg->points[1].x = cue_tip_frame.p.x();
            cue_msg->points[1].y = cue_tip_frame.p.y();
            cue_msg->points[1].z = cue_tip_frame.p.z();
            cue_msg->color.r = 0.93;
            cue_msg->color.g = 0.46;
            cue_msg->color.b = 0.0;
            cue_msg->color.a = 1.0;
            vis_marker_cue_publisher_->publish(cue_msg);
        }

        geometry_msgs::PoseStampedPtr debug_frame_msg = vis_marker_debug_frame_publisher_->allocate();
        if (debug_frame_msg)
        {
            debug_frame_msg->header.frame_id = std::string("/torso_lift_link");
            // debug_frame_msg->header.frame_id = std::string("/r_gripper_tool_frame");
            debug_frame_msg->header.stamp = ros::Time::now();
            debug_frame_msg->pose.position.x = debug_frame_.p.x();
            debug_frame_msg->pose.position.y = debug_frame_.p.y();
            debug_frame_msg->pose.position.z = debug_frame_.p.z();
            debug_frame_.M.GetQuaternion(qx, qy, qz, qw);
            debug_frame_msg->pose.orientation.x = qx;
            debug_frame_msg->pose.orientation.y = qy;
            debug_frame_msg->pose.orientation.z = qz;
            debug_frame_msg->pose.orientation.w = qw;
            vis_marker_debug_frame_publisher_->publish(debug_frame_msg);
        }

        geometry_msgs::PoseStampedPtr cue_frame_msg = vis_marker_cue_frame_publisher_->allocate();
        if (cue_frame_msg)
        {
            cue_frame_msg->header.frame_id = std::string("/torso_lift_link");
            // cue_frame_msg->header.frame_id = std::string("/r_gripper_tool_frame");
            cue_frame_msg->header.stamp = ros::Time::now();
            cue_frame_msg->pose.position.x = cue_frame_.p.x();
            cue_frame_msg->pose.position.y = cue_frame_.p.y();
            cue_frame_msg->pose.position.z = cue_frame_.p.z();
            cue_frame_.M.GetQuaternion(qx, qy, qz, qw);
            cue_frame_msg->pose.orientation.x = qx;
            cue_frame_msg->pose.orientation.y = qy;
            cue_frame_msg->pose.orientation.z = qz;
            cue_frame_msg->pose.orientation.w = qw;
            vis_marker_cue_frame_publisher_->publish(cue_frame_msg);
        }

        geometry_msgs::PoseStampedPtr right_gripper_frame_msg = vis_marker_right_gripper_frame_publisher_->allocate();
        if (right_gripper_frame_msg)
        {
            right_gripper_frame_msg->header.frame_id = std::string("/torso_lift_link");
            right_gripper_frame_msg->header.stamp = ros::Time::now();
            right_gripper_frame_msg->pose.position.x = right_gripper_frame_.p.x();
            right_gripper_frame_msg->pose.position.y = right_gripper_frame_.p.y();
            right_gripper_frame_msg->pose.position.z = right_gripper_frame_.p.z();
            double qx, qy, qz, qw;
            right_gripper_frame_.M.GetQuaternion(qx, qy, qz, qw);
            right_gripper_frame_msg->pose.orientation.x = qx;
            right_gripper_frame_msg->pose.orientation.y = qy;
            right_gripper_frame_msg->pose.orientation.z = qz;
            right_gripper_frame_msg->pose.orientation.w = qw;
            vis_marker_right_gripper_frame_publisher_->publish(right_gripper_frame_msg);
        }

    }
}

}
