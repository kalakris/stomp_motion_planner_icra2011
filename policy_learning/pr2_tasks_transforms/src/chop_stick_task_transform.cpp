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

  \file    chop_stick_task_transforms.h

  \author  Peter Pastor
  \date    Aug 24, 2010

**********************************************************************/

// system includes
#include <math.h>

// ros includes
#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/assert.h>

#include <dmp_motion_generation/constants.h>

// local includes
#include <pr2_tasks_transforms/chop_stick_task_transform.h>
#include <pr2_tasks_transforms/task_transforms_utilities.h>

USING_PART_OF_NAMESPACE_EIGEN

namespace pr2_tasks_transforms
{

ChopStickTaskTransform::ChopStickTaskTransform()
    : initialized_(false)
{

}

ChopStickTaskTransform::~ChopStickTaskTransform()
{
}

bool ChopStickTaskTransform::initialize(ros::NodeHandle& node_handle)
{
    node_handle_ = node_handle;
    publishing_counter_ = 0;

    ROS_ASSERT_FUNC(readParams());
    ROS_ASSERT_FUNC(initRTPublisher());

    task_pos_vel_acc_trajectory_point_ = VectorXd::Zero(N_CHOP_STICK_DOF * dmp::POS_VEL_ACC);

    return (initialized_ = true);
}

bool ChopStickTaskTransform::initRTPublisher()
{
    rosrt::init();

    geometry_msgs::PoseStamped left_gripper_debug_frame_visualization_marker;
    vis_marker_left_gripper_debug_frame_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("left_gripper_debug_frame"), 1), 100, left_gripper_debug_frame_visualization_marker));
    geometry_msgs::PoseStamped right_gripper_debug_frame_visualization_marker;
    vis_marker_right_gripper_debug_frame_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("right_gripper_debug_frame"), 1), 100, right_gripper_debug_frame_visualization_marker));

    geometry_msgs::PoseStamped left_chop_stick_frame_visualization_marker;
    vis_marker_left_chop_stick_frame_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("left_chop_stick_frame"), 1), 100, left_chop_stick_frame_visualization_marker));
    geometry_msgs::PoseStamped right_chop_stick_frame_visualization_marker;
    vis_marker_right_chop_stick_frame_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("right_chop_stick_frame"), 1), 100, right_chop_stick_frame_visualization_marker));

    geometry_msgs::PoseStamped left_gripper_frame_visualization_marker;
    vis_marker_left_gripper_frame_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("left_gripper_frame"), 1), 100, left_gripper_frame_visualization_marker));
    geometry_msgs::PoseStamped right_gripper_frame_visualization_marker;
    vis_marker_right_gripper_frame_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("right_gripper_frame"), 1), 100, right_gripper_frame_visualization_marker));

    visualization_msgs::Marker left_chop_stick_visualization_marker;
    left_chop_stick_visualization_marker.points.resize(2); // root and tip of the chop stick
    vis_marker_left_chop_stick_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise
            <visualization_msgs::Marker> (std::string("cue"), 1), 100, left_chop_stick_visualization_marker));
    visualization_msgs::Marker right_chop_stick_visualization_marker;
    right_chop_stick_visualization_marker.points.resize(2); // root and tip of the chop stick
    vis_marker_right_chop_stick_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise
            <visualization_msgs::Marker> (std::string("cue"), 1), 100, right_chop_stick_visualization_marker));

    return true;
}

bool ChopStickTaskTransform::readParams()
{
    ros::NodeHandle node_handle("/pr2_tasks_transform");
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("publishing_rate"), publishing_rate_));

    std::vector<double> gripper_to_chop_stick_offset;
    ROS_ASSERT_FUNC(policy_improvement_utilities::readDoubleArray(node_handle, std::string("gripper_to_chop_stick_offset"), gripper_to_chop_stick_offset));

    // set fixed offset between the r/l_gripper_tool_frame and chop stick
    gripper_to_chop_stick_offset_.M.Identity();
    gripper_to_chop_stick_offset_.p.x(gripper_to_chop_stick_offset[0]);
    gripper_to_chop_stick_offset_.p.y(gripper_to_chop_stick_offset[1]);
    gripper_to_chop_stick_offset_.p.z(gripper_to_chop_stick_offset[2]);

    std::vector<double> chop_stick_offset;
    ROS_ASSERT_FUNC(policy_improvement_utilities::readDoubleArray(node_handle, std::string("chop_stick_offset"), chop_stick_offset));
    chop_stick_offset_ = chop_stick_offset[2];

    return true;
}

bool ChopStickTaskTransform::getTaskTransform(const WhichArm arm,
                                              bool first_transform,
                                              const VectorXd& joint_pos_vel_acc_trajectory_point,
                                              const VectorXd& cartesian_pos_vel_acc_trajectory_point,
                                              VectorXd& cartesian_and_joint_pos_vel_acc_trajectory_point)
{
    ROS_ASSERT(initialized_);
    ROS_ASSERT(cartesian_and_joint_pos_vel_acc_trajectory_point.size() == (N_CHOP_STICK_DOF) * dmp::POS_VEL_ACC);

    double forearm_roll_angle_offset = 0;
    double wrist_roll_angle_offset = 0;

    if (first_transform)
    {
        computeAngleOffsets(joint_pos_vel_acc_trajectory_point, forearm_roll_angle_offset, wrist_roll_angle_offset);
    }

    for (int i = 0; i < dmp::N_CART; ++i)
    {
        cartesian_and_joint_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC) = cartesian_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC);
    }

    KDL::Rotation tmp_rotation = KDL::Rotation::Quaternion(cartesian_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (dmp::_QX_ * dmp::POS_VEL_ACC)),
                                                           cartesian_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (dmp::_QY_ * dmp::POS_VEL_ACC)),
                                                           cartesian_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (dmp::_QZ_ * dmp::POS_VEL_ACC)),
                                                           cartesian_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (dmp::_QW_ * dmp::POS_VEL_ACC)));

    double roll, pitch, yaw;
    tmp_rotation.GetRPY(roll, pitch, yaw);

    cartesian_and_joint_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (0 * dmp::POS_VEL_ACC)) = roll;
    cartesian_and_joint_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (1 * dmp::POS_VEL_ACC)) = pitch;
    cartesian_and_joint_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (2 * dmp::POS_VEL_ACC)) = yaw;


    // account for continous joint and set joint trajectory point
    for (int i = dmp::N_CART + dmp::N_CART; i < dmp::N_CART + dmp::N_CART + dmp::N_JOINTS; ++i)
    {
        if (i == dmp::N_CART + dmp::N_CART + dmp::CONTINOUS_FOREARM_ROLL_JOINT)
        {
            cartesian_and_joint_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC) = joint_pos_vel_acc_trajectory_point((i - (dmp::N_CART + dmp::N_CART)) * dmp::POS_VEL_ACC) + (forearm_roll_angle_offset * 2.0 * M_PI);
        }
        else if (i == dmp::N_CART + dmp::N_CART + dmp::CONTINOUS_WRIST_ROLL_JOINT)
        {
            cartesian_and_joint_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC) = joint_pos_vel_acc_trajectory_point((i - (dmp::N_CART + dmp::N_CART)) * dmp::POS_VEL_ACC) + (wrist_roll_angle_offset * 2.0 * M_PI);
        }
        else
        {
            cartesian_and_joint_pos_vel_acc_trajectory_point(i * dmp::POS_VEL_ACC) = joint_pos_vel_acc_trajectory_point((i - (dmp::N_CART + dmp::N_CART)) * dmp::POS_VEL_ACC);
        }
    }

    return true;
}

// REAL-TIME REQUIREMENTS
bool ChopStickTaskTransform::getTaskTransformFromCurrent(const KDL::Frame& frame,
                                                         const Eigen::VectorXd& joint_positions,
                                                         VectorXd& task_pos_vel_acc_values)
{
    ROS_ASSERT(task_pos_vel_acc_values.size() == dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS);
    KDL::Frame gripper_frame = frame * gripper_to_chop_stick_offset_.Inverse();
    for (int i = 0; i < dmp::N_CART; i++)
    {
        task_pos_vel_acc_values(i) = gripper_frame.p[i];
    }
    double roll, pitch, yaw;
    gripper_frame.M.GetRPY(roll, pitch, yaw);
    task_pos_vel_acc_values(dmp::N_CART + 0) = roll;
    task_pos_vel_acc_values(dmp::N_CART + 1) = pitch;
    task_pos_vel_acc_values(dmp::N_CART + 2) = yaw;
    for (int i = 0; i < dmp::N_JOINTS; i++)
    {
        task_pos_vel_acc_values(dmp::N_CART + dmp::N_CART + i) = joint_positions(i);
    }
    return true;
}

// REAL-TIME REQUIREMENTS
bool ChopStickTaskTransform::getRobotTransform(const WhichArm arm,
                                               const Eigen::VectorXd& task_pos_vel_acc_trajectory_point,
                                               Eigen::VectorXd& robot_pos_vel_acc_trajectory_point)
{

    double chop_x = task_pos_vel_acc_trajectory_point(dmp::_XX_ * dmp::POS_VEL_ACC);
    double chop_y = task_pos_vel_acc_trajectory_point(dmp::_YY_ * dmp::POS_VEL_ACC);
    double chop_z = task_pos_vel_acc_trajectory_point(dmp::_ZZ_ * dmp::POS_VEL_ACC);
    double chop_roll = task_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (0 * dmp::POS_VEL_ACC));
    double chop_pitch = task_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (1 * dmp::POS_VEL_ACC));
    double chop_yaw = task_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (2 * dmp::POS_VEL_ACC));

    KDL::Rotation chop_stick_rotation = KDL::Rotation::RPY(chop_roll, chop_pitch, chop_yaw);
    double qx, qy, qz, qw;

    switch(arm)
    {
        case ChopStickTaskTransform::LEFT_ARM:
        {
            left_chop_stick_frame_.p.x(chop_x);
            left_chop_stick_frame_.p.y(chop_y);
            left_chop_stick_frame_.p.z(chop_z);
            left_chop_stick_frame_.M = chop_stick_rotation;

            left_gripper_frame_ = left_chop_stick_frame_ * gripper_to_chop_stick_offset_.Inverse();

            robot_pos_vel_acc_trajectory_point(dmp::_XX_ * dmp::POS_VEL_ACC) = left_gripper_frame_.p.x();
            robot_pos_vel_acc_trajectory_point(dmp::_YY_ * dmp::POS_VEL_ACC) = left_gripper_frame_.p.y();
            robot_pos_vel_acc_trajectory_point(dmp::_ZZ_ * dmp::POS_VEL_ACC) = left_gripper_frame_.p.z();
            left_gripper_frame_.M.GetQuaternion(qx, qy, qz, qw);
            robot_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (0 * dmp::POS_VEL_ACC)) = qx;
            robot_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (1 * dmp::POS_VEL_ACC)) = qy;
            robot_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (2 * dmp::POS_VEL_ACC)) = qz;
            robot_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (3 * dmp::POS_VEL_ACC)) = qw;
            break;
        }
        case ChopStickTaskTransform::RIGHT_ARM:
        {
            right_chop_stick_frame_.p.x(chop_x);
            right_chop_stick_frame_.p.y(chop_y);
            right_chop_stick_frame_.p.z(chop_z);
            right_chop_stick_frame_.M = chop_stick_rotation;

            right_gripper_frame_ = right_chop_stick_frame_ * gripper_to_chop_stick_offset_.Inverse();

            robot_pos_vel_acc_trajectory_point(dmp::_XX_ * dmp::POS_VEL_ACC) = right_gripper_frame_.p.x();
            robot_pos_vel_acc_trajectory_point(dmp::_YY_ * dmp::POS_VEL_ACC) = right_gripper_frame_.p.y();
            robot_pos_vel_acc_trajectory_point(dmp::_ZZ_ * dmp::POS_VEL_ACC) = right_gripper_frame_.p.z();
            right_gripper_frame_.M.GetQuaternion(qx, qy, qz, qw);
            robot_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (0 * dmp::POS_VEL_ACC)) = qx;
            robot_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (1 * dmp::POS_VEL_ACC)) = qy;
            robot_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (2 * dmp::POS_VEL_ACC)) = qz;
            robot_pos_vel_acc_trajectory_point((dmp::N_CART * dmp::POS_VEL_ACC) + (3 * dmp::POS_VEL_ACC)) = qw;
            break;
        }
    }

//    KDL::Frame left_bridge_frame = left_gripper * left_gripper_to_bridge_offset_;
//
//    debug_frame_ = left_bridge_frame;
//
//    double offset = task_pos_vel_acc_trajectory_point(0 * dmp::POS_VEL_ACC);
    //    double roll = task_pos_vel_acc_trajectory_point(1 * dmp::POS_VEL_ACC);
    //    double pitch = task_pos_vel_acc_trajectory_point(2 * dmp::POS_VEL_ACC);
    //    double yaw = task_pos_vel_acc_trajectory_point(3 * dmp::POS_VEL_ACC);
//
//    KDL::Frame corrected_transform_frame(KDL::Rotation::RPY(roll, pitch, yaw));
//    corrected_transform_frame.p.z(offset);
//
//    right_gripper_frame_ = left_bridge_frame * corrected_transform_frame.Inverse();
//
//    right_gripper_frame_ = right_gripper_frame_ * right_gripper_to_cue_offset_.Inverse();
//
//    robot_pos_vel_acc_trajectory_point(0 * dmp::POS_VEL_ACC) = right_gripper_frame_.p.x();
//    robot_pos_vel_acc_trajectory_point(1 * dmp::POS_VEL_ACC) = right_gripper_frame_.p.y();
//    robot_pos_vel_acc_trajectory_point(2 * dmp::POS_VEL_ACC) = right_gripper_frame_.p.z();
//
//    double qx, qy, qz, qw;
//    right_gripper_frame_.M.GetQuaternion(qx, qy, qz, qw);
//    robot_pos_vel_acc_trajectory_point(3 * dmp::POS_VEL_ACC) = qx;
//    robot_pos_vel_acc_trajectory_point(4 * dmp::POS_VEL_ACC) = qy;
//    robot_pos_vel_acc_trajectory_point(5 * dmp::POS_VEL_ACC) = qz;
//    robot_pos_vel_acc_trajectory_point(6 * dmp::POS_VEL_ACC) = qw;
//
//    cue_frame_ = right_gripper_frame_ * right_gripper_to_cue_offset_;

    publishMarker();
    return true;
}

void ChopStickTaskTransform::getQuaternionIndices(const WhichArm arm, std::vector<int>& quaternion_indices)
{
    quaternion_indices.clear();
    switch(arm)
    {
        case ChopStickTaskTransform::LEFT_ARM:
        {
            // there are no quaternions contained
            //            quaternion_indices.push_back(dmp::N_CART + dmp::_QX_);
            //            quaternion_indices.push_back(dmp::N_CART + dmp::_QY_);
            //            quaternion_indices.push_back(dmp::N_CART + dmp::_QZ_);
            //            quaternion_indices.push_back(dmp::N_CART + dmp::_QW_);
            break;
        }
        case ChopStickTaskTransform::RIGHT_ARM:
        {
            // there are no quaternions contained
            //            quaternion_indices.push_back(dmp::N_CART + dmp::_QX_);
            //            quaternion_indices.push_back(dmp::N_CART + dmp::_QY_);
            //            quaternion_indices.push_back(dmp::N_CART + dmp::_QZ_);
            //            quaternion_indices.push_back(dmp::N_CART + dmp::_QW_);
            break;
        }
    }
}

void ChopStickTaskTransform::publishMarker()
{
    publishing_counter_++;
    if (publishing_counter_ % publishing_rate_ == 0)
    {
        publishing_counter_ = 0;
        double qx, qy, qz, qw;

//        visualization_msgs::MarkerPtr cue_msg = vis_marker_cue_publisher_->allocate();
//        if (cue_msg)
//        {
//            cue_msg->header.frame_id = std::string("/torso_lift_link");
//            cue_msg->header.stamp = ros::Time::now();
//            cue_msg->ns = "pr2_tasks_transform";
//            cue_msg->type = visualization_msgs::Marker::ARROW;
//            cue_msg->action = visualization_msgs::Marker::ADD;
//            cue_msg->id = 9;
//            cue_msg->scale.x = 0.025;
//            cue_msg->scale.y = 0.025;
//            double offset = 0.12;
//            KDL::Frame cue_root_offset_frame;
//            cue_root_offset_frame.p.z(-offset);
//            KDL::Frame cue_tip_offset_frame;
//            cue_tip_offset_frame.p.z(cue_offset_-offset);
//            KDL::Frame cue_root_frame = right_gripper_frame_ * right_gripper_to_cue_offset_ * cue_root_offset_frame;
//            KDL::Frame cue_tip_frame = right_gripper_frame_ * right_gripper_to_cue_offset_ * cue_tip_offset_frame;
//            cue_msg->points[0].x = cue_root_frame.p.x();
//            cue_msg->points[0].y = cue_root_frame.p.y();
//            cue_msg->points[0].z = cue_root_frame.p.z();
//            cue_msg->points[1].x = cue_tip_frame.p.x();
//            cue_msg->points[1].y = cue_tip_frame.p.y();
//            cue_msg->points[1].z = cue_tip_frame.p.z();
//            cue_msg->color.r = 0.93;
//            cue_msg->color.g = 0.46;
//            cue_msg->color.b = 0.0;
//            cue_msg->color.a = 1.0;
//            vis_marker_cue_publisher_->publish(cue_msg);
//        }
//
//        geometry_msgs::PoseStampedPtr debug_frame_msg = vis_marker_debug_frame_publisher_->allocate();
//        if (debug_frame_msg)
//        {
//            debug_frame_msg->header.frame_id = std::string("/torso_lift_link");
//            // debug_frame_msg->header.frame_id = std::string("/r_gripper_tool_frame");
//            debug_frame_msg->header.stamp = ros::Time::now();
//            debug_frame_msg->pose.position.x = debug_frame_.p.x();
//            debug_frame_msg->pose.position.y = debug_frame_.p.y();
//            debug_frame_msg->pose.position.z = debug_frame_.p.z();
//            debug_frame_.M.GetQuaternion(qx, qy, qz, qw);
//            debug_frame_msg->pose.orientation.x = qx;
//            debug_frame_msg->pose.orientation.y = qy;
//            debug_frame_msg->pose.orientation.z = qz;
//            debug_frame_msg->pose.orientation.w = qw;
//            vis_marker_debug_frame_publisher_->publish(debug_frame_msg);
//        }

        geometry_msgs::PoseStampedPtr left_chop_stick_frame_msg = vis_marker_left_chop_stick_frame_publisher_->allocate();
        if (left_chop_stick_frame_msg)
        {
            left_chop_stick_frame_msg->header.frame_id = std::string("/torso_lift_link");
            // cue_frame_msg->header.frame_id = std::string("/r_gripper_tool_frame");
            left_chop_stick_frame_msg->header.stamp = ros::Time::now();
            left_chop_stick_frame_msg->pose.position.x = left_chop_stick_frame_.p.x();
            left_chop_stick_frame_msg->pose.position.y = left_chop_stick_frame_.p.y();
            left_chop_stick_frame_msg->pose.position.z = left_chop_stick_frame_.p.z();
            left_chop_stick_frame_.M.GetQuaternion(qx, qy, qz, qw);
            left_chop_stick_frame_msg->pose.orientation.x = qx;
            left_chop_stick_frame_msg->pose.orientation.y = qy;
            left_chop_stick_frame_msg->pose.orientation.z = qz;
            left_chop_stick_frame_msg->pose.orientation.w = qw;
            vis_marker_left_chop_stick_frame_publisher_->publish(left_chop_stick_frame_msg);
        }

        geometry_msgs::PoseStampedPtr right_chop_stick_frame_msg = vis_marker_right_chop_stick_frame_publisher_->allocate();
        if (right_chop_stick_frame_msg)
        {
            right_chop_stick_frame_msg->header.frame_id = std::string("/torso_lift_link");
            // cue_frame_msg->header.frame_id = std::string("/r_gripper_tool_frame");
            right_chop_stick_frame_msg->header.stamp = ros::Time::now();
            right_chop_stick_frame_msg->pose.position.x = right_chop_stick_frame_.p.x();
            right_chop_stick_frame_msg->pose.position.y = right_chop_stick_frame_.p.y();
            right_chop_stick_frame_msg->pose.position.z = right_chop_stick_frame_.p.z();
            right_chop_stick_frame_.M.GetQuaternion(qx, qy, qz, qw);
            right_chop_stick_frame_msg->pose.orientation.x = qx;
            right_chop_stick_frame_msg->pose.orientation.y = qy;
            right_chop_stick_frame_msg->pose.orientation.z = qz;
            right_chop_stick_frame_msg->pose.orientation.w = qw;
            vis_marker_right_chop_stick_frame_publisher_->publish(right_chop_stick_frame_msg);
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

        geometry_msgs::PoseStampedPtr left_gripper_frame_msg = vis_marker_left_gripper_frame_publisher_->allocate();
        if (left_gripper_frame_msg)
        {
            left_gripper_frame_msg->header.frame_id = std::string("/torso_lift_link");
            left_gripper_frame_msg->header.stamp = ros::Time::now();
            left_gripper_frame_msg->pose.position.x = left_gripper_frame_.p.x();
            left_gripper_frame_msg->pose.position.y = left_gripper_frame_.p.y();
            left_gripper_frame_msg->pose.position.z = left_gripper_frame_.p.z();
            double qx, qy, qz, qw;
            left_gripper_frame_.M.GetQuaternion(qx, qy, qz, qw);
            left_gripper_frame_msg->pose.orientation.x = qx;
            left_gripper_frame_msg->pose.orientation.y = qy;
            left_gripper_frame_msg->pose.orientation.z = qz;
            left_gripper_frame_msg->pose.orientation.w = qw;
            vis_marker_left_gripper_frame_publisher_->publish(left_gripper_frame_msg);
        }

    }
}

}
