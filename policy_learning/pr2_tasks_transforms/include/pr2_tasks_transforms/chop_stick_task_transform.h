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

  \file    PoolTaskTransform.h

  \author  Peter Pastor
  \date    Aug 7, 2010

**********************************************************************/

#ifndef CHOPSTICKTASKTRANSFORM_H_
#define CHOPSTICKTASKTRANSFORM_H_

// system includes
#include <boost/shared_ptr.hpp>
#include <vector>

// ros includes
#include <ros/ros.h>

#include <rosrt/rosrt.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <Eigen/Eigen>


// local includes

namespace pr2_tasks_transforms
{

const int N_CHOP_STICK_DOF = dmp::N_CART + dmp::N_CART + dmp::N_JOINTS;

class ChopStickTaskTransform
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum WhichArm
    {
        LEFT_ARM = 1,
        RIGHT_ARM
    };

    ChopStickTaskTransform();
    virtual ~ChopStickTaskTransform();

    /*!
     * @param node_handle
     * @return
     */
    bool initialize(ros::NodeHandle& node_handle);

    /*!
    *
    * @param arm
    * @param first_transform
    * @param joint_pos_vel_acc_trajectory_point
    * @param cartesian_pos_vel_acc_trajectory_point
    * @param cartesian_and_joint_pos_vel_acc_trajectory_point
    * @return
    */
    bool getTaskTransform(const WhichArm arm,
                          bool first_transform,
                          const Eigen::VectorXd& joint_pos_vel_acc_trajectory_point,
                          const Eigen::VectorXd& cartesian_pos_vel_acc_trajectory_point,
                          Eigen::VectorXd& cartesian_and_joint_pos_vel_acc_trajectory_point);


    /*!
    *
    * @param frame
    * @param joint_positions
    * @param task_pos_vel_acc_values
    * REAL-TIME REQUIREMENT
    * @return
    */
    bool getTaskTransformFromCurrent(const KDL::Frame& frame,
                                     const Eigen::VectorXd& joint_positions,
                                     Eigen::VectorXd& task_pos_vel_acc_values);

    /*!
     *
     * @param arm
     * @param gripper_frame
     * @param task_pos_vel_acc_trajectory_point
     * @param robot_pos_vel_acc_trajectory_point
     * REAL-TIME REQUIREMENT
     * @return
     */
    bool getRobotTransform(const WhichArm arm,
                           const Eigen::VectorXd& task_pos_vel_acc_trajectory_point,
                           Eigen::VectorXd& robot_pos_vel_acc_trajectory_point);

    /*!
     * @return
     */
    std::vector<int> getNumDimensions();

    /*!
     * @param arm
     * @param quaternion_indices
     */
    void getQuaternionIndices(const WhichArm arm, std::vector<int>& quaternion_indices);

private:

    bool initialized_;

    ros::NodeHandle node_handle_;

    KDL::Frame gripper_to_chop_stick_offset_;

    KDL::Frame left_gripper_debug_frame_;
    KDL::Frame right_gripper_debug_frame_;
    KDL::Frame left_chop_stick_frame_;
    KDL::Frame right_chop_stick_frame_;
    KDL::Frame left_gripper_frame_;
    KDL::Frame right_gripper_frame_;

    Eigen::VectorXd task_pos_vel_acc_trajectory_point_;

    double chop_stick_offset_;

    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > vis_marker_left_gripper_debug_frame_publisher_;
    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > vis_marker_left_chop_stick_frame_publisher_;
    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > vis_marker_left_gripper_frame_publisher_;
    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > vis_marker_right_gripper_debug_frame_publisher_;
    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > vis_marker_right_chop_stick_frame_publisher_;
    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > vis_marker_right_gripper_frame_publisher_;

    boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > vis_marker_left_chop_stick_publisher_;
    boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > vis_marker_right_chop_stick_publisher_;

    int publishing_rate_;
    int publishing_counter_;

    /*!
     * @return
     */
    bool initRTPublisher();

    /*!
     * @return
     */
    bool readParams();

    /*!
     *
     * @param right_gripper
     * @param left_gripper
     * @param arm_joint_positions
     * @param task_pos_vel_acc_trajectory_point
     * @return
     */
    bool getArmTaskTransform(const KDL::Frame& right_gripper,
                             const KDL::Frame& left_gripper,
                             const Eigen::VectorXd& arm_joint_positions,
                             Eigen::VectorXd& task_pos_vel_acc_trajectory_point);

    /*!
     */
    void publishMarker();

};

inline std::vector<int> ChopStickTaskTransform::getNumDimensions()
{
    std::vector<int> dimensions;
    // left arm
    dimensions.push_back(N_CHOP_STICK_DOF);
    // right arm
    dimensions.push_back(N_CHOP_STICK_DOF);
    return dimensions;
}

}


#endif /* CHOPSTICKTASKTRANSFORM_H_ */
