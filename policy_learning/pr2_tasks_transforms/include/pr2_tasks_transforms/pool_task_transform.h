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

#ifndef POOLTASKTRANSFORM_H_
#define POOLTASKTRANSFORM_H_

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

const int N_POOL_DOF = 4;

class PoolTaskTransform
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum WhichArm
    {
        LEFT_ARM = 1,
        RIGHT_ARM
    };

    PoolTaskTransform();
    virtual ~PoolTaskTransform();

    /*!
     * @param node_handle
     * @return
     */
    bool initialize(ros::NodeHandle& node_handle);

    /*!
     * @param arm
     * @param first_transform
     * @param right_cue_gripper
     * @param left_bridge_gripper
     * @param right_cue_gripper_pos_vel_acc_trajectory_point
     * @return
     */
    bool getTaskTransform(const WhichArm arm, bool first_transform,
                          const KDL::Frame& right_gripper,
                          const KDL::Frame& left_gripper,
                          const Eigen::VectorXd& arm_joint_positions,
                          Eigen::VectorXd& task_pos_vel_acc_trajectory_point);

    /*!
     * @param arm
     * @param first_transform
     * @param right_cue_gripper
     * @param left_bridge_gripper
     * @param right_cue_gripper_pos_vel_acc_trajectory_point
     * REAL-TIME REQUIREMENT
     * @return
     */
    bool getTaskTransformFromCurrent(const WhichArm arm,
                                     const KDL::Frame& right_gripper,
                                     const KDL::Frame& left_gripper,
                                     const Eigen::VectorXd& arm_joint_positions,
                                     Eigen::VectorXd& task_pos_trajectory_point);

    /*!
    *
    * @param arm
    * @param right_cue_gripper
    * @param left_bridge_gripper
    * @param task_pos_vel_acc_trajectory_point
    * @return
    * REAL-TIME REQUIREMENT
    */
    bool getRobotTransform(const KDL::Frame& left_gripper,
                           const Eigen::VectorXd& task_pos_vel_acc_trajectory_point,
                           Eigen::VectorXd& robot_pos_vel_acc_trajectory_point);

    /*!
     * @param cue_frame
     * @param left_bridge_gripper
     * @param right_cue_gripper
     * @return
     * REAL-TIME REQUIREMENT
     */
//    bool convertToRightGripperFrame(const Eigen::VectorXd& cue_frame, bool first_transform,
//                                    const KDL::Frame& left_bridge_gripper,
//                                    KDL::FrameVel& right_cue_gripper);

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

    KDL::Frame right_gripper_to_cue_offset_;
    KDL::Frame left_gripper_to_bridge_offset_;

    KDL::Frame debug_frame_;
    KDL::Frame cue_frame_;
    KDL::Frame right_gripper_frame_;

    Eigen::VectorXd left_task_pos_vel_acc_trajectory_point_;
    Eigen::VectorXd right_task_pos_vel_acc_trajectory_point_;

    double cue_offset_;

    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > vis_marker_cue_frame_publisher_;
    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > vis_marker_right_gripper_frame_publisher_;
    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > vis_marker_debug_frame_publisher_;

    boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > vis_marker_cue_publisher_;

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
    bool getRightArmTaskTransform(const KDL::Frame& right_gripper,
                                  const KDL::Frame& left_gripper,
                                  const Eigen::VectorXd& arm_joint_positions,
                                  Eigen::VectorXd& task_pos_vel_acc_trajectory_point);

    /*!
     *
     * @param right_gripper
     * @param left_gripper
     * @param arm_joint_positions
     * @param task_pos_vel_acc_trajectory_point
     * @return
     */
    bool getLeftArmTaskTransform(const KDL::Frame& right_gripper,
                                 const KDL::Frame& left_gripper,
                                 const Eigen::VectorXd& arm_joint_positions,
                                 Eigen::VectorXd& task_pos_vel_acc_trajectory_point);

//    /*! computes the tip location of the cue stick given left and right tool frame.
//     *
//     * @param right_gripper
//     * @param left_gripper
//     * @param cue_tip_position
//     * @return
//     */
//    bool computeCueTipLocation(const KDL::Frame& right_gripper, const KDL::Frame& left_gripper, KDL::Vector& cue_tip_position);

    /*!
     */
    void publishMarker();

};

inline std::vector<int> PoolTaskTransform::getNumDimensions()
{
    std::vector<int> dimensions;
    // left arm
    dimensions.push_back(dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS);
    // right arm
    dimensions.push_back(N_POOL_DOF  + dmp::N_JOINTS);
    return dimensions;
}

}


#endif /* POOLTASKTRANSFORM_H_ */
