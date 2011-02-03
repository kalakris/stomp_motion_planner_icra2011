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

  \file    default_task_transform.h

  \author  Peter Pastor
  \date    Aug 9, 2010

**********************************************************************/

#ifndef DEFAULTTASKTRANSFORM_H_
#define DEFAULTTASKTRANSFORM_H_

// system includes
#include <boost/shared_ptr.hpp>

// ros includes
#include <ros/ros.h>
#include <rosrt/rosrt.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl/frames.hpp>
#include <kdl/framevel.hpp>
#include <Eigen/Eigen>

#include <dmp_motion_generation/constants.h>

// local includes

namespace pr2_tasks_transforms
{

class DefaultTaskTransform
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum WhichArm
    {
        LEFT_ARM = 1,
        RIGHT_ARM
    };

    DefaultTaskTransform();
    virtual ~DefaultTaskTransform();

    /*!
     * @param node_handle
     * @return
     */
    bool initialize(ros::NodeHandle& node_handle);

    /*!
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
     * @param cartesian_and_joint_pos_vel_acc_trajectory_point
     * @return
     * REAL-TIME REQUIREMENTS
     */
    bool getTaskTransformFromCurrent(const KDL::Frame& frame,
                                     const Eigen::VectorXd& joint_positions,
                                     Eigen::VectorXd& cartesian_and_joint_pos_vel_acc_trajectory_point);

    /*!
     * @param arm
     * @param quaternion_indices
     */
    void getQuaternionIndices(const WhichArm arm, std::vector<int>& quaternion_indices);

    // bool getRobotFrame(const Eigen::VectorXd& task_frame, const KDL::Frame& left_bridge_gripper, KDL::FrameVel& right_cue_gripper);

    /*!
     * @return
     */
    std::vector<int> getNumDimensions();

private:

    bool initialized_;

    ros::NodeHandle node_handle_;

    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > vis_marker_cue_frame_publisher_;
    boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > vis_marker_cue_publisher_;

    int publishing_rate_;
    int publishing_counter_;

//    /*!
//     * @return
//     */
//    bool initRTPublisher();
//
//    /*!
//     * @return
//     */
//    bool readParams();
//
//    /*!
//     */
//    void publishMarker();

};

inline std::vector<int> DefaultTaskTransform::getNumDimensions()
{
    std::vector<int> dimensions;
    dimensions.push_back(dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS);
    dimensions.push_back(dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS);
    return dimensions;
}

}

#endif /* DEFAULTTASKTRANSFORM_H_ */
