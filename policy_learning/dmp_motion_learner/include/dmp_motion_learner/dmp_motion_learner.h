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

#ifndef DMP_MOTION_LEARNER_NODE_H_
#define DMP_MOTION_LEARNER_NODE_H_

// system includes
#include <vector>

// ros includes
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

//#include <kdl/chain.hpp>
//#include <kdl/frames.hpp>
//#include <kdl/chainfksolver.hpp>

// #include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <pr2_tasks_transforms/task_transforms.h>

#include <dmp_motion_generation/dynamic_movement_primitive.h>
#include <dmp_motion_generation/constants.h>

// local includes
#include <dmp_motion_learner/LearnJointSpaceDMP.h>
#include <dmp_motion_learner/LearnCartesianSpaceDMP.h>
#include <dmp_motion_learner/LearnDualCartesianSpaceDMP.h>

namespace dmp_learner
{

class DMPMotionLearner
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! constructor
     *
     * @return
     */
    DMPMotionLearner(ros::NodeHandle& node_handle);

    /*! destructor
     *
     * @return
     */
    ~DMPMotionLearner();

    /*!
     * @param advertise_service
     * @return
     */
    bool initialize();

    /*!
     * @return
     */
    int run();

    /*!
     * @param request
     * @param response
     * @return
     */
    bool learnJointSpaceDMPFromBagFile(dmp_motion_learner::LearnJointSpaceDMP::Request &request, dmp_motion_learner::LearnJointSpaceDMP::Response &response);

    /*!
     * @param request
     * @param response
     * @return
     */
    bool learnCartesianAndJointSpaceDMPFromBagFile(dmp_motion_learner::LearnCartesianSpaceDMP::Request &request, dmp_motion_learner::LearnCartesianSpaceDMP::Response &response);

    /*!
     * @param request
     * @param response
     * @return
     */
    bool learnDualCartesianAndJointSpaceDMPFromBagFile(dmp_motion_learner::LearnDualCartesianSpaceDMP::Request &request, dmp_motion_learner::LearnDualCartesianSpaceDMP::Response &response);

private:

    /*!
     */
    bool initialized_;

    /*!
     */
    ros::NodeHandle node_handle_;
    ros::NodeHandle trajectory_node_handle_;

    std::string robot_description_;

    /*!
     */
    std::vector<ros::Time> time_stamps_;

    /*!
     */
    ros::ServiceServer learn_joint_space_dmp_from_bag_file_server_;
    ros::ServiceServer learn_cartesian_and_joint_space_dmp_from_bag_file_server_;
    ros::ServiceServer learn_dual_cartesian_and_joint_space_dmp_from_bag_file_server_;

    /*!
     */
//    KDL::Chain left_arm_kdl_chain_;
//    KDL::Chain right_arm_kdl_chain_;
//    KDL::JntArray left_arm_jnt_pos_;
//    KDL::JntArray right_arm_jnt_pos_;
//    boost::shared_ptr<KDL::ChainFkSolverPos> left_arm_jnt_to_pose_solver_;
//    boost::shared_ptr<KDL::ChainFkSolverPos> right_arm_jnt_to_pose_solver_;

    std::vector<std::string> left_arm_joint_names_;
    std::vector<std::string> right_arm_joint_names_;

    /*!
     */
    KDL::Frame gripper_offset_;
    KDL::Frame link_pose_;

    /*!
     */
    ros::Publisher marker_publisher_;
    geometry_msgs::PoseStamped pose_stamped_;

    /*!
     */
    int forearm_roll_angle_offset_;
    int wrist_roll_angle_offset_;

    /*!
     */
    pr2_tasks_transforms::TaskTransforms task_transform_;

    /*!
     * @param root_frame
     * @param tip_frame
     * @param info
     * @return
     */
    bool createJointToPoseSolver();

    bool createJointNames();

    /*!
     * @param bag_file_name
     * @param joint_names
     * @param trajectory
     * @param info
     * @return
     */
    bool getJointTrajectoryFromBagFile(const std::string& bag_file_name, const std::vector<std::string>& joint_names,
                                       dmp::Trajectory& trajectory, std::string& info);
    /*!
    *
    * @param bag_file_name
    * @param joint_names
    * @param joint_states
    * @param info
    * @return
    */
    bool getJointTrajectoryFromBagFile(const std::string& bag_file_name, const std::vector<std::string>& joint_names,
                                       std::vector<sensor_msgs::JointState>& joint_states, std::string& info);

    /*!
     * @param joint_trajectory_point
     * @param cartesian_trajectory_point
     * @param info
     * @return
     */
    bool doForwardKinematics(const Eigen::VectorXd& joint_trajectory_point, Eigen::VectorXd& cartesian_trajectory_point, std::string& info);

    /*!
     * @param joint_trajectory_point
     */
    void computeAngleOffsets(const Eigen::VectorXd& joint_trajectory_point);

    /*!
     * @param joint_trajectory_point
     * @param cartesian_trajectory_point
     * @param cartesian_and_joint_trajectory_point
     */
    void setCartesianAndJointTrajectoryPoint(const Eigen::VectorXd& joint_trajectory_point, const Eigen::VectorXd& cartesian_trajectory_point,
                                             Eigen::VectorXd& cartesian_and_joint_trajectory_point);

    /*!
     * @param cartesian_trajectory_point
     */
    void publishMarker(const Eigen::VectorXd& cartesian_trajectory_point);

};

}

#endif /* DMP_MOTION_LEARNER_NODE_H_ */
