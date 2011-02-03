/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#ifndef POLICY_IMPROVEMENT_UTILITIES_KDL_CHAIN_WRAPPER_H_
#define POLICY_IMPROVEMENT_UTILITIES_KDL_CHAIN_WRAPPER_H_

// system includes
#include <boost/shared_ptr.hpp>

// ros includes
#include <ros/ros.h>

#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <sensor_msgs/JointState.h>

// local includes

namespace policy_improvement_utilities
{

class KDLChainWrapper
{
public:
    KDLChainWrapper();
    virtual ~KDLChainWrapper();

    /**
     * Initialize the KDL Chain wrapper
     * @param node_handle
     * @param root_frame
     * @param tip_frame
     * @return
     */
    bool initialize(ros::NodeHandle& node_handle, const std::string& root_frame, const std::string& tip_frame);

    /**
     * Gets the number of joints in the chain
     * @return
     */
    int getNumJoints();

    /**
     * Perform forward kinematics (w/ velocities) for an input JointState message
     * @param joint_state
     * @param frame_vel
     * @return
     */
    bool forwardKinematicsVel(const sensor_msgs::JointState& joint_state, KDL::FrameVel& frame_vel);

    /**
     * Perform forward kinematics (w/ velocities) for an input JntArrayVel
     * @param jnt_array_vel
     * @param frame_vel
     * @return
     */
    bool forwardKinematicsVel(const KDL::JntArrayVel& jnt_array_vel, KDL::FrameVel& frame_vel);

    /**
     * Perform forward kinematics for an input JointState message
     * @param joint_state
     * @param frame_vel
     * @return
     */
    bool forwardKinematics(const sensor_msgs::JointState& joint_state, KDL::Frame& frame);

    /**
     * Perform forward kinematics for an input KDL JntArray
     * @param jnt_array
     * @param frame
     * @return
     */
    bool forwardKinematics(const KDL::JntArray& jnt_array, KDL::Frame& frame);

    /**
     * Converts a sensor_msgs::JointState ROS message into a KDL::JntArrayVel object
     * @param joint_state
     * @param jnt_array_vel
     * @return
     */
    bool jointStateMsgToJntArrayVel(const sensor_msgs::JointState& joint_state, KDL::JntArrayVel& jnt_array_vel) const;

    /**
     * Converts a sensor_msgs::JointState ROS message into a KDL::JntArray object
     * @param joint_state
     * @param jnt_array_vel
     * @return
     */
    bool jointStateMsgToJntArray(const sensor_msgs::JointState& joint_state, KDL::JntArray& jnt_array) const;

    void getJointNames(std::vector<std::string>& joint_names);

private:

    void createJointNameMappings();

    bool initialized_;
    std::string tip_frame_;
    std::string root_frame_;
    ros::NodeHandle node_handle_;

    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    boost::shared_ptr<KDL::ChainFkSolverVel> jnt_to_pose_vel_solver_;
    boost::shared_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;

    std::map<std::string, int> joint_name_to_number_;
    std::vector<std::string> joint_number_to_name_;
    int num_joints_;

};

}

#endif /* POLICY_IMPROVEMENT_UTILITIES_KDL_CHAIN_WRAPPER_H_ */
