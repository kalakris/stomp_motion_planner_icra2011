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

#include <policy_improvement_utilities/kdl_chain_wrapper.h>
#include <policy_improvement_utilities/assert.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/assert.h>
#include <map>

namespace policy_improvement_utilities
{

KDLChainWrapper::KDLChainWrapper()
{
}

KDLChainWrapper::~KDLChainWrapper()
{
}

int KDLChainWrapper::getNumJoints()
{
    ROS_ASSERT(initialized_);
    return num_joints_;
}

bool KDLChainWrapper::initialize(ros::NodeHandle& node_handle, const std::string& root_frame, const std::string& tip_frame)
{
    node_handle_ = node_handle;
    root_frame_ = root_frame;
    tip_frame_ = tip_frame;

    // get URDF
    std::string robot_description;
    if(!node_handle.getParam("/robot_description", robot_description))
    {
        ROS_ERROR("Could not retrieve >>robot_description<< from paramserver");
        return false;
    }

    // create kdl tree
    if (!kdl_parser::treeFromString(robot_description, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree.");
        return false;
    }

    // create the chain given the tree
    if (!kdl_tree_.getChain(root_frame_, tip_frame_, kdl_chain_))
    {
        ROS_ERROR("Failed to get kdl chain from %s to %s.", root_frame_.c_str(), tip_frame_.c_str());
        return false;
    }
    num_joints_ = kdl_chain_.getNrOfJoints();

    // create the joint to pose solver
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jnt_to_pose_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));

    // create urdf name to kdl joint index mapping and vice versa:
    createJointNameMappings();

    return (initialized_ = true);
}


void KDLChainWrapper::createJointNameMappings()
{
    joint_number_to_name_.clear();
    joint_name_to_number_.clear();

    int joint_number=0;
    for (int i=0; i<int(kdl_chain_.segments.size()); ++i)
    {
        const KDL::Segment& segment = kdl_chain_.segments[i];
        const KDL::Joint& joint = segment.getJoint();
        if (joint.getType() != KDL::Joint::None)
        {
            std::string name = segment.getJoint().getName();
            joint_number_to_name_.push_back(name);
            joint_name_to_number_.insert(std::make_pair(name, joint_number));
            ++joint_number;
        }
    }
}

bool KDLChainWrapper::forwardKinematicsVel(const sensor_msgs::JointState& joint_state, KDL::FrameVel& frame_vel)
{
    ROS_ASSERT(initialized_);

    KDL::JntArrayVel jnt_array_vel(num_joints_);
    ROS_ASSERT_FUNC(jointStateMsgToJntArrayVel(joint_state, jnt_array_vel));
    jnt_to_pose_vel_solver_->JntToCart(jnt_array_vel, frame_vel);
//    ROS_INFO_STREAM("in = " << jnt_array.q.data << ". out = "
//                    << frame_vel.p.p.data[0] << ","
//                    << frame_vel.p.p.data[1] << ","
//                    << frame_vel.p.p.data[2]);
    return true;
}

bool KDLChainWrapper::forwardKinematicsVel(const KDL::JntArrayVel& jnt_array_vel, KDL::FrameVel& frame_vel)
{
    ROS_ASSERT(initialized_);
    jnt_to_pose_vel_solver_->JntToCart(jnt_array_vel, frame_vel);
    return true;
}

bool KDLChainWrapper::forwardKinematics(const sensor_msgs::JointState& joint_state, KDL::Frame& frame)
{
    ROS_ASSERT(initialized_);

    KDL::JntArray jnt_array(num_joints_);
    ROS_ASSERT_FUNC(jointStateMsgToJntArray(joint_state, jnt_array));
    jnt_to_pose_solver_->JntToCart(jnt_array, frame);

//    ROS_INFO_STREAM("in = " << jnt_array.data << ". out = "
//                    << frame.p.data[0] << ","
//                    << frame.p.data[1] << ","
//                    << frame.p.data[2]);
    return true;
}

bool KDLChainWrapper::forwardKinematics(const KDL::JntArray& jnt_array, KDL::Frame& frame)
{
    ROS_ASSERT(initialized_);
    jnt_to_pose_solver_->JntToCart(jnt_array, frame);
//    ROS_INFO_STREAM("in = " << jnt_array.data << ". out = "
//                    << frame.p.data[0] << ","
//                    << frame.p.data[1] << ","
//                    << frame.p.data[2]);
    return true;
}

bool KDLChainWrapper::jointStateMsgToJntArrayVel(const sensor_msgs::JointState& joint_state, KDL::JntArrayVel& jnt_array_vel) const
{
    ROS_ASSERT(initialized_);

    unsigned int joints_found = 0;

    // loop through the joint state message and fill up the jnt_array_vel
    for (int i=0; i<int(joint_state.name.size()); ++i)
    {
        std::map<std::string, int>::const_iterator it = joint_name_to_number_.find(joint_state.name[i]);
        if (it==joint_name_to_number_.end())
            continue;
        ++joints_found;
        int index = it->second;
        jnt_array_vel.q(index) = joint_state.position[i];
        jnt_array_vel.qdot(index) = joint_state.velocity[i];
    }
    if (joints_found!=jnt_array_vel.q.rows())
    {
        ROS_WARN("KDLChainWrapper::jointStateMsgToJntArrayVel - only %d out of %d joints were assigned", joints_found, jnt_array_vel.q.rows());
    }
    return true;
}

bool KDLChainWrapper::jointStateMsgToJntArray(const sensor_msgs::JointState& joint_state, KDL::JntArray& jnt_array) const
{
    ROS_ASSERT(initialized_);

    unsigned int joints_found = 0;

    // loop through the joint state message and fill up the jnt_array_vel
    for (int i=0; i<int(joint_state.name.size()); ++i)
    {
        std::map<std::string, int>::const_iterator it = joint_name_to_number_.find(joint_state.name[i]);
        if (it==joint_name_to_number_.end())
            continue;
        ++joints_found;
        int index = it->second;
        jnt_array(index) = joint_state.position[i];
    }
    if (joints_found!=jnt_array.rows())
    {
        ROS_WARN("KDLChainWrapper::jointStateMsgToJntArray - only %d out of %d joints were assigned", joints_found, jnt_array.rows());
    }
    return true;
}

void KDLChainWrapper::getJointNames(std::vector<std::string>& joint_names)
{
    joint_names = joint_number_to_name_;
}

}
