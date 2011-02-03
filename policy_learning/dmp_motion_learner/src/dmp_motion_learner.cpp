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

// system includes
#include <math.h>

// ros includes
//#include <kdl/frames.hpp>
//#include <kdl_parser/kdl_parser.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <Eigen/Eigen>
// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

#include <angles/angles.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>

#include <dmp_motion_generation/parameters.h>
#include <dmp_motion_generation/math_helper.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

// local includes
#include <dmp_motion_learner/dmp_motion_learner.h>

using namespace pr2_tasks_transforms;

namespace dmp_learner
{

const std::string root_frame = std::string("torso_lift_link");
const std::string left_arm_tip_frame = std::string("l_gripper_tool_frame");
const std::string right_arm_tip_frame = std::string("r_gripper_tool_frame");

DMPMotionLearner::DMPMotionLearner(ros::NodeHandle& node_handle) :
    initialized_(false), node_handle_(node_handle), trajectory_node_handle_(node_handle, "trajectory")
{
}

DMPMotionLearner::~DMPMotionLearner()
{
}

bool DMPMotionLearner::initialize()
{

    learn_joint_space_dmp_from_bag_file_server_ = node_handle_.advertiseService("learn_joint_space_dmp_from_bag_file",
                                                                                &DMPMotionLearner::learnJointSpaceDMPFromBagFile, this);
    learn_cartesian_and_joint_space_dmp_from_bag_file_server_ = node_handle_.advertiseService("learn_cartesian_and_joint_space_dmp_from_bag_file",
                                                                                              &DMPMotionLearner::learnCartesianAndJointSpaceDMPFromBagFile,
                                                                                              this);

    learn_dual_cartesian_and_joint_space_dmp_from_bag_file_server_ = node_handle_.advertiseService("learn_dual_cartesian_and_joint_space_dmp_from_bag_file",
                                                                                              &DMPMotionLearner::learnDualCartesianAndJointSpaceDMPFromBagFile,
                                                                                              this);

    marker_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped> ("dmp_learn_pose", 100);

    ros::NodeHandle node_handle;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("robot_description"), robot_description_));

    // ROS_ASSERT_FUNC(createJointToPoseSolver());
    ROS_ASSERT_FUNC(createJointNames());

    ROS_ASSERT_FUNC(task_transform_.initialize(node_handle_));

    return (initialized_ = true);
}

int DMPMotionLearner::run()
{
    ros::spin();
    return 0;
}

bool DMPMotionLearner::learnJointSpaceDMPFromBagFile(dmp_motion_learner::LearnJointSpaceDMP::Request &request,
                                                     dmp_motion_learner::LearnJointSpaceDMP::Response &response)
{

    dmp::Trajectory joint_trajectory(node_handle_);
    if (!joint_trajectory.initialize(request.joint_names, dmp::DEFAULT_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Could not initialize trajectory.");
        response.info.assign(std::string("Could not initialize trajectory."));
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    if(!getJointTrajectoryFromBagFile(request.bag_file_name, request.joint_names, joint_trajectory, response.info))
    {
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    if (!joint_trajectory.writeToCLMCFile(request.data_directory_name + std::string("/joint_demo.traj")))
    {
        ROS_ERROR("Could not write joint trajectory.");
        response.info.assign(std::string("Could not write joint trajectory."));
        response.return_code = dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    // if (!dmp_trajectory.correctTimings(time_stamps_))
    // {
    //   ROS_ERROR("Could not correct timings in trajectory.");
    //   return false;
    // }
    // double sampling_frequency = dmp_trajectory.getSamplingFrequency();

    //	dmp_trajectory.rescalePositionAndComputeDerivativesAndCutout(NUM_RESCALING_TRAJECTORY_POINTS, static_cast<int>(ADDITIONAL_TIME_IN_SEC_FOR_SMOOTHING * sampling_frequency));
    //	if (!dmp_trajectory.writeToCLMCFile(std::string(data_directory_name_ + "d00002")))
    //	{
    //		ROS_ERROR("Could not write rescaled trajectory.");
    //		return false;
    //	}


    if (!joint_trajectory.resample(time_stamps_, dmp::ADDITIONAL_TIME_IN_SEC_FOR_SMOOTHING))
    {
        ROS_ERROR("Could not resample trajectory.");
        response.info.assign(std::string("Could not resample trajectory."));
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    if (!joint_trajectory.setSamplingFrequency(dmp::DEFAULT_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Could not set default sampling frequency.");
        response.info.assign(std::string("Could not set default sampling frequency."));
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    if (!joint_trajectory.writeToCLMCFile(request.data_directory_name + std::string("/joint_demo_rescaled.traj")))
    {
        ROS_ERROR("Could not write rescaled trajectory.");
        response.info.assign(std::string("Could not write rescaled trajectory."));
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    dmp::DynamicMovementPrimitive dmp(node_handle_);
    if (!dmp.initialize(static_cast<int>(request.joint_names.size()), request.dmp_id))
    {
        ROS_ERROR("Could not initialize dmp with id %i.", request.dmp_id);
        response.info.assign(std::string("Could not initialize dmp ") + dmp::MathHelper::getString(request.dmp_id) + ".");
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    if (!dmp.learnFromTrajectory(joint_trajectory))
    {
        ROS_ERROR("Learning dmp from trajectory was not successful.");
        response.info.assign(std::string("Could not learn dmp from trajectory."));
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    // set dmp info
    dmp.setDescription(std::string("Joint space DMP learned from ") + request.bag_file_name + std::string("."));

    dmp_motion_generation::DynamicMovementPrimitive dmp_msg;
    if (!dmp.writeToMessage(dmp_msg))
    {
        ROS_ERROR("Could not create DMP message.");
        response.info.assign(std::string("Could not create DMP message."));
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    ROS_INFO("Successful learned joint space DMP with id %i.", request.dmp_id);
    response.dmp = dmp_msg;
    response.info.assign(std::string("Successful learned joint space DMP with id ") + dmp::MathHelper::getString(request.dmp_id) + std::string("."));
    response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
}

bool DMPMotionLearner::learnCartesianAndJointSpaceDMPFromBagFile(dmp_motion_learner::LearnCartesianSpaceDMP::Request &request,
                                                                 dmp_motion_learner::LearnCartesianSpaceDMP::Response &response)
{

    std::string variable_names_key_word;
    if((request.type == dmp_motion_learner::LearnCartesianSpaceDMP::Request::LEFT_ARM_NO_TRANSFORM)
            || (request.type == dmp_motion_learner::LearnCartesianSpaceDMP::Request::LEFT_ARM_POOL_TRANSFORM))
    {
        variable_names_key_word.assign("cart_and_joint_left_arm");
    }
    else if(request.type == dmp_motion_learner::LearnCartesianSpaceDMP::Request::RIGHT_ARM_NO_TRANSFORM)
    {
        variable_names_key_word.assign("cart_and_joint_right_arm");
    }
    else if(request.type == dmp_motion_learner::LearnCartesianSpaceDMP::Request::RIGHT_ARM_POOL_TRANSFORM)
    {
        variable_names_key_word.assign("pool_and_joint_right_arm");
    }
    else if(request.type == dmp_motion_learner::LearnCartesianSpaceDMP::Request::RIGHT_ARM_CHOP_STICK_TRANSFORM)
    {
        variable_names_key_word.assign("chop_stick_and_joint_right_arm");
    }
    else if(request.type == dmp_motion_learner::LearnCartesianSpaceDMP::Request::LEFT_ARM_CHOP_STICK_TRANSFORM)
    {
        variable_names_key_word.assign("chop_stick_and_joint_left_arm");
    }
    else
    {
        ROS_ERROR("Invalid type (%i)", request.type);
        response.info.assign(std::string("Invalid type specified."));
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    // create left and right arm joint trajectories
    dmp::Trajectory left_arm_joint_trajectory(trajectory_node_handle_);
    if (!left_arm_joint_trajectory.initialize(left_arm_joint_names_, dmp::DEFAULT_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Could not initialize left arm joint trajectory.");
        response.info.assign(std::string("Could not initialize left arm joint trajectory."));
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }
    dmp::Trajectory right_arm_joint_trajectory(trajectory_node_handle_);
    if (!right_arm_joint_trajectory.initialize(right_arm_joint_names_, dmp::DEFAULT_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Could not initialize right arm joint trajectory.");
        response.info.assign(std::string("Could not initialize right arm joint trajectory."));
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    // fill left and right arm joint trajectories from bag file
    if(!getJointTrajectoryFromBagFile(request.bag_file_name, left_arm_joint_names_, left_arm_joint_trajectory, response.info))
    {
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }
    if(!getJointTrajectoryFromBagFile(request.bag_file_name, right_arm_joint_names_, right_arm_joint_trajectory, response.info))
    {
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }
    ROS_ASSERT(left_arm_joint_trajectory.getLength() == right_arm_joint_trajectory.getLength());
    ROS_ASSERT(left_arm_joint_names_.size() == right_arm_joint_names_.size());
    int trajectory_length = left_arm_joint_trajectory.getLength();

    // write to file for debugging
//    if (!left_arm_joint_trajectory.writeToCLMCFile(request.data_directory_name + std::string("/left_arm_joint_demo.traj")))
//    {
//        ROS_ERROR("Could not write left arm joint trajectory.");
//        response.info.assign(std::string("Could not write left arm joint trajectory."));
//        response.return_code = dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_FAILED;
//        return true;
//    }
//    if (!right_arm_joint_trajectory.writeToCLMCFile(request.data_directory_name + std::string("/right_arm_joint_demo.traj")))
//    {
//        ROS_ERROR("Could not write right arm joint trajectory.");
//        response.info.assign(std::string("Could not write right arm joint trajectory."));
//        response.return_code = dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_FAILED;
//        return true;
//    }

    std::vector<std::string> dmp_task_variable_names;
    TaskTransforms::TransformType transform_type = static_cast<TaskTransforms::TransformType>(request.type);
    ROS_ASSERT_FUNC(task_transform_.getTaskVariableDescriptions(transform_type, dmp_task_variable_names));

//    for(int i=0; i<(int)dmp_task_variable_names.size(); ++i)
//    {
//        ROS_INFO("dmp_task_variable_names[%i] = %s", i, dmp_task_variable_names[i].c_str());
//    }

    const int num_joints = static_cast<int>(left_arm_joint_names_.size());

    const int num_variables = static_cast<int>(dmp_task_variable_names.size());
    ROS_ASSERT(num_variables % dmp::POS_VEL_ACC == 0);
    const int num_dimensions = num_variables / dmp::POS_VEL_ACC;

    VectorXd left_arm_joint_pos_vel_acc_trajectory_point = VectorXd::Zero(num_joints * dmp::POS_VEL_ACC);
    VectorXd right_arm_joint_pos_vel_acc_trajectory_point = VectorXd::Zero(num_joints * dmp::POS_VEL_ACC);
    VectorXd dmp_pos_vel_acc_trajectory_point = VectorXd::Zero(num_dimensions * dmp::POS_VEL_ACC);

    dmp::Trajectory dmp_trajectory(trajectory_node_handle_);
    ROS_ASSERT_FUNC(dmp_trajectory.initialize(dmp_task_variable_names, dmp::DEFAULT_SAMPLING_FREQUENCY, true));

//    ROS_INFO("transform_type = %i", (int)transform_type);
//    ROS_INFO("num_dimensions = %i", num_dimensions);

    bool first_transform = true;
    for (int i=0; i < trajectory_length; ++i)
    {
        ROS_ASSERT_FUNC(left_arm_joint_trajectory.getTrajectoryPoint(i, left_arm_joint_pos_vel_acc_trajectory_point));
        ROS_ASSERT_FUNC(right_arm_joint_trajectory.getTrajectoryPoint(i, right_arm_joint_pos_vel_acc_trajectory_point));

        // get the task related transform
        ROS_ASSERT_FUNC(task_transform_.getTaskTransform(transform_type, first_transform,
                                                         left_arm_joint_pos_vel_acc_trajectory_point,
                                                         right_arm_joint_pos_vel_acc_trajectory_point,
                                                         dmp_pos_vel_acc_trajectory_point));
        first_transform = false;

//        ros::spinOnce();
//        ros::Duration(0.0002).sleep();

        // add trajectory point
        ROS_ASSERT_FUNC(dmp_trajectory.add(dmp_pos_vel_acc_trajectory_point));
    }

    // write the trajectory to file for debugging
    // ROS_ASSERT_FUNC(dmp_trajectory.writeToCLMCFile(request.data_directory_name + std::string("/dmp_trajectory_demo.traj")));

    // trajectory contains a quaternion and needs to be normalized when resampling...
    std::vector<int> quaternion_indices;
    ROS_ASSERT_FUNC(task_transform_.getQuaternionIndices(transform_type, quaternion_indices));

    // resample the trajectory
    if (!dmp_trajectory.resample(time_stamps_, dmp::ADDITIONAL_TIME_IN_SEC_FOR_SMOOTHING, quaternion_indices))
    {
        ROS_ERROR("Could not resample trajectory.");
        response.info.assign(std::string("Could not resample trajectory."));
        response.return_code = dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    // write the trajectory to file for debugging
//    if (!dmp_trajectory.writeToCLMCFile(request.data_directory_name + std::string("/dmp_trajectory_demo_rescaled.traj")))
//    {
//        ROS_ERROR("Could not write rescaled trajectory.");
//        response.info.assign(std::string("Could not write rescaled trajectory."));
//        return true;
//    }

    dmp::DynamicMovementPrimitive dmp(node_handle_);
    if (!dmp.initialize(num_dimensions, request.dmp_id))
    {
        ROS_ERROR("Could not initialize dmp.");
        response.info.assign(std::string("Could not initialize dmp."));
        response.return_code = dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    // ROS_INFO("Start learning dmp from trajectory...");
    if (!dmp.learnFromTrajectory(dmp_trajectory))
    {
        ROS_ERROR("Learning dmp from trajectory was not successful.");
        response.info.assign(std::string("Could not learn dmp from trajectory."));
        response.return_code = dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    // set dmp info
    dmp.setDescription(std::string("DMP learned from ") + request.bag_file_name + std::string("."));

    dmp_motion_generation::DynamicMovementPrimitive dmp_msg;
    if (!dmp.writeToMessage(dmp_msg))
    {
        ROS_ERROR("Could not create DMP message.");
        response.info.assign(std::string("Could not create DMP message."));
        response.return_code = dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_FAILED;
        return true;
    }

    ROS_INFO("Successfully learned cartesian and joint space DMP with id %i.", dmp.getID());
    response.dmp = dmp_msg;
    response.info.assign(std::string("Successfully learned DMP with id .") + dmp::MathHelper::getString(dmp.getID()));
    response.return_code = dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
}

bool DMPMotionLearner::learnDualCartesianAndJointSpaceDMPFromBagFile(dmp_motion_learner::LearnDualCartesianSpaceDMP::Request &request,
                                                                     dmp_motion_learner::LearnDualCartesianSpaceDMP::Response &response)
{

    dmp_motion_learner::LearnCartesianSpaceDMP::Request single_arm_request;
    dmp_motion_learner::LearnCartesianSpaceDMP::Response single_arm_response;

    single_arm_request.bag_file_name = request.bag_file_name;
    single_arm_request.dmp_id = request.dmp_id;
    single_arm_request.data_directory_name = request.data_directory_name;
    single_arm_request.type = request.left_arm_type;

    ROS_ASSERT_FUNC(learnCartesianAndJointSpaceDMPFromBagFile(single_arm_request, single_arm_response));
    ROS_ASSERT(single_arm_response.return_code == dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL);

    dmp_motion_generation::DynamicMovementPrimitive left_arm_dmp = single_arm_response.dmp;

    single_arm_request.type = request.right_arm_type;

    ROS_ASSERT_FUNC(learnCartesianAndJointSpaceDMPFromBagFile(single_arm_request, single_arm_response));
    ROS_ASSERT(single_arm_response.return_code == dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL);

    dmp_motion_generation::DynamicMovementPrimitive dual_arm_dmp = single_arm_response.dmp;
    dual_arm_dmp.transformation_systems.insert(dual_arm_dmp.transformation_systems.end(),
                                                left_arm_dmp.transformation_systems.begin(),
                                                left_arm_dmp.transformation_systems.end());

    response.return_code = dmp_motion_learner::LearnDualCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL;
    response.dmp = dual_arm_dmp;
    return true;
}

//bool DMPMotionLearner::createJointToPoseSolver()
//{
//    KDL::Tree kdl_tree;
//    if (!kdl_parser::treeFromString(robot_description_, kdl_tree))
//    {
//        ROS_ERROR("Failed to construct kdl tree.");
//        return false;
//    }
//
//    ROS_INFO("Getting chain from %s to %s.", root_frame.c_str(), left_arm_tip_frame.c_str());
//    // create the tree given the tree
//    if (!kdl_tree.getChain(root_frame, left_arm_tip_frame, left_arm_kdl_chain_))
//    {
//        ROS_ERROR("Failed to get kdl chain from %s to %s.", root_frame.c_str(), left_arm_tip_frame.c_str());
//        return false;
//    }
//    // create the joint to pose solver given the chain
//    left_arm_jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(left_arm_kdl_chain_));
//    left_arm_jnt_pos_.resize(left_arm_kdl_chain_.getNrOfJoints());
//
//    ROS_INFO("Getting chain from %s to %s.", root_frame.c_str(), right_arm_tip_frame.c_str());
//    // create the tree given the tree
//    if (!kdl_tree.getChain(root_frame, right_arm_tip_frame, right_arm_kdl_chain_))
//    {
//        ROS_ERROR("Failed to get kdl chain from %s to %s.", root_frame.c_str(), right_arm_tip_frame.c_str());
//        return false;
//    }
//    // create the joint to pose solver given the chain
//    right_arm_jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(right_arm_kdl_chain_));
//    right_arm_jnt_pos_.resize(right_arm_kdl_chain_.getNrOfJoints());
//    return true;
//}

bool DMPMotionLearner::createJointNames()
{
    left_arm_joint_names_.clear();
    left_arm_joint_names_.push_back("l_shoulder_pan_joint");
    left_arm_joint_names_.push_back("l_shoulder_lift_joint");
    left_arm_joint_names_.push_back("l_upper_arm_roll_joint");
    left_arm_joint_names_.push_back("l_elbow_flex_joint");
    left_arm_joint_names_.push_back("l_forearm_roll_joint");
    left_arm_joint_names_.push_back("l_wrist_flex_joint");
    left_arm_joint_names_.push_back("l_wrist_roll_joint");

    right_arm_joint_names_.clear();
    right_arm_joint_names_.push_back("r_shoulder_pan_joint");
    right_arm_joint_names_.push_back("r_shoulder_lift_joint");
    right_arm_joint_names_.push_back("r_upper_arm_roll_joint");
    right_arm_joint_names_.push_back("r_elbow_flex_joint");
    right_arm_joint_names_.push_back("r_forearm_roll_joint");
    right_arm_joint_names_.push_back("r_wrist_flex_joint");
    right_arm_joint_names_.push_back("r_wrist_roll_joint");
    return true;
}

bool DMPMotionLearner::getJointTrajectoryFromBagFile(const std::string& bag_file_name,
                                                     const std::vector<std::string>& joint_names,
                                                     std::vector<sensor_msgs::JointState>& joint_states, std::string& info)
{
    joint_states.clear();
    try
    {
        rosbag::Bag bag(bag_file_name, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery("/joint_states"));

        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
        {
            sensor_msgs::JointState::ConstPtr joint_state = msg.instantiate<sensor_msgs::JointState> ();
            if (joint_state != NULL)
            {
                joint_states.push_back(*joint_state);
            }
        }
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Problem when reading from bag file %s.", bag_file_name.c_str());
        info.assign(std::string("Problem when reading from bag file ") + bag_file_name + std::string("."));
        return false;
    }

    return true;
}

bool DMPMotionLearner::getJointTrajectoryFromBagFile(const std::string& bag_file_name, const std::vector<std::string>& joint_names,
                                                     dmp::Trajectory& joint_trajectory, std::string& info)
{

    time_stamps_.clear();

    const int num_joints = static_cast<int> (joint_names.size());
    VectorXd joint_positions = VectorXd::Zero(num_joints * dmp::POS_VEL_ACC);

    try
    {
        rosbag::Bag bag(bag_file_name, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery("/joint_states"));

        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
        {
            sensor_msgs::JointState::ConstPtr joint_state = msg.instantiate<sensor_msgs::JointState> ();
            if (joint_state != NULL)
            {
                int index = 0;
                int num_joints_found = 0;
                for (std::vector<std::string>::const_iterator vsi = joint_state->name.begin(); vsi != joint_state->name.end(); vsi++)
                {
                    for (int i = 0; i < num_joints; i++)
                    {
                        if (vsi->compare(joint_names[i]) == 0)
                        {
                            if ((i == dmp::CONTINOUS_FOREARM_ROLL_JOINT) || (i == dmp::CONTINOUS_WRIST_ROLL_JOINT))
                            {
                                // ROS_ERROR("joint_positions(%i) = %f", i, angles::normalize_angle(joint_state->position[index]));
                                joint_positions(i * dmp::POS_VEL_ACC) = angles::normalize_angle(joint_state->position[index]);
                            }
                            else
                            {
                                joint_positions(i * dmp::POS_VEL_ACC) = joint_state->position[index];
                            }
                            num_joints_found++;
                        }
                    }
                    index++;
                }
                if (num_joints_found == num_joints)
                {
                    if (!joint_trajectory.add(joint_positions))
                    {
                        ROS_ERROR("Could not add trajectory point.");
                        info.assign(std::string("Could not add trajectory point."));
                        return false;
                    }
                    time_stamps_.push_back(joint_state->header.stamp);
                }
                else
                {
                    ROS_ERROR("Number of joints is %i, but there have been %i matches.", num_joints, num_joints_found);
                    info.assign(std::string("Number of joints is ") + dmp::MathHelper::getString(num_joints) + std::string(", but there have been ")
                    + dmp::MathHelper::getString(num_joints_found) + std::string(" matches."));
                    return false;
                }
            }
            else
            {
                ROS_ERROR("Could not read bag file named %s.", bag_file_name.c_str());
                info.assign(std::string("Could not read bag file named ") + bag_file_name);
                return false;
            }
        }
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Problem when reading from bag file %s.", bag_file_name.c_str());
        info.assign(std::string("Problem when reading from bag file ") + bag_file_name + std::string("."));
        return false;
    }

    return true;
}

//bool DMPMotionLearner::doForwardKinematics(const VectorXd& joint_trajectory_point, VectorXd& cartesian_trajectory_point, std::string& info)
//{
//
//    if((joint_trajectory_point.size() % dmp::POS_VEL_ACC) != 0)
//    {
//        ROS_ERROR("Joint trajectory point does not contain position, velocity, and acceleration information.");
//        info.assign(std::string("Joint trajectory point does not contain position, velocity, and acceleration information."));
//        return false;
//    }
//
//    if(cartesian_trajectory_point.size() != ((dmp::N_CART + dmp::N_QUAT) * dmp::POS_VEL_ACC))
//    {
//        ROS_ERROR("Cartesian trajectory point has size %i, but should have size %i.", cartesian_trajectory_point.size(), (dmp::N_CART + dmp::N_QUAT) * dmp::POS_VEL_ACC);
//        info.assign(std::string("Cartesian trajectory point has size ") + dmp::MathHelper::getString(cartesian_trajectory_point.size()) + std::string(", but should have size ")
//        + dmp::MathHelper::getString((dmp::N_CART + dmp::N_QUAT) * dmp::POS_VEL_ACC) + std::string("."));
//        return false;
//    }
//
//    VectorXd joint_positions = VectorXd::Zero(joint_trajectory_point.size() / dmp::POS_VEL_ACC);
//    for (int i=0; i<joint_positions.size(); ++i)
//    {
//        joint_positions(i) = joint_trajectory_point((i*dmp::_POS_) * dmp::POS_VEL_ACC);
//    }
//
//    // do forward kinematics
//    jnt_pos_.data = joint_positions;
//    jnt_to_pose_solver_->JntToCart(jnt_pos_, link_pose_);
//
//    // transform into the frame of reference we are interested in the particular task
//    link_pose_ = link_pose_ * gripper_offset_;
//
//    double tmp_q0_x, tmp_q1_y, tmp_q2_z, tmp_q3_w;
//    link_pose_.M.GetQuaternion(tmp_q0_x, tmp_q1_y, tmp_q2_z, tmp_q3_w);
//
//    cartesian_trajectory_point(0 * dmp::POS_VEL_ACC) = link_pose_.p.x();
//    cartesian_trajectory_point(1 * dmp::POS_VEL_ACC) = link_pose_.p.y();
//    cartesian_trajectory_point(2 * dmp::POS_VEL_ACC) = link_pose_.p.z();
//    cartesian_trajectory_point(3 * dmp::POS_VEL_ACC) = tmp_q0_x;
//    cartesian_trajectory_point(4 * dmp::POS_VEL_ACC) = tmp_q1_y;
//    cartesian_trajectory_point(5 * dmp::POS_VEL_ACC) = tmp_q2_z;
//    cartesian_trajectory_point(6 * dmp::POS_VEL_ACC) = tmp_q3_w;
//
//    return true;
//}

void DMPMotionLearner::publishMarker(const VectorXd& cartesian_trajectory_point)
{
    // publish obtained pose for visualization
    pose_stamped_.header.seq = pose_stamped_.header.seq + 1;
    pose_stamped_.header.stamp = ros::Time::now();
    pose_stamped_.pose.position.x = cartesian_trajectory_point(0 * dmp::POS_VEL_ACC);
    pose_stamped_.pose.position.y = cartesian_trajectory_point(1 * dmp::POS_VEL_ACC);
    pose_stamped_.pose.position.z = cartesian_trajectory_point(2 * dmp::POS_VEL_ACC);
    pose_stamped_.pose.orientation.x = cartesian_trajectory_point(3 * dmp::POS_VEL_ACC);
    pose_stamped_.pose.orientation.y = cartesian_trajectory_point(4 * dmp::POS_VEL_ACC);
    pose_stamped_.pose.orientation.z = cartesian_trajectory_point(5 * dmp::POS_VEL_ACC);
    pose_stamped_.pose.orientation.w = cartesian_trajectory_point(6 * dmp::POS_VEL_ACC);
    marker_publisher_.publish(pose_stamped_);
}



}
