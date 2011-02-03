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

// system includes
#include <boost/filesystem.hpp>

// ros includes
#include <ros/assert.h>

#include <dmp_motion_controller/AddToExecuteDMPQueue.h>
#include <dmp_motion_controller/AddToDualArmExecuteDMPQueue.h>
#include <dmp_motion_controller/WriteTrajectories.h>

#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/assert.h>

// local includes
#include <pr2_tasks/dmp_executor.h>

using namespace pr2_tasks_transforms;

namespace pr2_tasks
{

DMPExecutor::DMPExecutor() :
    initialized_(false)
{
}

DMPExecutor::~DMPExecutor()
{
}

bool DMPExecutor::initialize(ros::NodeHandle& node_handle)
{
    node_handle_ = node_handle;

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("left_arm_ik_dmp_controller_name"), left_arm_ik_dmp_controller_name_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("right_arm_ik_dmp_controller_name"), right_arm_ik_dmp_controller_name_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("dual_arm_ik_dmp_controller_name"), dual_arm_ik_dmp_controller_name_));

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("left_arm_joint_dmp_controller_name"), left_arm_joint_dmp_controller_name_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("right_arm_joint_dmp_controller_name"), right_arm_joint_dmp_controller_name_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("dual_arm_joint_dmp_controller_name"), dual_arm_joint_dmp_controller_name_));

    // initalize controller services
    left_arm_execute_dmp_service_client_ = node_handle_.serviceClient<dmp_motion_controller::AddToExecuteDMPQueue> ("/" + left_arm_ik_dmp_controller_name_ + "/add_to_execute_dmp_queue", true);
    ROS_ASSERT(left_arm_execute_dmp_service_client_);
    left_arm_write_trajectories_client_ = node_handle_.serviceClient<dmp_motion_controller::WriteTrajectories> ("/" + left_arm_ik_dmp_controller_name_ + "/write_trajectories", true);
    ROS_ASSERT(left_arm_write_trajectories_client_);

    right_arm_execute_dmp_service_client_ = node_handle_.serviceClient<dmp_motion_controller::AddToExecuteDMPQueue> ("/" + right_arm_ik_dmp_controller_name_ + "/add_to_execute_dmp_queue", true);
    ROS_ASSERT(right_arm_execute_dmp_service_client_);
    right_arm_write_trajectories_client_ = node_handle_.serviceClient<dmp_motion_controller::WriteTrajectories> ("/" + right_arm_ik_dmp_controller_name_ + "/write_trajectories", true);
    ROS_ASSERT(right_arm_write_trajectories_client_);

    right_arm_execute_joint_dmp_service_client_ = node_handle_.serviceClient<dmp_motion_controller::AddToExecuteDMPQueue> ("/" + right_arm_joint_dmp_controller_name_ + "/add_to_execute_dmp_queue", true);
    ROS_ASSERT(right_arm_execute_joint_dmp_service_client_);
    left_arm_execute_joint_dmp_service_client_ = node_handle_.serviceClient<dmp_motion_controller::AddToExecuteDMPQueue> ("/" + left_arm_joint_dmp_controller_name_ + "/add_to_execute_dmp_queue", true);
    ROS_ASSERT(left_arm_execute_joint_dmp_service_client_);

    dual_arm_cartesian_space_execute_dmp_service_client_ = node_handle_.serviceClient<dmp_motion_controller::AddToDualArmExecuteDMPQueue> ("/" + dual_arm_ik_dmp_controller_name_ + "/add_to_execute_dmp_queue", true);
    ROS_ASSERT(dual_arm_cartesian_space_execute_dmp_service_client_);

    dual_arm_joint_space_execute_dmp_service_client_ = node_handle_.serviceClient<dmp_motion_controller::AddToDualArmExecuteDMPQueue> ("/" + dual_arm_joint_dmp_controller_name_ + "/add_to_execute_dmp_queue", true);
    ROS_ASSERT(dual_arm_joint_space_execute_dmp_service_client_);

    // listen to dmp execution statistics message
//    left_arm_dmp_execution_statistics_subscriber_ = node_handle_.subscribe("/" + left_arm_dmp_ik_controller_name_ + "/dmp_execution_statistics",
//                                                                           1, &DMPExecutor::executionStatisticsCallback, this);
    right_arm_dmp_execution_statistics_subscriber_ = node_handle_.subscribe("/" + right_arm_ik_dmp_controller_name_ + "/dmp_execution_statistics",
                                                                            1, &DMPExecutor::executionStatisticsCallback, this);

    right_arm_joint_dmp_execution_statistics_subscriber_ = node_handle_.subscribe("/" + right_arm_joint_dmp_controller_name_ +  "/dmp_execution_statistics",
                                                                            1, &DMPExecutor::executionStatisticsCallback, this);

    // initialize the controller switcher
    ROS_ASSERT_FUNC(controller_switcher_.initialize());

    return (initialized_ = true);
}

bool DMPExecutor::createDirectory(const std::string& directory_name, const int iteration_number)
{
    if (boost::filesystem::exists(directory_name))
    {
        if(iteration_number == 1)
        {
            ROS_WARN("Deleting %s and saving new rollouts.", directory_name.c_str());
            if (directory_name.find("/tmp") == std::string::npos)
            {
                ROS_ERROR("Directory %s does not contain \"/tmp\" and therefore to risky to delete.", directory_name.c_str());
                return false;
            }
            if (directory_name.find("dmp") == std::string::npos)
            {
                ROS_ERROR("Directory %s does not contain \"dmp\" and therefore to risky to delete.", directory_name.c_str());
                return false;
            }
            if (!boost::filesystem::remove_all(directory_name.c_str()))
            {
                ROS_ERROR("Could not delete directory %s.", directory_name.c_str());
                return false;
            }
        }
    }
    try
    {
        boost::filesystem::create_directory(directory_name.c_str());
    }
    catch(boost::filesystem::filesystem_error e)
    {
        ROS_ERROR("Could not create directory %s :%s", directory_name.c_str(), e.what());
        return false;
    }
    return true;
}

bool DMPExecutor::executeDMPAndWait(dmp::DynamicMovementPrimitive& dmp,
                                    pr2_tasks_transforms::TaskTransforms::TransformType transform_type,
                                    dmp_motion_controller::DMPExecutionStatistics& execution_statistics,
                                    ros::Duration& execution_duration,
                                    ros::Duration& timeout,
                                    bool write_debug, const int iteration_number)
{
    ROS_ASSERT(initialized_);

    DMPExecutor::WhichArm which_arm = DMPExecutor::RIGHT_ARM;
    if((transform_type == TaskTransforms::RIGHT_ARM_NO_TRANSFORM)
            || (transform_type == TaskTransforms::RIGHT_ARM_POOL_TRANSFORM))
    {
        ROS_ASSERT_FUNC(controller_switcher_.switchRightArmController(right_arm_ik_dmp_controller_name_));
        which_arm = DMPExecutor::RIGHT_ARM;
    }
    else
    {
        ROS_ASSERT_FUNC(controller_switcher_.switchLeftArmController(left_arm_ik_dmp_controller_name_));
        which_arm = DMPExecutor::LEFT_ARM;
    }

    // prepare DMP message for execution
    dmp_motion_controller::AddToExecuteDMPQueue::Request execute_dmp_request;
    dmp_motion_controller::AddToExecuteDMPQueue::Response execute_dmp_response;
    dmp_motion_generation::DynamicMovementPrimitive dmp_message;
    ROS_ASSERT_FUNC(dmp.writeToMessage(dmp_message));
    execute_dmp_request.dmps.push_back(dmp_message);
    execute_dmp_request.execution_durations.push_back(execution_duration.toSec());
    execute_dmp_request.types.push_back(transform_type);

    // call the execution service
    execution_stats_received_ = false;
    switch(which_arm)
    {
        case DMPExecutor::LEFT_ARM:
        {
            ROS_ASSERT_FUNC(left_arm_execute_dmp_service_client_.call(execute_dmp_request, execute_dmp_response));
            break;
        }
        case DMPExecutor::RIGHT_ARM:
        {
            ROS_ASSERT_FUNC(right_arm_execute_dmp_service_client_.call(execute_dmp_request, execute_dmp_response));
            break;
        }
    }

    // wait for the execution stats message
    if (!waitForExecutionStatistics(timeout))
    {
        ROS_ERROR("DMPExecutor: Timed out waiting for execution statistics message.");
        return false;
    }
    execution_statistics = execution_statistics_;

    if(write_debug)
    {
        // write debug traj to disk
        ROS_INFO("Write executed trajectories...");
        std::string directory_name = std::string("/tmp/");
        switch(which_arm)
        {
            case DMPExecutor::LEFT_ARM:
            {
                directory_name.append(left_arm_ik_dmp_controller_name_);
                break;
            }
            case DMPExecutor::RIGHT_ARM:
            {
                directory_name.append(right_arm_ik_dmp_controller_name_);
                break;
            }
        }
        ROS_ASSERT_FUNC(createDirectory(directory_name, iteration_number));

        policy_improvement_utilities::appendTrailingSlash(directory_name);
        directory_name.append(std::string("dmp_") + policy_improvement_utilities::getString(dmp_message.dmp_id) + std::string("_") + policy_improvement_utilities::getString(iteration_number));

        dmp_motion_controller::WriteTrajectories::Request write_traj_request;
        dmp_motion_controller::WriteTrajectories::Response write_traj_response;
        write_traj_request.filename = directory_name;

        switch(which_arm)
        {
            case DMPExecutor::LEFT_ARM:
            {
                ROS_ASSERT_FUNC(left_arm_write_trajectories_client_.call(write_traj_request, write_traj_response));
                break;
            }
            case DMPExecutor::RIGHT_ARM:
            {
                ROS_ASSERT_FUNC(right_arm_write_trajectories_client_.call(write_traj_request, write_traj_response));
                break;
            }
        }
        ROS_ASSERT(write_traj_response.return_code == dmp_motion_controller::WriteTrajectories::Response::SERVICE_CALL_SUCCESSFUL);
    }
    else
    {
        ROS_INFO("Skipping writing debug trajectories.");
    }

    return true;
}


bool DMPExecutor::executeJointDMPAndWait(dmp::DynamicMovementPrimitive& dmp,
                                         ros::Duration& execution_duration,
                                         ros::Duration& timeout)
{
    ROS_ASSERT(initialized_);

    ROS_ASSERT_FUNC(controller_switcher_.switchRightArmController(right_arm_joint_dmp_controller_name_));

    // prepare DMP message for execution
    dmp_motion_controller::AddToExecuteDMPQueue::Request execute_dmp_request;
    dmp_motion_controller::AddToExecuteDMPQueue::Response execute_dmp_response;
    dmp_motion_generation::DynamicMovementPrimitive dmp_message;
    ROS_ASSERT_FUNC(dmp.writeToMessage(dmp_message));
    execute_dmp_request.dmps.push_back(dmp_message);
    execute_dmp_request.execution_durations.push_back(execution_duration.toSec());
    execute_dmp_request.types.push_back(0);

    // call the execution service
    execution_stats_received_ = false;
    ROS_ASSERT_FUNC(right_arm_execute_joint_dmp_service_client_.call(execute_dmp_request, execute_dmp_response));

    // wait for the execution stats message
    if (!waitForExecutionStatistics(timeout))
    {
        ROS_ERROR("DMPExecutor: Timed out waiting for execution statistics message.");
        return false;
    }

    // switch back controller...
    ROS_ASSERT_FUNC(controller_switcher_.switchRightArmController(right_arm_ik_dmp_controller_name_));

//    if(write_debug)
//    {
//        // write debug traj to disk
//        ROS_INFO("Write executed trajectories...");
//        std::string directory_name = std::string("/tmp/");
//        switch(which_arm)
//        {
//            case DMPExecutor::LEFT_ARM:
//            {
//                directory_name.append(left_arm_dmp_controller_name_);
//                break;
//            }
//            case DMPExecutor::RIGHT_ARM:
//            {
//                directory_name.append(right_arm_dmp_controller_name_);
//                break;
//            }
//        }
//        ROS_ASSERT_FUNC(createDirectory(directory_name, iteration_number));
//
//        policy_improvement_utilities::appendTrailingSlash(directory_name);
//        directory_name.append(std::string("dmp_") + policy_improvement_utilities::getString(dmp_message.dmp_id) + std::string("_") + policy_improvement_utilities::getString(iteration_number));
//
//        dmp_motion_controller::WriteTrajectories::Request write_traj_request;
//        dmp_motion_controller::WriteTrajectories::Response write_traj_response;
//        write_traj_request.filename = directory_name;
//
//        switch(which_arm)
//        {
//            case DMPExecutor::LEFT_ARM:
//            {
//                ROS_ASSERT_FUNC(left_arm_write_trajectories_client_.call(write_traj_request, write_traj_response));
//                break;
//            }
//            case DMPExecutor::RIGHT_ARM:
//            {
//                ROS_ASSERT_FUNC(right_arm_write_trajectories_client_.call(write_traj_request, write_traj_response));
//                break;
//            }
//        }
//        ROS_ASSERT(write_traj_response.return_code == dmp_motion_controller::WriteTrajectories::Response::SERVICE_CALL_SUCCESSFUL);
//    }
//    else
//    {
//        ROS_INFO("Skipping writing debug trajectories.");
//    }

    return true;
}


bool DMPExecutor::executeDMPAndWait(dmp::DynamicMovementPrimitive& right_arm_dmp,
                                    dmp_motion_controller::DMPExecutionStatistics& execution_statistics,
                                    ros::Duration& right_arm_execution_duration,
                                    ros::Duration& timeout,
                                    const double right_arm_movement_duration,
                                    dmp::DynamicMovementPrimitive& left_arm_dmp,
                                    const double left_arm_execution_duration,
                                    bool write_debug, const int iteration_number)
{
    ROS_ASSERT(initialized_);

    ROS_ASSERT_FUNC(controller_switcher_.switchRightArmController(right_arm_ik_dmp_controller_name_));
    ROS_ASSERT_FUNC(controller_switcher_.switchLeftArmController(left_arm_ik_dmp_controller_name_));

    // prepare right arm DMP message for execution
    dmp_motion_controller::AddToExecuteDMPQueue::Request right_arm_dmp_request;
    dmp_motion_controller::AddToExecuteDMPQueue::Response right_arm_dmp_response;
    dmp_motion_generation::DynamicMovementPrimitive right_arm_dmp_message;
    ROS_ASSERT_FUNC(right_arm_dmp.writeToMessage(right_arm_dmp_message));
    right_arm_dmp_request.dmps.push_back(right_arm_dmp_message);
    right_arm_dmp_request.execution_durations.push_back(right_arm_execution_duration.toSec());
    right_arm_dmp_request.types.push_back(TaskTransforms::RIGHT_ARM_POOL_TRANSFORM);

    // call the execution service
    execution_stats_received_ = false;
    ROS_ASSERT_FUNC(right_arm_execute_dmp_service_client_.call(right_arm_dmp_request, right_arm_dmp_response));

    // wait for the movement duration to be over...
    ros::Duration wait_duration;
    wait_duration = ros::Duration(right_arm_movement_duration - 0.75);
    wait_duration.sleep();

    // prepare and execute left arm DMP
    dmp_motion_controller::AddToExecuteDMPQueue::Request left_arm_dmp_request;
    dmp_motion_controller::AddToExecuteDMPQueue::Response left_arm_dmp_response;
    dmp_motion_generation::DynamicMovementPrimitive left_arm_dmp_message;
    ROS_ASSERT_FUNC(left_arm_dmp.writeToMessage(left_arm_dmp_message));
    left_arm_dmp_request.dmps.push_back(left_arm_dmp_message);
    left_arm_dmp_request.execution_durations.push_back(left_arm_execution_duration);
    left_arm_dmp_request.types.push_back(TaskTransforms::LEFT_ARM_POOL_TRANSFORM);
    ROS_ASSERT_FUNC(left_arm_execute_dmp_service_client_.call(left_arm_dmp_request, left_arm_dmp_response));

    // wait for the execution stats message
    if (!waitForExecutionStatistics(timeout))
    {
        ROS_ERROR("DMPExecutor: Timed out waiting for execution statistics message.");
        return false;
    }
    execution_statistics = execution_statistics_;

    if(write_debug)
    {
        // write debug traj to disk
        ROS_INFO("Write executed trajectories...");
        std::string directory_name = std::string("/tmp/");
        directory_name.append(right_arm_ik_dmp_controller_name_);
        ROS_ASSERT_FUNC(createDirectory(directory_name, iteration_number));

        policy_improvement_utilities::appendTrailingSlash(directory_name);
        directory_name.append(std::string("dmp_") + policy_improvement_utilities::getString(right_arm_dmp_message.dmp_id) + std::string("_") + policy_improvement_utilities::getString(iteration_number));

        dmp_motion_controller::WriteTrajectories::Request write_traj_request;
        dmp_motion_controller::WriteTrajectories::Response write_traj_response;
        write_traj_request.filename = directory_name;

        ROS_ASSERT_FUNC(right_arm_write_trajectories_client_.call(write_traj_request, write_traj_response));
        ROS_ASSERT(write_traj_response.return_code == dmp_motion_controller::WriteTrajectories::Response::SERVICE_CALL_SUCCESSFUL);

    }
    else
    {
        ROS_INFO("Skipping writing debug trajectories.");
    }

    return true;
}

bool DMPExecutor::executeDMPAndWait(dmp::DynamicMovementPrimitive& dual_arm_dmp,
                                    TaskTransforms::TransformType left_arm_transform_type,
                                    TaskTransforms::TransformType right_arm_transform_type,
                                    dmp_motion_controller::DMPExecutionStatistics& execution_statistics,
                                    ros::Duration& execution_duration,
                                    ros::Duration& timeout,
                                    bool write_debug, const int iteration_number)
{

    // switch the controller and start the dual arm controller (if it is not already running)
    ROS_ASSERT_FUNC(controller_switcher_.switchLeftArmController(left_arm_ik_dmp_controller_name_));
    ROS_ASSERT_FUNC(controller_switcher_.switchRightArmController(right_arm_ik_dmp_controller_name_));
    ROS_ASSERT_FUNC(controller_switcher_.startController(dual_arm_ik_dmp_controller_name_));

    // form the request
    dmp_motion_controller::AddToDualArmExecuteDMPQueue::Request dual_arm_request;
    dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response dual_arm_response;

    dmp_motion_generation::DynamicMovementPrimitive left_arm_dmp_msg;
    ROS_ASSERT_FUNC(dual_arm_dmp.writeToMessage(left_arm_dmp_msg));
    dmp_motion_generation::DynamicMovementPrimitive right_arm_dmp_msg;
    ROS_ASSERT_FUNC(dual_arm_dmp.writeToMessage(right_arm_dmp_msg));

    dual_arm_request.left_arm_dmps.push_back(left_arm_dmp_msg);
    dual_arm_request.right_arm_dmps.push_back(right_arm_dmp_msg);

    dual_arm_request.execution_durations.push_back(execution_duration.toSec());

    dual_arm_request.left_arm_types.push_back(left_arm_transform_type);
    dual_arm_request.right_arm_types.push_back(right_arm_transform_type);

    // call the execution service
    execution_stats_received_ = false;
    ROS_ASSERT_FUNC(dual_arm_cartesian_space_execute_dmp_service_client_.call(dual_arm_request, dual_arm_response));
    // ROS_INFO_STREAM(dual_arm_response.info);

    // wait for the execution stats message
    if (!waitForExecutionStatistics(timeout))
    {
        ROS_ERROR("DMPExecutor: Timed out waiting for execution statistics message");
        return false;
    }
    execution_statistics = execution_statistics_;

    if(write_debug)
    {
        // write debug traj to disk
        ROS_INFO("Write executed trajectories...");

        std::string left_arm_directory_name = std::string("/tmp/");
        left_arm_directory_name.append(left_arm_ik_dmp_controller_name_);
        ROS_ASSERT_FUNC(createDirectory(left_arm_directory_name, iteration_number));
        policy_improvement_utilities::appendTrailingSlash(left_arm_directory_name);
        left_arm_directory_name.append(std::string("dmp_") + policy_improvement_utilities::getString(left_arm_dmp_msg.dmp_id) + std::string("_") + policy_improvement_utilities::getString(iteration_number));

        std::string right_arm_directory_name = std::string("/tmp/");
        right_arm_directory_name.append(right_arm_ik_dmp_controller_name_);
        ROS_ASSERT_FUNC(createDirectory(right_arm_directory_name, iteration_number));
        policy_improvement_utilities::appendTrailingSlash(right_arm_directory_name);
        right_arm_directory_name.append(std::string("dmp_") + policy_improvement_utilities::getString(right_arm_dmp_msg.dmp_id) + std::string("_") + policy_improvement_utilities::getString(iteration_number));

        dmp_motion_controller::WriteTrajectories::Request left_arm_write_traj_request;
        dmp_motion_controller::WriteTrajectories::Request right_arm_write_traj_request;
        dmp_motion_controller::WriteTrajectories::Response write_traj_response;
        left_arm_write_traj_request.filename = left_arm_directory_name;
        right_arm_write_traj_request.filename = right_arm_directory_name;

        ROS_ASSERT_FUNC(left_arm_write_trajectories_client_.call(left_arm_write_traj_request, write_traj_response));
        ROS_ASSERT(write_traj_response.return_code == dmp_motion_controller::WriteTrajectories::Response::SERVICE_CALL_SUCCESSFUL);
        ROS_ASSERT_FUNC(right_arm_write_trajectories_client_.call(right_arm_write_traj_request, write_traj_response));
        ROS_ASSERT(write_traj_response.return_code == dmp_motion_controller::WriteTrajectories::Response::SERVICE_CALL_SUCCESSFUL);
    }
    else
    {
        // ROS_INFO("Skipping writing debug trajectories.");
    }

    return true;
}

bool DMPExecutor::executeDMPAndWait(dmp::DynamicMovementPrimitive& dual_arm_dmp,
                                    dmp_motion_controller::DMPExecutionStatistics& execution_statistics,
                                    ros::Duration& execution_duration,
                                    ros::Duration& timeout)
{

    // switch the controller and start the dual arm controller (if it is not already running)
    ROS_ASSERT_FUNC(controller_switcher_.switchLeftArmController(left_arm_joint_dmp_controller_name_));
    ROS_ASSERT_FUNC(controller_switcher_.switchRightArmController(right_arm_joint_dmp_controller_name_));
    ROS_ASSERT_FUNC(controller_switcher_.startController(dual_arm_joint_dmp_controller_name_));

    // form the request
    dmp_motion_controller::AddToDualArmExecuteDMPQueue::Request dual_arm_request;
    dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response dual_arm_response;

    dmp_motion_generation::DynamicMovementPrimitive left_arm_dmp_msg;
    ROS_ASSERT_FUNC(dual_arm_dmp.writeToMessage(left_arm_dmp_msg));
    left_arm_dmp_msg.transformation_systems.erase(left_arm_dmp_msg.transformation_systems.begin(),
                                                  left_arm_dmp_msg.transformation_systems.begin() + dmp::N_JOINTS);

    dmp_motion_generation::DynamicMovementPrimitive right_arm_dmp_msg;
    ROS_ASSERT_FUNC(dual_arm_dmp.writeToMessage(right_arm_dmp_msg));
    right_arm_dmp_msg.transformation_systems.erase(right_arm_dmp_msg.transformation_systems.begin() + dmp::N_JOINTS,
                                                  right_arm_dmp_msg.transformation_systems.end());

    dual_arm_request.left_arm_dmps.push_back(left_arm_dmp_msg);
    dual_arm_request.right_arm_dmps.push_back(right_arm_dmp_msg);

    dual_arm_request.execution_durations.push_back(execution_duration.toSec());

    dual_arm_request.left_arm_types.push_back(0);
    dual_arm_request.right_arm_types.push_back(0);

    // call the execution service
    execution_stats_received_ = false;
    ROS_ASSERT_FUNC(dual_arm_joint_space_execute_dmp_service_client_.call(dual_arm_request, dual_arm_response));
    ROS_INFO_STREAM(dual_arm_response.info);

    // wait for the execution stats message
    if (!waitForExecutionStatistics(timeout))
    {
        ROS_ERROR("DMPExecutor: Timed out waiting for execution statistics message");
        return false;
    }
    execution_statistics = execution_statistics_;

    // ROS_ASSERT_FUNC(controller_switcher_.switchLeftArmController(left_arm_ik_dmp_controller_name_));
    // ROS_ASSERT_FUNC(controller_switcher_.switchRightArmController(right_arm_ik_dmp_controller_name_));

    return true;
}


void DMPExecutor::executionStatisticsCallback(const dmp_motion_controller::DMPExecutionStatisticsConstPtr& msg)
{
    execution_statistics_ = *msg;
    execution_stats_received_mutex_.lock();
    execution_stats_received_ = true;
    execution_stats_received_mutex_.unlock();
}

bool DMPExecutor::waitForExecutionStatistics(ros::Duration& timeout)
{
    bool esr = false;
    ros::Duration time_spent = ros::Duration(0.0);
    ros::Duration tick = ros::Duration(0.1);

    while (!esr && time_spent < timeout && node_handle_.ok())
    {
        ros::spinOnce();
        execution_stats_received_mutex_.lock();
        esr = execution_stats_received_;
        execution_stats_received_mutex_.unlock();
        if (!esr)
        {
            tick.sleep();
            time_spent += tick;
        }
    }

    return esr;
}

}
