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

#include <pr2_tasks/dmp_task.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/spin.h>

#include <boost/pointer_cast.hpp>

using namespace policy_improvement_utilities;

namespace pr2_tasks
{

DMPTask::DMPTask() :
    initialized_(false), iteration_number_(0)
{
}

DMPTask::~DMPTask()
{
}

bool DMPTask::initialize(ros::NodeHandle& node_handle, int num_time_steps)
{
    node_handle_ = node_handle;
    num_time_steps_ = num_time_steps;

    // read stuff from param server
    ROS_ASSERT_FUNC(readParams());

    // initialize the dmp
    dmp_.reset(new dmp::DynamicMovementPrimitive(node_handle_));
    reset_dmp_.reset(new dmp::DynamicMovementPrimitive(node_handle_));

    // initialize the dmp executor:
    dmp_executor_.initialize(node_handle_);

    // call the child class initializer
    ROS_ASSERT_FUNC(taskInitialize());

    // initialize the policy
    policy_.reset(new library::DMPPolicy());
    policy_->initialize(dmp_);

    return (initialized_ = true);
}

bool DMPTask::readParams()
{
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("control_cost_weight"), control_cost_weight_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("num_time_steps"), num_time_steps_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("movement_duration"), movement_duration_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("execution_duration"), execution_duration_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("task_dmp_id"), task_dmp_id_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("reset_dmp_id"), reset_dmp_id_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("reset_movement_duration"), reset_movement_duration_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("dmp_sampling_frequency"), dmp_sampling_frequency_));
    if(dmp_sampling_frequency_ < 1e-5)
    {
        ROS_ERROR("Sampling frequency (%f) not valid.", dmp_sampling_frequency_);
        return false;
    }
    dmp_dt_ = static_cast<double>(1.0) / dmp_sampling_frequency_;

    // we have extra time steps due to the discrepancy between movement_duration and execution_duration:
    num_time_steps_total_ = int((execution_duration_ / movement_duration_) * num_time_steps_);
    // ROS_INFO("total time steps = %d", num_time_steps_total_);

    execution_duration_ros_ = ros::Duration(execution_duration_);
    execution_timeout_ros_ = ros::Duration(execution_duration_*1.5);
    return true;
}

bool DMPTask::execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number)
{
    ros::spinOnce();
    ROS_ASSERT(initialized_);

    iteration_number_ = iteration_number;

    // setup the dmp with the new parameters
    ROS_ASSERT_FUNC(dmp_->setThetas(parameters));
    ROS_ASSERT_FUNC(dmp_->setup(dmp_sampling_frequency_));
    ROS_ASSERT_FUNC(dmp_->setDuration(movement_duration_, dmp_sampling_frequency_));

    bool offline_check_succeeded = false;
    ROS_ASSERT_FUNC(checkOfflineExecution(offline_check_succeeded, costs));
    if(offline_check_succeeded)
    {
        // reset canonical system
        ROS_ASSERT_FUNC(dmp_->setup(dmp_sampling_frequency_));

        // execute dmp
        ROS_ASSERT_FUNC(execute());

        // calculate the cost:
        ROS_ASSERT_FUNC(computeCosts(costs));
    }
    return true;
}

bool DMPTask::getPolicy(boost::shared_ptr<library::Policy>& policy)
{
    ROS_ASSERT(initialized_);
    policy = policy_;
    return true;
}

bool DMPTask::setPolicy(const boost::shared_ptr<library::Policy> policy)
{
    ROS_ASSERT(initialized_);
    policy_ = boost::dynamic_pointer_cast<library::DMPPolicy>(policy);
    return true;
}

bool DMPTask::getControlCostWeight(double& control_cost_weight)
{
    ROS_ASSERT(initialized_);
    control_cost_weight = control_cost_weight_;
    return true;
}

}
