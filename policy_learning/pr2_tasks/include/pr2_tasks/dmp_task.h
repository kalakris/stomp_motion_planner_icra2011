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

#ifndef DMP_TASK_H_
#define DMP_TASK_H_

// ros includes
#include <ros/ros.h>

#include <task_manager/task.h>
#include <task_manager/controller_switcher.h>

#include <policy_library/dmp_policy.h>
#include <policy_library/single_parameter_policy.h>
#include <policy_library/mixed_policy.h>

#include <dmp_motion_controller/DMPExecutionStatistics.h>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <policy_improvement_utilities/kdl_chain_wrapper.h>

// local includes
#include <pr2_tasks/dmp_executor.h>

namespace pr2_tasks
{

class DMPTask : public task_manager_interface::Task
{
public:
    DMPTask();
    virtual ~DMPTask();

    /**
     * Initializes the task for a given number of time steps
     * @param num_time_steps
     * @return
     */
    bool initialize(ros::NodeHandle& node_handle, int num_time_steps);

    /**
     * Executes the task for the given policy parameters, and returns the costs per timestep
     * @param parameters [num_dimensions] num_parameters - policy parameters to execute
     * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
     * @return
     */
    bool execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number = 0);

    /**
     * Get the Policy object of this Task
     * @param policy
     * @return
     */
    bool getPolicy(boost::shared_ptr<library::Policy>& policy);

    /**
     * Sets the Policy object of this Task
     * @param policy
     * @return
     */
    bool setPolicy(const boost::shared_ptr<library::Policy> policy);

    /**
     * Gets the weight of the control cost
     * @param control_cost_weight
     * @return
     */
    bool getControlCostWeight(double& control_cost_weight);


protected:

    virtual bool execute() = 0;

    virtual bool taskInitialize()=0;

    virtual bool checkOfflineExecution(bool& offline_check_succeeded, Eigen::VectorXd& costs)=0;

    /**
     * Get the costs over time
     * @param joint_states
     * @param costs
     */
    virtual bool computeCosts(Eigen::VectorXd& costs)=0;

    bool initialized_;
    int iteration_number_;

    ros::NodeHandle node_handle_;

    int num_time_steps_;            /**< Number of time steps for RL / movement duration only */
    int num_time_steps_total_;      /**< Number of time steps for the entire movement (execution duration) */
    double movement_duration_;
    double execution_duration_;
    double reset_movement_duration_;
    double dmp_sampling_frequency_;
    double dmp_dt_;

    int task_dmp_id_;
    int reset_dmp_id_;

    ros::Duration execution_duration_ros_;
    ros::Duration execution_timeout_ros_;

    double control_cost_weight_;

    DMPExecutor dmp_executor_;
    dmp_motion_controller::DMPExecutionStatistics execution_statistics_;

    boost::shared_ptr<library::DMPPolicy> policy_;

    boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp_;
    boost::shared_ptr<dmp::DynamicMovementPrimitive> reset_dmp_;

private:

    /**
     * Read params from param server
     */
    bool readParams();

};

}

#endif /* DMP_TASK_H_ */
