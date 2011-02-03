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

#ifndef DMP_JOINT_SPACE_WAYPOINT_TASK_H_
#define DMP_JOINT_SPACE_WAYPOINT_TASK_H_

// ros includes
#include <ros/ros.h>

#include <task_manager/task.h>
#include <task_manager/controller_switcher.h>

#include <policy_library/dmp_policy.h>

#include <dmp_motion_controller/DMPExecutionStatistics.h>
#include <policy_improvement_utilities/kdl_chain_wrapper.h>
#include <kdl/chainfksolvervel_recursive.hpp>

// local includes
#include <pr2_tasks/joint_states_logger.h>
#include <pr2_tasks/dmp_executor.h>
#include <pr2_tasks/dmp_task.h>

namespace pr2_tasks
{

class DMPJointSpaceWaypointTask : public DMPTask
{
public:
    DMPJointSpaceWaypointTask();
    virtual ~DMPJointSpaceWaypointTask();

protected:

    bool preExecute();
    bool postExecute();

    bool execute();

    bool checkOfflineExecution(bool& offline_check_succeeded, Eigen::VectorXd& costs);

    bool taskInitialize();

    bool computeCosts(Eigen::VectorXd& costs);

    /**
     * Read params from param server
     */
    bool readParams();

    /**
     * Displays the waypoint as a marker in rviz
     */
    void displayWaypoint(Eigen::VectorXd& position, int color);

    ros::Publisher marker_publisher_;
    int num_dimensions_;

    Eigen::VectorXd start_;
    Eigen::VectorXd goal_;
    Eigen::VectorXd cart_waypoint_;
    double cart_waypoint_time_;

    std::string root_frame_;
    std::string tip_frame_;

    double waypoint_cost_weight_;

    JointStatesLogger joint_states_logger_;

    // stuff required for the cost function:
    policy_improvement_utilities::KDLChainWrapper kdl_chain_wrapper_;
    std::vector<sensor_msgs::JointState> joint_states_;

private:
    bool reset();

    bool run_offline_;
};

}

#endif /* DMP_JOINT_SPACE_WAYPOINT_TASK_H_ */
