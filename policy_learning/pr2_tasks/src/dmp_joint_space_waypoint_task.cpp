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

// ros includes
#include <ros/ros.h>
#include <ros/assert.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/spin.h>

#include <dmp_motion_controller/AddToExecuteDMPQueue.h>
#include <dmp_motion_controller/WriteTrajectories.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <task_manager/task_manager.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

#include <pr2_tasks_transforms/task_transforms.h>

// local includes
#include <pr2_tasks/dmp_joint_space_waypoint_task.h>

using namespace dmp;
using namespace policy_improvement_utilities;
using namespace pr2_tasks_transforms;

PLUGINLIB_DECLARE_CLASS(pr2_tasks, DMPJointSpaceWaypointTask, pr2_tasks::DMPJointSpaceWaypointTask, task_manager_interface::Task)

namespace pr2_tasks
{

DMPJointSpaceWaypointTask::DMPJointSpaceWaypointTask()
{
}

DMPJointSpaceWaypointTask::~DMPJointSpaceWaypointTask()
{
}

bool DMPJointSpaceWaypointTask::taskInitialize()
{
    // read stuff from param server
    ROS_ASSERT_FUNC(readParams());

    // display waypoint in rviz
    marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);

    // create FK solver for cost function
    kdl_chain_wrapper_.initialize(node_handle_, root_frame_, tip_frame_);

    num_dimensions_ = kdl_chain_wrapper_.getNumJoints();

    // initialize the dmp
    int dmp_id_ = 1;
    dmp_->initialize(num_dimensions_, dmp_id_);

    // learn from min jerk
    dmp_->learnFromMinJerk(start_, goal_, movement_duration_, dmp_dt_);

    // initialize joint states logger
    ROS_ASSERT_FUNC(joint_states_logger_.initialize(node_handle_));

    // create FK solver for cost function
    kdl_chain_wrapper_.initialize(node_handle_, root_frame_, tip_frame_);

    return true;
}

bool DMPJointSpaceWaypointTask::readParams()
{
    node_handle_.param("waypoint_cost_weight", waypoint_cost_weight_, 100000.0);
    node_handle_.param("root_frame", root_frame_, std::string("torso_lift_link"));
    node_handle_.param("tip_frame", tip_frame_, std::string("r_gripper_tool_frame"));
    node_handle_.param("run_offline", run_offline_, false);

    ROS_ASSERT_FUNC(readEigenVector(node_handle_, "joint_start", start_));
    ROS_ASSERT_FUNC(readEigenVector(node_handle_, "joint_goal", goal_));
    ROS_ASSERT_FUNC(readEigenVector(node_handle_, "cart_waypoint", cart_waypoint_));

    node_handle_.param("cart_waypoint_time", cart_waypoint_time_, 1.0);

    return true;
}

bool DMPJointSpaceWaypointTask::preExecute()
{
    // reset the system
    if (!reset())
    {
        ROS_ERROR("DMPJointSpaceWaypointTask: Reset failed");
        return false;
    }

    // start logging
    joint_states_logger_.startLogging();

    return true;
}

bool DMPJointSpaceWaypointTask::postExecute()
{

    // stop logging
    joint_states_logger_.stopLogging();

    // get the binned joint states:
    joint_states_logger_.getJointStates(execution_statistics_.start_time, execution_statistics_.end_time, num_time_steps_total_, joint_states_);

    return true;
}

bool DMPJointSpaceWaypointTask::execute()
{
    // call pre-execute to reset the system and start logging:
    ROS_ASSERT_FUNC(preExecute());

    // TODO: remove spin_sleeps...
    spin_sleep(ros::Duration(0.2));
    ROS_ASSERT_FUNC(dmp_executor_.executeDMPAndWait(*dmp_, TaskTransforms::RIGHT_ARM_NO_TRANSFORM,
                                                    execution_statistics_, execution_duration_ros_,
                                                    execution_timeout_ros_, true));
    spin_sleep(ros::Duration(0.2));

    // call post-execute to stop logging:
    ROS_ASSERT_FUNC(postExecute());
    return true;
}

bool DMPJointSpaceWaypointTask::computeCosts(Eigen::VectorXd& costs)
{
    int waypoint_time_index = int( num_time_steps_total_ * cart_waypoint_time_ / execution_duration_);

    costs = Eigen::VectorXd::Zero(num_time_steps_);

    // waypoint cost
    KDL::Frame frame;
    kdl_chain_wrapper_.forwardKinematics(joint_states_[waypoint_time_index], frame);
    double dist = 0.0;
    Eigen::VectorXd point = Eigen::VectorXd::Zero(3);
    for (int i=0; i<3; ++i)
    {
        point(i) = frame.p(i);
        double d = frame.p(i) - cart_waypoint_(i);
        dist += d*d;
    }
    costs(waypoint_time_index) = dist * waypoint_cost_weight_;

    displayWaypoint(cart_waypoint_, 0);
    displayWaypoint(point, 1);

    // display the entire trajectory for debugging:
    /*for (int t=0; t<num_time_steps_total_; t+=10)
    {
        kdl_chain_wrapper_.forwardKinematics(joint_states_[t], frame);
        for (int i=0; i<3; ++i)
        {
            point(i) = frame.p(i);
        }
        displayWaypoint(point, 999+t);
    }*/

    return true;
}

bool DMPJointSpaceWaypointTask::reset()
{
    int dmp_id = 2;
    DynamicMovementPrimitive dmp(node_handle_);
    dmp.initialize(num_dimensions_, dmp_id);

    // learn from min jerk
    dmp.learnFromMinJerk(goal_, start_, movement_duration_, dmp_dt_);
    dmp.setup(dmp_sampling_frequency_);

    ROS_ASSERT_FUNC(dmp_executor_.executeDMPAndWait(dmp, TaskTransforms::RIGHT_ARM_NO_TRANSFORM,
                                                    execution_statistics_, execution_duration_ros_, execution_timeout_ros_));
    
    return true;
}

void DMPJointSpaceWaypointTask::displayWaypoint(Eigen::VectorXd& position, int color)
{

    double sphere_radius = 0.03;

    visualization_msgs::Marker marker;
    marker.header.frame_id = root_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "DMPJointSpaceWaypointTask";
    marker.id = color;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position(0);
    marker.pose.position.y = position(1);
    marker.pose.position.z = position(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = sphere_radius;
    marker.scale.y = sphere_radius;
    marker.scale.z = sphere_radius;
    if (color==0)
    {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    else if (color==1)
    {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else
    {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_publisher_.publish(marker);

}

bool DMPJointSpaceWaypointTask::checkOfflineExecution(bool& offline_check_succeeded, Eigen::VectorXd& costs)
{
    if (!run_offline_)
    {
        offline_check_succeeded = true;
        return true;
    }

    dmp_->setup(dmp_sampling_frequency_);

    dmp::Trajectory trajectory(node_handle_);

    double traj_sampling_frequency = double(num_time_steps_total_) / execution_duration_;

    if (!trajectory.initialize(num_dimensions_*3, traj_sampling_frequency, num_time_steps_total_+1, num_dimensions_*3, dmp::Trajectory::eDEBUG_DATA_TRACE))
    {
        ROS_ERROR("Could not initialize trajectory.");
        return false;
    }

    dmp_->propagateFull(trajectory, execution_duration_, num_time_steps_total_);

    // convert this trajectory into joint_states
    //std::vector<sensor_msgs::JointState> joint_states;
    joint_states_.clear();
    std::vector<std::string> joint_names;
    kdl_chain_wrapper_.getJointNames(joint_names);
    for (int t=0; t<num_time_steps_total_; ++t)
    {
        sensor_msgs::JointState js;
        js.name = joint_names;
        js.position.resize(num_dimensions_,0.0);
        js.velocity.resize(num_dimensions_,0.0);
        js.effort.resize(num_dimensions_,0.0);

        for (int d=0; d<num_dimensions_; ++d)
            js.position[d] = trajectory.getTrajectoryPosition(t, d);
        joint_states_.push_back(js);
    }

    computeCosts(costs);

    offline_check_succeeded = false;

    return true;
}

/*
bool DMPJointSpaceWaypointTask::executeFake(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs)
{
    dmp_->setThetas(parameters);
    dmp_->setup(dmp_sampling_frequency_);

    dmp::Trajectory trajectory(node_handle_);

    double traj_sampling_frequency = double(num_time_steps_total_) / execution_duration_;

    if (!trajectory.initialize(num_dimensions_*3, traj_sampling_frequency, num_time_steps_total_+1, num_dimensions_*3, dmp::Trajectory::eDEBUG_DATA_TRACE))
    {
        ROS_ERROR("Could not initialize trajectory.");
        return false;
    }

    dmp_->propagateFull(trajectory, execution_duration_, num_time_steps_total_);

    // convert this trajectory into joint_states
    std::vector<sensor_msgs::JointState> joint_states;
    std::vector<std::string> joint_names;
    kdl_chain_wrapper_.getJointNames(joint_names);
    for (int t=0; t<num_time_steps_total_; ++t)
    {
        sensor_msgs::JointState js;
        js.name = joint_names;
        js.position.resize(num_dimensions_,0.0);
        js.velocity.resize(num_dimensions_,0.0);
        js.effort.resize(num_dimensions_,0.0);

        for (int d=0; d<num_dimensions_; ++d)
            js.position[d] = trajectory.getTrajectoryPosition(t, d);
        joint_states.push_back(js);
    }

    computeCosts(joint_states, costs);
    spin_sleep(ros::Duration(0.2));

    return true;
}

bool DMPJointSpaceWaypointTask::execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs)
{
    ros::spinOnce();
    if (execute_real_)
        return executeReal(parameters, costs);
    else
        return executeFake(parameters, costs);
}
*/
}
