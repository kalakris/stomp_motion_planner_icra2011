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
#include <sstream>

// ros includes
#include <ros/ros.h>
#include <ros/assert.h>

#include <pluginlib/class_list_macros.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/spin.h>
#include <visualization_msgs/Marker.h>

#include <dmp_motion_learner/LearnCartesianSpaceDMP.h>
// #include <dmp_motion_learner/LearnDualCartesianSpaceDMP.h>

#include <dmp_motion_controller/AddToExecuteDMPQueue.h>
#include <dmp_motion_controller/WriteTrajectories.h>

#include <boost/lexical_cast.hpp>

// local includes
#include <pr2_tasks/pool_task.h>
#include <task_manager/task_manager.h>

using namespace dmp;
using namespace policy_improvement_utilities;
using boost::lexical_cast;
using namespace pr2_tasks_transforms;
USING_PART_OF_NAMESPACE_EIGEN

PLUGINLIB_DECLARE_CLASS(pr2_tasks, PoolTask, pr2_tasks::PoolTask, task_manager_interface::Task)

namespace pr2_tasks
{

const double POOL_TASK_HUGE_COST = 100000;
// TODO: change this...
const int LEFT_ARM_DIMENSIONS = dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS;
const int RIGHT_ARM_DIMENSIONS = 4 + dmp::N_JOINTS;

const char* POOL_TASK_COST_LOG_FILE_NAME = "/tmp/pi2_statistics/pool_task_costs.txt";

PoolTask::PoolTask()
{
}

PoolTask::~PoolTask()
{
}


bool PoolTask::initialize(ros::NodeHandle& node_handle, int num_time_steps)
{
    node_handle_ = node_handle;
    num_time_steps_ = num_time_steps;

    // read stuff from param server
    ROS_ASSERT_FUNC(readParams());

    // initialize the dmp
    right_arm_dmp_.reset(new dmp::DynamicMovementPrimitive(node_handle_));
    dual_arm_reset_dmp_.reset(new dmp::DynamicMovementPrimitive(node_handle_));
    left_arm_dmp_.reset(new dmp::DynamicMovementPrimitive(node_handle_));
    right_arm_joint_reset_dmp_.reset(new dmp::DynamicMovementPrimitive(node_handle_));

    single_parameter_policy_.reset(new library::SingleParameterPolicy());
    num_left_arm_parameters_ = 2;
    ROS_ASSERT_FUNC(single_parameter_policy_->initialize(num_left_arm_parameters_));
    ROS_ASSERT_FUNC(single_parameter_policy_->setID(right_arm_task_dmp_id_));
    ROS_ASSERT_FUNC(single_parameter_policy_->setNumTimeSteps(num_time_steps_));
    std::vector<VectorXd> single_parameters;
    for (int i=0; i<num_left_arm_parameters_; ++i)
    {
        VectorXd single_parameter = VectorXd::Zero(1);
        single_parameters.push_back(single_parameter);
    }
    ROS_ASSERT_FUNC(single_parameter_policy_->setParameters(single_parameters));

    // call the child class initializer
    ROS_ASSERT_FUNC(taskInitialize());

    // initialize the dmp policy
    dmp_policy_.reset(new library::DMPPolicy());
    ROS_ASSERT_FUNC(dmp_policy_->initialize(right_arm_dmp_));
    ROS_ASSERT_FUNC(dmp_policy_->setID(right_arm_task_dmp_id_));
    ROS_ASSERT_FUNC(dmp_policy_->setNumTimeSteps(num_time_steps_));

    std::vector<boost::shared_ptr<library::Policy> > policies;
    policies.push_back(dmp_policy_);
    policies.push_back(single_parameter_policy_);

    mixed_policy_.reset(new library::MixedPolicy());
    ROS_ASSERT_FUNC(mixed_policy_->initialize(policies));
    ROS_ASSERT_FUNC(mixed_policy_->setNumTimeSteps(num_time_steps_));

    // initialize the dmp executor:
    ROS_ASSERT_FUNC(dmp_executor_.initialize(node_handle_));

    local_iteration_number_ = 0;

    return (initialized_ = true);
}

bool PoolTask::getPolicy(boost::shared_ptr<library::Policy>& policy)
{
    ROS_ASSERT(initialized_);
    policy = mixed_policy_;
    return true;
}

bool PoolTask::setPolicy(const boost::shared_ptr<library::Policy> policy)
{
    ROS_ASSERT(initialized_);
    mixed_policy_ = boost::dynamic_pointer_cast<library::MixedPolicy>(policy);
    return true;
}

bool PoolTask::getControlCostWeight(double& control_cost_weight)
{
    ROS_ASSERT(initialized_);
    control_cost_weight = control_cost_weight_;
    return true;
}

bool PoolTask::taskInitialize()
{
    // read stuff from param server
    ROS_ASSERT_FUNC(readParams());

    // num_dmp_dimensions_ = left_arm_dimensions + right_arm_dimensions;
    num_dmp_dimensions_ = RIGHT_ARM_DIMENSIONS;

    right_arm_start_ = VectorXd::Zero(num_dmp_dimensions_);
    right_arm_goal_ = VectorXd::Zero(num_dmp_dimensions_);

    left_arm_start_ = VectorXd::Zero(LEFT_ARM_DIMENSIONS);
    left_arm_goal_ = VectorXd::Zero(LEFT_ARM_DIMENSIONS);

    dual_arm_start_ = VectorXd::Zero(RIGHT_ARM_DIMENSIONS + LEFT_ARM_DIMENSIONS);
    dual_arm_goal_ = VectorXd::Zero(RIGHT_ARM_DIMENSIONS + LEFT_ARM_DIMENSIONS);

    // create FK solver for cost function
    right_arm_kdl_chain_wrapper_.initialize(node_handle_, root_frame_, right_arm_tip_frame_);
    right_arm_kdl_chain_wrapper_.getJointNames(joint_names_);

    // initialize persisten service client to learn dmps
    learn_dmp_service_client_
            = node_handle_.serviceClient<dmp_motion_learner::LearnCartesianSpaceDMP> ("/dmp_motion_learner/learn_cartesian_and_joint_space_dmp_from_bag_file", true);
//    learn_dmp_service_client_
//            = node_handle_.serviceClient<dmp_motion_learner::LearnDualCartesianSpaceDMP> ("/dmp_motion_learner/learn_dual_cartesian_and_joint_space_dmp_from_bag_file", true);
    ROS_ASSERT(learn_dmp_service_client_);

    if(!initialize_dmp_from_file_)
    {
        // initialize the dmp from (minimum jerk) demonstration
        // ROS_ASSERT_FUNC(learnDMPFromBagFile(TaskTransforms::LEFT_ARM_POOL_TRANSFORM, left_arm_task_dmp_id_));
        // ROS_ASSERT_FUNC(learnDMPFromBagFile(TaskTransforms::RIGHT_ARM_POOL_TRANSFORM, right_arm_task_dmp_id_));
        ROS_ASSERT_FUNC(learnDMPFromBagFile());
        // overwrite dmp and initialize from minimum jerk
        ROS_ASSERT_FUNC(right_arm_dmp_->getInitialGoal(right_arm_goal_));
        ROS_ASSERT_FUNC(right_arm_dmp_->getInitialStart(right_arm_start_));
        ROS_ASSERT_FUNC(right_arm_dmp_->learnFromMinJerk(right_arm_start_, right_arm_goal_, right_arm_movement_duration_, dmp_dt_));
    }
    else
    {
        // initialize the dmp from file
        ROS_ASSERT_FUNC(initializeDMPFromFile());
        ROS_ASSERT_FUNC(right_arm_dmp_->getInitialGoal(right_arm_goal_));
        ROS_ASSERT_FUNC(right_arm_dmp_->getInitialStart(right_arm_start_));
    }

    // initialize task recorders
    joint_states_.initialize(node_handle_, "/joint_states");
    r_gripper_accelerometer_states_.initialize(node_handle_, "/accelerometer/r_gripper_motor");
    l_gripper_accelerometer_states_.initialize(node_handle_, "/accelerometer/l_gripper_motor");
    point_clouds_.initialize(node_handle_, "/ball_point_cloud");
    // r_gripper_tactile_states_.initialize(node_handle_, "/r_gripper_sensor_observer/tactile_sensor_info");
    // l_gripper_tactile_states_.initialize(node_handle_, "/l_gripper_sensor_observer/tactile_sensor_info");

    // display waypoint in rviz
    marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);

    l_gripper_acceleration_costs_ = VectorXd::Zero(num_time_steps_);
    effort_costs_ = VectorXd::Zero(num_time_steps_);
    ball_travel_distance_costs_ = VectorXd::Zero(num_time_steps_);
    cue_stick_exploration_boundary_costs_ = VectorXd::Zero(num_time_steps_);
    ball_offset_costs_ = VectorXd::Zero(num_time_steps_);

    is_ready_to_strike_ = false;

    return true;
}

bool PoolTask::readParams()
{
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("control_cost_weight"), control_cost_weight_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("num_time_steps"), num_time_steps_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("right_arm_movement_duration"), right_arm_movement_duration_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("left_arm_movement_duration"), left_arm_movement_duration_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("execution_duration"), right_arm_execution_duration_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("left_arm_task_dmp_id"), left_arm_task_dmp_id_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("right_arm_task_dmp_id"), right_arm_task_dmp_id_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("dual_arm_reset_dmp_id"), dual_arm_reset_dmp_id_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("reset_movement_duration"), reset_movement_duration_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("right_arm_joint_reset_movement_duration"), right_arm_joint_reset_movement_duration_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("dmp_sampling_frequency"), dmp_sampling_frequency_));
    if(dmp_sampling_frequency_ < 1e-5)
    {
        ROS_ERROR("Sampling frequency (%f) not valid.", dmp_sampling_frequency_);
        return false;
    }
    dmp_dt_ = static_cast<double>(1.0) / dmp_sampling_frequency_;

    // we have extra time steps due to the discrepancy between movement_duration and execution_duration:
    num_time_steps_total_ = int((right_arm_execution_duration_ / right_arm_movement_duration_) * num_time_steps_);
    // ROS_INFO("total time steps = %d", num_time_steps_total_);

    execution_duration_ros_ = ros::Duration(right_arm_execution_duration_);
    execution_timeout_ros_ = ros::Duration(right_arm_execution_duration_*1.5);

    ROS_ASSERT_FUNC(read(node_handle_, std::string("root_frame"), root_frame_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("right_arm_tip_frame"), right_arm_tip_frame_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("demonstration_bag_file"), demonstration_bag_file_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("left_arm_demonstration_file_name"), left_arm_demonstration_file_name_));

    demonstration_bag_file_ = ros::package::getPath("pr2_tasks") + std::string("/") + demonstration_bag_file_;
    left_arm_demonstration_file_name_ = ros::package::getPath("pr2_tasks") + std::string("/") + left_arm_demonstration_file_name_;

    ROS_ASSERT_FUNC(readEigenVector(node_handle_, "cue_offset", cue_offset_));

    ROS_ASSERT_FUNC(read(node_handle_, std::string("l_gripper_acceleration_weight"), l_gripper_acceleration_weight_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("effort_cost_weight"), effort_cost_weight_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("ball_travel_distance_weight"), ball_travel_distance_weight_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("ball_offset_weight"), ball_offset_weight_));

    ROS_ASSERT_FUNC(read(node_handle_, std::string("max_backward_distance"), max_backward_distance_));

    ROS_ASSERT_FUNC(read(node_handle_, std::string("initialize_dmp_from_file"), initialize_dmp_from_file_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("initial_dmp_file_name"), initial_dmp_file_name_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("initial_spp_file_name"), initial_spp_file_name_));

    ROS_ASSERT_FUNC(readStringArraySpaceSeparated(node_handle_, std::string("effort_cost_joint_names"), effort_cost_joint_names_));

    return true;
}

bool PoolTask::execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number)
{
    ros::spinOnce();
    ROS_ASSERT(initialized_);

    iteration_number_ = iteration_number;

    std::vector<VectorXd> dmp_parameters;
    dmp_parameters.insert(dmp_parameters.begin(), parameters.begin(), parameters.begin() + num_dmp_dimensions_);

    std::vector<VectorXd> single_parameters;
    single_parameters.insert(single_parameters.begin(), parameters.begin() + num_dmp_dimensions_, parameters.end());
    ROS_ASSERT_FUNC(setLeftArmPose(single_parameters));

    // setup the dmp with the new parameters
    ROS_ASSERT_FUNC(right_arm_dmp_->setThetas(dmp_parameters));
    ROS_ASSERT_FUNC(right_arm_dmp_->setup(dmp_sampling_frequency_));
    ROS_ASSERT_FUNC(right_arm_dmp_->setDuration(right_arm_movement_duration_, dmp_sampling_frequency_));

    bool offline_check_succeeded = false;
    ROS_ASSERT_FUNC(checkOfflineExecution(offline_check_succeeded, costs));
    if(offline_check_succeeded)
    {
        // reset canonical system
        ROS_ASSERT_FUNC(right_arm_dmp_->setup(dmp_sampling_frequency_));

        // execute dmp
        ROS_ASSERT_FUNC(execute());

        // calculate the cost:
        ROS_ASSERT_FUNC(computeCosts(costs));
    }
    return true;
}

bool PoolTask::setLeftArmPose(std::vector<VectorXd>& single_parameters)
{
    if((int)single_parameters.size() != num_left_arm_parameters_)
    {
        ROS_ERROR("(int)single_parameters.size() %i !=  %i num_left_arm_parameters_", (int)single_parameters.size(), num_left_arm_parameters_);
        ROS_ASSERT((int)single_parameters.size() == num_left_arm_parameters_);
    }

    double x_offset =  single_parameters[0](0);
    double yaw_offset = single_parameters[1](0);

    ROS_INFO("Left arm x offset is %f", x_offset);
    ROS_INFO("Left arm yaw offset is %f", yaw_offset);

    KDL::Frame left_gripper_offset_frame;
    left_gripper_offset_frame.p.x(x_offset);
    KDL::Rotation left_gripper_rotation = KDL::Rotation::RotZ(yaw_offset);
    left_gripper_offset_frame.M = left_gripper_rotation;

    KDL::Frame updated_left_arm_goal_frame = left_gripper_initial_start_frame_ * left_gripper_offset_frame;

    dual_arm_goal_(RIGHT_ARM_DIMENSIONS + dmp::_XX_) = updated_left_arm_goal_frame.p.x();
    dual_arm_goal_(RIGHT_ARM_DIMENSIONS + dmp::_YY_) = updated_left_arm_goal_frame.p.y();
    dual_arm_goal_(RIGHT_ARM_DIMENSIONS + dmp::_ZZ_) = updated_left_arm_goal_frame.p.z();
    updated_left_arm_goal_frame.M.GetQuaternion(dual_arm_goal_(RIGHT_ARM_DIMENSIONS + dmp::N_CART + dmp::_QX_),
                                                dual_arm_goal_(RIGHT_ARM_DIMENSIONS + dmp::N_CART + dmp::_QY_),
                                                dual_arm_goal_(RIGHT_ARM_DIMENSIONS + dmp::N_CART + dmp::_QZ_),
                                                dual_arm_goal_(RIGHT_ARM_DIMENSIONS + dmp::N_CART + dmp::_QW_));

    return true;
}

bool PoolTask::execute()
{
    // call pre-execute to reset the system and start logging:
    ROS_ASSERT_FUNC(preExecute());

    ROS_ASSERT_FUNC(left_arm_dmp_->setup(dmp_sampling_frequency_));
    left_arm_dmp_->unsetStart();

    // TODO: remove spin_sleeps...
    spin_sleep(ros::Duration(0.1));
    ROS_ASSERT_FUNC(dmp_executor_.executeDMPAndWait(*right_arm_dmp_,
                                                    execution_statistics_,
                                                    execution_duration_ros_,
                                                    execution_timeout_ros_,
                                                    right_arm_movement_duration_,
                                                    *left_arm_dmp_,
                                                    left_arm_movement_duration_,
                                                    false, iteration_number_));
    spin_sleep(ros::Duration(0.1));

    // call post-execute to stop logging:
    ROS_ASSERT_FUNC(postExecute());
    return true;
}

bool PoolTask::preExecute()
{
    // reset the system
    if(!is_ready_to_strike_)
    {
        ROS_INFO("Learning reset movement.");
        ROS_ASSERT_FUNC(learnLeftArmDMPFromBagFile());
        ROS_ASSERT_FUNC(learnResetDMP());
        ROS_ASSERT_FUNC(learnRightArmJointResetDMPFromBagFile());
        ROS_ASSERT_FUNC(reset());
    }

    // start logging
    ROS_ASSERT_FUNC(joint_states_.startRecording());
    ROS_ASSERT_FUNC(r_gripper_accelerometer_states_.startRecording());
    ROS_ASSERT_FUNC(l_gripper_accelerometer_states_.startRecording());
    ROS_ASSERT_FUNC(point_clouds_.startRecording());
    // ROS_ASSERT_FUNC(r_gripper_tactile_states_.startRecording());
    // ROS_ASSERT_FUNC(l_gripper_tactile_states_.startRecording());

    return true;
}

bool PoolTask::postExecute()
{
    ROS_ASSERT_FUNC(reset());
    is_ready_to_strike_ = true;

    // stop logging
    ROS_ASSERT_FUNC(joint_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, right_arm_movement_duration_, num_time_steps_total_, effort_cost_joint_names_));
    std::vector<std::string> left_arm;
    left_arm.push_back(std::string("left"));
    ROS_ASSERT_FUNC(r_gripper_accelerometer_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, right_arm_movement_duration_, num_time_steps_total_, left_arm));
    std::vector<std::string> right_arm;
    right_arm.push_back(std::string("right"));
    ROS_ASSERT_FUNC(l_gripper_accelerometer_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, right_arm_movement_duration_, num_time_steps_total_, right_arm));
    std::vector<std::string> empty_vector;
    ROS_ASSERT_FUNC(point_clouds_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, right_arm_movement_duration_, num_time_steps_total_, empty_vector));
    // ROS_ASSERT_FUNC(r_gripper_tactile_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, movement_duration_, num_time_steps_total_, empty_vector));
    // ROS_ASSERT_FUNC(l_gripper_tactile_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, movement_duration_, num_time_steps_total_, empty_vector));
    return true;
}


bool PoolTask::computeCosts(VectorXd& costs)
{
    ROS_ASSERT_FUNC(computeEffortCost());
    ROS_ASSERT_FUNC(computeLeftGripperAccelerationCost());

    ROS_ASSERT_FUNC(computeBallTravelDistanceFromLaser());
    ROS_ASSERT_FUNC(computeCueStickExplorationBoundaries());

    costs.setZero(num_time_steps_);
    costs = costs + l_gripper_acceleration_costs_;
    costs = costs + effort_costs_;
    costs = costs + ball_offset_costs_;
    costs = costs + ball_travel_distance_costs_;
    costs = costs + cue_stick_exploration_boundary_costs_;
    publishCosts(costs);

    cost_log_file_.open(POOL_TASK_COST_LOG_FILE_NAME, std::fstream::out | std::fstream::app);
    cost_log_file_ << local_iteration_number_ << std::endl;
    local_iteration_number_++;
    cost_log_file_ << iteration_number_ << std::endl;
    cost_log_file_ << ball_travel_distance_costs_.sum() << std::endl;
    cost_log_file_ << ball_offset_costs_.sum() << std::endl;
    cost_log_file_ << cue_stick_exploration_boundary_costs_.sum() << std::endl;
    cost_log_file_ << std::endl;
    cost_log_file_.close();

    return true;
}

bool PoolTask::computeLeftGripperAccelerationCost()
{
    l_gripper_acceleration_costs_.setZero(num_time_steps_);

    // add up the accelerations in the left gripper only for the time of the DMP being executed
    VectorXd l_gripper_accelerometer_data = VectorXd::Zero(num_time_steps_);
    for(int i=0; i<num_time_steps_; ++i)
    {
        l_gripper_accelerometer_data(i) = pow(l_gripper_accelerometer_states_.messages_[i].samples[0].x, 2)
                + pow(l_gripper_accelerometer_states_.messages_[i].samples[0].y, 2)
                + pow(l_gripper_accelerometer_states_.messages_[i].samples[0].z, 2);
    }
    l_gripper_acceleration_costs_ = l_gripper_accelerometer_data * l_gripper_acceleration_weight_;
    return true;
}

bool PoolTask::computeEffortCost()
{
    effort_costs_.setZero(num_time_steps_);

    // only add up the efforts in the right arm only for the time of the DMP being executed
    int num_joints = static_cast<int>(joint_states_.messages_[0].name.size());
    MatrixXd joint_effort_data = MatrixXd::Zero(num_time_steps_, num_joints);
    for(int i=0; i<num_time_steps_; ++i)
    {
        for (int j=0; j<num_joints; ++j)
        {
            joint_effort_data(i, j) = joint_states_.messages_[i].effort[j];
        }
    }
    effort_costs_ = joint_effort_data.rowwise().squaredNorm();
    effort_costs_ = effort_costs_ * effort_cost_weight_;
    return true;
}

bool PoolTask::computeBallTravelDistanceFromSecondImpact()
{
    ball_travel_distance_costs_.setZero(num_time_steps_);
    ROS_ASSERT(r_gripper_accelerometer_states_.times_.size() == 2);
    ball_travel_time_ = r_gripper_accelerometer_states_.times_[1].toSec() - r_gripper_accelerometer_states_.times_[0].toSec();

    ROS_INFO("Ball travel time (using second impact) is %f seconds.", ball_travel_time_);
    double cost = pow(1.0 / ball_travel_time_, 2);
    cost *= ball_travel_distance_weight_;
    ball_travel_distance_costs_(num_time_steps_-1) = cost;

    return true;
}

bool PoolTask::computeBallTravelDistanceFromLaser()
{
    ball_travel_time_ = right_arm_execution_duration_;
    double ball_offset = 0.5;

    if(point_clouds_.messages_.size() == 1)
    {
        ROS_ASSERT(point_clouds_.data_.size() == 1);
        ROS_ASSERT(point_clouds_.times_.size() == 1);
        ROS_ASSERT(r_gripper_accelerometer_states_.times_.size() > 0);

        ball_travel_time_ = point_clouds_.times_[0].toSec() - r_gripper_accelerometer_states_.times_[0].toSec();
        ball_offset = point_clouds_.data_[0];
        crossed_laser_scan_line_ = true;
    }
    else if(point_clouds_.messages_.size() == 0)
    {
        ROS_WARN("Ball did not cross scan line.");
        crossed_laser_scan_line_ = false;
    }
    else
    {
        ROS_ERROR("Size of point cloud messages (%i) is incorrect.", static_cast<int>(point_clouds_.messages_.size()));
        return false;
    }

    double cost = pow(ball_travel_time_, 2);
    cost *= ball_travel_distance_weight_;
    ball_travel_distance_costs_(num_time_steps_-1) = cost;

    cost = pow(ball_offset, 2);
    cost *= ball_offset_weight_;
    ball_offset_costs_(num_time_steps_-1) = cost;

    return true;
}

bool PoolTask::computeCueStickExplorationBoundaries()
{
    cue_stick_exploration_boundary_costs_.setZero(num_time_steps_);
    KDL::Frame initial_frame;
    ROS_ASSERT_FUNC(right_arm_kdl_chain_wrapper_.forwardKinematics(joint_states_.messages_[0], initial_frame));
    double cost = 0;
    KDL::Frame frame;
    for(int i=1; i<num_time_steps_; ++i)
    {
        // TODO: make this applicable when cue is not aligned with y axis.
        ROS_ASSERT_FUNC(right_arm_kdl_chain_wrapper_.forwardKinematics(joint_states_.messages_[i], frame));
        if(frame.p.y() < (initial_frame.p.y() + max_backward_distance_))
        {
            cost = POOL_TASK_HUGE_COST;
            break;
        }
    }
    cue_stick_exploration_boundary_costs_(num_time_steps_-1) = cost;
    return true;
}

bool PoolTask::checkOfflineExecution(bool& offline_check_succeeded, VectorXd& costs)
{
    VectorXd desired_coordinates = VectorXd::Zero(right_arm_dmp_->getNumTransformationSystems() * dmp::POS_VEL_ACC);
    int check_index = 0; // position trace of the dmp in pool task
    double start = 0;
    double desired_pos = 0;
    bool movement_finished = false;
    bool got_start_coordinates = false;
    while (!movement_finished)
    {
        ROS_ASSERT_FUNC(right_arm_dmp_->propagateStep(desired_coordinates, movement_finished));
        desired_pos = desired_coordinates(check_index);
        if(!got_start_coordinates)
        {
            got_start_coordinates = true;
            start = desired_pos;
        }
        if((start - desired_pos) < max_backward_distance_)
        {
            costs(num_time_steps_-1) = POOL_TASK_HUGE_COST;
            offline_check_succeeded = false;
            ROS_WARN("Boundary violation detected. Penalizing without execution (start=%f current=%f diff=%f max=%f).",
                     start, desired_pos, desired_pos - start, max_backward_distance_);
            return true;
        }
    }
    offline_check_succeeded = true;
    return true;
}

bool PoolTask::learnResetDMP()
{
    ROS_INFO("Learning dual arm dmp.");
    ROS_ASSERT_FUNC(dual_arm_reset_dmp_->initialize(LEFT_ARM_DIMENSIONS+RIGHT_ARM_DIMENSIONS, dual_arm_reset_dmp_id_));

    for (int i=0; i<RIGHT_ARM_DIMENSIONS; ++i)
    {
        dual_arm_goal_(i) = right_arm_start_(i);
        dual_arm_start_(i) = right_arm_goal_(i);
    }
    for (int i=0; i<LEFT_ARM_DIMENSIONS; ++i)
    {
        dual_arm_goal_(RIGHT_ARM_DIMENSIONS + i) = left_arm_start_(i);
        dual_arm_start_(RIGHT_ARM_DIMENSIONS + i) = left_arm_goal_(i);
    }

    // learn from min jerk
    ROS_ASSERT_FUNC(dual_arm_reset_dmp_->learnFromMinJerk(dual_arm_start_, dual_arm_goal_, reset_movement_duration_, dmp_dt_));
    return true;
}

bool PoolTask::reset()
{
    // setup reset dmp
    ROS_ASSERT_FUNC(dual_arm_reset_dmp_->setup(dual_arm_goal_, reset_movement_duration_, dmp_sampling_frequency_));
    dual_arm_reset_dmp_->unsetStart();

    ros::Duration execution_duration = ros::Duration(reset_movement_duration_);
    ros::Duration timeout = ros::Duration(2*reset_movement_duration_);

    dmp_motion_controller::DMPExecutionStatistics unused_execution_statistics;

    ROS_ASSERT_FUNC(dmp_executor_.executeDMPAndWait(*dual_arm_reset_dmp_,
                                                    TaskTransforms::LEFT_ARM_POOL_TRANSFORM,
                                                    TaskTransforms::RIGHT_ARM_POOL_TRANSFORM,
                                                    unused_execution_statistics,
                                                    execution_duration, timeout));

    ROS_ASSERT_FUNC(resetRightArmConfiguration());

    return true;
}

bool PoolTask::initializeDMPFromFile()
{
    ROS_ASSERT_FUNC(right_arm_dmp_->readFromDisc(initial_dmp_file_name_));
    ROS_ASSERT_FUNC(single_parameter_policy_->readFromDisc(initial_spp_file_name_));
    return true;
}

bool PoolTask::resetRightArmConfiguration()
{

    // setup reset dmp
    ROS_ASSERT_FUNC(right_arm_joint_reset_dmp_->setup(dmp_sampling_frequency_));
    right_arm_joint_reset_dmp_->unsetStart();

    double right_arm_reset_duration = 2.0;
    ros::Duration r_arm_reset_execution_duration = ros::Duration(right_arm_reset_duration);
    ros::Duration timeout = ros::Duration(4.0*right_arm_reset_duration);

    ROS_ASSERT_FUNC(dmp_executor_.executeJointDMPAndWait(*right_arm_joint_reset_dmp_, r_arm_reset_execution_duration, timeout));

    return true;
}

bool PoolTask::learnDMPFromBagFile()
{
//    dmp_motion_learner::LearnDualCartesianSpaceDMP::Request learn_dmp_request;
//    dmp_motion_learner::LearnDualCartesianSpaceDMP::Response learn_dmp_response;
//
//    learn_dmp_request.left_arm_type = static_cast<int>(TaskTransforms::LEFT_ARM_POOL_TRANSFORM);
//    learn_dmp_request.right_arm_type = static_cast<int>(TaskTransforms::RIGHT_ARM_POOL_TRANSFORM);
//    learn_dmp_request.bag_file_name = demonstration_bag_file_;
//    learn_dmp_request.data_directory_name = "/tmp";
//    learn_dmp_request.dmp_id = left_arm_task_dmp_id_;
//
//    ROS_ASSERT_FUNC(learn_dmp_service_client_.call(learn_dmp_request, learn_dmp_response));
//    if (learn_dmp_response.return_code != dmp_motion_learner::LearnDualCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL)
//    {
//        ROS_ERROR("Learn cartesian space DMP from bag file service call was not successful.");
//        return false;
//    }
//
//    ROS_ASSERT_FUNC(dmp_->initFromMessage(learn_dmp_response.dmp));

    dmp_motion_learner::LearnCartesianSpaceDMP::Request learn_dmp_request;
    dmp_motion_learner::LearnCartesianSpaceDMP::Response learn_dmp_response;

    learn_dmp_request.type = static_cast<int>(TaskTransforms::RIGHT_ARM_POOL_TRANSFORM);
    learn_dmp_request.bag_file_name = demonstration_bag_file_;
    learn_dmp_request.data_directory_name = "/tmp";
    learn_dmp_request.dmp_id = right_arm_task_dmp_id_;

    ROS_ASSERT_FUNC(learn_dmp_service_client_.call(learn_dmp_request, learn_dmp_response));
    if (learn_dmp_response.return_code != dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL)
    {
        ROS_ERROR("Learn cartesian space DMP from bag file service call was not successful.");
        return false;
    }

    ROS_ASSERT_FUNC(right_arm_dmp_->initFromMessage(learn_dmp_response.dmp));

    return true;
}

bool PoolTask::learnLeftArmDMPFromBagFile()
{

    ROS_INFO("Learning left arm DMP.");
    ROS_ASSERT_FUNC(left_arm_dmp_->initialize(LEFT_ARM_DIMENSIONS, left_arm_task_dmp_id_));

    dmp_motion_learner::LearnCartesianSpaceDMP::Request learn_dmp_request;
    dmp_motion_learner::LearnCartesianSpaceDMP::Response learn_dmp_response;

    learn_dmp_request.type = static_cast<int>(TaskTransforms::LEFT_ARM_POOL_TRANSFORM);
    learn_dmp_request.bag_file_name = left_arm_demonstration_file_name_;
    learn_dmp_request.data_directory_name = "/tmp";
    learn_dmp_request.dmp_id = left_arm_task_dmp_id_;

    ROS_ASSERT_FUNC(learn_dmp_service_client_.call(learn_dmp_request, learn_dmp_response));
    if (learn_dmp_response.return_code != dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL)
    {
        ROS_ERROR("Learn left arm cartesian space DMP from bag file service call was not successful.");
        return false;
    }

    ROS_ASSERT_FUNC(left_arm_dmp_->initFromMessage(learn_dmp_response.dmp));

    ROS_ASSERT_FUNC(left_arm_dmp_->getInitialStart(left_arm_start_));
    ROS_ASSERT_FUNC(left_arm_dmp_->getInitialGoal(left_arm_goal_));

    left_gripper_initial_start_frame_.p.x(left_arm_start_(dmp::_XX_));
    left_gripper_initial_start_frame_.p.y(left_arm_start_(dmp::_YY_));
    left_gripper_initial_start_frame_.p.z(left_arm_start_(dmp::_ZZ_));
    KDL::Rotation left_gripper_goal_rotation = KDL::Rotation::Quaternion(left_arm_start_(dmp::N_CART+dmp::_QX_),
                                                                         left_arm_start_(dmp::N_CART+dmp::_QY_),
                                                                         left_arm_start_(dmp::N_CART+dmp::_QZ_),
                                                                         left_arm_start_(dmp::N_CART+dmp::_QW_));
    left_gripper_initial_start_frame_.M = left_gripper_goal_rotation;

    // learn from min jerk
    ROS_ASSERT_FUNC(left_arm_dmp_->learnFromMinJerk(left_arm_start_, left_arm_goal_, left_arm_movement_duration_, dmp_dt_));

    return true;
}

bool PoolTask::learnRightArmJointResetDMPFromBagFile()
{

    ROS_INFO("Learning right arm joint reset DMP.");
    ROS_ASSERT_FUNC(right_arm_joint_reset_dmp_->initialize(dmp::N_JOINTS, 42));

    VectorXd right_arm_joint_start = VectorXd::Zero(dmp::N_JOINTS);
    VectorXd right_arm_joint_goal = VectorXd::Zero(dmp::N_JOINTS);

    for (int i=0; i<dmp::N_JOINTS; ++i)
    {
        right_arm_joint_start[i] = right_arm_goal_[RIGHT_ARM_DIMENSIONS - dmp::N_JOINTS + i];
        right_arm_joint_goal[i] = right_arm_start_[RIGHT_ARM_DIMENSIONS - dmp::N_JOINTS + i];
    }

    // learn from min jerk
    ROS_ASSERT_FUNC(right_arm_joint_reset_dmp_->learnFromMinJerk(right_arm_joint_goal, right_arm_joint_goal, right_arm_joint_reset_movement_duration_, dmp_dt_));

    return true;
}

void PoolTask::publishText(const std::string& text, const Vector3d& color, const int row_index)
{

    const double font_size = 0.08;

    visualization_msgs::Marker marker;
    marker.header.frame_id = root_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "PoolTask";
    marker.id = row_index + 42;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 1.1 - (row_index*(font_size+0.03));
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = font_size;
    marker.scale.y = font_size;
    marker.scale.z = font_size;
    marker.lifetime = ros::Duration();
    marker.color.a = 1.0;

    marker.text = text;
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);

    marker_publisher_.publish(marker);

}

void PoolTask::publishCosts(const VectorXd& costs)
{

    std::string display_text;
    Vector3d color;
    color(0) = 1;
    color(1) = 1;
    color(2) = 1;

    std::string info("");
    std::stringstream ss;

    int precision = 3;
    ss.precision(precision);
    ss << std::fixed;

    try
    {
        ss.str("");
        ss.clear();
        ss << iteration_number_;
        display_text.assign(std::string("iteration ") + ss.str());
        ss.str("");
        ss.clear();
        ss << costs.sum();
        display_text.append(std::string(" - total cost: ") + ss.str());
        publishText(display_text, color, 0);

//        ss.str("");
//        ss.clear();
//        ss << effort_costs_.sum();
//        display_text.assign(std::string("effort cost: ") + ss.str());
//        publishText(display_text, color, 1);
//
//        ss.str("");
//        ss.clear();
//        ss << l_gripper_acceleration_costs_.sum();
//        display_text.assign(std::string("left gripper acceleration cost: ") + ss.str());
//        publishText(display_text, color, 2);

        ss.str("");
        ss.clear();
        ss << ball_travel_distance_costs_.sum();
        display_text.assign(std::string("ball speed cost: ") + ss.str());
        ss.str("");
        ss.clear();
        ss << ball_travel_time_;
        display_text.append(std::string(" (") + ss.str() + std::string(" sec)"));
        publishText(display_text, color, 1);

        ss.str("");
        ss.clear();
        ss << ball_offset_costs_.sum();
        display_text.assign(std::string("ball offset cost: ") + ss.str());
        if(crossed_laser_scan_line_)
        {
            ss.str("");
            ss.clear();
            ss << point_clouds_.data_[0];
            display_text.append(std::string(" (") + ss.str() + std::string(")"));
        }
        else
        {
            display_text.append(std::string(" (failed)"));
        }
        publishText(display_text, color, 2);

//        if( cue_stick_exploration_boundary_costs_.sum() > 0.1)
//        {
            ss.str("");
            ss.clear();
            ss << cue_stick_exploration_boundary_costs_.sum();
            display_text.assign(std::string("cue stick boundary violation cost: ") + ss.str());
            publishText(display_text, color, 3);
//        }

    }
    catch(boost::bad_lexical_cast &)
    {
        ROS_ERROR("Could not convert costs to string for display purposes.");
    }

}

}
