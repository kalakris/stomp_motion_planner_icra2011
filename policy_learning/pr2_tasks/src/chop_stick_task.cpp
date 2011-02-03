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

/** \author Peter Pastor */

// system includes

// ros includes
#include <ros/ros.h>
#include <ros/assert.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/spin.h>

#include <visualization_msgs/Marker.h>

#include <dmp_motion_learner/LearnDualCartesianSpaceDMP.h>

#include <dmp_motion_controller/AddToExecuteDMPQueue.h>
#include <dmp_motion_controller/WriteTrajectories.h>

#include <boost/lexical_cast.hpp>

#include <angles/angles.h>

// local includes
#include <pr2_tasks/chop_stick_task.h>
#include <task_manager/task_manager.h>

using namespace dmp;
using namespace policy_improvement_utilities;
using boost::lexical_cast;
using namespace pr2_tasks_transforms;
USING_PART_OF_NAMESPACE_EIGEN

PLUGINLIB_DECLARE_CLASS(pr2_tasks, ChopStickTask, pr2_tasks::ChopStickTask, task_manager_interface::Task)

namespace pr2_tasks
{

const int LEFT_ARM_DIMENSIONS = dmp::N_CART + dmp::N_CART + dmp::N_JOINTS;
const int RIGHT_ARM_DIMENSIONS = dmp::N_CART + dmp::N_CART + dmp::N_JOINTS;

const char* CHOP_STICK_TASK_COST_LOG_FILE_NAME = "/tmp/pi2_statistics/chop_stick_costs.txt";

ChopStickTask::ChopStickTask()
{
}

ChopStickTask::~ChopStickTask()
{
}

bool ChopStickTask::initialize(ros::NodeHandle& node_handle, int num_time_steps)
{
    node_handle_ = node_handle;
    num_time_steps_ = num_time_steps;

    // read stuff from param server
    ROS_ASSERT_FUNC(readParams());

    // initialize the dmp
    dmp_.reset(new dmp::DynamicMovementPrimitive(node_handle_));
    reset_dmp_.reset(new dmp::DynamicMovementPrimitive(node_handle_));

    ROS_ASSERT_FUNC(taskInitialize());

    // initialize the dmp policy
    dmp_policy_.reset(new library::DMPPolicy());
    ROS_ASSERT_FUNC(dmp_policy_->initialize(dmp_));
    ROS_ASSERT_FUNC(dmp_policy_->setID(task_dmp_id_));
    ROS_ASSERT_FUNC(dmp_policy_->setNumTimeSteps(num_time_steps_));

    // initialize the dmp executor:
    ROS_ASSERT_FUNC(dmp_executor_.initialize(node_handle_));

    local_iteration_number_ = 0;
    return (initialized_ = true);
}

bool ChopStickTask::getPolicy(boost::shared_ptr<library::Policy>& policy)
{
    ROS_ASSERT(initialized_);
    policy = dmp_policy_;
    return true;
}

bool ChopStickTask::setPolicy(const boost::shared_ptr<library::Policy> policy)
{
    ROS_ASSERT(initialized_);
    dmp_policy_ = boost::dynamic_pointer_cast<library::DMPPolicy>(policy);
    return true;
}

bool ChopStickTask::getControlCostWeight(double& control_cost_weight)
{
    ROS_ASSERT(initialized_);
    control_cost_weight = control_cost_weight_;
    return true;
}

bool ChopStickTask::taskInitialize()
{
    // read stuff from param server
	//    ROS_ASSERT_FUNC(readParams());

    num_dmp_dimensions_ = RIGHT_ARM_DIMENSIONS + LEFT_ARM_DIMENSIONS;

    dual_arm_dmp_start_ = VectorXd::Zero(num_dmp_dimensions_);
    dual_arm_dmp_goal_ = VectorXd::Zero(num_dmp_dimensions_);

    dual_arm_joint_reset_dmp_start_ = VectorXd::Zero(dmp::N_JOINTS + dmp::N_JOINTS);
    dual_arm_joint_reset_dmp_goal_ = VectorXd::Zero(dmp::N_JOINTS + dmp::N_JOINTS);

    // create FK solver for cost function
    right_arm_kdl_chain_wrapper_.initialize(node_handle_, root_frame_, right_arm_tip_frame_);
    right_arm_kdl_chain_wrapper_.getJointNames(r_arm_effort_cost_joint_names_);
    left_arm_kdl_chain_wrapper_.initialize(node_handle_, root_frame_, left_arm_tip_frame_);
    left_arm_kdl_chain_wrapper_.getJointNames(l_arm_effort_cost_joint_names_);

    // initialize persisten service client to learn dmps
    // learn_dmp_service_client_ = node_handle_.serviceClient<dmp_motion_learner::LearnCartesianSpaceDMP> ("/dmp_motion_learner/learn_cartesian_and_joint_space_dmp_from_bag_file", true);
    learn_dmp_service_client_ = node_handle_.serviceClient<dmp_motion_learner::LearnDualCartesianSpaceDMP> ("/dmp_motion_learner/learn_dual_cartesian_and_joint_space_dmp_from_bag_file", true);
    ROS_ASSERT(learn_dmp_service_client_);

    if(!initialize_dmp_from_file_)
    {
        // initialize the dmp from (minimum jerk) demonstration
        ROS_ASSERT_FUNC(learnTaskDMPFromBagFile());
        // overwrite dmp and initialize from minimum jerk
        ROS_ASSERT_FUNC(dmp_->getInitialGoal(dual_arm_dmp_goal_));
        ROS_ASSERT_FUNC(dmp_->getInitialStart(dual_arm_dmp_start_));
        // we no longer use the minimum jerk initialization
        // ROS_ASSERT_FUNC(dmp_->learnFromMinJerk(dual_arm_dmp_start_, dual_arm_dmp_goal_, task_movement_duration_, dmp_dt_));
    }
    else
    {
        // initialize the dmp from file
        ROS_ASSERT_FUNC(initializeDMPFromFile());
        ROS_ASSERT_FUNC(dmp_->getInitialGoal(dual_arm_dmp_goal_));
        ROS_ASSERT_FUNC(dmp_->getInitialStart(dual_arm_dmp_start_));
    }

    // initialize task recorders
    joint_states_.initialize(node_handle_, "/joint_states");
    r_gripper_accelerometer_states_.initialize(node_handle_, "/accelerometer/r_gripper_motor");
    l_gripper_accelerometer_states_.initialize(node_handle_, "/accelerometer/l_gripper_motor");
    imu_states_.initialize(node_handle_, "/imu/data");
    // r_gripper_tactile_states_.initialize(node_handle_, "/r_gripper_sensor_observer/tactile_sensor_info");
    // l_gripper_tactile_states_.initialize(node_handle_, "/l_gripper_sensor_observer/tactile_sensor_info");

    // display waypoint in rviz
    marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);

    l_gripper_acceleration_costs_ = VectorXd::Zero(num_time_steps_);
    r_gripper_acceleration_costs_ = VectorXd::Zero(num_time_steps_);
    l_arm_effort_costs_ = VectorXd::Zero(num_time_steps_);
    r_arm_effort_costs_ = VectorXd::Zero(num_time_steps_);
    l_gripper_pressure_costs_ = VectorXd::Zero(num_time_steps_);
    r_gripper_pressure_costs_ = VectorXd::Zero(num_time_steps_);
    imu_costs_ = VectorXd::Zero(num_time_steps_);

    is_ready_ = false;

    return true;
}

bool ChopStickTask::readParams()
{

    ROS_ASSERT_FUNC(read(node_handle_, std::string("right_arm_tip_frame"), right_arm_tip_frame_));

    ROS_ASSERT_FUNC(read(node_handle_, std::string("num_time_steps"), num_time_steps_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("task_movement_duration"), task_movement_duration_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("task_execution_duration"), task_execution_duration_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("task_dmp_id"), task_dmp_id_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("reset_dmp_id"), reset_dmp_id_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("reset_movement_duration"), reset_movement_duration_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("reset_execution_duration"), reset_execution_duration_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("dmp_sampling_frequency"), dmp_sampling_frequency_));
    if(dmp_sampling_frequency_ < 1e-5)
    {
        ROS_ERROR("Sampling frequency (%f) not valid.", dmp_sampling_frequency_);
        return false;
    }
    dmp_dt_ = static_cast<double>(1.0) / dmp_sampling_frequency_;

    // we have extra time steps due to the discrepancy between movement_duration and execution_duration:
    num_time_steps_total_ = int((task_execution_duration_ / task_movement_duration_) * num_time_steps_);
    ROS_INFO("total time steps = %d", num_time_steps_total_);

    task_execution_duration_ros_ = ros::Duration(task_execution_duration_);
    task_execution_timeout_duration_ros_ = ros::Duration(task_execution_duration_*1.5);
    reset_execution_duration_ros_ = ros::Duration(reset_execution_duration_);
    reset_execution_timeout_duration_ros_ = ros::Duration(reset_execution_duration_*1.5);

    ROS_ASSERT_FUNC(read(node_handle_, std::string("root_frame"), root_frame_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("left_arm_tip_frame"), left_arm_tip_frame_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("right_arm_tip_frame"), right_arm_tip_frame_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("demonstration_bag_file"), demonstration_bag_file_));
    demonstration_bag_file_ = ros::package::getPath("pr2_tasks") + std::string("/") + demonstration_bag_file_;

    ROS_ASSERT_FUNC(read(node_handle_, std::string("control_cost_weight"), control_cost_weight_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("l_gripper_acceleration_weight"), l_gripper_acceleration_weight_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("r_gripper_acceleration_weight"), r_gripper_acceleration_weight_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("l_gripper_pressure_weight"), l_gripper_pressure_weight_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("r_gripper_pressure_weight"), r_gripper_pressure_weight_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("l_arm_effort_weight"), l_arm_effort_weight_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("r_arm_effort_weight"), r_arm_effort_weight_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("imu_weight"), imu_weight_));

    ROS_ASSERT_FUNC(read(node_handle_, std::string("initialize_dmp_from_file"), initialize_dmp_from_file_));
    ROS_ASSERT_FUNC(read(node_handle_, std::string("initial_dmp_file_name"), initial_dmp_file_name_));

    // ROS_ASSERT_FUNC(readStringArraySpaceSeparated(node_handle_, std::string("r_arm_effort_cost_joint_names"), r_arm_effort_cost_joint_names_));
    // ROS_ASSERT_FUNC(readStringArraySpaceSeparated(node_handle_, std::string("l_arm_effort_cost_joint_names"), l_arm_effort_cost_joint_names_));

    return true;
}

bool ChopStickTask::execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number)
{
    ros::spinOnce();
    ROS_ASSERT(initialized_);

    iteration_number_ = iteration_number;

    // setup the dmp with the new parameters
    ROS_ASSERT_FUNC(dmp_->setThetas(parameters));
    ROS_ASSERT_FUNC(dmp_->setup(dmp_sampling_frequency_));
    ROS_ASSERT_FUNC(dmp_->setDuration(task_movement_duration_, dmp_sampling_frequency_));

    // reset canonical system
    ROS_ASSERT_FUNC(dmp_->setup(dmp_sampling_frequency_));

    // execute dmp
    ROS_ASSERT_FUNC(execute());

    // calculate the cost:
    ROS_ASSERT_FUNC(computeCosts(costs));
    return true;

}

bool ChopStickTask::execute()
{
    // call pre-execute to reset the system and start logging:
    ROS_ASSERT_FUNC(preExecute());

    // TODO: remove spin_sleeps...
    spin_sleep(ros::Duration(0.1));

    // Testing...
    // dmp_->unsetStart();

    ROS_ASSERT_FUNC(dmp_executor_.executeDMPAndWait(*dmp_,
                                                    TaskTransforms::LEFT_ARM_CHOP_STICK_TRANSFORM,
                                                    TaskTransforms::RIGHT_ARM_CHOP_STICK_TRANSFORM,
                                                    execution_statistics_,
                                                    task_execution_duration_ros_,
                                                    task_execution_timeout_duration_ros_));
    spin_sleep(ros::Duration(0.1));

    // call post-execute to stop logging:
    ROS_ASSERT_FUNC(postExecute());
    return true;
}

bool ChopStickTask::reset()
{
    // setup reset dmp to go to the start
    ROS_ASSERT_FUNC(reset_dmp_->setup(dual_arm_joint_reset_dmp_start_, reset_movement_duration_, dmp_sampling_frequency_));
    reset_dmp_->unsetStart();

    dmp_motion_controller::DMPExecutionStatistics unused_execution_statistics;
    ROS_ASSERT_FUNC(dmp_executor_.executeDMPAndWait(*reset_dmp_,
                                                    unused_execution_statistics,
                                                    reset_execution_duration_ros_,
                                                    reset_execution_timeout_duration_ros_));

    return true;
}

bool ChopStickTask::preExecute()
{
    // reset the system
    if(!is_ready_)
    {
        ROS_INFO("Learning reset movement.");
        ROS_ASSERT_FUNC(learnResetDMP());
        ROS_INFO("Resetting before first execution.");
        ROS_ASSERT_FUNC(reset());
    }

    // start logging
    ROS_ASSERT_FUNC(joint_states_.startRecording());
    ROS_ASSERT_FUNC(r_gripper_accelerometer_states_.startRecording());
    ROS_ASSERT_FUNC(l_gripper_accelerometer_states_.startRecording());
    ROS_ASSERT_FUNC(imu_states_.startRecording());
    // ROS_ASSERT_FUNC(r_gripper_tactile_states_.startRecording());
    // ROS_ASSERT_FUNC(l_gripper_tactile_states_.startRecording());

    return true;
}

bool ChopStickTask::postExecute()
{
    ROS_ASSERT_FUNC(reset());
    is_ready_ = true;

    std::vector<std::string> joint_names;
    joint_names.insert(joint_names.begin(), r_arm_effort_cost_joint_names_.begin(), r_arm_effort_cost_joint_names_.end());
    joint_names.insert(joint_names.begin(), l_arm_effort_cost_joint_names_.begin(), l_arm_effort_cost_joint_names_.end());

    // stop logging
    ROS_ASSERT_FUNC(joint_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, task_movement_duration_, num_time_steps_total_, joint_names));
    std::vector<std::string> left_arm;
    left_arm.push_back(std::string("left"));
    ROS_ASSERT_FUNC(r_gripper_accelerometer_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, task_movement_duration_, num_time_steps_total_, left_arm));
    std::vector<std::string> right_arm;
    right_arm.push_back(std::string("right"));
    ROS_ASSERT_FUNC(l_gripper_accelerometer_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, task_movement_duration_, num_time_steps_total_, right_arm));
    std::vector<std::string> empty_vector;
    ROS_ASSERT_FUNC(imu_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, task_movement_duration_, num_time_steps_total_, empty_vector));
    // ROS_ASSERT_FUNC(r_gripper_tactile_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, movement_duration_, num_time_steps_total_, empty_vector));
    // ROS_ASSERT_FUNC(l_gripper_tactile_states_.stopRecording(execution_statistics_.start_time, execution_statistics_.end_time, movement_duration_, num_time_steps_total_, empty_vector));
    return true;
}


bool ChopStickTask::computeCosts(VectorXd& costs)
{
    // ROS_ASSERT_FUNC(computeEffortCost());
    ROS_ASSERT_FUNC(computeImuCost());

    costs.setZero(num_time_steps_);
    costs = costs + l_gripper_acceleration_costs_;
    costs = costs + r_gripper_acceleration_costs_;
    costs = costs + l_arm_effort_costs_;
    costs = costs + r_arm_effort_costs_;
    costs = costs + l_gripper_pressure_costs_;
    costs = costs + r_gripper_pressure_costs_;
    costs = costs + imu_costs_;

    publishCosts(costs);

    cost_log_file_.open(CHOP_STICK_TASK_COST_LOG_FILE_NAME, std::fstream::out | std::fstream::app);
    cost_log_file_ << local_iteration_number_ << std::endl;
    local_iteration_number_++;
    cost_log_file_ << iteration_number_ << std::endl;

    cost_log_file_ << l_gripper_acceleration_costs_ << std::endl;
    cost_log_file_ << r_gripper_acceleration_costs_ << std::endl;
    cost_log_file_ << l_arm_effort_costs_ << std::endl;
    cost_log_file_ << r_arm_effort_costs_ << std::endl;
    cost_log_file_ << l_gripper_pressure_costs_ << std::endl;
    cost_log_file_ << r_gripper_pressure_costs_ << std::endl;
    cost_log_file_ << imu_costs_ << std::endl;

    cost_log_file_ << std::endl;
    cost_log_file_.close();

    return true;
}

bool ChopStickTask::computeImuCost()
{
    imu_costs_.setZero(num_time_steps_);

    double roll, pitch, yaw;
    double max_roll = 0;

    double final_roll = 0;
    double angular_threshold = 0.5;

    for(int i=0; i<num_time_steps_; ++i)
    {
        KDL::Rotation box_orientation = KDL::Rotation::Quaternion(imu_states_.messages_[i].orientation.x,
                                                                  imu_states_.messages_[i].orientation.y,
                                                                  imu_states_.messages_[i].orientation.z,
                                                                  imu_states_.messages_[i].orientation.w);
        box_orientation.GetRPY(roll, pitch, yaw);
        roll -= M_PI;

        double normalized_roll = angles::normalize_angle_positive(roll);
//        double normalized_pitch = angles::normalize_angle_positive(pitch);
//        double normalized_yaw = angles::normalize_angle_positive(yaw);

//        double initial_roll = 0;
//        if(i==0)
//        {
//            initial_roll = normalized_roll;
//        }
//        normalized_roll -= initial_roll;

        if(normalized_roll > 5)
        {
            normalized_roll = 0.0;
        }
        if(normalized_roll > max_roll)
        {
            max_roll = normalized_roll;
        }

        if(i==num_time_steps_-1)
        {
            final_roll = normalized_roll;
        }

    }

    if((final_roll < ((M_PI/2.0)-angular_threshold))
            || (final_roll > ((M_PI/2.0)+angular_threshold)))
    {
        max_roll += (M_PI/2.0);
    }

    double cost = M_PI + angular_threshold - max_roll;

    // assign final cost
    imu_costs_(num_time_steps_-1) = cost * imu_weight_;

    return true;
}

bool ChopStickTask::computeGripperAccelerationCost()
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

    r_gripper_acceleration_costs_.setZero(num_time_steps_);
    // add up the accelerations in the left gripper only for the time of the DMP being executed
    VectorXd r_gripper_accelerometer_data = VectorXd::Zero(num_time_steps_);
    for(int i=0; i<num_time_steps_; ++i)
    {
        r_gripper_accelerometer_data(i) = pow(r_gripper_accelerometer_states_.messages_[i].samples[0].x, 2)
                + pow(r_gripper_accelerometer_states_.messages_[i].samples[0].y, 2)
                + pow(r_gripper_accelerometer_states_.messages_[i].samples[0].z, 2);
    }
    r_gripper_acceleration_costs_ = r_gripper_accelerometer_data * r_gripper_acceleration_weight_;

    return true;
}

bool ChopStickTask::computeEffortCost()
{
    l_arm_effort_costs_.setZero(num_time_steps_);

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
    l_arm_effort_costs_ = joint_effort_data.rowwise().squaredNorm();
    l_arm_effort_costs_ = l_arm_effort_costs_ * l_arm_effort_weight_;

    r_arm_effort_costs_.setZero(num_time_steps_);

    // only add up the efforts in the right arm only for the time of the DMP being executed
    num_joints = static_cast<int>(joint_states_.messages_[0].name.size());
    joint_effort_data = MatrixXd::Zero(num_time_steps_, num_joints);
    for(int i=0; i<num_time_steps_; ++i)
    {
        for (int j=0; j<num_joints; ++j)
        {
            joint_effort_data(i, j) = joint_states_.messages_[i].effort[j];
        }
    }
    r_arm_effort_costs_ = joint_effort_data.rowwise().squaredNorm();
    r_arm_effort_costs_ = r_arm_effort_costs_ * r_arm_effort_weight_;
    return true;
}

bool ChopStickTask::computePressureCost()
{

    return true;
}

bool ChopStickTask::initializeDMPFromFile()
{
    ROS_ASSERT_FUNC(dmp_->readFromDisc(initial_dmp_file_name_));
    return true;
}


bool ChopStickTask::learnTaskDMPFromBagFile()
{
    dmp_motion_learner::LearnDualCartesianSpaceDMP::Request learn_dmp_request;
    dmp_motion_learner::LearnDualCartesianSpaceDMP::Response learn_dmp_response;

    learn_dmp_request.left_arm_type = static_cast<int>(TaskTransforms::LEFT_ARM_CHOP_STICK_TRANSFORM);
    learn_dmp_request.right_arm_type = static_cast<int>(TaskTransforms::RIGHT_ARM_CHOP_STICK_TRANSFORM);
    learn_dmp_request.bag_file_name = demonstration_bag_file_;
    learn_dmp_request.data_directory_name = "/tmp";
    learn_dmp_request.dmp_id = task_dmp_id_;

    ROS_ASSERT_FUNC(learn_dmp_service_client_.call(learn_dmp_request, learn_dmp_response));
    if (learn_dmp_response.return_code != dmp_motion_learner::LearnDualCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL)
    {
        ROS_ERROR("Learn cartesian space DMP from bag file service call was not successful.");
        return false;
    }

    ROS_ASSERT_FUNC(dmp_->initFromMessage(learn_dmp_response.dmp));
    return true;
}

bool ChopStickTask::learnResetDMP()
{
    ROS_INFO("Learning dual arm reset dmp.");
    ROS_ASSERT_FUNC(reset_dmp_->initialize(dmp::N_JOINTS + dmp::N_JOINTS, reset_dmp_id_));

    for (int i=0; i<dmp::N_JOINTS; ++i)
    {
        dual_arm_joint_reset_dmp_start_(i) = dual_arm_dmp_start_(dmp::N_CART + dmp::N_CART + i);
        dual_arm_joint_reset_dmp_goal_(i) = dual_arm_dmp_goal_(dmp::N_CART + dmp::N_CART + i);
        dual_arm_joint_reset_dmp_start_(dmp::N_JOINTS + i) = dual_arm_dmp_start_(dmp::N_CART + dmp::N_CART + dmp::N_JOINTS + dmp::N_CART + dmp::N_CART + i);
        dual_arm_joint_reset_dmp_goal_(dmp::N_JOINTS + i) = dual_arm_dmp_goal_(dmp::N_CART + dmp::N_CART + dmp::N_JOINTS + dmp::N_CART + dmp::N_CART + i);
    }

    // learn from min jerk
    ROS_ASSERT_FUNC(reset_dmp_->learnFromMinJerk(dual_arm_joint_reset_dmp_goal_, dual_arm_joint_reset_dmp_start_, reset_movement_duration_, dmp_dt_));
    return true;
}

//
//bool ChopStickTask::learnLeftArmDMPFromBagFile()
//{
//
//    ROS_INFO("Learning left arm DMP.");
//    ROS_ASSERT_FUNC(left_arm_dmp_->initialize(LEFT_ARM_DIMENSIONS, left_arm_task_dmp_id_));
//
//    dmp_motion_learner::LearnCartesianSpaceDMP::Request learn_dmp_request;
//    dmp_motion_learner::LearnCartesianSpaceDMP::Response learn_dmp_response;
//
//    learn_dmp_request.type = static_cast<int>(TaskTransforms::LEFT_ARM_POOL_TRANSFORM);
//    learn_dmp_request.bag_file_name = left_arm_demonstration_file_name_;
//    learn_dmp_request.data_directory_name = "/tmp";
//    learn_dmp_request.dmp_id = left_arm_task_dmp_id_;
//
//    ROS_ASSERT_FUNC(learn_dmp_service_client_.call(learn_dmp_request, learn_dmp_response));
//    if (learn_dmp_response.return_code != dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL)
//    {
//        ROS_ERROR("Learn left arm cartesian space DMP from bag file service call was not successful.");
//        return false;
//    }
//
//    ROS_ASSERT_FUNC(left_arm_dmp_->initFromMessage(learn_dmp_response.dmp));
//
//    ROS_ASSERT_FUNC(left_arm_dmp_->getInitialStart(left_arm_start_));
//    ROS_ASSERT_FUNC(left_arm_dmp_->getInitialGoal(left_arm_goal_));
//
//    left_gripper_initial_start_frame_.p.x(left_arm_start_(dmp::_XX_));
//    left_gripper_initial_start_frame_.p.y(left_arm_start_(dmp::_YY_));
//    left_gripper_initial_start_frame_.p.z(left_arm_start_(dmp::_ZZ_));
//    KDL::Rotation left_gripper_goal_rotation = KDL::Rotation::Quaternion(left_arm_start_(dmp::N_CART+dmp::_QX_),
//                                                                         left_arm_start_(dmp::N_CART+dmp::_QY_),
//                                                                         left_arm_start_(dmp::N_CART+dmp::_QZ_),
//                                                                         left_arm_start_(dmp::N_CART+dmp::_QW_));
//    left_gripper_initial_start_frame_.M = left_gripper_goal_rotation;
//
//    // learn from min jerk
//    ROS_ASSERT_FUNC(left_arm_dmp_->learnFromMinJerk(left_arm_start_, left_arm_goal_, left_arm_movement_duration_, dmp_dt_));
//
//    return true;
//}
//
//bool ChopStickTask::learnRightArmJointResetDMPFromBagFile()
//{
//
//    ROS_INFO("Learning right arm joint reset DMP.");
//    ROS_ASSERT_FUNC(right_arm_joint_reset_dmp_->initialize(dmp::N_JOINTS, 42));
//
//    VectorXd right_arm_joint_start = VectorXd::Zero(dmp::N_JOINTS);
//    VectorXd right_arm_joint_goal = VectorXd::Zero(dmp::N_JOINTS);
//
//    for (int i=0; i<dmp::N_JOINTS; ++i)
//    {
//        right_arm_joint_start[i] = right_arm_goal_[RIGHT_ARM_DIMENSIONS - dmp::N_JOINTS + i];
//        right_arm_joint_goal[i] = right_arm_start_[RIGHT_ARM_DIMENSIONS - dmp::N_JOINTS + i];
//    }
//
//    // learn from min jerk
//    ROS_ASSERT_FUNC(right_arm_joint_reset_dmp_->learnFromMinJerk(right_arm_joint_goal, right_arm_joint_goal, right_arm_joint_reset_movement_duration_, dmp_dt_));
//
//    return true;
//}

void ChopStickTask::publishText(const std::string& text, const Vector3d& color, const int row_index)
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
    marker.pose.position.z = 1.5 - (row_index*(font_size+0.03));
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

void ChopStickTask::publishCosts(const VectorXd& costs)
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

        ss.str("");
        ss.clear();
        ss << l_arm_effort_costs_.sum();
        display_text.assign(std::string("left arm effort cost: ") + ss.str());
        publishText(display_text, color, 1);

        ss.str("");
        ss.clear();
        ss << r_arm_effort_costs_.sum();
        display_text.assign(std::string("right arm effort cost: ") + ss.str());
        publishText(display_text, color, 2);

        ss.str("");
        ss.clear();
        ss << r_gripper_acceleration_costs_.sum();
        display_text.assign(std::string("right gripper accelerometer cost: ") + ss.str());
        publishText(display_text, color, 3);

        ss.str("");
        ss.clear();
        ss << l_gripper_acceleration_costs_.sum();
        display_text.assign(std::string("left gripper accelerometer cost: ") + ss.str());
        publishText(display_text, color, 4);

        ss.str("");
        ss.clear();
        ss << r_gripper_pressure_costs_.sum();
        display_text.assign(std::string("right gripper pressure cost: ") + ss.str());
        publishText(display_text, color, 5);

        ss.str("");
        ss.clear();
        ss << l_gripper_pressure_costs_.sum();
        display_text.assign(std::string("left gripper pressure cost: ") + ss.str());
        publishText(display_text, color, 6);

        ss.str("");
        ss.clear();
        ss << imu_costs_.sum();
        display_text.assign(std::string("imu cost: ") + ss.str());
        publishText(display_text, color, 7);

    }
    catch(boost::bad_lexical_cast &)
    {
        ROS_ERROR("Could not convert costs to string for display purposes.");
    }

}

}
