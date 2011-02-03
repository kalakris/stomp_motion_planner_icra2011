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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

 \file    dmp_motion_controller_test.cpp

 \author  Peter Pastor
 \date    Jun 22, 2010

 **********************************************************************/

// system includes
#include <string>
#include <sstream>
#include <math.h>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <gtest/gtest.h>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>

#include <policy_library/policy_library.h>
#include <task_manager/controller_switcher.h>

#include <Eigen/Eigen>
USING_PART_OF_NAMESPACE_EIGEN

#include <sensor_msgs/JointState.h>

#include <dmp_motion_generation/dynamic_movement_primitive.h>
#include <dmp_motion_generation/parameters.h>
#include <dmp_motion_generation/math_helper.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>

#include <dmp_motion_learner/LearnJointSpaceDMP.h>
#include <dmp_motion_learner/LearnCartesianSpaceDMP.h>

#include <dmp_motion_controller/dmp_joint_position_controller.h>

#include <dmp_motion_controller/AddToExecuteDMPQueue.h>
#include <dmp_motion_controller/WriteTrajectories.h>
#include <dmp_motion_controller/DMPExecutionStatistics.h>
#include <dmp_motion_controller/AddToDualArmExecuteDMPQueue.h>

#include <task_recorder/StartRecording.h>

#include <task_recorder/StopRecordingJointStates.h>
#include <pr2_msgs/AccelerometerState.h>
#include <task_recorder/StopRecordingAccelerometerStates.h>
// #include <pr2_msgs/PressureState.h>
// #include <task_recorder/StopRecordingPressureStates.h>
#include <slipgrip_controller/PR2GripperSensorData.h>
#include <task_recorder/StopRecordingTactileStates.h>

#include <task_recorder/task_recorder_utilities.h>

#include <pr2_tasks_transforms/task_transforms.h>

using namespace pr2_tasks_transforms;

const int TEST_DMP_ID_1 = 1;
const int TEST_DMP_ID_2 = 2;
const int TEST_DMP_ID_3 = 3;
const int TEST_DMP_ID_4 = 4;

class DMPMotionControllerTest
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum WhichArm
    {
        LEFT_ARM = 1,
        RIGHT_ARM
    };

    DMPMotionControllerTest(ros::NodeHandle& node_handle);
    ~DMPMotionControllerTest();

    bool initialize();

    bool runDualArmControllerTest(const int right_arm_dmp_id, const int left_arm_dmp_id, bool in_joint_space);

    bool runDualArmJointSpaceControllerTest(const int left_arm_dmp_id, const int right_arm_dmp_id);
    bool runDualArmCartesianSpaceControllerTest(const int left_arm_dmp_id, const int right_arm_dmp_id);

    bool runJointSpaceControllerTest(const std::string arm_prefix, const int dmp_id, const DMPMotionControllerTest::WhichArm arm);
    bool runCartesianSpaceControllerTest(const std::string arm_prefix, const int dmp_id, const DMPMotionControllerTest::WhichArm arm);

    bool learnJointSpaceDMP(const std::string arm_prefix, const std::string bag_file_name, const int dmp_id, const DMPMotionControllerTest::WhichArm arm);
    bool learnCartesianSpaceDMP(const std::string bag_file_name, const int dmp_id, const DMPMotionControllerTest::WhichArm arm);

    bool waitForExecutionStatistics(const double wait_duration);

    bool startRecording(const int dmp_id);
    bool stopRecording();

    std::string r_arm_bag_file_name_;
    std::string l_arm_bag_file_name_;

private:

    bool initialized_;

    void initializeServiceClients();

    bool readParameters();

    bool setupForReproduction(boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp);

    int num_samples_;

    std::string library_directory_name_;
    std::string data_directory_name_;
    std::string debug_directory_name_;

    bool use_durations_;
    double movement_duration_;
    double execution_duration_;
    double sampling_frequency_;

    std::vector<std::string> pr2_r_arm_joint_names_;
    std::vector<std::string> pr2_l_arm_joint_names_;

    std::vector<std::string> pr2_r_arm_trajectory_variable_names_;
    std::vector<std::string> pr2_l_arm_trajectory_variable_names_;

    int num_joints_;
    VectorXd start_;
    VectorXd goal_;

    std::vector<double> left_arm_transform_offsets_;
    std::vector<double> left_arm_transform_quat_;
    std::vector<double> right_arm_transform_offsets_;
    std::vector<double> right_arm_transform_quat_;

    double mse_error_threshold_;

    task_manager::ControllerSwitcher controller_switcher_;

    ros::NodeHandle node_handle_;

    ros::ServiceClient dual_arm_add_to_joint_space_dmp_queue_service_client_;
    ros::ServiceClient dual_arm_add_to_cartesian_space_dmp_queue_service_client_;

    ros::ServiceClient r_arm_add_to_joint_space_execute_dmp_queue_service_client_;
    ros::ServiceClient r_arm_write_joint_space_trajectories_service_client_;
    ros::ServiceClient r_arm_add_to_cartesian_space_execute_dmp_queue_service_client_;
    ros::ServiceClient r_arm_write_cartesian_space_trajectories_service_client_;

    ros::ServiceClient l_arm_add_to_joint_space_execute_dmp_queue_service_client_;
    ros::ServiceClient l_arm_write_joint_space_trajectories_service_client_;
    ros::ServiceClient l_arm_add_to_cartesian_space_execute_dmp_queue_service_client_;
    ros::ServiceClient l_arm_write_cartesian_space_trajectories_service_client_;

    ros::ServiceClient learn_joint_space_dmp_service_client_;
    ros::ServiceClient learn_cartesian_space_dmp_service_client_;

    ros::Subscriber r_arm_joint_space_statistics_subscriber_;
    ros::Subscriber r_arm_cartesian_space_statistics_subscriber_;

    ros::Subscriber l_arm_joint_space_statistics_subscriber_;
    ros::Subscriber l_arm_cartesian_space_statistics_subscriber_;

    bool execution_stats_received_;
    boost::mutex execution_stats_received_mutex_;
    dmp_motion_controller::DMPExecutionStatistics execution_statistics_;

    void executionStatisticsCallback(const dmp_motion_controller::DMPExecutionStatisticsPtr& dmp_execution_statistics);

    ros::ServiceClient start_recording_joint_states_service_client_;
    ros::ServiceClient stop_recording_joint_states_service_client_;

    ros::ServiceClient start_r_arm_recording_accelerometer_states_service_client_;
    ros::ServiceClient stop_r_arm_recording_accelerometer_states_service_client_;
    // ros::ServiceClient start_r_arm_recording_pressure_states_service_client_;
    // ros::ServiceClient stop_r_arm_recording_pressure_states_service_client_;
    ros::ServiceClient start_r_arm_recording_tactile_states_service_client_;
    ros::ServiceClient stop_r_arm_recording_tactile_states_service_client_;

    ros::ServiceClient start_l_arm_recording_accelerometer_states_service_client_;
    ros::ServiceClient stop_l_arm_recording_accelerometer_states_service_client_;
    // ros::ServiceClient start_l_arm_recording_pressure_states_service_client_;
    // ros::ServiceClient stop_l_arm_recording_pressure_states_service_client_;
    ros::ServiceClient start_l_arm_recording_tactile_states_service_client_;
    ros::ServiceClient stop_l_arm_recording_tactile_states_service_client_;

    boost::shared_ptr<library::PolicyLibrary<dmp::DynamicMovementPrimitive> > test_lib_;

    TaskTransforms::TransformType getTransformType(const DMPMotionControllerTest::WhichArm arm);
};

DMPMotionControllerTest::DMPMotionControllerTest(ros::NodeHandle& node_handle) :
    initialized_(false), node_handle_(node_handle)
{
}

DMPMotionControllerTest::~DMPMotionControllerTest()
{
}

bool DMPMotionControllerTest::initialize()
{
    ROS_ASSERT_FUNC(readParameters());
    initializeServiceClients();

    test_lib_.reset(new library::PolicyLibrary<dmp::DynamicMovementPrimitive>(node_handle_));
    ROS_ASSERT_FUNC(test_lib_->initialize(library_directory_name_));

    return (initialized_ = true);
}

bool DMPMotionControllerTest::readParameters()
{
    ros::NodeHandle node_handle("/dmp_motion_controller_test/dmp");

    // get package name and then get the package path
    std::string package_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("package_name"), package_name));

    std::string package_path = ros::package::getPath(package_name);
    policy_improvement_utilities::appendTrailingSlash(package_path);

    std::string library_base_directory_name;

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("library_base_directory_name"), library_base_directory_name));
    policy_improvement_utilities::appendTrailingSlash(library_base_directory_name);

    std::string sub_directory_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("library_directory_name"), sub_directory_name));
    library_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    policy_improvement_utilities::appendTrailingSlash(library_directory_name_);
    ROS_INFO("Library directory is set to %s.", library_directory_name_.c_str());

    sub_directory_name.clear();
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("data_directory_name"), sub_directory_name));
    data_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    policy_improvement_utilities::appendTrailingSlash(data_directory_name_);
    // ROS_INFO("Data directory is set to %s.", data_directory_name_.c_str());

    sub_directory_name.clear();
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("debug_directory_name"), sub_directory_name));
    debug_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    policy_improvement_utilities::appendTrailingSlash(debug_directory_name_);
    // ROS_INFO("Debug directory is set to %s.", debug_directory_name_.c_str());

    node_handle = ros::NodeHandle(std::string("/dmp_motion_controller_test/test_parameters"));

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("num_samples"), num_samples_));

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("use_durations"), use_durations_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("movement_duration"), movement_duration_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("execution_duration"), execution_duration_));

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("sampling_frequency"), sampling_frequency_));

    ROS_ASSERT_FUNC(policy_improvement_utilities::readDoubleArray(node_handle, std::string("left_arm_transform_offsets"), left_arm_transform_offsets_));
    ROS_ASSERT(left_arm_transform_offsets_.size() == 3);
    ROS_ASSERT_FUNC(policy_improvement_utilities::readDoubleArray(node_handle, std::string("left_arm_transform_quat"), left_arm_transform_quat_));
    ROS_ASSERT_FUNC(left_arm_transform_quat_.size() == 4);

    ROS_ASSERT_FUNC(policy_improvement_utilities::readDoubleArray(node_handle, std::string("right_arm_transform_offsets"), right_arm_transform_offsets_));
    ROS_ASSERT(right_arm_transform_offsets_.size() == 3);
    ROS_ASSERT_FUNC(policy_improvement_utilities::readDoubleArray(node_handle, std::string("right_arm_transform_quat"), right_arm_transform_quat_));
    ROS_ASSERT_FUNC(right_arm_transform_quat_.size() == 4);

    std::string bag_file_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("r_arm_bag_file_name"), bag_file_name));
    r_arm_bag_file_name_.assign(data_directory_name_ + bag_file_name);
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("l_arm_bag_file_name"), bag_file_name));
    l_arm_bag_file_name_.assign(data_directory_name_ + bag_file_name);

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("mse_error_threshold"), mse_error_threshold_));

    ROS_ASSERT_FUNC(policy_improvement_utilities::readStringArraySpaceSeparated(node_handle, std::string("r_arm_joint_names"), pr2_r_arm_joint_names_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::readStringArraySpaceSeparated(node_handle, std::string("l_arm_joint_names"), pr2_l_arm_joint_names_));

    ROS_ASSERT_FUNC(policy_improvement_utilities::readStringArraySpaceSeparated(node_handle, std::string("pr2_r_arm_joint_names"), pr2_r_arm_trajectory_variable_names_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::readStringArraySpaceSeparated(node_handle, std::string("pr2_l_arm_joint_names"), pr2_l_arm_trajectory_variable_names_));

    XmlRpc::XmlRpcValue start;
    if (!node_handle.getParam("start", start))
    {
        ROS_ERROR("Could not retrive parameter struct >>start<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        return false;
    }
    num_joints_ = start.size();
    start_ = VectorXd::Zero(num_joints_);
    for (int j = 0; j < start.size(); j++)
    {
        if (start[j].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            ROS_ERROR("Could not retrive parameter >>start<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
            ROS_ERROR("They are not specified a list of doubles, instead they are type %d, but should be type %d.", start[j].getType(),
                    XmlRpc::XmlRpcValue::TypeDouble);
            return false;
        }
        start_(j) = static_cast<double> (start[j]);
    }

    XmlRpc::XmlRpcValue goal;
    if (!node_handle.getParam("goal", goal))
    {
        ROS_ERROR("Could not retrive parameter struct >>goal<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        return false;
    }
    if (goal.size() != num_joints_)
    {
        ROS_ERROR("Goal has %i values specified and start has %i values specified.", goal.size(), start.size());
    }
    goal_ = VectorXd::Zero(num_joints_);
    for (int j = 0; j < goal.size(); j++)
    {
        if (goal[j].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            ROS_ERROR("Could not retrive parameter >>goal<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
            ROS_ERROR("They are not specified a list of doubles, instead they are type %d, but should be type %d.", goal[j].getType(),
                    XmlRpc::XmlRpcValue::TypeDouble);
            return false;
        }
        goal_(j) = static_cast<double> (goal[j]);
    }

    if (!controller_switcher_.initialize())
    {
        ROS_ERROR("Could not initialize controller switcher.");
        return false;
    }
    return true;

}

void DMPMotionControllerTest::initializeServiceClients()
{

    dual_arm_add_to_joint_space_dmp_queue_service_client_
            = node_handle_.serviceClient<dmp_motion_controller::AddToDualArmExecuteDMPQueue> ("/dual_arm_dmp_joint_position_controller/add_to_execute_dmp_queue");

    dual_arm_add_to_cartesian_space_dmp_queue_service_client_
            = node_handle_.serviceClient<dmp_motion_controller::AddToDualArmExecuteDMPQueue> ("/dual_arm_dmp_ik_controller/add_to_execute_dmp_queue");

    r_arm_add_to_joint_space_execute_dmp_queue_service_client_
            = node_handle_.serviceClient<dmp_motion_controller::AddToExecuteDMPQueue> ("/r_arm_dmp_joint_position_controller/add_to_execute_dmp_queue");
    r_arm_write_joint_space_trajectories_service_client_
            = node_handle_.serviceClient<dmp_motion_controller::WriteTrajectories> ("/r_arm_dmp_joint_position_controller/write_trajectories");

    r_arm_add_to_cartesian_space_execute_dmp_queue_service_client_
            = node_handle_.serviceClient<dmp_motion_controller::AddToExecuteDMPQueue> ("/r_arm_dmp_ik_controller/add_to_execute_dmp_queue");
    r_arm_write_cartesian_space_trajectories_service_client_
            = node_handle_.serviceClient<dmp_motion_controller::WriteTrajectories> ("/r_arm_dmp_ik_controller/write_trajectories");

    l_arm_add_to_joint_space_execute_dmp_queue_service_client_
            = node_handle_.serviceClient<dmp_motion_controller::AddToExecuteDMPQueue> ("/l_arm_dmp_joint_position_controller/add_to_execute_dmp_queue");
    l_arm_write_joint_space_trajectories_service_client_
            = node_handle_.serviceClient<dmp_motion_controller::WriteTrajectories> ("/l_arm_dmp_joint_position_controller/write_trajectories");

    l_arm_add_to_cartesian_space_execute_dmp_queue_service_client_
            = node_handle_.serviceClient<dmp_motion_controller::AddToExecuteDMPQueue> ("/l_arm_dmp_ik_controller/add_to_execute_dmp_queue");
    l_arm_write_cartesian_space_trajectories_service_client_
            = node_handle_.serviceClient<dmp_motion_controller::WriteTrajectories> ("/l_arm_dmp_ik_controller/write_trajectories");

    learn_joint_space_dmp_service_client_
            = node_handle_.serviceClient<dmp_motion_learner::LearnJointSpaceDMP> ("/dmp_motion_learner/learn_joint_space_dmp_from_bag_file");
    learn_cartesian_space_dmp_service_client_
            = node_handle_.serviceClient<dmp_motion_learner::LearnCartesianSpaceDMP> ("/dmp_motion_learner/learn_cartesian_and_joint_space_dmp_from_bag_file");

    r_arm_joint_space_statistics_subscriber_ = node_handle_.subscribe("/r_arm_dmp_joint_position_controller/dmp_execution_statistics", 1, &DMPMotionControllerTest::executionStatisticsCallback, this);
    r_arm_cartesian_space_statistics_subscriber_ = node_handle_.subscribe("/r_arm_dmp_ik_controller/dmp_execution_statistics", 1, &DMPMotionControllerTest::executionStatisticsCallback, this);

    l_arm_joint_space_statistics_subscriber_ = node_handle_.subscribe("/l_arm_dmp_joint_position_controller/dmp_execution_statistics", 1, &DMPMotionControllerTest::executionStatisticsCallback, this);
    l_arm_cartesian_space_statistics_subscriber_ = node_handle_.subscribe("/l_arm_dmp_ik_controller/dmp_execution_statistics", 1, &DMPMotionControllerTest::executionStatisticsCallback, this);

    std::string topic_name;
    std::string start_recording_service_name;
    std::string stop_recording_service_name;

    topic_name.assign("/joint_states");
    ROS_ASSERT_FUNC(task_recorder::getTopicName(topic_name));
    start_recording_service_name.assign(std::string("/task_recorder/start_recording_") + topic_name);
    stop_recording_service_name.assign(std::string("/task_recorder/stop_recording_") + topic_name);
    start_recording_joint_states_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording> (start_recording_service_name);
    stop_recording_joint_states_service_client_ = node_handle_.serviceClient<task_recorder::StopRecordingJointStates> (stop_recording_service_name);

    topic_name.assign("/accelerometer/r_gripper_motor");
    ROS_ASSERT_FUNC(task_recorder::getTopicName(topic_name));
    start_recording_service_name.assign(std::string("/task_recorder/start_recording_") + topic_name);
    stop_recording_service_name.assign(std::string("/task_recorder/stop_recording_") + topic_name);
    start_r_arm_recording_accelerometer_states_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording> (start_recording_service_name);
    stop_r_arm_recording_accelerometer_states_service_client_  = node_handle_.serviceClient<task_recorder::StopRecordingAccelerometerStates> (stop_recording_service_name);

    topic_name.assign("/accelerometer/l_gripper_motor");
    ROS_ASSERT_FUNC(task_recorder::getTopicName(topic_name));
    start_recording_service_name.assign(std::string("/task_recorder/start_recording_") + topic_name);
    stop_recording_service_name.assign(std::string("/task_recorder/stop_recording_") + topic_name);
    start_l_arm_recording_accelerometer_states_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording> (start_recording_service_name);
    stop_l_arm_recording_accelerometer_states_service_client_  = node_handle_.serviceClient<task_recorder::StopRecordingAccelerometerStates> (stop_recording_service_name);

    // topic_name.assign("/pressure/r_gripper_motor");
    // ROS_ASSERT_FUNC(task_recorder::getTopicName(topic_name));
    // start_recording_service_name.assign(std::string("/task_recorder/start_recording_") + topic_name);
    // stop_recording_service_name.assign(std::string("/task_recorder/stop_recording_") + topic_name);
    // start_r_arm_recording_pressure_states_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording> (start_recording_service_name);
    // stop_r_arm_recording_pressure_states_service_client_  = node_handle_.serviceClient<task_recorder::StopRecordingPressureStates> (stop_recording_service_name);
    //
    // topic_name.assign("/pressure/r_gripper_motor");
    // ROS_ASSERT_FUNC(task_recorder::getTopicName(topic_name));
    // start_recording_service_name.assign(std::string("/task_recorder/start_recording_") + topic_name);
    // stop_recording_service_name.assign(std::string("/task_recorder/stop_recording_") + topic_name);
    // start_r_arm_recording_pressure_states_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording> (start_recording_service_name);
    // stop_r_arm_recording_pressure_states_service_client_  = node_handle_.serviceClient<task_recorder::StopRecordingPressureStates> (stop_recording_service_name);

    topic_name.assign("/r_gripper_sensor_observer/tactile_sensor_info");
    ROS_ASSERT_FUNC(task_recorder::getTopicName(topic_name));
    start_recording_service_name.assign(std::string("/task_recorder/start_recording_") + topic_name);
    stop_recording_service_name.assign(std::string("/task_recorder/stop_recording_") + topic_name);
    start_r_arm_recording_tactile_states_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording> (start_recording_service_name);
    stop_r_arm_recording_tactile_states_service_client_  = node_handle_.serviceClient<task_recorder::StopRecordingTactileStates> (stop_recording_service_name);

    topic_name.assign("/l_gripper_sensor_observer/tactile_sensor_info");
    ROS_ASSERT_FUNC(task_recorder::getTopicName(topic_name));
    start_recording_service_name.assign(std::string("/task_recorder/start_recording_") + topic_name);
    stop_recording_service_name.assign(std::string("/task_recorder/stop_recording_") + topic_name);
    start_l_arm_recording_tactile_states_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording> (start_recording_service_name);
    stop_l_arm_recording_tactile_states_service_client_  = node_handle_.serviceClient<task_recorder::StopRecordingTactileStates> (stop_recording_service_name);
}

void DMPMotionControllerTest::executionStatisticsCallback(const dmp_motion_controller::DMPExecutionStatisticsPtr& dmp_execution_statistics)
{
    execution_statistics_ = *dmp_execution_statistics;
    execution_stats_received_mutex_.lock();
    execution_stats_received_ = true;
    execution_stats_received_mutex_.unlock();
}

bool DMPMotionControllerTest::waitForExecutionStatistics(const double wait_duration)
{
    bool statistics_received=false;
    ros::Duration time_spent = ros::Duration(0.0);
    ros::Duration tick = ros::Duration(0.1);
    ros::Duration timeout = ros::Duration(wait_duration);

    execution_stats_received_ = false;
    while(!statistics_received && time_spent < timeout)
    {
        ros::spinOnce();
        execution_stats_received_mutex_.lock();
        statistics_received = execution_stats_received_;
        execution_stats_received_mutex_.unlock();
        if (!statistics_received)
        {
            tick.sleep();
            time_spent += tick;
        }
    }

    ros::Duration(0.3).sleep();

    return statistics_received;
}

TaskTransforms::TransformType DMPMotionControllerTest::getTransformType(const DMPMotionControllerTest::WhichArm arm)
{
    TaskTransforms::TransformType transform_type = TaskTransforms::LEFT_ARM_NO_TRANSFORM;
    switch(arm)
    {
        case LEFT_ARM:
        {
            // transform_type = TaskTransforms::LEFT_ARM_NO_TRANSFORM;
            transform_type = TaskTransforms::LEFT_ARM_POOL_TRANSFORM;
            break;
        }
        case RIGHT_ARM:
        {
            // transform_type = TaskTransforms::RIGHT_ARM_NO_TRANSFORM;
            transform_type = TaskTransforms::RIGHT_ARM_POOL_TRANSFORM;
            break;
        }
    }
    return transform_type;
}

bool DMPMotionControllerTest::learnJointSpaceDMP(const std::string arm_prefix, const std::string bag_file_name, const int dmp_id, const DMPMotionControllerTest::WhichArm arm)
{
    dmp_motion_learner::LearnJointSpaceDMP::Request learn_joint_space_dmp_request;
    dmp_motion_learner::LearnJointSpaceDMP::Response learn_joint_space_dmp_response;

    learn_joint_space_dmp_request.joint_names.push_back(arm_prefix + std::string("shoulder_pan_joint"));
    learn_joint_space_dmp_request.joint_names.push_back(arm_prefix + std::string("shoulder_lift_joint"));
    learn_joint_space_dmp_request.joint_names.push_back(arm_prefix + std::string("upper_arm_roll_joint"));
    learn_joint_space_dmp_request.joint_names.push_back(arm_prefix + std::string("elbow_flex_joint"));
    learn_joint_space_dmp_request.joint_names.push_back(arm_prefix + std::string("forearm_roll_joint"));
    learn_joint_space_dmp_request.joint_names.push_back(arm_prefix + std::string("wrist_flex_joint"));
    learn_joint_space_dmp_request.joint_names.push_back(arm_prefix + std::string("wrist_roll_joint"));

    learn_joint_space_dmp_request.bag_file_name.assign(bag_file_name);
    learn_joint_space_dmp_request.data_directory_name.assign(data_directory_name_);
    learn_joint_space_dmp_request.dmp_id = dmp_id;

    ROS_ASSERT_FUNC(learn_joint_space_dmp_service_client_.call(learn_joint_space_dmp_request, learn_joint_space_dmp_response));
    ROS_ASSERT_FUNC(learn_joint_space_dmp_response.return_code == dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL);

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp;
    test_dmp.reset(new dmp::DynamicMovementPrimitive(node_handle_));
    ROS_ASSERT_FUNC(test_dmp->initFromMessage(learn_joint_space_dmp_response.dmp));
    ROS_ASSERT_FUNC(test_lib_->add(test_dmp));

    return true;
}

bool DMPMotionControllerTest::learnCartesianSpaceDMP(const std::string bag_file_name, const int dmp_id, const DMPMotionControllerTest::WhichArm arm)
{
    dmp_motion_learner::LearnCartesianSpaceDMP::Request learn_cartesian_space_dmp_request;
    dmp_motion_learner::LearnCartesianSpaceDMP::Response learn_cartesian_space_dmp_response;

    learn_cartesian_space_dmp_request.bag_file_name.assign(bag_file_name);
    learn_cartesian_space_dmp_request.data_directory_name.assign(data_directory_name_);
    learn_cartesian_space_dmp_request.dmp_id = dmp_id;
    learn_cartesian_space_dmp_request.type = getTransformType(arm);

    ROS_ASSERT_FUNC(learn_cartesian_space_dmp_service_client_.call(learn_cartesian_space_dmp_request, learn_cartesian_space_dmp_response));
    ROS_ASSERT_FUNC(learn_cartesian_space_dmp_response.return_code == dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL);

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp;
    test_dmp.reset(new dmp::DynamicMovementPrimitive(node_handle_));
    ROS_ASSERT_FUNC(test_dmp->initFromMessage(learn_cartesian_space_dmp_response.dmp));
    ROS_ASSERT_FUNC(test_lib_->add(test_dmp));

    return true;
}

bool DMPMotionControllerTest::runJointSpaceControllerTest(const std::string arm_prefix, const int dmp_id, const DMPMotionControllerTest::WhichArm arm)
{

    switch (arm)
    {
        case LEFT_ARM:
        {
            EXPECT_TRUE(controller_switcher_.switchLeftArmController(arm_prefix + std::string("arm_dmp_joint_position_controller")));
            break;
        }
        case RIGHT_ARM:
        {
            EXPECT_TRUE(controller_switcher_.switchRightArmController(arm_prefix + std::string("arm_dmp_joint_position_controller")));
            break;
        }
    }
    // ros::Duration(1.0).sleep();

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp;
    ROS_ASSERT_FUNC(test_lib_->getItem(dmp_id, test_dmp));

    ROS_ASSERT_FUNC(setupForReproduction(test_dmp));

    dmp_motion_controller::AddToExecuteDMPQueue::Request execute_test_dmp_request;
    dmp_motion_controller::AddToExecuteDMPQueue::Response execute_test_dmp_response;

    dmp_motion_generation::DynamicMovementPrimitive test_dmp_msg;

    ROS_ASSERT_FUNC(test_dmp->writeToMessage(test_dmp_msg));
    execute_test_dmp_request.dmps.push_back(test_dmp_msg);
    execute_test_dmp_request.types.push_back(0);

    double duration = execution_duration_;
    if(!use_durations_)
    {
        ROS_ASSERT_FUNC(test_dmp->getInitialDuration(duration));
    }
    execute_test_dmp_request.execution_durations.push_back(duration);

    switch (arm)
    {
        case LEFT_ARM:
        {
            ROS_ASSERT_FUNC(l_arm_add_to_joint_space_execute_dmp_queue_service_client_.call(execute_test_dmp_request, execute_test_dmp_response));
            break;
        }
        case RIGHT_ARM:
        {
            ROS_ASSERT_FUNC(r_arm_add_to_joint_space_execute_dmp_queue_service_client_.call(execute_test_dmp_request, execute_test_dmp_response));
            break;
        }
    }

    ROS_INFO_STREAM(execute_test_dmp_response.info);

    dmp_motion_controller::WriteTrajectories::Request write_trajectories_request;
    dmp_motion_controller::WriteTrajectories::Response write_trajectories_response;

    switch (arm)
    {
        case LEFT_ARM:
        {
            // ROS_ASSERT_FUNC(l_arm_write_joint_space_trajectories_service_client_.call(write_trajectories_request, write_trajectories_response));
            l_arm_write_joint_space_trajectories_service_client_.call(write_trajectories_request, write_trajectories_response);
            break;
        }
        case RIGHT_ARM:
        {
            // ROS_ASSERT_FUNC(r_arm_write_joint_space_trajectories_service_client_.call(write_trajectories_request, write_trajectories_response));
            r_arm_write_joint_space_trajectories_service_client_.call(write_trajectories_request, write_trajectories_response);
            break;
        }
    }
    ROS_INFO_STREAM(write_trajectories_response.info);

    // TODO: compare result...

    return true;
}

bool DMPMotionControllerTest::runCartesianSpaceControllerTest(const std::string arm_prefix, const int dmp_id, const DMPMotionControllerTest::WhichArm arm)
{

    // first, switch controller
    switch(arm)
    {
        case LEFT_ARM:
        {
            EXPECT_TRUE(controller_switcher_.switchLeftArmController(arm_prefix + std::string("arm_dmp_ik_controller")));
            break;
        }
        case RIGHT_ARM:
        {
            EXPECT_TRUE(controller_switcher_.switchRightArmController(arm_prefix + std::string("arm_dmp_ik_controller")));
            break;
        }
    }
    // ros::Duration(1.0).sleep();

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp;
    ROS_ASSERT_FUNC(test_lib_->getItem(dmp_id, test_dmp));
    ROS_ASSERT_FUNC(setupForReproduction(test_dmp));

    dmp_motion_controller::AddToExecuteDMPQueue::Request execute_test_dmp_request;
    dmp_motion_controller::AddToExecuteDMPQueue::Response execute_test_dmp_response;

    dmp_motion_generation::DynamicMovementPrimitive test_dmp_msg;
    ROS_ASSERT_FUNC(test_dmp->writeToMessage(test_dmp_msg));
    execute_test_dmp_request.dmps.push_back(test_dmp_msg);
    execute_test_dmp_request.types.push_back(static_cast<int>(getTransformType(arm)));

    double duration = execution_duration_;
    if(!use_durations_)
    {
        ROS_ASSERT_FUNC(test_dmp->getInitialDuration(duration));
    }
    execute_test_dmp_request.execution_durations.push_back(duration);

    switch (arm)
    {
        case LEFT_ARM:
        {
            ROS_ASSERT_FUNC(l_arm_add_to_cartesian_space_execute_dmp_queue_service_client_.call(execute_test_dmp_request, execute_test_dmp_response));
            break;
        }
        case RIGHT_ARM:
        {
            ROS_ASSERT_FUNC(r_arm_add_to_cartesian_space_execute_dmp_queue_service_client_.call(execute_test_dmp_request, execute_test_dmp_response));
            break;
        }
    }
    ROS_INFO_STREAM(execute_test_dmp_response.info);

    dmp_motion_controller::WriteTrajectories::Request write_trajectories_request;
    dmp_motion_controller::WriteTrajectories::Response write_trajectories_response;

    switch (arm)
    {
        case LEFT_ARM:
        {
            // ROS_ASSERT_FUNC(l_arm_write_cartesian_space_trajectories_service_client_.call(write_trajectories_request, write_trajectories_response));
            l_arm_write_cartesian_space_trajectories_service_client_.call(write_trajectories_request, write_trajectories_response);
            break;
        }
        case RIGHT_ARM:
        {
            // ROS_ASSERT_FUNC(r_arm_write_cartesian_space_trajectories_service_client_.call(write_trajectories_request, write_trajectories_response));
            r_arm_write_cartesian_space_trajectories_service_client_.call(write_trajectories_request, write_trajectories_response);
            break;
        }
    }
    ROS_INFO_STREAM(write_trajectories_response.info);

    // TODO: compare result...

    return true;
}

bool DMPMotionControllerTest::setupForReproduction(boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp)
{
    VectorXd goal = VectorXd::Zero(dmp->getNumTransformationSystems());
    ROS_ASSERT_FUNC(dmp->getInitialGoal(goal));
    double duration = movement_duration_;
    if(!use_durations_)
    {
        ROS_ASSERT_FUNC(dmp->getInitialDuration(duration));
    }
    ROS_ASSERT_FUNC(dmp->setup(goal, duration, sampling_frequency_));
    dmp->unsetStart();
    return true;
}

bool DMPMotionControllerTest::runDualArmJointSpaceControllerTest(const int right_arm_dmp_id, const int left_arm_dmp_id)
{

    EXPECT_TRUE(controller_switcher_.switchLeftArmController(std::string("l_arm_dmp_joint_position_controller")));
    EXPECT_TRUE(controller_switcher_.switchRightArmController(std::string("r_arm_dmp_joint_position_controller")));

    EXPECT_TRUE(controller_switcher_.startController(std::string("dual_arm_dmp_joint_position_controller")));
    // ros::Duration(1.0).sleep();

    EXPECT_TRUE(runDualArmControllerTest(right_arm_dmp_id, left_arm_dmp_id, true));

    // TODO: compare result...
    return true;
}

bool DMPMotionControllerTest::runDualArmCartesianSpaceControllerTest(const int right_arm_dmp_id, const int left_arm_dmp_id)
{

    EXPECT_TRUE(controller_switcher_.switchLeftArmController(std::string("l_arm_dmp_ik_controller")));
    EXPECT_TRUE(controller_switcher_.switchRightArmController(std::string("r_arm_dmp_ik_controller")));

    EXPECT_TRUE(controller_switcher_.startController(std::string("dual_arm_dmp_ik_controller")));
    // ros::Duration(1.0).sleep();

    EXPECT_TRUE(runDualArmControllerTest(right_arm_dmp_id, left_arm_dmp_id, false));

    // TODO: compare result...
    return true;
}

bool DMPMotionControllerTest::runDualArmControllerTest(const int right_arm_dmp_id, const int left_arm_dmp_id, bool in_joint_space)
{

    boost::shared_ptr<dmp::DynamicMovementPrimitive> left_arm_test_dmp;
    ROS_ASSERT_FUNC(test_lib_->getItem(left_arm_dmp_id, left_arm_test_dmp));
    ROS_ASSERT_FUNC(setupForReproduction(left_arm_test_dmp));

    boost::shared_ptr<dmp::DynamicMovementPrimitive> right_arm_test_dmp;
    ROS_ASSERT_FUNC(test_lib_->getItem(right_arm_dmp_id, right_arm_test_dmp));
    ROS_ASSERT_FUNC(setupForReproduction(right_arm_test_dmp));

    dmp_motion_controller::AddToDualArmExecuteDMPQueue::Request dual_arm_request;
    dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response dual_arm_response;

    dmp_motion_generation::DynamicMovementPrimitive left_arm_dmp_msg;
    ROS_ASSERT_FUNC(left_arm_test_dmp->writeToMessage(left_arm_dmp_msg));
    dmp_motion_generation::DynamicMovementPrimitive right_arm_dmp_msg;
    ROS_ASSERT_FUNC(right_arm_test_dmp->writeToMessage(right_arm_dmp_msg));

    dual_arm_request.left_arm_dmps.push_back(left_arm_dmp_msg);
    dual_arm_request.right_arm_dmps.push_back(right_arm_dmp_msg);
    double duration = execution_duration_;
    if(!use_durations_)
    {
        double left_arm_dmp_duration = 0;
        double right_arm_dmp_duration = 0;
        ROS_ASSERT_FUNC(left_arm_test_dmp->getInitialDuration(left_arm_dmp_duration));
        ROS_ASSERT_FUNC(right_arm_test_dmp->getInitialDuration(right_arm_dmp_duration));
        if(left_arm_dmp_duration < right_arm_dmp_duration)
        {
            duration = right_arm_dmp_duration;
        }
        else
        {
            duration = left_arm_dmp_duration;
        }
    }
    dual_arm_request.execution_durations.push_back(duration);

    if(in_joint_space)
    {
        dual_arm_request.left_arm_types.push_back(0);
        dual_arm_request.right_arm_types.push_back(0);
        ROS_ASSERT_FUNC(dual_arm_add_to_joint_space_dmp_queue_service_client_.call(dual_arm_request, dual_arm_response));
    }
    else
    {
        dual_arm_request.left_arm_types.push_back(getTransformType(LEFT_ARM));
        dual_arm_request.right_arm_types.push_back(getTransformType(RIGHT_ARM));
        ROS_ASSERT_FUNC(dual_arm_add_to_cartesian_space_dmp_queue_service_client_.call(dual_arm_request, dual_arm_response));
    }
    ROS_INFO_STREAM(dual_arm_response.info);

    dmp_motion_controller::WriteTrajectories::Request write_trajectories_request;
    dmp_motion_controller::WriteTrajectories::Response write_trajectories_response;

    l_arm_write_cartesian_space_trajectories_service_client_.call(write_trajectories_request, write_trajectories_response);
    r_arm_write_cartesian_space_trajectories_service_client_.call(write_trajectories_request, write_trajectories_response);
    ROS_INFO_STREAM(write_trajectories_response.info);

    // TODO: compare result...
    return true;

}

bool DMPMotionControllerTest::startRecording(const int dmp_id)
{
    task_recorder::StartRecording::Request start_request;
    task_recorder::StartRecording::Response start_response;

    start_request.id = dmp_id;

    // service call to record, filter, and crop the joint states
    start_request.topic_names.clear();
    start_request.topic_names.push_back("/joint_states");
    EXPECT_TRUE(start_recording_joint_states_service_client_.call(start_request, start_response));

    // service call to record, filter, and crop the accelerometer states of the right arm
    start_request.topic_names.clear();
    start_request.topic_names.push_back("/accelerometer/r_gripper_motor");
    EXPECT_TRUE(start_r_arm_recording_accelerometer_states_service_client_.call(start_request, start_response));

    // service call to record, filter, and crop the accelerometer states of the left arm
    start_request.topic_names.clear();
    start_request.topic_names.push_back("/accelerometer/l_gripper_motor");
    EXPECT_TRUE(start_l_arm_recording_accelerometer_states_service_client_.call(start_request, start_response));

    // // service call to record, filter, and crop the pressure states of the right arm
    // start_request.topic_names.clear();
    // start_request.topic_names.push_back("/pressure/r_gripper_motor");
    // EXPECT_TRUE(start_r_arm_recording_pressure_states_service_client_.call(start_request, start_response));
    //
    // // service call to record, filter, and crop the pressure states of the left arm
    // start_request.topic_names.clear();
    // start_request.topic_names.push_back("/pressure/l_gripper_motor");
    // EXPECT_TRUE(start_l_arm_recording_pressure_states_service_client_.call(start_request, start_response));

    // service call to record, filter, and crop the pressure states of the right arm
    start_request.topic_names.clear();
    start_request.topic_names.push_back("/r_gripper_sensor_observer/tactile_sensor_info");
    EXPECT_TRUE(start_r_arm_recording_tactile_states_service_client_.call(start_request, start_response));

    // service call to record, filter, and crop the pressure states of the left arm
    start_request.topic_names.clear();
    start_request.topic_names.push_back("/l_gripper_sensor_observer/tactile_sensor_info");
    EXPECT_TRUE(start_l_arm_recording_tactile_states_service_client_.call(start_request, start_response));

    return true;
}

bool DMPMotionControllerTest::stopRecording()
{

    // service call to querry filtered and cropped joint states
    task_recorder::StopRecordingJointStates::Request stop_joint_states_request;
    task_recorder::StopRecordingJointStates::Response stop_joint_states_response;
    stop_joint_states_request.crop_start_time = execution_statistics_.start_time;
    stop_joint_states_request.crop_end_time = execution_statistics_.end_time;
    stop_joint_states_request.num_samples = num_samples_;
    stop_recording_joint_states_service_client_.call(stop_joint_states_request, stop_joint_states_response);

    // service call to querry filtered and cropped joint states of the left and right arm
    task_recorder::StopRecordingAccelerometerStates::Request stop_accelerometer_states_request;
    task_recorder::StopRecordingAccelerometerStates::Response stop_accelerometer_states_response;
    stop_accelerometer_states_request.crop_start_time = execution_statistics_.start_time;
    stop_accelerometer_states_request.crop_end_time = execution_statistics_.end_time;
    stop_accelerometer_states_request.num_samples = num_samples_;
    stop_r_arm_recording_accelerometer_states_service_client_.call(stop_accelerometer_states_request, stop_accelerometer_states_response);
    stop_l_arm_recording_accelerometer_states_service_client_.call(stop_accelerometer_states_request, stop_accelerometer_states_response);

//    // service call to querry filtered and cropped joint states of the left and right arm
//    task_recorder::StopRecordingPressureStates::Request stop_pressure_states_request;
//    task_recorder::StopRecordingPressureStates::Response stop_pressure_states_response;
//    stop_pressure_states_request.crop_start_time = execution_statistics_.start_time;
//    stop_pressure_states_request.crop_end_time = execution_statistics_.end_time;
//    stop_pressure_states_request.num_samples = num_samples_;
//    stop_r_arm_recording_pressure_states_service_client_.call(stop_pressure_states_request, stop_pressure_states_response);
//    stop_l_arm_recording_pressure_states_service_client_.call(stop_pressure_states_request, stop_pressure_states_response);

    task_recorder::StopRecordingTactileStates::Request stop_tactile_states_request;
    task_recorder::StopRecordingTactileStates::Response stop_tactile_states_response;
    stop_tactile_states_request.crop_start_time = execution_statistics_.start_time;
    stop_tactile_states_request.crop_end_time = execution_statistics_.end_time;
    stop_tactile_states_request.num_samples = num_samples_;
    stop_r_arm_recording_tactile_states_service_client_.call(stop_tactile_states_request, stop_tactile_states_response);
    stop_l_arm_recording_tactile_states_service_client_.call(stop_tactile_states_request, stop_tactile_states_response);

    return true;
}

TEST(dmp_controller_test, BOTH_CONTROLLER_TEST)
{
    ros::NodeHandle node_handle("~");
    DMPMotionControllerTest dmp_controller_test(node_handle);
    EXPECT_TRUE(dmp_controller_test.initialize());

    bool run_joint_space_controller;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("run_joint_space_controller"), run_joint_space_controller));

    bool run_cartesian_space_controller;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("run_cartesian_space_controller"), run_cartesian_space_controller));

    int run_test_trials;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("run_test_trials"), run_test_trials));

    bool test_left_arm;
    bool test_right_arm;
    bool test_both_arms;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("test_left_arm"), test_left_arm));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("test_right_arm"), test_right_arm));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("test_both_arms"), test_both_arms));

    if(run_joint_space_controller)
    {
        if (test_right_arm)
        {
            EXPECT_TRUE(dmp_controller_test.learnJointSpaceDMP(std::string("r_"), dmp_controller_test.r_arm_bag_file_name_, TEST_DMP_ID_1, DMPMotionControllerTest::RIGHT_ARM));
        }
        if (test_left_arm)
        {
            EXPECT_TRUE(dmp_controller_test.learnJointSpaceDMP(std::string("l_"), dmp_controller_test.l_arm_bag_file_name_, TEST_DMP_ID_2, DMPMotionControllerTest::LEFT_ARM));
        }
    }
    if(run_cartesian_space_controller)
    {
        if (test_right_arm)
        {
            EXPECT_TRUE(dmp_controller_test.learnCartesianSpaceDMP(dmp_controller_test.r_arm_bag_file_name_, TEST_DMP_ID_3, DMPMotionControllerTest::RIGHT_ARM));
        }
        if (test_left_arm)
        {
            EXPECT_TRUE(dmp_controller_test.learnCartesianSpaceDMP(dmp_controller_test.l_arm_bag_file_name_, TEST_DMP_ID_4, DMPMotionControllerTest::LEFT_ARM));
        }
    }

    for (int i=0; i<run_test_trials; ++i)
    {
        if(run_joint_space_controller)
        {
            if (test_both_arms)
            {
                //EXPECT_TRUE(dmp_controller_test.startRecording(TEST_DMP_ID_3));
                EXPECT_TRUE(dmp_controller_test.runDualArmJointSpaceControllerTest(TEST_DMP_ID_1, TEST_DMP_ID_2));
                EXPECT_TRUE(dmp_controller_test.waitForExecutionStatistics(20.0));
                //EXPECT_TRUE(dmp_controller_test.stopRecording());
            }
            if(test_right_arm)
            {
                //EXPECT_TRUE(dmp_controller_test.startRecording(TEST_DMP_ID_1));
                EXPECT_TRUE(dmp_controller_test.runJointSpaceControllerTest(std::string("r_"), TEST_DMP_ID_1, DMPMotionControllerTest::RIGHT_ARM));
                EXPECT_TRUE(dmp_controller_test.waitForExecutionStatistics(20.0));
                //EXPECT_TRUE(dmp_controller_test.stopRecording());
            }
            if (test_left_arm)
            {
                //EXPECT_TRUE(dmp_controller_test.startRecording(TEST_DMP_ID_2));
                EXPECT_TRUE(dmp_controller_test.runJointSpaceControllerTest(std::string("l_"), TEST_DMP_ID_2, DMPMotionControllerTest::LEFT_ARM));
                EXPECT_TRUE(dmp_controller_test.waitForExecutionStatistics(20.0));
                //EXPECT_TRUE(dmp_controller_test.stopRecording());
            }
        }
        if(run_cartesian_space_controller)
        {
            if (test_both_arms)
            {
                //EXPECT_TRUE(dmp_controller_test.startRecording(TEST_DMP_ID_3));
                EXPECT_TRUE(dmp_controller_test.runDualArmCartesianSpaceControllerTest(TEST_DMP_ID_3, TEST_DMP_ID_4));
                EXPECT_TRUE(dmp_controller_test.waitForExecutionStatistics(20.0));
                //EXPECT_TRUE(dmp_controller_test.stopRecording());
            }
            if (test_right_arm)
            {
                //EXPECT_TRUE(dmp_controller_test.startRecording(TEST_DMP_ID_3));
                EXPECT_TRUE(dmp_controller_test.runCartesianSpaceControllerTest(std::string("r_"), TEST_DMP_ID_3, DMPMotionControllerTest::RIGHT_ARM));
                EXPECT_TRUE(dmp_controller_test.waitForExecutionStatistics(20.0));
                //EXPECT_TRUE(dmp_controller_test.stopRecording());
            }
            if (test_left_arm)
            {
                //EXPECT_TRUE(dmp_controller_test.startRecording(TEST_DMP_ID_4));
                EXPECT_TRUE(dmp_controller_test.runCartesianSpaceControllerTest(std::string("l_"), TEST_DMP_ID_4, DMPMotionControllerTest::LEFT_ARM));
                EXPECT_TRUE(dmp_controller_test.waitForExecutionStatistics(20.0));
                //EXPECT_TRUE(dmp_controller_test.stopRecording());
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmp_controller_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
