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

 \file    task_recorder_test.cpp

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

// system includes
#include <string>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <rosbag/player.h>

//#include <boost/thread.hpp>
//#include <boost/foreach.hpp>

// #include <roslib/Clock.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>

//#include <task_recorder/StartRecording.h>
//#include <task_recorder/StopRecordingJointStates.h>
//#include <task_recorder/StopRecordingAccelerometerStates.h>

#include <task_recorder/task_recorder_utilities.h>

#include <task_recorder/accumulator.h>
#include <std_msgs/Float64.h>
#include <task_recorder_test/TaskRecorderTest.h>
#include <task_recorder/AccumulatedTrialStatistics.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

//#include <pr2_msgs/PressureState.h>
//#include <pr2_msgs/AccelerometerState.h>

using namespace task_recorder;

class TaskRecorderTest
{

public:
    TaskRecorderTest(ros::NodeHandle& node_handle);

    bool initialize();

    bool testAccumulator();

    // bool runJointStatesTest();

    // bool generateClock(const double total_duration);

    // void playBagFile(const std::vector<std::string>& bag_file_names);

    // void getStartAndEndTimes(ros::Time& start_time, ros::Time& end_time);

    // template <typename MessageType>
    // bool getTimingsFromBagFile(const std::string& bag_file_name, const std::string& topic_name, ros::Time& start_time, ros::Time& end_time);

    // template <typename MessageType>
    // bool setTimeStampsInBagFile(const std::string& bag_file_name, const std::string& topic_name, double& total_duration);

private:

    bool initialized_;

    ros::NodeHandle node_handle_;

    std::string data_bag_file_name_;

//    ros::Time joint_state_start_time_;
//    ros::Time joint_state_end_time_;

    int num_samples_;
//    int num_iterations_;

//    ros::ServiceClient start_recording_joint_states_service_client_;
//    ros::ServiceClient stop_recording_joint_states_service_client_;

//    ros::ServiceClient start_recording_accelerometer_states_service_client_;
//    ros::ServiceClient stop_recording_accelerometer_states_service_client_;
};

TaskRecorderTest::TaskRecorderTest(ros::NodeHandle& node_handle) :
    initialized_(false), node_handle_(node_handle)
{
}

bool TaskRecorderTest::initialize()
{

    // load bag file
    std::string package_name;

    EXPECT_TRUE(policy_improvement_utilities::read(node_handle_, std::string("package_name"), package_name));
    std::string package_path = ros::package::getPath(package_name);
    policy_improvement_utilities::appendTrailingSlash(package_path);

    EXPECT_TRUE(policy_improvement_utilities::read(node_handle_, std::string("num_samples"), num_samples_));
//    EXPECT_TRUE(policy_improvement_utilities::read(node_handle_, std::string("num_iterations"), num_iterations_));

    EXPECT_TRUE(policy_improvement_utilities::read(node_handle_, std::string("data_bag_file_name"), data_bag_file_name_));
    data_bag_file_name_.assign(package_path + data_bag_file_name_);

    ROS_INFO_STREAM("Read: " << data_bag_file_name_);

//    std::string topic_name;
//    std::string start_recording_service_name;
//    std::string stop_recording_service_name;
//
//    topic_name.assign("/joint_states");
//    ROS_ASSERT_FUNC(task_recorder::getTopicName(topic_name));
//    start_recording_service_name.assign(std::string("/task_recorder/start_recording_") + topic_name);
//    stop_recording_service_name.assign(std::string("/task_recorder/stop_recording_") + topic_name);
//    start_recording_joint_states_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording> (start_recording_service_name);
//    stop_recording_joint_states_service_client_ = node_handle_.serviceClient<task_recorder::StopRecordingJointStates> (stop_recording_service_name);
//
//    topic_name.assign("/accelerometer/r_gripper_motor");
//    ROS_ASSERT_FUNC(task_recorder::getTopicName(topic_name));
//    start_recording_service_name.assign(std::string("/task_recorder/start_recording_") + topic_name);
//    stop_recording_service_name.assign(std::string("/task_recorder/stop_recording_") + topic_name);
//    start_recording_accelerometer_states_service_client_ = node_handle_.serviceClient<task_recorder::StartRecording> (start_recording_service_name);
//    stop_recording_accelerometer_states_service_client_  = node_handle_.serviceClient<task_recorder::StopRecordingAccelerometerStates> (stop_recording_service_name);

    initialized_ = true;
    return true;
}

//void TaskRecorderTest::playBagFile(const std::vector<std::string>& bag_file_names)
//{
//    // play bag file
//    rosbag::PlayerOptions opts;
//    opts.bags = bag_file_names;
//    // publish the clock time
//    opts.bag_time = true;
//    opts.bag_time_frequency = 1000;
//
//    opts.quiet = true;
//    opts.advertise_sleep = ros::WallDuration(0.0);
//
//    rosbag::Player player(opts);
//    try
//    {
//        player.publish();
//    }
//    catch (std::runtime_error& e)
//    {
//        ROS_ERROR("%s", e.what());
//    }
//}

//bool TaskRecorderTest::generateClock(const double total_duration)
//{
//
//    std::vector<ros::Time> time_stamps;
//    double dt = 0.001;
//
//    ros::Time start_time(0.0);
//    ros::Time time = start_time;
//    while (time < start_time.fromSec(total_duration))
//    {
//        time += ros::Duration(dt);
//        time_stamps.push_back(time);
//    }
//
//    std::vector<roslib::Clock> clock_stamps;
//    for (int i=0; i<static_cast<int>(time_stamps.size()); ++i)
//    {
//        roslib::Clock clock;
//        clock.clock = time_stamps[i];
//        clock_stamps.push_back(clock);
//    }
//
//    try
//    {
//        rosbag::Bag bag;
//        bag.open(clock_bag_file_name_, rosbag::bagmode::Write);
//        for (int i = 0; i < static_cast<int> (clock_stamps.size()); ++i)
//        {
//            bag.write("/clock", clock_stamps[i].clock, clock_stamps[i]);
//        }
//        bag.close();
//    }
//    catch (rosbag::BagIOException ex)
//    {
//        ROS_ERROR("Problem when writing to bag file named %s.", clock_bag_file_name_.c_str());
//        return false;
//    }
//
//    return true;
//}

//template <typename MessageType>
//bool TaskRecorderTest::setTimeStampsInBagFile(const std::string& bag_file_name, const std::string& topic_name, double& total_duration)
//{
//
//    bool start_time_set;
//    ros::Time start_time;
//
//    std::vector<MessageType> messages;
//
//    try
//    {
//        rosbag::Bag bag;
//        bag.open(bag_file_name, rosbag::bagmode::Read);
//
//        std::vector<std::string> topics;
//        topics.push_back(std::string(topic_name));
//        rosbag::View view(bag, rosbag::TopicQuery(topics));
//
//        start_time_set = false;
//        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
//        {
//            typename MessageType::ConstPtr msg_ptr = msg.instantiate<MessageType>();
//            if (msg_ptr != NULL)
//            {
//                if(!start_time_set)
//                {
//                    start_time_set = true;
//                    start_time = msg_ptr->header.stamp - ros::Duration().fromNSec(1);
//                }
//                MessageType m = *msg_ptr;
//                ros::Duration offset(m.header.stamp - start_time);
//                m.header.stamp = ros::Time(0.0) + offset;
//
//                if(offset.toSec() > total_duration)
//                {
//                    total_duration = offset.toSec();
//                }
//                messages.push_back(m);
//            }
//        }
//        bag.close();
//    }
//    catch (rosbag::BagIOException ex)
//    {
//        ROS_ERROR("Problem when reading from bag file %s.", data_bag_file_name_.c_str());
//        return false;
//    }
//
//    try
//    {
//        rosbag::Bag bag;
//        bag.open(test_data_bag_file_name_, rosbag::bagmode::Append);
//        for (int i = 0; i < static_cast<int> (messages.size()); ++i)
//        {
//            bag.write(std::string(topic_name), messages[i].header.stamp, messages[i]);
//        }
//        bag.close();
//
//    }
//    catch (rosbag::BagIOException ex)
//    {
//        ROS_ERROR("Problem when writing bag file %s.", data_bag_file_name_.c_str());
//        return false;
//    }
//
//    return start_time_set;
//}

//template<typename MessageType>
//bool TaskRecorderTest::getTimingsFromBagFile(const std::string& bag_file_name, const std::string& topic_name, ros::Time& start_time, ros::Time& end_time)
//{
//
//    bool start_time_set;
//    try
//    {
//        rosbag::Bag bag;
//        bag.open(bag_file_name, rosbag::bagmode::Read);
//
//        std::vector<std::string> topics;
//        topics.push_back(std::string(topic_name));
//        rosbag::View view(bag, rosbag::TopicQuery(topics));
//
//        start_time_set = false;
//        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
//        {
//            typename MessageType::ConstPtr msg_ptr = msg.instantiate<MessageType>();
//            if (msg_ptr != NULL)
//            {
//                if(!start_time_set)
//                {
//                    start_time_set = true;
//                    start_time = msg_ptr->header.stamp;
//                }
//                end_time = msg_ptr->header.stamp;
//            }
//        }
//        bag.close();
//    }
//    catch (rosbag::BagIOException ex)
//    {
//        ROS_ERROR("Problem when reading from bag file %s.", data_bag_file_name_.c_str());
//        return false;
//    }
//
//    return true;
//}

//void TaskRecorderTest::getStartAndEndTimes(ros::Time& start_time, ros::Time& end_time)
//{
//    EXPECT_TRUE(getTimingsFromBagFile<sensor_msgs::JointState>(data_bag_file_name_, std::string("/joint_states"), start_time, end_time));
//    ros::Time earliest_start_time = start_time;
//    ros::Time latest_end_time = end_time;
//    EXPECT_TRUE(getTimingsFromBagFile<pr2_msgs::PressureState>(data_bag_file_name_, std::string("/pressure/r_gripper_motor"), start_time, end_time));
//    if(start_time < earliest_start_time)
//    {
//        start_time = earliest_start_time;
//    }
//    if(end_time > latest_end_time)
//    {
//        end_time = latest_end_time;
//    }
//}
//
//bool TaskRecorderTest::runJointStatesTest()
//{
//
//    EXPECT_TRUE(initialize());
//    ROS_INFO("Running %i test iterations.", num_iterations_);
//
//    for(int i=0; i<num_iterations_; i++)
//    {
//        ros::Time start_time;
//        ros::Time end_time;
//        getStartAndEndTimes(start_time, end_time);
//
//        ros::Duration duration = end_time - start_time;
//
//        std::vector<std::string> bag_file_names;
//        bag_file_names.push_back(data_bag_file_name_);
//
//        // play bag file
//        boost::thread play_bag_file_thread( boost::bind( &TaskRecorderTest::playBagFile, this, bag_file_names ) );
//
//        task_recorder::StartRecording::Request start_request;
//        task_recorder::StartRecording::Response start_response;
//
//        // service call to record, filter, and crop the joint states
//        start_request.topic_names.clear();
//        start_request.topic_names.push_back("/joint_states");
//        EXPECT_TRUE(start_recording_joint_states_service_client_.call(start_request, start_response));
//
//        // service call to record, filter, and crop the accelerometer states
//        start_request.topic_names.clear();
//        start_request.topic_names.push_back("/accelerometer/r_gripper_motor");
//        EXPECT_TRUE(start_recording_accelerometer_states_service_client_.call(start_request, start_response));
//
//        // make sure that the bag file is still playing (and publishing the clock) in order
//        // for the next service call to work
//        ros::Duration(duration * static_cast<double>(0.8)).sleep();
//
//        ros::Time crop_start_time = start_time + ros::Duration(duration * static_cast<double>(0.3));
//        ros::Time crop_end_time = start_time + ros::Duration(duration * static_cast<double>(0.6));
//
//        // service call to querry filtered and cropped joint states
//        task_recorder::StopRecordingAccelerometerStates::Request stop_accelerometer_states_request;
//        task_recorder::StopRecordingAccelerometerStates::Response stop_accelerometer_states_response;
//        stop_accelerometer_states_request.crop_start_time = crop_start_time;
//        stop_accelerometer_states_request.crop_end_time = crop_end_time;
//        stop_accelerometer_states_request.num_samples = num_samples_;
//        stop_recording_accelerometer_states_service_client_.call(stop_accelerometer_states_request, stop_accelerometer_states_response);
//
//        // service call to querry filtered and cropped joint states
//        task_recorder::StopRecordingJointStates::Request stop_joint_states_request;
//        task_recorder::StopRecordingJointStates::Response stop_joint_states_response;
//        stop_joint_states_request.crop_start_time = crop_start_time;
//        stop_joint_states_request.crop_end_time = crop_end_time;
//        stop_joint_states_request.num_samples = num_samples_;
//        stop_recording_joint_states_service_client_.call(stop_joint_states_request, stop_joint_states_response);
//
//        // wait for the bag to finish
//        play_bag_file_thread.join();
//
//    }
//    return true;
//}
//
//TEST(task_recorder_test, runJointStatesTest)
//{
//    ros::NodeHandle node_handle("~");
//    TaskRecorderTest task_recorder_test(node_handle);
//    EXPECT_TRUE(task_recorder_test.runJointStatesTest());
//}

bool TaskRecorderTest::testAccumulator()
{

    Accumulator accumulator;
    EXPECT_TRUE(accumulator.initialize(2, num_samples_));

    std::vector<double> data_vector_1;
    std::vector<double> data_vector_2;

    srand ( time(NULL) );

    for(int i=0; i<num_samples_; ++i)
    {
        double rand_num = (rand() % 100) + 1;
        double noise = 1.0/rand_num;

        data_vector_1.push_back(i+noise);
        data_vector_2.push_back(noise);
    }


    for(int i=0; i<10; ++i)
    {
        accumulator.add(0, data_vector_1);
        accumulator.add(1, data_vector_2);
    }

    std::vector<task_recorder::AccumulatedTrialStatistics> accumulated_trial_statistics;
    EXPECT_TRUE(accumulator.getAccumulatedTrialStatistics(accumulated_trial_statistics));

    try
    {
        rosbag::Bag bag(data_bag_file_name_, rosbag::bagmode::Write);
        for (int i=0; i<int(accumulated_trial_statistics.size()); ++i)
        {
            bag.write("/accumulated_trial_statistics", ros::Time(1) + ros::Duration(i), accumulated_trial_statistics[i]);
        }
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s: %s", data_bag_file_name_.c_str(), ex.what());
        return false;
    }

    return true;
}

TEST(task_recorder_test, testAccumulator)
{
    ros::NodeHandle node_handle("~");
    TaskRecorderTest task_recorder_test(node_handle);
    EXPECT_TRUE(task_recorder_test.initialize());
    EXPECT_TRUE(task_recorder_test.testAccumulator());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_recorder_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
