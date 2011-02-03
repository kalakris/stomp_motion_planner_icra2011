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

 \file    pressure_states_recorder.cpp

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

// system includes

// ros includes
#include <policy_improvement_utilities/bspline.h>
#include <policy_improvement_utilities/assert.h>

#include <geometry_msgs/PointStamped.h>

// local includes
#include <task_recorder/pressure_states_recorder.h>

namespace task_recorder
{

const int num_pressure_sensors = 22;

PressureStatesRecorder::PressureStatesRecorder()
{
}

PressureStatesRecorder::~PressureStatesRecorder()
{
}

bool PressureStatesRecorder::initialize(ros::NodeHandle& node_handle, const std::string& topic_name)
{
    filtered_data_.resize(num_pressure_sensors);
    unfiltered_data_.resize(num_pressure_sensors);
    ROS_ASSERT_FUNC(((filters::MultiChannelFilterBase<double>&)filter_).configure(num_pressure_sensors, node_handle.getNamespace() + std::string("/HighPass"), node_handle));
    return initializeBase(node_handle, topic_name);
}

bool PressureStatesRecorder::filterAndCrop(const ros::Time& start_time, const ros::Time& end_time, int num_samples,
                                           std::vector<pr2_msgs::PressureState>& filtered_and_cropped_pressure_states)
{

    int num_pressure_states = recorder_io_.messages_.size();
    if (num_pressure_states == 0)
    {
        ROS_ERROR("Zero pressure states have been logged.");
        return false;
    }

    // figure out when our data starts and ends
    ros::Time our_start_time = recorder_io_.messages_[0].header.stamp;
    ros::Time our_end_time = recorder_io_.messages_[num_pressure_states - 1].header.stamp;

    if (our_start_time > start_time || our_end_time < end_time)
    {
        ROS_ERROR("Requested times have not been recorded!");
        ROS_ERROR_STREAM("Recorded start and end times : " << our_start_time << " to " << our_end_time);
        ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
        return false;
    }

    // fit bspline and resample the position and effort trajectories and compute the velocities
    ROS_ASSERT_FUNC(resample(recorder_io_.messages_, start_time, end_time, num_samples, filtered_and_cropped_pressure_states));
    ROS_ASSERT(static_cast<int>(filtered_and_cropped_pressure_states.size()) == num_samples);

    recorder_io_.messages_.clear();
    recorder_io_.messages_ = filtered_and_cropped_pressure_states;
    return true;
}

bool PressureStatesRecorder::transformMessages(pr2_msgs::PressureState& pressure_state)
{
    return true;
}

bool PressureStatesRecorder::resample(std::vector<pr2_msgs::PressureState>& pressure_states, const ros::Time& start_time, const ros::Time& end_time,
                                      const int num_samples, std::vector<pr2_msgs::PressureState>& resampled_pressure_states)
{

    ROS_ASSERT_FUNC(!pressure_states.empty());
    ROS_ASSERT_FUNC(removeDuplicates<pr2_msgs::PressureState>(pressure_states));

    int num_pressure_states = static_cast<int> (pressure_states.size());

    double mean_dt = 0.0;
    std::vector<double> input_vector;
    ROS_ASSERT_FUNC(computeMeanDtAndInputVector<pr2_msgs::PressureState>(pressure_states, mean_dt, input_vector));

    ros::Duration interval = static_cast<ros::Duration> (end_time - start_time) * (1.0 / double(num_samples - 1));

    double wave_length = interval.toSec() * 2;

    resampled_pressure_states.clear();
    std::string frame_id = pressure_states[0].header.frame_id;
    std::vector<double> input_querry(num_samples);
    for (int i = 0; i < num_samples; i++)
    {
        pr2_msgs::PressureState ps;
        ps.header.frame_id = frame_id;
        ps.header.seq = i;
        ps.header.stamp = static_cast<ros::Time> (start_time.toSec() + i * interval.toSec());
        ps.l_finger_tip.resize(num_pressure_sensors);
        ps.r_finger_tip.resize(num_pressure_sensors);
        input_querry[i] = ps.header.stamp.toSec();
        resampled_pressure_states.push_back(ps);
    }

    std::vector<double> target_vector;
    std::vector<double> resampled_vector;
    for (int i = 0; i < num_pressure_sensors; ++i)
    {
        // left finger tip
        target_vector.clear();
        resampled_vector.clear();
        for (int j = 0; j < num_pressure_states; ++j)
        {
            target_vector.push_back(pressure_states[j].l_finger_tip[i]);
            resampled_vector.push_back(pressure_states[j].l_finger_tip[i]);
        }
        if (!policy_improvement_utilities::resample(input_vector, target_vector, wave_length, input_querry, resampled_vector, false))
        {
            ROS_ERROR("Could not rescale pressure vector %i, splining failed.", i);
            return false;
        }
        for (int j = 0; j < num_samples; ++j)
        {
            resampled_pressure_states[j].l_finger_tip[i] = resampled_vector[j];
        }

        // right finger tip
        target_vector.clear();
        resampled_vector.clear();
        for (int j = 0; j < num_pressure_states; ++j)
        {
            target_vector.push_back(pressure_states[j].r_finger_tip[i]);
            resampled_vector.push_back(pressure_states[j].r_finger_tip[i]);
        }
        if (!policy_improvement_utilities::resample(input_vector, target_vector, wave_length, input_querry, resampled_vector, false))
        {
            ROS_ERROR("Could not rescale pressure vector %i, splining failed.", i);
            return false;
        }
        for (int j = 0; j < num_samples; ++j)
        {
            resampled_pressure_states[j].r_finger_tip[i] = resampled_vector[j];
        }
    }
    return true;
}

}
