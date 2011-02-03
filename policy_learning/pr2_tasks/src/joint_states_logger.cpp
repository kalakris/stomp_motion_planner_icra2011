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

#include <pr2_tasks/joint_states_logger.h>
#include <boost/interprocess/sync/scoped_lock.hpp>

namespace pr2_tasks
{

JointStatesLogger::JointStatesLogger():
        initialized_(false)
{
}

JointStatesLogger::~JointStatesLogger()
{
}

bool JointStatesLogger::initialize(ros::NodeHandle& node_handle)
{
    node_handle_ = node_handle;

    // listen to /joint_states
    joint_states_subscriber_ = node_handle_.subscribe("/joint_states", 100, &JointStatesLogger::jointStateCallback, this);

    return (initialized_ = true);
}

void JointStatesLogger::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
    joint_states_mutex_.lock();
    if (logging_)
    {
        joint_states_.push_back(*joint_state);
    }
    joint_states_mutex_.unlock();
}

void JointStatesLogger::startLogging()
{
    joint_states_mutex_.lock();
    logging_ = true;
    joint_states_.clear();
    joint_states_mutex_.unlock();
}

void JointStatesLogger::stopLogging()
{
    joint_states_mutex_.lock();
    logging_ = false;
    joint_states_mutex_.unlock();
}

bool JointStatesLogger::getJointStates(const ros::Time& start_time, const ros::Time& end_time, int num_samples, std::vector<sensor_msgs::JointState>& binned_joint_states)
{
    boost::interprocess::scoped_lock<boost::mutex> lock(joint_states_mutex_);
    int num_joint_states = joint_states_.size();
    if (num_joint_states==0)
    {
        ROS_ERROR("Zero joint states have been logged.");
        return false;
    }

    // figure out when our data starts and ends
    ros::Time our_start_time = joint_states_[0].header.stamp;
    ros::Time our_end_time = joint_states_[num_joint_states-1].header.stamp;

    if (our_start_time > start_time || our_end_time < end_time)
    {
        ROS_ERROR("Requested times have not been recorded!");
        ROS_ERROR_STREAM("Recorded start and end times: " << our_start_time << " to " << our_end_time);
        ROS_ERROR_STREAM("Requested start and end times: " << start_time << " to " << end_time);
        return false;
    }

    ros::Duration interval = (end_time - start_time);
    interval *= 1.0/double(num_samples-1);

    binned_joint_states.clear();

    // TODO: there should really be some kind of zero-phase FIR low-pass filter here

    int our_data_ptr=0;
    ros::Time bin_data_time = start_time;
    for (int bin_data_ptr = 0; bin_data_ptr < num_samples; ++bin_data_ptr)
    {
        // advance the our_data_ptr until it's ahead of bin_data_time:
        while (our_data_ptr < num_joint_states && joint_states_[our_data_ptr].header.stamp < bin_data_time)
            ++our_data_ptr;

        if (our_data_ptr >= num_joint_states)
            break;

        binned_joint_states.push_back(joint_states_[our_data_ptr]);

        bin_data_time += interval;
    }

    return true;
}

}
