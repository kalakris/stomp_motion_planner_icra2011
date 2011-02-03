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

 \file    task_recorder.h

 \author  Peter Pastor
 \date    Jul 12, 2010

 **********************************************************************/

#ifndef TASK_RECORDER_H_
#define TASK_RECORDER_H_

// system includes
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

// ros includes
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/assert.h>

// local includes
#include <task_recorder/task_recorder_io.h>
#include <task_recorder/task_recorder_utilities.h>
#include <task_recorder/StartRecording.h>
#include <task_recorder/AccumulatedTrialStatistics.h>

namespace task_recorder
{

template<class MessageType, class StopServiceType>
    class TaskRecorder
    {

    public:

        typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;
        typedef boost::shared_ptr<StopServiceType const> StopServiceTypeConstPtr;

        /*!
         * @return
         */
        TaskRecorder();
        virtual ~TaskRecorder();

        /*!
         *
         * @param node_handle
         * @param topic_name
         * @param type
         * @return
         */
        bool initializeBase(ros::NodeHandle& node_handle, const std::string& topic_name);

        /*!
         */
        void startRecording();

        /*!
         */
        void stopRecording();

        /*!
         * @param request
         * @param response
         * @return
         */
        bool startRecording(task_recorder::StartRecording::Request& request, task_recorder::StartRecording::Response& response);

        /*!
         * @param request
         * @param response
         * @return
         */
        bool stopRecording(typename StopServiceType::Request& request, typename StopServiceType::Response& response);

    protected:

        bool initialized_;
        bool is_recording_;

        TaskRecorderIO<MessageType> recorder_io_;

    private:

        ros::ServiceServer start_recording_service_server_;
        ros::ServiceServer stop_recording_service_server_;

        ros::Subscriber subscriber_;

        boost::mutex mutex_;
        bool logging_;

        void recordMessagesCallback(const MessageTypeConstPtr message);

        virtual bool filterAndCrop(const ros::Time& start_time, const ros::Time& end_time, const double movement_duration, int num_samples,
                                   std::vector<MessageType>& filter_and_cropped_messages, std::vector<std::string>& message_names,
                                   std::vector<ros::Time>& times, std::vector<double>& data) = 0;

        virtual bool transformMessages(MessageType& message) = 0;

        virtual bool getAccumulatedTrialStatistics(std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> >& vector_of_accumulated_trial_statistics) = 0;
        virtual void setMessageNames(std::vector<task_recorder::AccumulatedTrialStatistics>& accumulated_trial_statistics) = 0;
        virtual void getSignalNames(const int signal_index, std::string& signal_name) = 0;

    };

    template<class MessageType, class StopServiceType>
    TaskRecorder<MessageType, StopServiceType>::TaskRecorder() :
        initialized_(false), is_recording_(false)
    {
    }

template<class MessageType, class StopServiceType>
    TaskRecorder<MessageType, StopServiceType>::~TaskRecorder()
    {
    }

template<class MessageType, class StopServiceType>
    bool TaskRecorder<MessageType, StopServiceType>::initializeBase(ros::NodeHandle& node_handle, const std::string& topic_name)
    {

        ROS_ASSERT_FUNC(recorder_io_.initialize(node_handle, topic_name));

        std::string name = topic_name;
        ROS_ASSERT_FUNC(getTopicName(name));
        std::string start_recording_service_name = std::string("start_recording_") + name;
        std::string stop_recording_service_name = std::string("stop_recording_") + name;

        start_recording_service_server_ = recorder_io_.node_handle_.advertiseService(start_recording_service_name, &TaskRecorder<MessageType, StopServiceType>::startRecording, this);
        stop_recording_service_server_ = recorder_io_.node_handle_.advertiseService(stop_recording_service_name, &TaskRecorder<MessageType, StopServiceType>::stopRecording, this);

        return (initialized_ = true);
    }

template<class MessageType, class StopServiceType>
    void TaskRecorder<MessageType, StopServiceType>::recordMessagesCallback(const MessageTypeConstPtr message)
    {
        mutex_.lock();
        if (logging_)
        {
            MessageType msg = *message;
            transformMessages(msg);
            recorder_io_.messages_.push_back(msg);
        }
        mutex_.unlock();
    }

template<class MessageType, class StopServiceType>
    void TaskRecorder<MessageType, StopServiceType>::startRecording()
    {
        ROS_INFO("Start recording topic named %s.", recorder_io_.topic_name_.c_str());
        subscriber_ = recorder_io_.node_handle_.subscribe(recorder_io_.topic_name_, 10000, &TaskRecorder<MessageType, StopServiceType>::recordMessagesCallback, this);
        mutex_.lock();
        logging_ = true;
        recorder_io_.messages_.clear();
        mutex_.unlock();
    }

template<class MessageType, class StopServiceType>
    void TaskRecorder<MessageType, StopServiceType>::stopRecording()
    {
        ROS_INFO("Stop recording topic named %s.", recorder_io_.topic_name_.c_str());
        mutex_.lock();
        logging_ = false;
        mutex_.unlock();
        subscriber_.shutdown();
    }

template<class MessageType, class StopServiceType>
    bool TaskRecorder<MessageType, StopServiceType>::startRecording(task_recorder::StartRecording::Request& request,
                                                                    task_recorder::StartRecording::Response& response)
    {
        startRecording();
        recorder_io_.setId(request.id);
        response.return_code = task_recorder::StartRecording::Response::SERVICE_CALL_SUCCESSFUL;
        return true;
    }

template<class MessageType, class StopServiceType>
    bool TaskRecorder<MessageType, StopServiceType>::stopRecording(typename StopServiceType::Request& request, typename StopServiceType::Response& response)
    {

        stopRecording();
        std::vector<MessageType> filtered_and_cropped_messages;
        std::vector<ros::Time> times;
        std::vector<double> data;

        if (!filterAndCrop(request.crop_start_time, request.crop_end_time, request.movement_duration, request.num_samples,
                           filtered_and_cropped_messages, request.message_names, times, data))
        {
            ROS_ERROR("Could not filter and crop messages.");
            response.return_code = StopServiceType::Response::SERVICE_CALL_FAILED;
            return true;
        }

        // write out data files
        boost::thread( boost::bind( &TaskRecorderIO<MessageType>::writeRecordedData, recorder_io_) );

        // write out statistics files
        std::vector<std::vector<task_recorder::AccumulatedTrialStatistics> > vector_of_accumulated_trial_statistics;
        ROS_ASSERT_FUNC(getAccumulatedTrialStatistics(vector_of_accumulated_trial_statistics));
        boost::thread( boost::bind( &TaskRecorderIO<MessageType>::writeStatistics, recorder_io_, vector_of_accumulated_trial_statistics ) );

        response.filtered_and_cropped_messages = filtered_and_cropped_messages;
        response.times = times;
        response.data = data;
        response.return_code = StopServiceType::Response::SERVICE_CALL_SUCCESSFUL;
        return true;
    }
}

#endif /* TASK_RECORDER_H_ */
