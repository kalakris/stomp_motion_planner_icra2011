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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
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
 *********************************************************************/

/** \author Peter Pastor */

#ifndef DMP_CONTROLLER_H_
#define DMP_CONTROLLER_H_

// system includes
// #include <deque>
#include <map>
#include <pthread.h>

// ros includes
#include <ros/ros.h>

#include <rosrt/rosrt.h>

#include <dmp_motion_generation/dynamic_movement_primitive.h>
#include <dmp_motion_generation/trajectory.h>

#include <geometry_msgs/PointStamped.h>

#include <sensor_msgs/JointState.h>

#include <tf/transform_listener.h>

#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>

#include <pr2_tasks_transforms/task_transforms.h>

#include <boost/circular_buffer.hpp>

// local includes
#include <dmp_motion_controller/AddToExecuteDMPQueue.h>
#include <dmp_motion_controller/WriteTrajectories.h>
// #include <dmp_motion_controller/ControllerStatus.h>
#include <dmp_motion_controller/DMPExecutionStatistics.h>

#include <dmp_motion_controller/dmp_controller_common.h>

namespace dmp_controller
{

/*! \class The DMPController class contains the interface for controller that use set
 * points generated by a Dynamic Movement Primitive (DMP).
 */
class DMPController
{

public:

    /*! constructor
     */
    DMPController();

    /*! destructor
     */
    ~DMPController();

    /*!
     */
    enum RepresentationType
    {
        UNINITIALIZED = -1, JOINT, CARTESIAN_AND_JOINT
    };

    /*!
     * @return
     */
    bool initialize(ros::NodeHandle& node_handle, const std::vector<int>& num_dofs, RepresentationType dmp_representation_type = UNINITIALIZED);

    /*!
     * @param request
     * @param response
     * @return
     */
    bool addToExecuteDMPQueue(dmp_motion_controller::AddToExecuteDMPQueue::Request& request, dmp_motion_controller::AddToExecuteDMPQueue::Response& response);

    /*!
     * @param request
     * @param response
     * @return
     */
    bool writeTrajectories(dmp_motion_controller::WriteTrajectories::Request& request, dmp_motion_controller::WriteTrajectories::Response& response);

    /*!
     * @param request
     * @param response
     * @return
     */
    // bool getControllerStatus(dmp_motion_controller::ControllerStatus::Request& request, dmp_motion_controller::ControllerStatus::Response& response);

    /*!
     * @return
     */
    std::vector<int> getNumDofs() const;
    std::string getVariableNamesKeyWord() const;

    /*!
     * @return
     */
    bool tryGetLock();

    /*!
     */
    void freeLock();
    void incrementDMPQueueIndex();

    /*!
     * @return
     */
    boost::shared_ptr<dmp::DynamicMovementPrimitive> getCurrentDMP();

    /*!
     * @return
     */
    double getCurrentExecutionDuration();

    /*!
     * @return
     */
    pr2_tasks_transforms::TaskTransforms::TransformType getCurrentType();

    /*!
     * @param trajectory_point
     */
    void addDebugDesired(Eigen::VectorXd& trajectory_point);
    /*!
     * @param trajectory_point
     */
    void addDebugActual(Eigen::VectorXd& trajectory_point);

    /*!
     */
    void incrementSkippedCycles();

    /*!
     * @param start_time
     */
    void setStartTime(const ros::Time start_time);

    /*!
     * @param end_time
     */
    void setEndTime(const ros::Time end_time);
    bool firstDMPControlCycle();


    /*!
     * @return
     */
    bool isQueueEmpty();

    bool addToQueue(std::vector<DMPStruct> dmp_structs);

private:

    /*! node handle
     */
    ros::NodeHandle node_handle_;

    /*!
     */
    // std::vector<std::string> joint_names_;

    /*!
     */
    std::vector<int> num_dofs_;

    /*! \brief mutex lock for accessing dmp circular buffer
     */
    pthread_mutex_t dmp_cmd_lock_;

    boost::circular_buffer<DMPStruct> run_buffer_;
    boost::circular_buffer<DMPStruct> completed_buffer_;

    /*!
     */
    std::string variable_names_key_word_;

    /*! for diagnostics
     */
    int skipped_updates_;

    /*!
     */
    ros::ServiceServer add_to_execute_dmp_queue_service_server_;
    ros::ServiceServer write_trajectories_service_server_;
    // ros::ServiceServer get_controller_status_service_server_;

    /*!
     */
    boost::shared_ptr<rosrt::Publisher<dmp_motion_controller::DMPExecutionStatistics> > dmp_execution_statistics_publisher_;

    /*!
     */
    bool first_control_cycle_;
    ros::Time start_time_;
    ros::Time end_time_;

    /*!
     */
    void publishStatistics();

    /*!
     * @param dmp_representation_type
     * @return
     */
    bool setControllerVariables(RepresentationType dmp_representation_type, const std::string controller_namespace);

};

// inline functions

// REAL-TIME REQUIREMENTS
inline bool DMPController::tryGetLock()
{
    if (pthread_mutex_trylock(&dmp_cmd_lock_) == 0) // never forget the inverse check again...
    {
        if (!run_buffer_.empty())
        {
            return true;
        }
    }
    return false;
}
// REAL-TIME REQUIREMENTS
inline void DMPController::freeLock()
{
    pthread_mutex_unlock(&dmp_cmd_lock_);
}

// REAL-TIME REQUIREMENTS
inline void DMPController::incrementDMPQueueIndex()
{
    // WARNING: dmp_cmd_lock_ need to be set !!
    publishStatistics();
    // HACK
//    if(run_buffer_.front().type == pr2_tasks_transforms::TaskTransforms::RIGHT_ARM_POOL_TRANSFORM)
//    {
//        completed_buffer_.push_back(run_buffer_.front());
//    }
    run_buffer_.pop_front();
    first_control_cycle_=true;
}

// REAL-TIME REQUIREMENTS
inline bool DMPController::isQueueEmpty()
{
    bool is_empty = false;
    if (pthread_mutex_trylock(&dmp_cmd_lock_) == 0) // never forget the inverse check again...
    {
        is_empty = run_buffer_.empty();
        pthread_mutex_unlock(&dmp_cmd_lock_);
    }
    return is_empty;
}

// REAL-TIME REQUIREMENTS
inline boost::shared_ptr<dmp::DynamicMovementPrimitive> DMPController::getCurrentDMP()
{
    return run_buffer_.front().dmp;
}
// REAL-TIME REQUIREMENTS
inline double DMPController::getCurrentExecutionDuration()
{
    return run_buffer_.front().execution_duration;
}
// REAL-TIME REQUIREMENTS
inline pr2_tasks_transforms::TaskTransforms::TransformType DMPController::getCurrentType()
{
    return run_buffer_.front().type;
}

inline void DMPController::addDebugDesired(Eigen::VectorXd& trajectory_point)
{
    run_buffer_.front().debug_object_desired->add(trajectory_point);
}
inline void DMPController::addDebugActual(Eigen::VectorXd& trajectory_point)
{
    run_buffer_.front().debug_object_actual->add(trajectory_point);
}

inline void DMPController::incrementSkippedCycles()
{
    skipped_updates_++;
}
inline void DMPController::setStartTime(const ros::Time start_time)
{
    start_time_ = start_time;
}
inline void DMPController::setEndTime(const ros::Time end_time)
{
    end_time_ = end_time;
}
inline bool DMPController::firstDMPControlCycle()
{
    if(first_control_cycle_)
    {
        first_control_cycle_ = false;
        return true;
    }
    return false;
}

inline std::string DMPController::getVariableNamesKeyWord() const
{
    return variable_names_key_word_;
}
inline std::vector<int> DMPController::getNumDofs() const
{
    return num_dofs_;
}

}

#endif /* DMP_CONTROLLER_H_ */
