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

#ifndef DMP_EXECUTOR_H_
#define DMP_EXECUTOR_H_

// system includes
#include <boost/thread.hpp>

// ros includes
#include <ros/ros.h>
#include <dmp_motion_controller/DMPExecutionStatistics.h>
#include <dmp_motion_generation/dynamic_movement_primitive.h>

#include <task_manager/controller_switcher.h>

#include <pr2_tasks_transforms/task_transforms.h>

//local includes

namespace pr2_tasks
{

class DMPExecutor
{
public:

    enum WhichArm
    {
        LEFT_ARM = 1,
        RIGHT_ARM
    };

    DMPExecutor();
    virtual ~DMPExecutor();

    bool initialize(ros::NodeHandle& node_handle);

    /*!
     * @param dmp
     * @param execution_statistics
     * @param execution_duration
     * @param timeout
     * @param transform
     * @param write_debug
     * @return
     */
    bool executeDMPAndWait(dmp::DynamicMovementPrimitive& dmp,
                           pr2_tasks_transforms::TaskTransforms::TransformType transform_type,
                           dmp_motion_controller::DMPExecutionStatistics& execution_statistics,
                           ros::Duration& execution_duration,
                           ros::Duration& timeout,
                           bool write_debug = false, const int iteration_number = 0);

    /*!
     *
     * @param right_arm_dmp
     * @param transform_type
     * @param execution_statistics
     * @param execution_duration
     * @param timeout
     * @param movement_duration
     * @param right_arm_dmp
     * @param write_debug
     * @param iteration_number
     * @return
     */
    bool executeDMPAndWait(dmp::DynamicMovementPrimitive& right_arm_dmp,
                           dmp_motion_controller::DMPExecutionStatistics& execution_statistics,
                           ros::Duration& right_arm_execution_duration,
                           ros::Duration& timeout,
                           const double right_arm_movement_duration,
                           dmp::DynamicMovementPrimitive& left_arm_dmp,
                           const double left_arm_execution_duration,
                           bool write_debug, const int iteration_number);

    /*!
     * @param left_arm_dmp
     * @param right_arm_dmp
     * @param execution_statistics
     * @param execution_duration
     * @param timeout
     * @param left_arm_transform
     * @param right_arm_transform
     * @param write_debug
     * @return
     */
    bool executeDMPAndWait(dmp::DynamicMovementPrimitive& dual_arm_dmp,
                           pr2_tasks_transforms::TaskTransforms::TransformType left_arm_transform_type,
                           pr2_tasks_transforms::TaskTransforms::TransformType right_arm_transform_type,
                           dmp_motion_controller::DMPExecutionStatistics& execution_statistics,
                           ros::Duration& execution_duration,
                           ros::Duration& timeout,
                           bool write_debug = false, const int iteration_number = 0);

    /*!
     *
     * @param dual_arm_dmp
     * @param execution_statistics
     * @param execution_duration
     * @param timeout
     * @return
     */
    bool executeDMPAndWait(dmp::DynamicMovementPrimitive& dual_arm_dmp,
                           dmp_motion_controller::DMPExecutionStatistics& execution_statistics,
                           ros::Duration& execution_duration,
                           ros::Duration& timeout);

    /*!
     * @param dmp
     * @param execution_duration
     * @param timeout
     * @return
     */
    bool executeJointDMPAndWait(dmp::DynamicMovementPrimitive& dmp,
                                ros::Duration& execution_duration,
                                ros::Duration& timeout);

    /*!
     * @param msg
     */
    void executionStatisticsCallback(const dmp_motion_controller::DMPExecutionStatisticsConstPtr& msg);

private:

    bool initialized_;
    ros::NodeHandle node_handle_;

    bool execution_stats_received_;

    std::string left_arm_ik_dmp_controller_name_;
    std::string right_arm_ik_dmp_controller_name_;
    std::string dual_arm_ik_dmp_controller_name_;

    std::string left_arm_joint_dmp_controller_name_;
    std::string right_arm_joint_dmp_controller_name_;
    std::string dual_arm_joint_dmp_controller_name_;

    boost::mutex execution_stats_received_mutex_;
    dmp_motion_controller::DMPExecutionStatistics execution_statistics_;

    task_manager::ControllerSwitcher controller_switcher_;

    ros::ServiceClient left_arm_execute_dmp_service_client_;
    ros::ServiceClient left_arm_write_trajectories_client_;

    ros::ServiceClient right_arm_execute_dmp_service_client_;
    ros::ServiceClient right_arm_write_trajectories_client_;

    ros::ServiceClient right_arm_execute_joint_dmp_service_client_;
    ros::ServiceClient left_arm_execute_joint_dmp_service_client_;

    ros::Subscriber left_arm_dmp_execution_statistics_subscriber_;
    ros::Subscriber right_arm_dmp_execution_statistics_subscriber_;

    ros::Subscriber right_arm_joint_dmp_execution_statistics_subscriber_;

    ros::ServiceClient dual_arm_joint_space_execute_dmp_service_client_;
    ros::ServiceClient dual_arm_cartesian_space_execute_dmp_service_client_;

    bool waitForExecutionStatistics(ros::Duration& timeout);

    bool createDirectory(const std::string& directory_name, const int iteration_number);

};

}

#endif /* DMP_EXECUTOR_H_ */
