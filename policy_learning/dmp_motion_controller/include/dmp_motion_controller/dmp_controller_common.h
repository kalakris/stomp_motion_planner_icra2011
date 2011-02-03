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

  \file    dmp_controller_common.h

  \author  Peter Pastor
  \date    Jul 6, 2010

**********************************************************************/

#ifndef DMP_CONTROLLER_COMMON_H_
#define DMP_CONTROLLER_COMMON_H_

// system includes

// ros includes
#include <ros/ros.h>
#include <pr2_mechanism_model/robot.h>
#include <dmp_motion_generation/dynamic_movement_primitive.h>
#include <dmp_motion_generation/dmp_debug.h>
#include <geometry_msgs/Transform.h>

#include <pr2_tasks_transforms/task_transforms.h>

// local includes
#include <dmp_motion_controller/joint_position_controller.h>
#include <dmp_motion_controller/AddToExecuteDMPQueue.h>

namespace dmp_controller
{

static const double CONTROLLER_SAMPLING_FREQUENCY = 1000.0;

/*! this variables need to be protected by a mutex
 */
struct DMPStruct
{
    boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp;
    double execution_duration;
    pr2_tasks_transforms::TaskTransforms::TransformType type;
    boost::shared_ptr<dmp::DMPDebug> debug_object_desired;
    boost::shared_ptr<dmp::DMPDebug> debug_object_actual;
};

class DMPControllerCommon
{

public:

    static bool initJointPositionController(ros::NodeHandle node_handle, pr2_mechanism_model::RobotState *robot, std::vector<JointPositionController>& joint_position_controllers);

    static bool setDMPStructFromRequest(ros::NodeHandle& node_handle, dmp_motion_controller::AddToExecuteDMPQueue::Request& request, dmp_motion_controller::AddToExecuteDMPQueue::Response& response,
                                        const std::vector<int>& num_dofs, const std::string variable_names_key_word, std::vector<DMPStruct>& dmp_structs);

private:

};

}

#endif /* DMP_CONTROLLER_COMMON_H_ */
