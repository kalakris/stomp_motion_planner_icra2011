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

  \file    dmp_controller_common.cpp

  \author  Peter Pastor
  \date    Jul 6, 2010

**********************************************************************/

// system includes

// ros includes
#include <dmp_motion_generation/constants.h>
#include <dmp_motion_generation/math_helper.h>

// local includes
#include <dmp_motion_controller/dmp_controller_common.h>

namespace dmp_controller
{

bool DMPControllerCommon::initJointPositionController(ros::NodeHandle node_handle, pr2_mechanism_model::RobotState* robot_state, std::vector<JointPositionController>& joint_position_controllers)
{
    joint_position_controllers.clear();
    std::string joint_names_string;
    node_handle.param<std::string> ("joint_names", joint_names_string, "");
    std::stringstream ss(joint_names_string);
    std::string joint_name;

    while (ss >> joint_name)
    {
        ros::NodeHandle joint_node_handle(node_handle, joint_name);
        // ROS_INFO("joint: %s", joint_name.c_str());
        dmp_controller::JointPositionController joint_controller;
        if (!joint_controller.init(robot_state, joint_node_handle))
        {
            ROS_ERROR("Could not initialize joint controller for joint %s.", joint_name.c_str());
            return false;
        }
        joint_position_controllers.push_back(joint_controller);
    }

    return (static_cast<int>(joint_position_controllers.size()) == dmp::N_JOINTS);
}

bool DMPControllerCommon::setDMPStructFromRequest(ros::NodeHandle& node_handle, dmp_motion_controller::AddToExecuteDMPQueue::Request& request, dmp_motion_controller::AddToExecuteDMPQueue::Response& response,
                                                  const std::vector<int>& num_dofs, const std::string variable_names_key_word, std::vector<DMPStruct>& dmp_structs)
{

    if(request.dmps.size() != request.execution_durations.size())
    {
        ROS_ERROR("There are %i dmps but only %i execution durations.", (int)request.dmps.size(), (int)request.execution_durations.size());
        response.info.assign(std::string("There are ") + dmp::MathHelper::getString((int)request.dmps.size()) + std::string(" dmps but only ")
         + dmp::MathHelper::getString(request.execution_durations.size()) + std::string(" execution durations."));
        response.return_code = dmp_motion_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
        return false;
    }

    if(request.dmps.size() != request.types.size())
    {
        ROS_ERROR("There are %i dmps but only %i types.", (int)request.dmps.size(), (int)request.types.size());
        response.info.assign(std::string("There are ") + dmp::MathHelper::getString((int)request.dmps.size()) + std::string(" dmps but only ")
         + dmp::MathHelper::getString(request.execution_durations.size()) + std::string(" types."));
        response.return_code = dmp_motion_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
        return false;
    }

    dmp_structs.clear();
    for (int i=0; i<static_cast<int>(request.dmps.size()); ++i)
    {
        DMPStruct dmp_struct;
        dmp_struct.dmp.reset(new dmp::DynamicMovementPrimitive(node_handle));

        ROS_INFO("Adding DMP with id %i and execution duration %f.", request.dmps[i].dmp_id, request.execution_durations[i]);

        if (!dmp_struct.dmp->initFromMessage(request.dmps[i]))
        {
            ROS_ERROR("Could not set DMP with id %i.", request.dmps[i].dmp_id);
            response.info.assign(std::string("Could not set DMP with id ") + dmp::MathHelper::getString(request.dmps[i].dmp_id) + std::string("."));
            response.return_code = dmp_motion_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
            return false;
        }
        if (!dmp_struct.dmp->isInitialized())
        {
            ROS_ERROR("DMP with id %i is not initialized.", request.dmps[i].dmp_id);
            response.info.assign(std::string("DMP with id ") + dmp::MathHelper::getString(request.dmps[i].dmp_id) + std::string(" is not initialized."));
            response.return_code = dmp_motion_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
            return false;
        }
        if (!dmp_struct.dmp->isSetup())
        {
            ROS_ERROR("DMP with id %i is not setup.", request.dmps[i].dmp_id);
            response.info.assign(std::string("DMP with id ") + dmp::MathHelper::getString(request.dmps[i].dmp_id) + std::string(" is not setup."));
            response.return_code = dmp_motion_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
            return false;
        }

        dmp_struct.execution_duration = request.execution_durations[i];
        dmp_struct.type = static_cast<pr2_tasks_transforms::TaskTransforms::TransformType>(request.types[i]);

        ROS_ASSERT(dmp_struct.type >= 0);
        ROS_ASSERT(dmp_struct.type < static_cast<int>(num_dofs.size()));

        dmp_struct.debug_object_actual.reset(new dmp::DMPDebug(node_handle));
        if (dmp_struct.debug_object_actual->initialize(request.dmps[i].dmp_id, variable_names_key_word, num_dofs[dmp_struct.type], dmp::POS_VEL_ACC, 0, CONTROLLER_SAMPLING_FREQUENCY))
        {
            dmp_struct.debug_object_actual->setTrunkFileName(std::string("item_") + dmp::MathHelper::getString(request.dmps[i].dmp_id) + std::string("_actual_")
                    + dmp::MathHelper::getString(static_cast<int> (dmp_structs.size())));
        }
        else
        {
            ROS_ERROR("Could not initialize DMP debug object to record the actual trajectory.");
        }

        dmp_struct.debug_object_desired.reset(new dmp::DMPDebug(node_handle));
        if (dmp_struct.debug_object_desired->initialize(request.dmps[i].dmp_id, variable_names_key_word, num_dofs[dmp_struct.type], dmp::POS_VEL_ACC, 0, CONTROLLER_SAMPLING_FREQUENCY))
        {
            dmp_struct.debug_object_desired->setTrunkFileName(std::string("item_") + dmp::MathHelper::getString(request.dmps[i].dmp_id) + std::string("_desired_")
                    + dmp::MathHelper::getString(static_cast<int> (dmp_structs.size())));
        }
        else
        {
            ROS_ERROR("Could not initialize DMP debug object to record the desired trajectory.");
        }

        dmp_structs.push_back(dmp_struct);
    }
    return true;
}

}
