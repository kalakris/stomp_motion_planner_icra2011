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

  \file    DMPDualArmJointPositionController.cpp

  \author  Peter Pastor
  \date    Jul 26, 2010

**********************************************************************/

// system includes

// ros includes
#include <pluginlib/class_list_macros.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>

// local includes
#include <dmp_motion_controller/dmp_dual_arm_joint_position_controller.h>
#include <dmp_motion_controller/AddToExecuteDMPQueue.h>

PLUGINLIB_DECLARE_CLASS(dmp_motion_controller, DMPDualArmJointPositionController, dmp_controller::DMPDualArmJointPositionController, pr2_controller_interface::Controller)

namespace dmp_controller
{

const int SIZE_RUN_QUEUE = 100;

DMPDualArmJointPositionController::DMPDualArmJointPositionController() :
    initialized_(false)
{
    int mutex_return_code = pthread_mutex_init(&dmp_cmd_lock_, NULL);
    if (mutex_return_code != 0)
    {
        ROS_ERROR("Could not initialize mutex (%i : %s).", mutex_return_code, strerror(mutex_return_code));
    }
    mutex_return_code = pthread_mutex_unlock(&dmp_cmd_lock_);
    if (mutex_return_code != 0)
    {
        ROS_ERROR("Could not unlock mutex (%i : %s).", mutex_return_code, strerror(mutex_return_code));
    }
}

DMPDualArmJointPositionController::~DMPDualArmJointPositionController()
{
}

bool DMPDualArmJointPositionController::init(pr2_mechanism_model::RobotState* robot_state, ros::NodeHandle& node_handle)
{
    node_handle_ = node_handle;
    assert(robot_state);

    // get pointer to left and right ik controllers
    ROS_ASSERT_FUNC(getChildController());

    // pre-allocate memory
    dual_arm_dmp_buffer_.set_capacity(SIZE_RUN_QUEUE);

    // advertise services
    execute_dual_arm_dmp_service_server_ = node_handle_.advertiseService("add_to_execute_dmp_queue", &DMPDualArmJointPositionController::addToExecuteDMPQueue, this);

    return (initialized_ = true);
}

bool DMPDualArmJointPositionController::getChildController()
{
    // get a pointer to the ik controllers
    std::string left_arm_controller_name;
    std::string right_arm_controller_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("left_arm_controller"), left_arm_controller_name));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("right_arm_controller"), right_arm_controller_name));

    ros::Duration timeout(20);
    ros::Time start_time = ros::Time::now();
    bool found_controller = false;
    do
    {
      found_controller = getController<ChildController>(left_arm_controller_name, AFTER_ME, left_joint_controller_);
      if (!found_controller)
      {
        ros::Duration(0.5).sleep();
      }
    }
    while (!found_controller && ((start_time + timeout) > ros::Time::now()));
    if (!found_controller)
    {
      ROS_ERROR("Could not connect to left ik controller \"%s\"", left_arm_controller_name.c_str());
      return false;
    }

    start_time = ros::Time::now();
    found_controller = false;
    do
    {
      found_controller = getController<ChildController>(right_arm_controller_name, AFTER_ME, right_joint_controller_);
      if (!found_controller)
      {
        ros::Duration(0.5).sleep();
      }
    }
    while (!found_controller && ((start_time + timeout) > ros::Time::now()));
    if (!found_controller)
    {
      ROS_ERROR("Could not connect to right ik controller \"%s\"", right_arm_controller_name.c_str());
      return false;
    }
    return true;
}

void DMPDualArmJointPositionController::starting()
{
}

void DMPDualArmJointPositionController::update()
{
    if(tryGetLock())
    {
        if(!dual_arm_dmp_buffer_.empty())
        {
            if((right_joint_controller_->isQueueEmpty()) && (left_joint_controller_->isQueueEmpty()))
            {

                if(!right_joint_controller_->addToQueue(dual_arm_dmp_buffer_.front().right_arm_dmp_structs))
                {
                    ROS_ERROR("Could not add right arm dmp to execute queue.");
                }
                if(!left_joint_controller_->addToQueue(dual_arm_dmp_buffer_.front().left_arm_dmp_structs))
                {
                    ROS_ERROR("Could not add left arm dmp to execute queue.");
                }

                dual_arm_dmp_buffer_.pop_front();
            }
        }
        freeLock();
    }

}

void DMPDualArmJointPositionController::stopping()
{

}

bool DMPDualArmJointPositionController::initXml(pr2_mechanism_model::RobotState* robot, TiXmlElement* config)
{
    ros::NodeHandle node_handle(config->Attribute("name"));
    return init(robot, node_handle);
}

bool DMPDualArmJointPositionController::addToExecuteDMPQueue(dmp_motion_controller::AddToDualArmExecuteDMPQueue::Request& request, dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response& response)
{
    if ((request.left_arm_dmps.size() != request.left_arm_types.size())
            || (request.right_arm_dmps.size() != request.right_arm_types.size())
            || (request.execution_durations.size() != request.right_arm_types.size()))
    {
        response.info.assign("Number of dmps, transforms, and execution_durations does not match.");
        response.return_code = dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
        return true;
    }

    if (pthread_mutex_lock(&dmp_cmd_lock_) == 0)
    {
        for (int i=0; i<static_cast<int>(request.execution_durations.size()); ++i)
        {

            dmp_motion_controller::AddToExecuteDMPQueue::Request left_arm_request;
            dmp_motion_controller::AddToExecuteDMPQueue::Request right_arm_request;

            left_arm_request.dmps = request.left_arm_dmps;
            left_arm_request.execution_durations = request.execution_durations;
            left_arm_request.types = request.left_arm_types;
            right_arm_request.dmps = request.right_arm_dmps;
            right_arm_request.execution_durations = request.execution_durations;
            right_arm_request.types = request.right_arm_types;

            dmp_motion_controller::AddToExecuteDMPQueue::Response left_arm_response;
            dmp_motion_controller::AddToExecuteDMPQueue::Response right_arm_response;

            DualArmDMPStruct dual_arm_dmp_struct;
            ros::NodeHandle right_arm_node_handle("/r_arm_dmp_joint_position_controller");
            if(!DMPControllerCommon::setDMPStructFromRequest(right_arm_node_handle, right_arm_request, right_arm_response, right_joint_controller_->getNumJoints(),
                                                             right_joint_controller_->getVariableNamesKeyWord(), dual_arm_dmp_struct.right_arm_dmp_structs))
            {
                // setting dmp_structs failed...
                return true;
            }
            ros::NodeHandle left_arm_node_handle("/l_arm_dmp_joint_position_controller");
            if(!DMPControllerCommon::setDMPStructFromRequest(left_arm_node_handle, left_arm_request, left_arm_response, left_joint_controller_->getNumJoints(),
                                                             left_joint_controller_->getVariableNamesKeyWord(), dual_arm_dmp_struct.left_arm_dmp_structs))
            {
                // setting dmp_structs failed...
                return true;
            }

            dual_arm_dmp_buffer_.push_back(dual_arm_dmp_struct);
        }
        pthread_mutex_unlock(&dmp_cmd_lock_);
    }

    return true;
}

}
