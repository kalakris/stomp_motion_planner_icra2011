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

 \file    controller_switcher.cpp

 \author  Peter Pastor
 \date    Jun 22, 2010

 **********************************************************************/

#include <task_manager/controller_switcher.h>

#include <policy_improvement_utilities/assert.h>

namespace task_manager
{

ControllerSwitcher::ControllerSwitcher() :
    initialized_(false)
{
}

ControllerSwitcher::~ControllerSwitcher()
{

}

bool ControllerSwitcher::initialize()
{
    // TODO: think about making this a persistent service call including checking for errors.
    switch_controller_client_ = node_handle_.serviceClient<pr2_mechanism_msgs::SwitchController> ("/pr2_controller_manager/switch_controller");
    list_controller_client_ = node_handle_.serviceClient<pr2_mechanism_msgs::ListControllers> ("/pr2_controller_manager/list_controllers");

    initialized_ = true;
    return initialized_;
}

bool ControllerSwitcher::startController(const std::string controller_name)
{
    ROS_ASSERT(initialized_);

    if(isControllerRunning(controller_name))
    {
       ROS_DEBUG("Controller %s is already running, not starting.", controller_name.c_str());
       return true;
    }

    pr2_mechanism_msgs::SwitchController::Request switch_controller_request;
    pr2_mechanism_msgs::SwitchController::Response switch_controller_response;

    switch_controller_request.start_controllers.clear();
    switch_controller_request.start_controllers.push_back(controller_name);
    switch_controller_request.stop_controllers.clear();
    switch_controller_request.strictness = switch_controller_request.STRICT;

    ROS_ASSERT_FUNC(switch_controller_client_.call(switch_controller_request, switch_controller_response));
    ROS_ASSERT(switch_controller_response.ok);
    return true;
}

bool ControllerSwitcher::stopController(const std::string controller_name)
{
    ROS_ASSERT(initialized_);
    if(!isControllerRunning(controller_name))
    {
       ROS_DEBUG("Controller %s is not running, not stopping.", controller_name.c_str());
       return true;
    }

    pr2_mechanism_msgs::SwitchController::Request switch_controller_request;
    pr2_mechanism_msgs::SwitchController::Response switch_controller_response;

    switch_controller_request.start_controllers.clear();
    switch_controller_request.stop_controllers.clear();
    switch_controller_request.stop_controllers.push_back(controller_name);
    switch_controller_request.strictness = switch_controller_request.STRICT;

    ROS_ASSERT_FUNC(switch_controller_client_.call(switch_controller_request, switch_controller_response));
    ROS_ASSERT(switch_controller_response.ok);
    return true;
}

bool ControllerSwitcher::switchLeftArmController(const std::string& new_left_arm_controller_name)
{
    return switchController(new_left_arm_controller_name, LEFT);
}
bool ControllerSwitcher::switchRightArmController(const std::string& new_right_arm_controller_name)
{
    return switchController(new_right_arm_controller_name, RIGHT);
}

bool ControllerSwitcher::switchController(const std::string& new_controller_name, Arm arm)
{

    ROS_ASSERT(initialized_);

    pr2_mechanism_msgs::SwitchController::Request switch_controller_request;
    pr2_mechanism_msgs::SwitchController::Response switch_controller_response;

    switch_controller_request.start_controllers.clear();
    switch_controller_request.start_controllers.push_back(new_controller_name);
    switch_controller_request.stop_controllers.clear();
    switch_controller_request.strictness = switch_controller_request.STRICT;

    std::vector<std::string> left_arm_controllers;
    std::vector<std::string> right_arm_controllers;
    if (!getRunningArmController(left_arm_controllers, right_arm_controllers))
    {
        ROS_ERROR("Could not get running arm controllers.");
        return false;
    }

    switch (arm)
    {
        case LEFT:
            for (std::vector<std::string>::iterator vi = left_arm_controllers.begin(); vi != left_arm_controllers.end(); vi++)
            {
                if (!vi->compare(new_controller_name))
                {
                    ROS_DEBUG("Controller %s is already running, not switching controller.", new_controller_name.c_str());
                    return true;
                }
                else
                {
                    switch_controller_request.stop_controllers.push_back(*vi);
                }
            }
            break;

        case RIGHT:
            for (std::vector<std::string>::iterator vi = right_arm_controllers.begin(); vi != right_arm_controllers.end(); vi++)
            {
                if (!vi->compare(new_controller_name))
                {
                    ROS_DEBUG("Controller %s is already running, not switching controller.", new_controller_name.c_str());
                    return true;
                }
                else
                {
                    switch_controller_request.stop_controllers.push_back(*vi);
                }
            }
            break;

        default:
            ROS_ERROR("Arm type (%i) is unknown.", (int)arm);
            return false;
    }

//    for (std::vector<std::string>::iterator vi = switch_controller_request.stop_controllers.begin(); vi != switch_controller_request.stop_controllers.end(); vi++)
//    {
//        ROS_INFO("Switching from %s.", vi->c_str());
//    }
//    for (std::vector<std::string>::iterator vi = switch_controller_request.start_controllers.begin(); vi != switch_controller_request.start_controllers.end(); vi++)
//    {
//        ROS_INFO("To %s.", vi->c_str());
//    }

    if (!switch_controller_client_.call(switch_controller_request, switch_controller_response))
    {
        ROS_ERROR("Switch controller service call failed.");
        return false;
    }

    if (!switch_controller_response.ok)
    {
        ROS_ERROR("Could not switch controller.");
        return false;
    }

    ROS_DEBUG("Successfully switched to %s.", new_controller_name.c_str());
    return true;
}

bool ControllerSwitcher::isControllerRunning(const std::string& controller_name)
{

    pr2_mechanism_msgs::ListControllers::Request list_controller_request;
    pr2_mechanism_msgs::ListControllers::Response list_controller_response;
    ROS_ASSERT_FUNC(listControllers(list_controller_request, list_controller_response));

    for (int i = 0; i < (int)list_controller_response.controllers.size(); i++)
    {
        if (!list_controller_response.controllers[i].compare(controller_name))
        {
            if (!list_controller_response.state[i].compare(std::string("running")))
            {
                ROS_DEBUG("Contorller %s is %s.", controller_name.c_str(), list_controller_response.state[i].c_str());
                return true;
            }
            else
            {
                ROS_DEBUG("Controller %s is %s.", controller_name.c_str(), list_controller_response.state[i].c_str());
                return false;
            }
        }
    }

    ROS_ERROR("Controller %s is not loaded.", controller_name.c_str());
    return false;
}

bool ControllerSwitcher::listControllers(pr2_mechanism_msgs::ListControllers::Request& list_controller_request, pr2_mechanism_msgs::ListControllers::Response& list_controller_response)
{
    list_controller_response.controllers.clear();
    list_controller_response.state.clear();

    int num_attempts = 0;
    int num_max_attempts = 5;
    for (num_attempts = 0; num_attempts < num_max_attempts; num_attempts++)
    {
        if (!list_controller_client_.call(list_controller_request, list_controller_response))
        {
            ROS_ERROR("List controller service call failed %i consecutive times.", num_attempts);
        }
        else
        {
            break;
        }
    }
    if (num_attempts == num_max_attempts)
    {
        ROS_ERROR("List controller service call failed %i consecutive times... and therefore too often", num_attempts);
        return false;
    }

    if (list_controller_response.controllers.size() != list_controller_response.state.size())
    {
        ROS_ERROR("Response invalid");
        return false;
    }
    return true;
}

bool ControllerSwitcher::getRunningArmController(std::vector<std::string>& left_arm_controllers, std::vector<std::string>& right_arm_controllers)
{

    left_arm_controllers.clear();
    right_arm_controllers.clear();

    pr2_mechanism_msgs::ListControllers::Request list_controller_request;
    pr2_mechanism_msgs::ListControllers::Response list_controller_response;
    ROS_ASSERT_FUNC(listControllers(list_controller_request, list_controller_response));

    for (int i = 0; i < (int)list_controller_response.controllers.size(); i++)
    {
        if (!list_controller_response.controllers[i].compare(0, 5, "l_arm"))
        {
            if (!list_controller_response.state[i].compare(std::string("running")))
            {
                left_arm_controllers.push_back(list_controller_response.controllers[i]);
                ROS_DEBUG(">> left arm: %s", list_controller_response.controllers[i].c_str());
            }
        }
        if (!list_controller_response.controllers[i].compare(0, 5, "r_arm"))
        {
            if (!list_controller_response.state[i].compare(std::string("running")))
            {
                right_arm_controllers.push_back(list_controller_response.controllers[i]);
                ROS_DEBUG(">> right arm: %s", list_controller_response.controllers[i].c_str());
            }
        }
    }
    return true;
}

}
