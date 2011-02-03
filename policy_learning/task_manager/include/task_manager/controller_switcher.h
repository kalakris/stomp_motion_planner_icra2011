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

 \file    controller_switcher.h

 \author  Peter Pastor
 \date    Jun 22, 2010

 **********************************************************************/

#ifndef CONTROLLER_SWITCHER_H_
#define CONTROLLER_SWITCHER_H_

// system includes
#include <string>

// ros includes
#include <ros/ros.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>

// local includes

namespace task_manager
{

class ControllerSwitcher
{

public:

    /*!
     * @return
     */
    ControllerSwitcher();
    ~ControllerSwitcher();

    /*!
     * @return
     */
    bool initialize();

    /*!
     * @return
     */
    bool startController(const std::string controller_name);
    bool stopController(const std::string controller_name);

    /*!
     * @param new_left_arm_controller_name
     * @return
     */
    bool switchLeftArmController(const std::string& new_left_arm_controller_name);

    /*!
     * @param new_right_arm_controller_name
     * @return
     */
    bool switchRightArmController(const std::string& new_right_arm_controller_name);

private:

    bool initialized_;

    ros::ServiceClient switch_controller_client_;
    ros::ServiceClient list_controller_client_;

    enum Arm {
        LEFT=0, RIGHT
    };

    /*!
     * @param new_controller_name
     * @param arm
     * @return
     */
    bool switchController(const std::string& new_controller_name, Arm arm);

    /*!
     * @param left_arm_controllers
     * @param right_arm_controllers
     * @return
     */
    bool getRunningArmController(std::vector<std::string>& left_arm_controllers, std::vector<std::string>& right_arm_controllers);

    /*!
     * @param controller_name
     * @return
     */
    bool isControllerRunning(const std::string& controller_name);

    /*!
     * @param list_controller_request
     * @param list_controller_response
     * @return
     */
    bool listControllers(pr2_mechanism_msgs::ListControllers::Request& list_controller_request, pr2_mechanism_msgs::ListControllers::Response& list_controller_response);

    /*! Since we are using service calls we need to make sure that the node handle does not go out of scope.
     */
    ros::NodeHandle node_handle_;

};

}

#endif /* CONTROLLER_SWITCHER_H_ */
