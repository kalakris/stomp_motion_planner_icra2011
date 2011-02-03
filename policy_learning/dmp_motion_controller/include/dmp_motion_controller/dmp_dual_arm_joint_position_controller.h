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

  \file    dmp_dual_arm_ik_controller.h

  \author  Peter Pastor
  \date    Jul 26, 2010

**********************************************************************/

#ifndef DMP_DUAL_ARM_JOINT_POSITION_CONTROLLER_H_
#define DMP_DUAL_ARM_JOINT_POSITION_CONTROLLER_H_

// system includes
#include <pthread.h>

// ros includes
#include <ros/ros.h>
#include <rosrt/rosrt.h>
#include <pr2_controller_interface/controller.h>

#include <boost/circular_buffer.hpp>

// local includes
#include <dmp_motion_controller/dmp_controller_common.h>
#include <dmp_motion_controller/dmp_joint_position_controller.h>
#include <dmp_motion_controller/AddToDualArmExecuteDMPQueue.h>

namespace dmp_controller
{

class DMPDualArmJointPositionController : public pr2_controller_interface::Controller
{

    typedef DMPJointPositionController ChildController;

public:

    /*!
     * @return
     */
    DMPDualArmJointPositionController();
    virtual ~DMPDualArmJointPositionController();

    /*!
     * @param robot_state
     * @param node_handle
     * @return
     */
    bool init(pr2_mechanism_model::RobotState* robot_state, ros::NodeHandle& node_handle);

    /*!
     */
    void starting();

    /*!
     */
    void update();

    /*!
     */
    void stopping();

    /*!
     * @return
     */
    bool initXml(pr2_mechanism_model::RobotState* robot, TiXmlElement* config);

    /*!
     * @param request
     * @param response
     * @return
     */
    bool addToExecuteDMPQueue(dmp_motion_controller::AddToDualArmExecuteDMPQueue::Request& request, dmp_motion_controller::AddToDualArmExecuteDMPQueue::Response& response);

private:

    bool initialized_;

    ros::NodeHandle node_handle_;

    ChildController* left_joint_controller_;
    ChildController* right_joint_controller_;

    ros::ServiceServer execute_dual_arm_dmp_service_server_;

    /*!
     * @return
     */
    bool getChildController();

    /*!
     */
    struct DualArmDMPStruct
    {
        std::vector<DMPStruct> left_arm_dmp_structs;
        std::vector<DMPStruct> right_arm_dmp_structs;
    };
    boost::circular_buffer<DualArmDMPStruct> dual_arm_dmp_buffer_;

    /*!
     * @return
     */
    bool tryGetLock();
    void freeLock();

    /*! \brief mutex lock for accessing dmp circular buffer
     */
    pthread_mutex_t dmp_cmd_lock_;

};

// REAL-TIME REQUIREMENTS
inline bool DMPDualArmJointPositionController::tryGetLock()
{
    if (pthread_mutex_trylock(&dmp_cmd_lock_) == 0) // never forget the inverse check again...
    {
        return true;
    }
    return false;
}
// REAL-TIME REQUIREMENTS
inline void DMPDualArmJointPositionController::freeLock()
{
    pthread_mutex_unlock(&dmp_cmd_lock_);
}

}

#endif /* DMP_DUAL_ARM_JOINT_POSITION_CONTROLLER_H_ */
