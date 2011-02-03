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

#ifndef JOINT_VELOCITY_FILTERED_CONTROLLER_H_
#define JOINT_VELOCITY_FILTERED_CONTROLLER_H_

// ros includes
#include <ros/ros.h>
#include <ros/rt.h>

#include <boost/shared_ptr.hpp>

#include <pr2_controller_interface/controller.h>

#include <control_toolbox/pid.h>
#include <control_toolbox/pid_gains_setter.h>

#include <pr2_controllers_msgs/JointControllerState.h>

#include <filters/transfer_function.h>

namespace dmp_controller
{

class JointVelocityFilteredController : public pr2_controller_interface::Controller
{
public:

    /*!
     */
    JointVelocityFilteredController();
    ~JointVelocityFilteredController();

    /*!
     */
    bool init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle& node_handle);

    void setCommand(const double cmd);

    /*!
     * @return
     */
    double getCommand() const;

    /*!
     * @return
     */
    double getFilteredVelocity() const;

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */

    void starting();

    /*!
     */
    void update();

    /*!
     */
    void stopping();

    void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
    void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min);

    std::string getJointName();
    pr2_mechanism_model::JointState* joint_state_; /**< Joint we're controlling. */
    ros::Duration dt_;

    double command_; /**< Last commanded position. */

private:

    /*!
     */
    ros::NodeHandle node_handle_;
    pr2_mechanism_model::RobotState *robot_; /**< Pointer to robot structure. */
    control_toolbox::Pid pid_controller_; /**< Internal PID controller. */

    /*!
     */
    ros::Time last_time_; /**< Last time stamp of update. */

    /*!
     */
    int publisher_rate_;
    int publisher_buffer_size_;
    int publisher_counter_;

    /*!
     */
    double error_;

    /*!
     */
    boost::shared_ptr<rosrt::Publisher<pr2_controllers_msgs::JointControllerState> > controller_state_publisher_;

    /*!
     */
    filters::MultiChannelTransferFunctionFilter<double> filter_velocity_;
    std::vector<double> velocity_;

    /*!
     */
    double filtered_velocity_;

    /*!
     */
    void publish();

};

inline double JointVelocityFilteredController::getFilteredVelocity() const
{
    return filtered_velocity_;
}

} // namespace

#endif /* JOINT_VELOCITY_FILTERED_CONTROLLER_H_ */
