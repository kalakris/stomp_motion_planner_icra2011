/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/** \author Peter Pastor */

// system includes

// ros includes
#include <pluginlib/class_list_macros.h>
#include <dmp_motion_generation/constants.h>

#include <policy_improvement_utilities/assert.h>

// local includes
#include <dmp_motion_controller/dmp_joint_position_controller.h>
#include <dmp_motion_controller/dmp_controller_common.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

PLUGINLIB_DECLARE_CLASS(dmp_motion_controller, DMPJointPositionController, dmp_controller::DMPJointPositionController, pr2_controller_interface::Controller)

namespace dmp_controller
{

  DMPJointPositionController::DMPJointPositionController() // : current_desired_joint_cmd_is_set_(false)
{
}

DMPJointPositionController::~DMPJointPositionController()
{
    // TODO: check for objects that are not boost shared pointer
}

bool DMPJointPositionController::init(pr2_mechanism_model::RobotState* robot_state, ros::NodeHandle& node_handle)
{

    ROS_ASSERT_FUNC(DMPControllerCommon::initJointPositionController(node_handle, robot_state, joint_position_controllers_));
    num_joints_ = static_cast<int>(joint_position_controllers_.size());

    cmd_.positions.resize(num_joints_);
    cmd_.velocities.resize(num_joints_);
    cmd_.accelerations.resize(num_joints_);

    current_desired_joint_cmd_.positions.resize(num_joints_);
    current_desired_joint_cmd_.velocities.resize(num_joints_);
    current_desired_joint_cmd_.accelerations.resize(num_joints_);

    start_pos_point_ = VectorXd::Zero(num_joints_);
    current_pos_point_ = VectorXd::Zero(num_joints_);

    trajectory_pos_vel_acc_point_ = VectorXd::Zero(num_joints_ * dmp::POS_VEL_ACC);

    debug_trajectory_point_ = VectorXd::Zero(num_joints_ * dmp::POS_VEL_ACC);

    std::vector<int> num_dofs;
    num_dofs.push_back(num_joints_);
    if (!dmp_controller_.initialize(node_handle, num_dofs, DMPController::JOINT))
    {
        ROS_ERROR("Could not initialize base.");
        initialized_ = false;
        return initialized_;
    }

    std::string debug_joint_name;
    if(node_handle.getNamespace().compare(0,6,std::string("/l_arm")) == 0)
    {
        debug_joint_name.assign("debug_pr2_l_arm_joint_names");
    }
    else if(node_handle.getNamespace().compare(0,6,std::string("/r_arm")) == 0)
    {
        debug_joint_name.assign("debug_pr2_r_arm_joint_names");
    }
    else
    {
        ROS_ERROR("Invalid namespace: %s", node_handle.getNamespace().c_str());
    }

    dmp_debug_.reset(new dmp::DMPDebug(node_handle));
    if (!dmp_debug_->initialize(0, debug_joint_name, num_joints_, dmp::POS_VEL_ACC, dmp::POS_VEL_ACC, CONTROLLER_SAMPLING_FREQUENCY))
    {
        ROS_WARN("Initialization of the debug object is skipped.");
    }

    return (initialized_ = true);
}

bool DMPJointPositionController::initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config)
{
    ros::NodeHandle node_handle(config->Attribute("name"));
    return init(robot, node_handle);
}

void DMPJointPositionController::starting()
{
    for (int i = 0; i < static_cast<int>(joint_position_controllers_.size()); ++i)
    {
        joint_position_controllers_[i].starting();
    }
    //    current_desired_joint_cmd_is_set_ = false;
    skip_first_ = false;
}

void DMPJointPositionController::update()
{

    if (dmp_controller_.tryGetLock())
    {
        bool movement_finished = true;
        current_desired_joint_cmd_ = getCommand(movement_finished);
        // current_desired_joint_cmd_is_set_ = true;
        setCommand(current_desired_joint_cmd_);
        if (movement_finished)
        {
            dmp_controller_.setEndTime(ros::Time::now());
            dmp_controller_.incrementDMPQueueIndex();
        }
        dmp_controller_.freeLock();

        addDebugInformation();
    }
    else
    {
//         if (!current_desired_joint_cmd_is_set_)
//         {
// //             for (int i = 0; i < num_joints_; i++)
// //             {
// //                 current_desired_joint_cmd_.positions[i] = joint_position_controllers_[i].getJointPosition();
// //                 current_desired_joint_cmd_.velocities[i] = 0.0;
// //             }
//              current_desired_joint_cmd_is_set_ = true;
//         }
//         setCommand(current_desired_joint_cmd_);

        dmp_controller_.freeLock();
        dmp_controller_.incrementSkippedCycles();
    }

    for (int i = 0; i < static_cast<int>(joint_position_controllers_.size()); i++)
    {
        joint_position_controllers_[i].update();
    }

}

void DMPJointPositionController::addDebugInformation()
{
    for (int i = 0; i < num_joints_ * dmp::POS_VEL_ACC; i++)
    {
        debug_trajectory_point_[i] = 0.0;
    }

    if (skip_first_)
    {
        debug_trajectory_point_(0) = static_cast<double> (joint_position_controllers_[0].dt_.nsec);
        debug_trajectory_point_(1) = static_cast<double> (joint_position_controllers_[0].dt_.sec);
        int extra_index = 3;
        for (int i = extra_index; i < extra_index + num_joints_; ++i)
        {
            debug_trajectory_point_(i) = joint_position_controllers_[i - extra_index].getCommandedEffort();
        }
        for (int i = extra_index + num_joints_; i < (2 * num_joints_) + extra_index; i++)
        {
            // TODO: fix this
            debug_trajectory_point_(i) = 0;
            // trajectory_point[i] = joint_states_[i - (extra_index + num_joints_)]->dt_.nsec;
        }
    }
    else
    {
        skip_first_ = true;
    }

    if (!dmp_debug_->add(debug_trajectory_point_))
    {

    }
}

trajectory_msgs::JointTrajectoryPoint DMPJointPositionController::getCommand(bool &movement_finished)
{

    for (int i = 0; i < num_joints_; i++)
    {
        current_pos_point_(i) = joint_position_controllers_[i].getJointPosition();
        trajectory_pos_vel_acc_point_(i * dmp::POS_VEL_ACC + 0) = joint_position_controllers_[i].getJointPosition();
        trajectory_pos_vel_acc_point_(i * dmp::POS_VEL_ACC + 1) = joint_position_controllers_[i].getJointVelocity();
        trajectory_pos_vel_acc_point_(i * dmp::POS_VEL_ACC + 2) = 0.0;
    }
    dmp_controller_.addDebugActual(trajectory_pos_vel_acc_point_);

    if (!dmp_controller_.getCurrentDMP()->isStartSet())
    {
        for (int i = 0; i < num_joints_; i++)
        {
            start_pos_point_(i) = joint_position_controllers_[i].getJointPosition();
        }
        if(!dmp_controller_.getCurrentDMP()->changeStart(start_pos_point_))
        {
            ROS_ERROR("Could not change start of the DMP.");
        }
    }

    if (dmp_controller_.firstDMPControlCycle())
    {
        dmp_controller_.setStartTime(ros::Time::now());
    }

    double execution_duration = dmp_controller_.getCurrentExecutionDuration();
    int num_samples = execution_duration * CONTROLLER_SAMPLING_FREQUENCY;
    if (!dmp_controller_.getCurrentDMP()->propagateStep(trajectory_pos_vel_acc_point_, movement_finished, execution_duration, num_samples))
    {

    }
    dmp_controller_.addDebugDesired(trajectory_pos_vel_acc_point_);

    for (int i = 0; i < num_joints_; i++)
    {
        cmd_.positions[i] = trajectory_pos_vel_acc_point_(i * dmp::POS_VEL_ACC + 0);
        cmd_.velocities[i] = trajectory_pos_vel_acc_point_(i * dmp::POS_VEL_ACC + 1);
        cmd_.accelerations[i] = trajectory_pos_vel_acc_point_(i * dmp::POS_VEL_ACC + 2);
    }

    return cmd_;
}

void DMPJointPositionController::setCommand(const trajectory_msgs::JointTrajectoryPoint &cmd)
{
    for (int i = 0; i < num_joints_; i++)
    {
        joint_position_controllers_[i].setCommand(cmd_.positions[i]);
    }
}

}
