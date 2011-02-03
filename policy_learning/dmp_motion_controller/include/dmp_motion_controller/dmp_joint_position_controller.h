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

#ifndef DMP_JOINT_POSITION_CONTROLLER_H_
#define DMP_JOINT_POSITION_CONTROLLER_H_

// system include
#include <boost/shared_ptr.hpp>

// ros include
#include <ros/node_handle.h>
#include <angles/angles.h>
#include <control_toolbox/pid.h>

#include <pr2_controller_interface/controller.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <dmp_motion_generation/dynamic_movement_primitive.h>
#include <dmp_motion_generation/dmp_debug.h>

#include <pr2_controllers_msgs/JointTrajectoryActionGoal.h>

#include <Eigen/Eigen>

// local include
#include <dmp_motion_controller/dmp_controller.h>
#include <dmp_motion_controller/joint_position_controller.h>

namespace dmp_controller
{

class DMPJointPositionController : public pr2_controller_interface::Controller
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! constructor
     *
     */
    DMPJointPositionController();

    /*! descructor
     *
     */
    ~DMPJointPositionController();

    /*!
     * @param robot_state
     * @param n
     * @return
     */
    bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &node);

    /*!
     * @return
     */
    void starting();

    /*!
     *
     */
    void update();

    /*!
     *
     * @param robot
     * @param config
     * @return
     */
    bool initXml(pr2_mechanism_model::RobotState *robot, TiXmlElement *config);

    /*!
     * @return
     */
    bool isQueueEmpty();

    /*!
     * @param dmp_structs
     * @return
     */
    bool addToQueue(std::vector<DMPStruct> dmp_structs);

    /*!
     * @return
     */
    std::string getVariableNamesKeyWord() const;
    std::vector<int> getNumJoints() const;

private:
    /*!
     */
    bool initialized_;

    /*!
     */
    int num_joints_;

    /*!
     */
    DMPController dmp_controller_;

    /*!
     * @param cmd
     */
    void setCommand(const trajectory_msgs::JointTrajectoryPoint &cmd);
    /*!
    * @param movement_finished
    * @return
    */
    trajectory_msgs::JointTrajectoryPoint getCommand(bool &movement_finished);

    /*!
     */
    void addDebugInformation();

    /*! commands that are set by the dmp and passed to the underlying PID controllers
     */
    trajectory_msgs::JointTrajectoryPoint cmd_;

    /*!
     */
    // bool current_desired_joint_cmd_is_set_;
    trajectory_msgs::JointTrajectoryPoint current_desired_joint_cmd_;

    /*! for debugging purpose
     */
    boost::shared_ptr<dmp::DMPDebug> dmp_debug_;
    bool skip_first_;

    /*!
     */
    Eigen::VectorXd start_pos_point_;
    Eigen::VectorXd trajectory_pos_vel_acc_point_;
    Eigen::VectorXd current_pos_point_;
    Eigen::VectorXd debug_trajectory_point_;

    /*!
     */
    std::vector<JointPositionController> joint_position_controllers_;

};

inline std::string DMPJointPositionController::getVariableNamesKeyWord() const
{
    return dmp_controller_.getVariableNamesKeyWord();
}
inline std::vector<int> DMPJointPositionController::getNumJoints() const
{
    return dmp_controller_.getNumDofs();
}

inline bool DMPJointPositionController::isQueueEmpty()
{
    return dmp_controller_.isQueueEmpty();
}
inline bool DMPJointPositionController::addToQueue(std::vector<DMPStruct> dmp_structs)
{
    return dmp_controller_.addToQueue(dmp_structs);
}

}

#endif /* DMP_JOINT_POSITION_CONTROLLER_H_ */
