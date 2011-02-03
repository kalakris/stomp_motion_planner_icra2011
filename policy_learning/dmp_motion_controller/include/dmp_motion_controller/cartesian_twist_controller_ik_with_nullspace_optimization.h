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

#ifndef CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H_
#define CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H_

// system includes
#include <vector>
#include <boost/scoped_ptr.hpp>

// ros includes
#include <ros/ros.h>

#include <rosrt/rosrt.h>

#include <pr2_controller_interface/controller.h>
#include <control_toolbox/pid.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "kdl/chainfksolverpos_recursive.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Core>

#include <geometry_msgs/Twist.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/chain.h>
#include <tf/transform_datatypes.h>

#include <filters/transfer_function.h>

// local includes
#include <dmp_motion_controller/joint_position_controller.h>

#include <dmp_motion_controller/JointPositionVelocityStamped.h>
#include <dmp_motion_controller/PoseTwistStamped.h>
#include <dmp_motion_controller/NullspaceTermStamped.h>
#include <dmp_motion_controller/ControllerStatus.h>

namespace dmp_controller
{

class CartesianTwistControllerIkWithNullspaceOptimization : public pr2_controller_interface::Controller
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     */
    CartesianTwistControllerIkWithNullspaceOptimization();
    ~CartesianTwistControllerIkWithNullspaceOptimization();

    /*!
     * @param robot_state
     * @param node_handle
     * @return
     */
    bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &node_handle);

    /*!
     * @return
     */
    void starting();

    /*!
     */
    void update();

    /*! input of the cartesian twist controller
     */
    KDL::Frame kdl_pose_desired_;
    KDL::Twist kdl_twist_desired_;

    /*! output
     */
    KDL::Frame kdl_pose_measured_;
    KDL::Twist kdl_twist_measured_;

    KDL::Frame kdl_real_pose_measured_;

    /*! only for debugging...
     */
    KDL::JntArray kdl_current_joint_positions_;
    KDL::JntArrayVel kdl_current_joint_velocities_;

    KDL::JntArray kdl_desired_joint_positions_;

    /*! input to the controller for the nullspace optimization part
     */
    Eigen::VectorXd rest_posture_joint_configuration_;

private:

    /*!
     * @return
     */
    bool initMechanismChain();

    /*!
     * @return
     */
    bool readParameters();

    /*!
     * @return
     */
    bool initCartesianPidControllers();
    /*!
     * @return
     */
    bool initNullspacePidControllers();

    /*!
     * @return
     */
    bool initRTPublisher();

    /*! robot description
     */
    pr2_mechanism_model::RobotState *robot_state_;
    pr2_mechanism_model::Chain mechanism_chain_;

    /*!
     */
    ros::NodeHandle node_handle_;

    /*!
     */
    KDL::Twist kdl_twist_error_;
    KDL::Chain kdl_chain_;
    KDL::Jacobian kdl_chain_jacobian_;

    /*!
     */
    boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

    /*!
     */
    double damping_;

    /*!
     */
    ros::Time last_time_;
    ros::Duration dt_;

    /*!
     */
    double ff_trans_;
    double ff_rot_;

    /*! feedback pid controllers (translation and rotation)
     */
    std::vector<control_toolbox::Pid> cartesian_fb_pid_controllers_;

    /*! feedback pid controllers (nullspace)
     */
    std::vector<control_toolbox::Pid> nullspace_fb_pid_controllers_;

    /*!
     */
    std::vector<JointPositionController> joint_position_controllers_;

    /*!
     */
    int num_joints_;

    Eigen::VectorXd eigen_desired_cartesian_velocities_;

    Eigen::VectorXd eigen_desired_joint_positions_;
    Eigen::VectorXd eigen_desired_joint_velocities_;

    Eigen::MatrixXd eigen_chain_jacobian_;

    Eigen::MatrixXd eigen_jac_times_jac_transpose_;
    Eigen::MatrixXd eigen_jjt_inverse_;
    Eigen::MatrixXd eigen_jac_pseudo_inverse_;
    Eigen::MatrixXd eigen_identity_;

    Eigen::VectorXd eigen_nullspace_term_;
    Eigen::MatrixXd eigen_nullspace_projector_;
    Eigen::VectorXd eigen_nullspace_error_;

    /*!
     */
    filters::MultiChannelTransferFunctionFilter<double> pose_filter_;
    std::vector<double> pose_unfiltered_data_;
    std::vector<double> pose_filtered_data_;

    /*!
     */
    int publisher_rate_;
    int publisher_counter_;
    int publisher_buffer_size_;
    int header_sequence_number_;

    /*!
     */
    boost::shared_ptr<rosrt::Publisher<dmp_motion_controller::PoseTwistStamped> > pose_twist_desired_publisher_;
    boost::shared_ptr<rosrt::Publisher<dmp_motion_controller::PoseTwistStamped> > pose_twist_actual_publisher_;
    boost::shared_ptr<rosrt::Publisher<dmp_motion_controller::NullspaceTermStamped> > nullspace_term_publisher_;
    boost::shared_ptr<rosrt::Publisher<dmp_motion_controller::ControllerStatus> > controller_status_publisher_;

    /*!
     */
    void publish();

};

}

#endif /* CARTESIAN_TWIST_CONTROLLER_IK_WITH_NULLSPACE_OPTIMIZATION_H_ */
