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
#include <boost/thread.hpp>
#include <sstream>

// ros includes
#include <ros/callback_queue.h>
#include <pluginlib/class_list_macros.h>
#include <dmp_motion_generation/constants.h>
#include <angles/angles.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>

// local includes
#include <dmp_motion_controller/dmp_ik_controller.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

PLUGINLIB_DECLARE_CLASS(dmp_motion_controller, DMPIkController, dmp_controller::DMPIkController, pr2_controller_interface::Controller)

namespace dmp_controller
{

DMPIkController::DMPIkController() :
    initialized_(false), publishing_rate_(15), publishing_counter_(0), publisher_buffer_size_(0), visualization_line_counter_(0),
    visualization_line_rate_(5), visualization_line_max_points_(100), visualization_line_points_index_(0), keep_restposture_fixed_for_testing_(false), last_frame_set_(false)
{
}

DMPIkController::~DMPIkController()
{
}

bool DMPIkController::init(pr2_mechanism_model::RobotState* robot_state, ros::NodeHandle& node_handle)
{

    node_handle_ = node_handle;

    ROS_ASSERT_FUNC(readParameters());
    ROS_ASSERT_FUNC(initRTPublisher());

    ROS_ASSERT_FUNC(task_frame_transformer_.initialize(node_handle_, robot_state));
    std::vector<int> num_dofs;
    ROS_ASSERT_FUNC(task_frame_transformer_.getNumDimensions(num_dofs));
    for (int i = 0; i < static_cast<int> (num_dofs.size()); ++i)
    {
        VectorXd trajectory_point = VectorXd::Zero(num_dofs[i]);
        dmp_trajectory_points_.push_back(trajectory_point);
        VectorXd pos_vel_acc_trajectory_point = VectorXd::Zero(num_dofs[i] * dmp::POS_VEL_ACC);
        dmp_pos_vel_acc_trajectory_points_.push_back(pos_vel_acc_trajectory_point);
    }

    int num_dof = dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS;

    next_waypoint_.positions.resize(num_dof);
    next_waypoint_.velocities.resize(num_dof);
    next_waypoint_.accelerations.resize(num_dof);

    controller_trajectory_point_ = VectorXd::Zero(num_dof * dmp::POS_VEL_ACC);
    // current_desired_position_ = VectorXd::Zero(num_dof);

    if (!dmp_controller_.initialize(node_handle_, num_dofs, DMPController::CARTESIAN_AND_JOINT))
    {
        ROS_ERROR("Could not initialize dmp controller.");
        initialized_ = false;
        return initialized_;
    }

    cart_controller_.reset(new ChildController());
    if (!cart_controller_->init(robot_state, node_handle_))
    {
        ROS_ERROR("Could not initialize cartesian controller.");
        initialized_ = false;
        return initialized_;
    }

    for (int i = 0; i < dmp::N_CART; i++)
    {
        actual_endeffector_linear_twist_(i) = 0.0;
        desired_endeffector_linear_twist_(i) = 0.0;
    }

    joint_pos_vel_acc_point_ = VectorXd::Zero(dmp::N_JOINTS * dmp::POS_VEL_ACC);

    return (initialized_ = true);
}

bool DMPIkController::getArmRelatedVariables(const std::string& handle_namespace, std::string& controller_handle_namespace)
{
    if(handle_namespace.compare(0,6,std::string("/l_arm")) == 0)
    {
        controller_handle_namespace.assign("/l_arm_dmp_ik_controller");
    }
    else if(handle_namespace.compare(0,6,std::string("/r_arm")) == 0)
    {
        controller_handle_namespace.assign("/r_arm_dmp_ik_controller");
    }
    else
    {
        ROS_ERROR("Invalid namespace: %s", handle_namespace.c_str());
        return false;
    }
    return true;
}

bool DMPIkController::readParameters()
{
    ROS_INFO("Reading parameters...");
    std::string controller_handle_namespace;
    ROS_ASSERT_FUNC(getArmRelatedVariables(node_handle_.getNamespace(), controller_handle_namespace));
    ros::NodeHandle controller_handle(controller_handle_namespace);
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(controller_handle, std::string("root_name"), root_name_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(controller_handle, std::string("keep_restposture_fixed_for_testing"), keep_restposture_fixed_for_testing_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("publisher_buffer_size"), publisher_buffer_size_));
    ROS_INFO("Reading parameters... done");
    return true;
}

bool DMPIkController::initRTPublisher()
{
    rosrt::init();

    visualization_msgs::Marker visualization_pose_marker_actual;
    viz_marker_actual_arrow_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise
            <visualization_msgs::Marker> (std::string("dmp_actual_arrow"), 1), publisher_buffer_size_, visualization_pose_marker_actual));

    visualization_msgs::Marker visualization_pose_marker_desired;
    viz_marker_desired_arrow_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise
            <visualization_msgs::Marker> (std::string("dmp_desired_arrow"), 1), publisher_buffer_size_, visualization_pose_marker_desired));

    visualization_msgs::Marker visualization_line_marker_actual;
    visualization_line_marker_actual.points.resize(visualization_line_max_points_);
    viz_marker_actual_line_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise
            <visualization_msgs::Marker> (std::string("dmp_actual_line"), 1), publisher_buffer_size_, visualization_line_marker_actual));

    visualization_msgs::Marker visualization_line_marker_desired;
    visualization_line_marker_desired.points.resize(visualization_line_max_points_);
    viz_marker_desired_line_publisher_.reset(new rosrt::Publisher<visualization_msgs::Marker>(node_handle_.advertise
            <visualization_msgs::Marker> (std::string("dmp_desired_line"), 1), publisher_buffer_size_, visualization_line_marker_desired));

    geometry_msgs::PoseStamped pose_stamped_actual_msg;
    pose_actual_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("dmp_pose_actual"), 1), publisher_buffer_size_, pose_stamped_actual_msg));

    geometry_msgs::PoseStamped pose_stamped_desired_msg;
    pose_desired_publisher_.reset(new rosrt::Publisher<geometry_msgs::PoseStamped>(node_handle_.advertise
            <geometry_msgs::PoseStamped> (std::string("dmp_pose_desired"), 1), publisher_buffer_size_, pose_stamped_desired_msg));

    return true;
}

bool DMPIkController::initXml(pr2_mechanism_model::RobotState* robot, TiXmlElement* config)
{
    ros::NodeHandle node_handle(config->Attribute("name"));
    return init(robot, node_handle);
}

// REAL-TIME REQUIREMENTS
void DMPIkController::starting()
{
    cart_controller_->starting();
}

// REAL-TIME REQUIREMENTS
void DMPIkController::update()
{
    if (dmp_controller_.tryGetLock())
    {
        if (!getCommand(next_waypoint_))
        {
            // gripper_controller_.reset();
            visualization_line_points_index_ = 0;
            dmp_controller_.incrementDMPQueueIndex();
        }
        else
        {
            setCommand(next_waypoint_);
            visualize();
        }
        dmp_controller_.freeLock();
    }
    else
    {
        dmp_controller_.freeLock();
        dmp_controller_.incrementSkippedCycles();
    }
    cart_controller_->update();
}

// REAL-TIME REQUIREMENTS
void DMPIkController::stopping()
{
    cart_controller_->stopping();
}

// REAL-TIME REQUIREMENTS
bool DMPIkController::getCommand(dmp_motion_controller::Waypoint& next_waypoint)
{

    // use the desired pose and twist
    KDL::Frame pose_desired_gripper_robot = cart_controller_->kdl_pose_desired_;
    KDL::Twist twist_desired_gripper_robot = cart_controller_->kdl_twist_desired_;
//    KDL::Frame pose_desired_gripper_task = pose_desired_gripper_robot * endeffector_offset_;
//    KDL::Twist twist_desired_gripper_task = twist_desired_gripper_robot.RefPoint(pose_desired_gripper_robot.M * endeffector_offset_.p);

    // get current goal of the DMP for logging purpose
    // dmp_controller_.getCurrentDMP()->getGoal(current_goal_);

    // get measured pose and twist
    KDL::Frame pose_measured_gripper_robot = cart_controller_->kdl_real_pose_measured_;
    KDL::Twist twist_measured_gripper_robot = cart_controller_->kdl_twist_measured_;

//    KDL::Frame pose_measured_gripper_task;
//    KDL::Twist twist_measured_gripper_task;
//    ROS_ASSERT_FUNC(task_frame_transformer_.getActualRobotTransform(dmp_controller_.getCurrentType(),
//                                                                    cart_controller_->kdl_real_pose_measured_,
//                                                                    cart_controller_->kdl_twist_measured_,
//                                                                    pose_measured_gripper_task,
//                                                                    twist_measured_gripper_task));

    setTrajectoryPointFromCurrent(cart_controller_->kdl_real_pose_measured_, cart_controller_->kdl_twist_measured_);
    dmp_controller_.addDebugActual(controller_trajectory_point_);

    for (int i = 0; i < dmp::N_CART; i++)
    {
        actual_endeffector_linear_twist_[i] = cart_controller_->kdl_twist_measured_(i);
    }

    // set start to current state in case it was not set before
    if (!dmp_controller_.getCurrentDMP()->isStartSet())
    {
        last_frame_set_ = false;

        int transform_type = static_cast<int>(dmp_controller_.getCurrentType());

        for(int i=0; i<dmp::N_JOINTS; ++i)
        {
            joint_pos_vel_acc_point_(i * dmp::POS_VEL_ACC) = cart_controller_->rest_posture_joint_configuration_(i);
        }

        ROS_ASSERT_FUNC(task_frame_transformer_.getTaskTransformFromCurrent(dmp_controller_.getCurrentType(),
                                                                            pose_measured_gripper_robot,
                                                                            joint_pos_vel_acc_point_,
                                                                            dmp_trajectory_points_[transform_type]));

        // set start of the dmp to current
        dmp_controller_.getCurrentDMP()->changeStart(dmp_trajectory_points_[transform_type]);

//        setStartConfigurationFromCurrent(pose_measured_gripper_robot);
//        // adapt joint goal for the continuous joints depending on current joint configuration.
//        // get joint angle goal configuration recorded during demonstration.
//        // TODO: move this in separate function
//        if (!dmp_controller_.getCurrentDMP()->getInitialStart(initial_joint_start_))
//        {
//            ROS_ERROR("Could not get initial start configuration.");
//            return false;
//        }
//        if (!dmp_controller_.getCurrentDMP()->getInitialGoal(initial_joint_goal_))
//        {
//            ROS_ERROR("Could not get initial goal configuration.");
//            return false;
//        }
//        computeAndSetJointGoal();
    }

    if(dmp_controller_.firstDMPControlCycle())
    {
        dmp_controller_.setStartTime(ros::Time::now());
    }

    // run dmp and obtain reference trajectory point
    bool movement_finished = false;
    double execution_duration = dmp_controller_.getCurrentExecutionDuration();
    int num_samples = execution_duration * CONTROLLER_SAMPLING_FREQUENCY;
    if(!transformCommand(controller_trajectory_point_, movement_finished, execution_duration, num_samples))
    {
        ROS_ERROR("Could not transform command into %i.", static_cast<int>(dmp_controller_.getCurrentType()));
    }

    dmp_controller_.addDebugDesired(controller_trajectory_point_);

    for (int i = 0; i < dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS; i++)
    {
        next_waypoint_.positions[i] = controller_trajectory_point_(i * dmp::POS_VEL_ACC + 0);
        next_waypoint_.velocities[i] = controller_trajectory_point_(i * dmp::POS_VEL_ACC + 1);
        next_waypoint_.accelerations[i] = controller_trajectory_point_(i * dmp::POS_VEL_ACC + 2);
    }

    if(movement_finished)
    {
        dmp_controller_.setEndTime(ros::Time::now());
        return false;
    }
    return true;
}

// REAL-TIME REQUIREMENTS
void DMPIkController::setCommand(const dmp_motion_controller::Waypoint& waypoint)
{

    // set the desired pose
    for (int i = 0; i < dmp::N_CART; i++)
    {
        cart_controller_->kdl_pose_desired_.p(i) = waypoint.positions[i];
    }
    cart_controller_->kdl_pose_desired_.M = KDL::Rotation::Quaternion(waypoint.positions[dmp::N_CART + dmp::_QX_], waypoint.positions[dmp::N_CART + dmp::_QY_],
                                                                      waypoint.positions[dmp::N_CART + dmp::_QZ_], waypoint.positions[dmp::N_CART + dmp::_QW_]);

    // set desired linear velocities
    for (int i = 0; i < dmp::N_CART; i++)
    {
        cart_controller_->kdl_twist_desired_(i) = waypoint.velocities[i];
        desired_endeffector_linear_twist_[i] = waypoint.velocities[i];
    }

    current_frame_ = cart_controller_->kdl_pose_desired_;
    if (last_frame_set_)
    {
        cart_controller_->kdl_twist_desired_ =
                diff(current_frame_, last_frame_, static_cast<double> (1.0 / CONTROLLER_SAMPLING_FREQUENCY)).RefPoint(cart_controller_->kdl_pose_measured_.p - current_frame_.p);
    }
    last_frame_ = current_frame_;
    last_frame_set_ = true;

    // gripper offset
    // cart_controller_->kdl_pose_desired_ = cart_controller_->kdl_pose_desired_ * endeffector_offset_.Inverse();
    // cart_controller_->kdl_twist_desired_.RefPoint(cart_controller_->kdl_pose_desired_.M * (-endeffector_offset_.p));

    if (!keep_restposture_fixed_for_testing_)
    {
        for (int i = 0; i < dmp::N_JOINTS; i++)
        {
            cart_controller_->rest_posture_joint_configuration_(i) = waypoint.positions[dmp::N_CART + dmp::N_QUAT + i];
        }
    }

    // update goal orientation if needed
    //	if((dmp_controller_.getCurrentDMP()->getProgress() > start_grasp_orientation_adaptation_)
    //			&& (dmp_controller_.getCurrentDMP()->getProgress() < end_grasp_orientation_adaptation_)
    //			&& (dmp_controller_.getCurrentDMP()->getTaskType() == dmp::DMPParameters::eGRASP))
    //	{
    //		goal_estimator_.updateGoalOrientation(dmp_controller_.getCurrentDMP(), desired_endeffector_linear_twist_);
    //	}

}

// REAL-TIME REQUIREMENTS
bool DMPIkController::transformCommand(Eigen::VectorXd& trajectory_point,
                                       bool& movement_finished,
                                       const double execution_duration,
                                       const int num_samples)
{

    pr2_tasks_transforms::TaskTransforms::TransformType transform_type = dmp_controller_.getCurrentType();
    int transform_type_id = static_cast<int>(transform_type);

    ROS_ASSERT_FUNC(dmp_controller_.getCurrentDMP()->propagateStep(dmp_pos_vel_acc_trajectory_points_[transform_type_id], movement_finished, execution_duration, num_samples));
    ROS_ASSERT_FUNC(task_frame_transformer_.getRobotTransform(transform_type, dmp_pos_vel_acc_trajectory_points_[transform_type_id], controller_trajectory_point_));

    return true;
}

// REAL-TIME REQUIREMENTS
void DMPIkController::setTrajectoryPointFromCurrent(KDL::Frame& pose_measured_gripper_robot, KDL::Twist twist_measured_gripper_robot)
{
    for (int i = 0; i < dmp::N_CART; i++)
    {
        controller_trajectory_point_(i * dmp::POS_VEL_ACC + 0) = pose_measured_gripper_robot.p[i];
        controller_trajectory_point_(i * dmp::POS_VEL_ACC + 1) = twist_measured_gripper_robot.vel[i];
    }
    pose_measured_gripper_robot.M.GetQuaternion(quat_(dmp::_QX_), quat_(dmp::_QY_), quat_(dmp::_QZ_), quat_(dmp::_QW_));
    for (int i = 0; i < dmp::N_QUAT; i++)
    {
        controller_trajectory_point_((dmp::N_CART * dmp::POS_VEL_ACC) + (i * dmp::POS_VEL_ACC + 0)) = quat_(i);
        controller_trajectory_point_((dmp::N_CART * dmp::POS_VEL_ACC) + (i * dmp::POS_VEL_ACC + 1)) = 0.0;
    }

    const int CONTINOUS_FOREARM_ROLL_JOINT = 4;
    const int CONTINOUS_WRIST_ROLL_JOINT = 6;

    for (int i = 0; i < dmp::N_JOINTS; i++)
    {
        if ((i == CONTINOUS_FOREARM_ROLL_JOINT) || (i == CONTINOUS_WRIST_ROLL_JOINT))
        {
            controller_trajectory_point_(((dmp::N_CART + dmp::N_QUAT) * dmp::POS_VEL_ACC) + (i * dmp::POS_VEL_ACC + 0))
                    = angles::normalize_angle(cart_controller_->kdl_current_joint_positions_(i));
        }
        else
        {
            controller_trajectory_point_(((dmp::N_CART + dmp::N_QUAT) * dmp::POS_VEL_ACC) + (i * dmp::POS_VEL_ACC + 0))
                    = cart_controller_->kdl_current_joint_positions_(i);
        }
        controller_trajectory_point_(((dmp::N_CART + dmp::N_QUAT) * dmp::POS_VEL_ACC) + (i * dmp::POS_VEL_ACC + 1)) = 0.0;
    }
    for (int i = 0; i < dmp::N_CART + dmp::N_QUAT + dmp::N_JOINTS; i++)
    {
        // trajectory_point_(i * dmp::POS_VEL_ACC + 2) = current_goal_(i);
        // WARNING: set acceleration to zero
        controller_trajectory_point_(i * dmp::POS_VEL_ACC + 2) = 0.0;
    }
}

//void DMPIkController::setStartConfigurationFromCurrent(KDL::Frame& pose_measured_gripper_center)
//{
////    // get current gripper center position
////        for (int i = 0; i < dmp::N_CART; i++)
////        {
////            // WARNING: here we set the desired position to the current
////            current_desired_position_(i) = pose_measured_gripper_center.p[i];
////        }
////
////        // get current gripper center orientation
////        pose_measured_gripper_center.M.GetQuaternion(quat_(dmp::_QX_), quat_(dmp::_QY_), quat_(dmp::_QZ_), quat_(dmp::_QW_));
////        for (int i = 0; i < dmp::N_QUAT; i++)
////        {
////            current_desired_position_(dmp::N_CART + i) = quat_(i);
////        }
////        // set current joint configuration to current rest posture (not current joint configuration) to obtain smooth transitions
////        for (int i = 0; i < dmp::N_JOINTS; i++)
////        {
////            current_desired_position_(dmp::N_CART + dmp::N_QUAT + i) = cart_controller_->rest_posture_joint_configuration_(i);
////        }
//
//}

// REAL-TIME REQUIREMENTS
//bool DMPIkController::computeAndSetJointGoal()
//{
//    double updated_goal;
//    double joint_angle;
//    int angle_offset;
//
//    joint_angle = initial_joint_start_(CONTINOUS_FOREARM_ROLL_JOINT) - cart_controller_->rest_posture_joint_configuration_(CONTINOUS_FOREARM_ROLL_JOINT);
//    angle_offset = 0;
//    while (joint_angle >= M_PI)
//    {
//        angle_offset--;
//        joint_angle -= 2.0 * M_PI;
//    }
//    while (joint_angle < -M_PI)
//    {
//        angle_offset++;
//        joint_angle += 2.0 * M_PI;
//    }
//
//    updated_goal = initial_joint_goal_(CONTINOUS_FOREARM_ROLL_JOINT) + (angle_offset * 2.0 * M_PI);
//    if (!dmp_controller_.getCurrentDMP()->changeGoal(updated_goal, dmp::N_CART + dmp::N_QUAT + CONTINOUS_FOREARM_ROLL_JOINT))
//    {
//        // TODO: do something about it...
//    }
//
//    joint_angle = initial_joint_start_(CONTINOUS_WRIST_ROLL_JOINT) - cart_controller_->rest_posture_joint_configuration_(CONTINOUS_WRIST_ROLL_JOINT);
//    angle_offset = 0;
//    while (joint_angle >= M_PI)
//    {
//        angle_offset--;
//        joint_angle -= 2.0 * M_PI;
//    }
//    while (joint_angle < -M_PI)
//    {
//        angle_offset++;
//        joint_angle += 2.0 * M_PI;
//    }
//
//    updated_goal = initial_joint_goal_(CONTINOUS_WRIST_ROLL_JOINT) + (angle_offset * 2.0 * M_PI);
//    if (!dmp_controller_.getCurrentDMP()->changeGoal(updated_goal, dmp::N_CART + dmp::N_QUAT + CONTINOUS_WRIST_ROLL_JOINT))
//    {
//        // TODO: do something about it...
//    }
//
//    return true;
//}

// REAL-TIME REQUIREMENTS
void DMPIkController::visualize()
{

    publishing_counter_++;
    if (publishing_counter_ % publishing_rate_ == 0)
    {
        publishing_counter_ = 0;

        Eigen::Matrix<double, 3, 1> velocity_vec;
        Eigen::Matrix<double, 3, 1> world_vec;
        Eigen::Quaternion<double> eigen_quat;

        visualization_msgs::MarkerPtr vis_marker_actual_arrow = viz_marker_actual_arrow_publisher_->allocate();
        if(vis_marker_actual_arrow)
        {
            vis_marker_actual_arrow->header.frame_id = std::string("/") + root_name_;
            vis_marker_actual_arrow->header.stamp = ros::Time::now();
            vis_marker_actual_arrow->ns = "dmp_motion_controller";
            vis_marker_actual_arrow->type = visualization_msgs::Marker::ARROW;
            vis_marker_actual_arrow->action = visualization_msgs::Marker::ADD;
            vis_marker_actual_arrow->id = 0;
            vis_marker_actual_arrow->scale.x = 0.8 * desired_endeffector_linear_twist_(0);
            vis_marker_actual_arrow->scale.y = 0.8 * desired_endeffector_linear_twist_(1);
            vis_marker_actual_arrow->scale.z = 0.8 * desired_endeffector_linear_twist_(2);
            vis_marker_actual_arrow->color.r = 0.0f;
            vis_marker_actual_arrow->color.g = 0.0f;
            vis_marker_actual_arrow->color.b = 1.0f;
            vis_marker_actual_arrow->color.a = 0.5;
            vis_marker_actual_arrow->lifetime = ros::Duration();
            vis_marker_actual_arrow->pose.position.x = cart_controller_->kdl_real_pose_measured_.p.x();
            vis_marker_actual_arrow->pose.position.y = cart_controller_->kdl_real_pose_measured_.p.y();
            vis_marker_actual_arrow->pose.position.z = cart_controller_->kdl_real_pose_measured_.p.z();
            double qx, qy, qz, qw;
            cart_controller_->kdl_real_pose_measured_.M.GetQuaternion(qx, qy, qz, qw);
            vis_marker_actual_arrow->pose.orientation.x = qx;
            vis_marker_actual_arrow->pose.orientation.y = qy;
            vis_marker_actual_arrow->pose.orientation.z = qz;
            vis_marker_actual_arrow->pose.orientation.w = qw;

            for (int i = 0; i < dmp::N_CART; i++)
            {
                velocity_vec(i) = actual_endeffector_linear_twist_(i);
                world_vec(i) = 0.0;
            }
            world_vec(0) = 1.0;
            eigen_quat.setFromTwoVectors(world_vec, velocity_vec);

            double length = velocity_vec.norm();
            if (length > 0.01)
            {
                vis_marker_actual_arrow->scale.x = length;
            }
            else
            {
                vis_marker_actual_arrow->scale.x = 0.01;
            }
            vis_marker_actual_arrow->scale.y = 0.25;
            vis_marker_actual_arrow->scale.z = 0.25;

            if((isinf(eigen_quat.x()) == 0) && (isnan(eigen_quat.x()) == 0) && (isinf(eigen_quat.y()) == 0) && (isnan(eigen_quat.y()) == 0)
                    && (isinf(eigen_quat.z()) == 0) && (isnan(eigen_quat.z()) == 0) && (isinf(eigen_quat.w()) == 0) && (isnan(eigen_quat.w()) == 0) )
            {
                vis_marker_actual_arrow->pose.orientation.x = eigen_quat.x();
                vis_marker_actual_arrow->pose.orientation.y = eigen_quat.y();
                vis_marker_actual_arrow->pose.orientation.z = eigen_quat.z();
                vis_marker_actual_arrow->pose.orientation.w = eigen_quat.w();
            }
            viz_marker_actual_arrow_publisher_->publish(vis_marker_actual_arrow);
        }
        else
        {
            ROS_ERROR("skipping visualization");
        }

        visualization_msgs::MarkerPtr vis_marker_desired_arrow = viz_marker_desired_arrow_publisher_->allocate();
        if (vis_marker_desired_arrow)
        {
            vis_marker_desired_arrow->header.frame_id = std::string("/") + root_name_;
            vis_marker_desired_arrow->header.stamp = ros::Time::now();
            vis_marker_desired_arrow->ns = "dmp_motion_controller";
            vis_marker_desired_arrow->type = visualization_msgs::Marker::ARROW;
            vis_marker_desired_arrow->action = visualization_msgs::Marker::ADD;
            vis_marker_desired_arrow->id = 0;
            vis_marker_desired_arrow->scale.x = 0.8 * desired_endeffector_linear_twist_(0);
            vis_marker_desired_arrow->scale.y = 0.8 * desired_endeffector_linear_twist_(1);
            vis_marker_desired_arrow->scale.z = 0.8 * desired_endeffector_linear_twist_(2);
            vis_marker_desired_arrow->color.r = 0.0f;
            vis_marker_desired_arrow->color.g = 1.0f;
            vis_marker_desired_arrow->color.b = 0.0f;
            vis_marker_desired_arrow->color.a = 0.5;
            vis_marker_desired_arrow->lifetime = ros::Duration();
            vis_marker_desired_arrow->pose.position.x = next_waypoint_.positions[dmp::_XX_];
            vis_marker_desired_arrow->pose.position.y = next_waypoint_.positions[dmp::_YY_];
            vis_marker_desired_arrow->pose.position.z = next_waypoint_.positions[dmp::_ZZ_];
            vis_marker_desired_arrow->pose.orientation.x = next_waypoint_.positions[dmp::N_CART + dmp::_QX_];
            vis_marker_desired_arrow->pose.orientation.y = next_waypoint_.positions[dmp::N_CART + dmp::_QY_];
            vis_marker_desired_arrow->pose.orientation.z = next_waypoint_.positions[dmp::N_CART + dmp::_QZ_];
            vis_marker_desired_arrow->pose.orientation.w = next_waypoint_.positions[dmp::N_CART + dmp::_QW_];

            for (int i = 0; i < dmp::N_CART; i++)
            {
                velocity_vec(i) = desired_endeffector_linear_twist_(i);
                world_vec(i) = 0.0;
            }
            world_vec(0) = 1.0;
            eigen_quat.setFromTwoVectors(world_vec, velocity_vec);

            double length = velocity_vec.norm();
            if (length > 0.01)
            {
                vis_marker_desired_arrow->scale.x = length;
            }
            else
            {
                vis_marker_desired_arrow->scale.x = 0.01;
            }
            vis_marker_desired_arrow->scale.y = 0.25;
            vis_marker_desired_arrow->scale.z = 0.25;

            if((isinf(eigen_quat.x()) == 0) && (isnan(eigen_quat.x()) == 0) && (isinf(eigen_quat.y()) == 0) && (isnan(eigen_quat.y()) == 0)
                    && (isinf(eigen_quat.z()) == 0) && (isnan(eigen_quat.z()) == 0) && (isinf(eigen_quat.w()) == 0) && (isnan(eigen_quat.w()) == 0) )
            {
                vis_marker_desired_arrow->pose.orientation.x = eigen_quat.x();
                vis_marker_desired_arrow->pose.orientation.y = eigen_quat.y();
                vis_marker_desired_arrow->pose.orientation.z = eigen_quat.z();
                vis_marker_desired_arrow->pose.orientation.w = eigen_quat.w();
            }

            viz_marker_desired_arrow_publisher_->publish(vis_marker_desired_arrow);
        }

        geometry_msgs::PoseStampedPtr pose_actual = pose_actual_publisher_->allocate();
        if(pose_actual)
        {
            pose_actual->header.frame_id = std::string("/") + root_name_;
            pose_actual->header.stamp = ros::Time::now();
            pose_actual->header.seq = 0;
            pose_actual->pose.position.x = cart_controller_->kdl_real_pose_measured_.p.x();
            pose_actual->pose.position.y = cart_controller_->kdl_real_pose_measured_.p.y();
            pose_actual->pose.position.z = cart_controller_->kdl_real_pose_measured_.p.z();
            double qx, qy, qz, qw;
            cart_controller_->kdl_real_pose_measured_.M.GetQuaternion(qx, qy, qz, qw);
            pose_actual->pose.orientation.x = qx;
            pose_actual->pose.orientation.y = qy;
            pose_actual->pose.orientation.z = qz;
            pose_actual->pose.orientation.w = qw;
            pose_actual_publisher_->publish(pose_actual);
        }
        else
        {
            ROS_ERROR("skipping visualization");
        }

        geometry_msgs::PoseStampedPtr pose_desired = pose_desired_publisher_->allocate();
        if(pose_desired)
        {
            pose_desired->header.frame_id = std::string("/") + root_name_;
            pose_desired->header.stamp = ros::Time::now();
            pose_desired->header.seq = 0;
            pose_desired->pose.position.x = next_waypoint_.positions[dmp::_XX_];
            pose_desired->pose.position.y = next_waypoint_.positions[dmp::_YY_];
            pose_desired->pose.position.z = next_waypoint_.positions[dmp::_ZZ_];
            pose_desired->pose.orientation.x = next_waypoint_.positions[dmp::N_CART + dmp::_QX_];
            pose_desired->pose.orientation.y = next_waypoint_.positions[dmp::N_CART + dmp::_QY_];
            pose_desired->pose.orientation.z = next_waypoint_.positions[dmp::N_CART + dmp::_QZ_];
            pose_desired->pose.orientation.w = next_waypoint_.positions[dmp::N_CART + dmp::_QW_];
            pose_desired_publisher_->publish(pose_desired);
        }
        else
        {
            ROS_ERROR("skipping visualization");
        }

        visualization_line_counter_++;
        if (visualization_line_counter_ % visualization_line_rate_ == 0)
        {
            visualization_line_counter_ = 0;

            visualization_msgs::MarkerPtr vis_marker_actual_line = viz_marker_actual_line_publisher_->allocate();
            if (vis_marker_actual_line)
            {
                vis_marker_actual_line->header.frame_id = std::string("/") + root_name_;
                vis_marker_actual_line->header.stamp = ros::Time::now();
                vis_marker_actual_line->header.seq = 0;
                vis_marker_actual_line->ns = "dmp_motion_controller";
                vis_marker_actual_line->type = visualization_msgs::Marker::LINE_STRIP;
                vis_marker_actual_line->id = 2;
                vis_marker_actual_line->scale.x = 0.006;
                vis_marker_actual_line->scale.y = 0.006;
                vis_marker_actual_line->scale.z = 0.006;
                // vis_marker_actual_line->lifetime = ros::Duration();
                vis_marker_actual_line->color.r = 0.0f;
                vis_marker_actual_line->color.g = 0.0f;
                vis_marker_actual_line->color.b = 1.0f;
                vis_marker_actual_line->color.a = 0.4;
                geometry_msgs::Point point;
                point.x = cart_controller_->kdl_real_pose_measured_.p.x();
                point.y = cart_controller_->kdl_real_pose_measured_.p.y();
                point.z = cart_controller_->kdl_real_pose_measured_.p.z();
                for (int i = visualization_line_points_index_; i < visualization_line_max_points_; ++i)
                {
                    vis_marker_actual_line->points[i] = point;
                }
                viz_marker_actual_line_publisher_->publish(vis_marker_actual_line);
            }
            else
            {
                ROS_ERROR("skipping visualization (viz_marker_actual_line)");
            }

            visualization_msgs::MarkerPtr vis_marker_desired_line = viz_marker_desired_line_publisher_->allocate();
            if (vis_marker_desired_line)
            {
                vis_marker_desired_line->header.frame_id = std::string("/") + root_name_;
                vis_marker_desired_line->header.stamp = ros::Time::now();
                vis_marker_desired_line->header.seq = 0;
                vis_marker_desired_line->ns = "dmp_motion_controller";
                vis_marker_desired_line->type = visualization_msgs::Marker::LINE_STRIP;
                vis_marker_desired_line->id = 3;
                vis_marker_desired_line->scale.x = 0.006;
                vis_marker_desired_line->scale.y = 0.006;
                vis_marker_desired_line->scale.z = 0.006;
                // vis_marker_desired_line->lifetime = ros::Duration();
                vis_marker_desired_line->header.seq = 0;
                vis_marker_desired_line->color.r = 0.0f;
                vis_marker_desired_line->color.g = 1.0f;
                vis_marker_desired_line->color.b = 0.0f;
                vis_marker_desired_line->color.a = 0.4;
                geometry_msgs::Point point;
                point.x = next_waypoint_.positions[dmp::_XX_];
                point.y = next_waypoint_.positions[dmp::_YY_];
                point.z = next_waypoint_.positions[dmp::_ZZ_];
                for (int i = visualization_line_points_index_; i < visualization_line_max_points_; ++i)
                {
                    vis_marker_desired_line->points[i] = point;
                }
                viz_marker_desired_line_publisher_->publish(vis_marker_desired_line);
            }
            else
            {
                ROS_ERROR("skipping visualization (viz_marker_actual_line)");
            }

            visualization_line_points_index_++;
        }
    }
}

}
