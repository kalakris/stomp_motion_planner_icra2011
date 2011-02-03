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

 \file    dmp_motion_unit.cpp

 \author  Peter Pastor
 \date    September 29, 2009

 **********************************************************************/

// system includes
#include <math.h>
#include <sstream>
#include <errno.h>

// ros includes
#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>

// local includes
#include <dmp_motion_generation/dynamic_movement_primitive.h>
#include <dmp_motion_generation/constants.h>
#include <dmp_motion_generation/math_helper.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

namespace dmp
{

static const char* DYNAMIC_MOVEMENT_PRIMITIVE_FILE_NAME = "DynamicMovementPrimitive";
static const char* DYNAMIC_MOVEMENT_PRIMITIVE_TOPIC_NAME = "dynamic_movement_primitive";

static const int DMP_ID_OFFSET = 10000;
static const double NOT_ASSIGNED = -666.666;

//

DynamicMovementPrimitive::DynamicMovementPrimitive(ros::NodeHandle& node_handle) :
    initialized_(false), item_id_(-1), node_handle_(node_handle), params_(node_handle_), dmp_debug_(node_handle_)
{
}

DynamicMovementPrimitive::~DynamicMovementPrimitive()
{
}

bool DynamicMovementPrimitive::initialize(int num_transformation_systems, int dmp_id, Parameters::Version version)
{
    Parameters dmp_params(node_handle_);
    if (!dmp_params.initialize(version))
    {
        ROS_ERROR("Could not initialize dmp parameters from node handle");
        initialized_ = false;
        return initialized_;
    }

    lwr::Parameters lwr_params(node_handle_);
    if (!lwr_params.initialize())
    {
        ROS_ERROR("Could not initialize lwr parameters from node handle");
        initialized_ = false;
        return initialized_;
    }
    return initialize(num_transformation_systems, dmp_params, lwr_params, dmp_id, version);
}

bool DynamicMovementPrimitive::initialize(int num_transformation_systems, Parameters dmp_params, int dmp_id, Parameters::Version version)
{
    lwr::Parameters lwr_params(node_handle_);
    if (!lwr_params.initialize())
    {
        ROS_ERROR("Could not initialize LWR parameters from node handle");
        initialized_ = false;
        return initialized_;
    }
    return initialize(num_transformation_systems, dmp_params, lwr_params, dmp_id, version);
}

bool DynamicMovementPrimitive::initialize(int num_transformation_systems, lwr::Parameters lwr_params, int dmp_id, Parameters::Version version)
{
    Parameters dmp_params(node_handle_);
    if (!dmp_params.initialize(version))
    {
        ROS_ERROR("Could not initialize dmp parameters from node handle");
        initialized_ = false;
        return initialized_;
    }
    return initialize(num_transformation_systems, dmp_params, lwr_params, dmp_id, version);
}

bool DynamicMovementPrimitive::initialize(int num_transformation_systems, Parameters dmp_params, lwr::Parameters lwr_params, int dmp_id,
                                          Parameters::Version version)
{

    // initialize directory name and id in the base class to generate item_name_
    if (!initializeBase(dmp_params.library_directory_name_, dmp_id, std::string(DYNAMIC_MOVEMENT_PRIMITIVE_FILE_NAME)))
    {
        ROS_ERROR("Could not initialize base.");
        initialized_ = false;
        return initialized_;
    }
    // ROS_INFO("Initializing item %s with id %i.", item_name_.c_str(), item_id_);

    // assign dmp parameters
    params_ = dmp_params;

    // overwrite number of transformation system in dmp_params
    if (num_transformation_systems <= 0)
    {
        ROS_ERROR("Number of transformation system %i is not valid", num_transformation_systems);
        initialized_ = false;
        return initialized_;
    }
    params_.num_transformation_systems_ = num_transformation_systems;

    // initialized transformation systems using the lwr parameters
    transformation_systems_.resize(params_.num_transformation_systems_);
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        // ROS_INFO("Initializing transformation system %i named %s.",i,item_name_.c_str());
        if (!transformation_systems_[i].initialize(dmp_params.library_directory_name_, item_id_, i, lwr_params))
        {
            ROS_ERROR("Could not initialize transformation system named %s.", item_name_.c_str());
            initialized_ = false;
            return initialized_;
        }
        // transformation_systems_[i].delta_goal_fit_ = NOT_ASSIGNED;
    }

    // set the version of the dmp formulation
    params_.version_ = version;

    // set canonical system to pre-defined state
    resetCanonicalState();

    // allocate some memory
    initialize();

    initialized_ = true;
    return initialized_;
}

void DynamicMovementPrimitive::initialize()
{
    trajectory_target_function_input_.clear();
    debug_trajectory_point_
            = VectorXd::Zero(NUM_DEBUG_CANONICAL_SYSTEM_VALUES + (params_.num_transformation_systems_ * NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES));
}

bool DynamicMovementPrimitive::initFromMessage(const dmp_motion_generation::DynamicMovementPrimitive& dmp_msg)
{

    initialized_ = dmp_msg.initialized;
    params_.version_ = static_cast<Parameters::Version> (dmp_msg.version);

    params_.k_gain_ = dmp_msg.k_gain;
    params_.d_gain_ = dmp_msg.d_gain;

    //    params_.alpha_z_ = dmp_msg.alpha_z;
    //    params_.beta_z_ = dmp_msg.beta_z;
    //    params_.alpha_s_ = dmp_msg.alpha_s;

    params_.is_learned_ = dmp_msg.is_learned;
    params_.is_setup_ = dmp_msg.is_setup;
    params_.is_start_set_ = dmp_msg.is_start_set;

    canonical_system_.x = dmp_msg.canonical_x;
    canonical_system_.time = dmp_msg.canonical_time;

    params_.tau_ = dmp_msg.duration;
    params_.initial_tau_ = dmp_msg.initial_duration;
    params_.delta_t_ = dmp_msg.delta_t;
    params_.initial_delta_t_ = dmp_msg.initial_delta_t;

    params_.library_directory_name_.assign(dmp_msg.library_directory_name);

    params_.teaching_duration_ = dmp_msg.teaching_duration;
    params_.execution_duration_ = dmp_msg.execution_duration;

    params_.can_sys_cutoff_ = dmp_msg.can_sys_cutoff;
    params_.alpha_x_ = dmp_msg.alpha_x;

    params_.num_samples_ = dmp_msg.num_samples;

    // initialize directory name and id in the base class to generate item_name_
    if (!initializeBase(params_.library_directory_name_, dmp_msg.dmp_id, std::string(DYNAMIC_MOVEMENT_PRIMITIVE_FILE_NAME)))
    {
        ROS_ERROR("Could not initialize base.");
        initialized_ = false;
        return initialized_;
    }

    params_.num_transformation_systems_ = static_cast<int> (dmp_msg.transformation_systems.size());
    transformation_systems_.resize(params_.num_transformation_systems_);
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        if (!transformation_systems_[i].initFromMessage(dmp_msg.transformation_systems[i]))
        {
            return false;
        }
    }

    if (!dmp_debug_.initialize(item_id_, std::string("debug_run"), params_.num_transformation_systems_,
                               NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES, NUM_DEBUG_CANONICAL_SYSTEM_VALUES, DEFAULT_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Could not initialize debug object.");
        initialized_ = false;
        return initialized_;
    }

    initialize();

    return true;
}

bool DynamicMovementPrimitive::writeToMessage(dmp_motion_generation::DynamicMovementPrimitive& dmp_msg)
{

    dmp_msg.initialized = initialized_;
    dmp_msg.dmp_id = item_id_;
    dmp_msg.version = params_.version_;

    dmp_msg.is_learned = params_.is_learned_;
    dmp_msg.is_setup = params_.is_setup_;
    dmp_msg.is_start_set = params_.is_start_set_;

    dmp_msg.canonical_x = canonical_system_.x;
    dmp_msg.canonical_time = canonical_system_.time;

    dmp_msg.duration = params_.tau_;
    dmp_msg.initial_duration = params_.initial_tau_;
    dmp_msg.delta_t = params_.delta_t_;
    dmp_msg.initial_delta_t = params_.initial_delta_t_;

    dmp_msg.library_directory_name.assign(params_.library_directory_name_);

    dmp_msg.teaching_duration = params_.teaching_duration_;
    dmp_msg.execution_duration = params_.execution_duration_;

    dmp_msg.can_sys_cutoff = params_.can_sys_cutoff_;
    dmp_msg.alpha_x = params_.alpha_x_;
    dmp_msg.k_gain = params_.k_gain_;
    dmp_msg.d_gain = params_.d_gain_;

    //    dmp_msg.alpha_z = params_.alpha_z_;
    //    dmp_msg.beta_z = params_.beta_z_;
    //    dmp_msg.alpha_s = params_.alpha_s_;

    dmp_msg.num_samples = params_.num_samples_;

    dmp_msg.transformation_systems.clear();
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        dmp_motion_generation::TransformationSystem transformation_system;
        if (!transformation_systems_[i].writeToMessage(transformation_system))
        {
            return false;
        }
        dmp_msg.transformation_systems.push_back(transformation_system);
    }
    return true;
}

bool DynamicMovementPrimitive::reInitializeParams()
{
    return params_.initialize();
}

bool DynamicMovementPrimitive::learnFromThetas(const std::vector<VectorXd>& thetas, const VectorXd &initial_start, const VectorXd &initial_goal,
                                               const double sampling_frequency, const double initial_duration)
{
    if (!initialized_)
    {
        ROS_ERROR("DMP is not initialized.");
        params_.is_learned_ = false;
        return params_.is_learned_;
    }

    ros::NodeHandle node_handle;

    // if dmp_debug is initialized, then debug information will be stored in debug_directory_name.
    if (!dmp_debug_.initialize(item_id_, std::string("debug_learn"), params_.num_transformation_systems_, NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES,
                               NUM_DEBUG_CANONICAL_SYSTEM_VALUES, sampling_frequency))
    {
        ROS_WARN("Debugging object remains uninitialized and will not be used.");
    }

    // set y0 to start state of trajectory and set goal to end of the trajectory
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        // set initial start and initial goal
        transformation_systems_[i].setInitialStart(initial_start(i));
        transformation_systems_[i].setInitialGoal(initial_goal(i));
    }

    params_.alpha_x_ = -log(params_.can_sys_cutoff_);
    params_.teaching_duration_ = initial_duration;

    params_.delta_t_ = static_cast<double> (1.0) / sampling_frequency;
    params_.initial_delta_t_ = params_.delta_t_;

    params_.tau_ = params_.teaching_duration_;
    params_.initial_tau_ = params_.tau_;

    switch (params_.version_)
    {
        case Parameters::ICRA2009:

            break;

        case Parameters::NIPS2003:

            break;

        default:
            ROS_ERROR("DMP version (%i) not valid.", params_.version_);
            params_.is_learned_ = false;
            return params_.is_learned_;
    }

    if (!setThetas(thetas))
    {
        ROS_ERROR("Could not set theta parameters.");
        params_.is_learned_ = false;
        return params_.is_learned_;
    }

    params_.is_learned_ = true;
    return params_.is_learned_;
}

bool DynamicMovementPrimitive::learnFromMinJerk(const VectorXd &start, const VectorXd &goal, const double duration, const double delta_t)
{
    if (!initialized_)
    {
        ROS_ERROR("DMP motion unit is not initialized, not learning from minimum jerk trajectory.");
        params_.is_learned_ = false;
        return params_.is_learned_;
    }

    double sampling_frequency = static_cast<double> (1.0) / delta_t;

    std::vector<std::string> variable_names;
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        std::stringstream ss;
        ss << i;
        variable_names.push_back(std::string("dummy_") + ss.str());
    }

    dmp::Trajectory min_jerk_trajectory(node_handle_);
    if (!min_jerk_trajectory.initialize(variable_names, sampling_frequency))
    {
        ROS_ERROR("Could not initialize trajectory.");
        params_.is_learned_ = false;
        return params_.is_learned_;
    }

    if (!MathHelper::generateMinJerkTrajectory(start, goal, duration, delta_t, min_jerk_trajectory))
    {
        ROS_ERROR("Could not generate minimum jerk trajectory.");
        params_.is_learned_ = false;
        return params_.is_learned_;
    }
    if (!learnFromTrajectory(min_jerk_trajectory))
    {
        ROS_ERROR("Could not learn from minimum jerk trajectory.");
        params_.is_learned_ = false;
        return params_.is_learned_;
    }

    return (params_.is_learned_ = true);
}

bool DynamicMovementPrimitive::learnFromTrajectory(const Trajectory &trajectory)
{
    if (!initialized_)
    {
        ROS_ERROR("DMP motion unit is not initialized, not learning from trajectory.");
        params_.is_learned_ = false;
        return params_.is_learned_;
    }

    int num_rows = trajectory.getLength();
    if (num_rows < MIN_NUM_DATA_POINTS)
    {
        ROS_ERROR("Trajectory has %i rows, but should have at least %i.", num_rows, MIN_NUM_DATA_POINTS);
        params_.is_learned_ = false;
        return params_.is_learned_;
    }

    double sampling_frequency = trajectory.getSamplingFrequency();
    if (sampling_frequency <= 0)
    {
        ROS_ERROR("Sampling frequency %f [Hz] of the trajectory is not valid.",sampling_frequency);
        params_.is_learned_ = false;
        return params_.is_learned_;
    }

    // if dmp_debug is initialized, then debug information will be stored in debug_directory_name.
    if (!dmp_debug_.initialize(item_id_, std::string("debug_learn"), params_.num_transformation_systems_, NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES,
                               NUM_DEBUG_CANONICAL_SYSTEM_VALUES, sampling_frequency))
    {
        ROS_WARN("Debugging object remains uninitialized and will not be used.");
    }

    // set teaching duration to the duration of the trajectory
    params_.teaching_duration_ = static_cast<double> (num_rows) / static_cast<double> (sampling_frequency);

    params_.delta_t_ = static_cast<double> (1.0) / sampling_frequency;
    params_.initial_delta_t_ = params_.delta_t_;
    params_.tau_ = params_.teaching_duration_;
    params_.initial_tau_ = params_.tau_;

    // compute alpha_x such that the canonical system drops below the cutoff when the trajectory has finished
    params_.alpha_x_ = -log(params_.can_sys_cutoff_);

    double mse_total = 0.0;
    double normalized_mse_total = 0.0;
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        transformation_systems_[i].trajectory_target_.clear();
        transformation_systems_[i].resetMSE();
    }
    trajectory_target_function_input_.clear();

    // reset canonical system
    resetCanonicalState();

    // obtain start and goal position
    VectorXd start = VectorXd::Zero(params_.num_transformation_systems_);
    if (!trajectory.getStartPosition(start))
    {
        ROS_ERROR("Could not get the start position of the trajectory");
        params_.is_learned_ = false;
        return params_.is_learned_;
    }
    VectorXd goal = VectorXd::Zero(params_.num_transformation_systems_);
    if (!trajectory.getEndPosition(goal))
    {
        ROS_ERROR("Could not get the goal position of the trajectory");
        params_.is_learned_ = false;
        return params_.is_learned_;
    }

    // set y0 to start state of trajectory and set goal to end of the trajectory
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        // TODO: check whether all this is necessary
        transformation_systems_[i].reset();

        // set start and goal
        transformation_systems_[i].setStart(start(i));
        transformation_systems_[i].setGoal(goal(i));

        // set current state to start state (position and velocity)
        transformation_systems_[i].setState(start(i), 0.0);
    }

    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        transformation_systems_[i].setInitialStart(transformation_systems_[i].y0_);
        transformation_systems_[i].setInitialGoal(transformation_systems_[i].goal_);
    }

    dmp_debug_.clear();
    for (int row_index = 0; row_index < num_rows; row_index++)
    {

        // set transformation target
        for (int i = 0; i < params_.num_transformation_systems_; i++)
        {
            transformation_systems_[i].t_ = trajectory.getTrajectoryPosition(row_index, i);
            transformation_systems_[i].td_ = trajectory.getTrajectoryVelocity(row_index, i);
            transformation_systems_[i].tdd_ = trajectory.getTrajectoryAcceleration(row_index, i);
            transformation_systems_[i].f_ = 0.0;
            transformation_systems_[i].ft_ = 0.0;
        }

        // fit the target function
        if (!integrateAndFit())
        {
            ROS_ERROR("Could not integrate system and fit the target function");
            params_.is_learned_ = false;
            return params_.is_learned_;
        }

        if (dmp_debug_.isInitialized())
        {
            int index = 0;
            debug_trajectory_point_(index) = canonical_system_.x;
            index++;
            debug_trajectory_point_(index) = canonical_system_.time;
            index++;
            debug_trajectory_point_(index) = 0;
            index++;
            for (int i = NUM_DEBUG_CANONICAL_SYSTEM_VALUES; i < params_.num_transformation_systems_ + NUM_DEBUG_CANONICAL_SYSTEM_VALUES; i++)
            {
                debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].y_;
                index++;
                debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].t_;
                index++;
                debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].td_;
                index++;
                debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].tdd_;
                index++;
                debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].f_;
                index++;
                debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].ft_;
                index++;
            }
            if (!dmp_debug_.add(debug_trajectory_point_))
            {
                ROS_ERROR("Could not add trajectory point to (learn) debug object.");
                params_.is_learned_ = false;
                return params_.is_learned_;
            }
        }
    }

    if (!learnTransformationTarget())
    {
        ROS_ERROR("Could not learn transformation target.");
        params_.is_learned_ = false;
        return params_.is_learned_;
    }

    mse_total = 0.0;
    normalized_mse_total = 0.0;
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        double mse;
        double normalized_mse;
        if (transformation_systems_[i].getMSE(mse))
        {
            mse_total += mse;
        }
        if (transformation_systems_[i].getNormalizedMSE(normalized_mse))
        {
            normalized_mse_total += normalized_mse;
        }
        transformation_systems_[i].resetMSE();
    }

    if (!writeDebugTrajectory())
    {
        ROS_WARN("Could not write debug trajectory.");
    }

    // we are done...
    ROS_INFO("Successfully learned DMP with id %i from trajectory.", item_id_);
    params_.is_learned_ = true;
    return params_.is_learned_;
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::getInitialStart(VectorXd &initial_start)
{
    if (initial_start.size() != params_.num_transformation_systems_)
    {
        return false;
    }
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        initial_start(i) = transformation_systems_[i].initial_y0_;
    }
    return true;
}
// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::getInitialGoal(VectorXd &initial_goal)
{
    if (initial_goal.size() != params_.num_transformation_systems_)
    {
        return false;
    }
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        initial_goal(i) = transformation_systems_[i].initial_goal_;
    }
    return true;
}
// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::getGoal(VectorXd &goal)
{
    if (goal.size() != params_.num_transformation_systems_)
    {
        return false;
    }
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        goal(i) = transformation_systems_[i].goal_;
    }
    return true;
}

bool DynamicMovementPrimitive::setup(const double sampling_frequency)
{
    VectorXd start = VectorXd::Zero(params_.num_transformation_systems_);
    VectorXd goal = VectorXd::Zero(params_.num_transformation_systems_);
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        start(i) = transformation_systems_[i].initial_y0_;
        goal(i) = transformation_systems_[i].initial_goal_;
    }
    return setup(start, goal, params_.initial_tau_, sampling_frequency);
}

//bool DynamicMovementPrimitive::setup(const std::vector<double>& goal, const double sampling_frequency)
//{
//    if (static_cast<int> (goal.size()) != params_.num_transformation_systems_)
//    {
//        ROS_ERROR("Could not setup DMP unit. Provided size of goal vector (%i) does not match number of transformation systems (%i).", static_cast<int>(goal.size()), params_.num_transformation_systems_);
//        return false;
//    }
//    Eigen::Map<VectorXd> goal_array = VectorXd::Map(&goal(0), goal.size());
//    return setup(goal_array, sampling_frequency);
//}

bool DynamicMovementPrimitive::setup(const VectorXd &goal, const double movement_duration, const double sampling_frequency)
{
    VectorXd start = VectorXd(params_.num_transformation_systems_);
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        start(i) = transformation_systems_[i].initial_y0_;
    }
    if (!setup(start, goal, movement_duration, sampling_frequency))
    {
        params_.is_setup_ = false;
        return params_.is_setup_;
    }

    // start has not been specified, need to be set before the DMP can be propagated
    params_.is_start_set_ = false;

    params_.is_setup_ = true;
    return params_.is_setup_;
}

//bool DynamicMovementPrimitive::setup(const std::vector<double>& start, const std::vector<double>& goal, const double movement_duration,
//                                     const double sampling_frequency)
//{
//    if (start.size() != transformation_systems_.size())
//    {
//        ROS_ERROR("The dimension of the start vector %i does not coincide with the number of transformation systems %i.", static_cast<int>(start.size()), static_cast<int>(transformation_systems_.size()));
//        params_.is_setup_ = false;
//        return params_.is_setup_;
//    }
//    if (goal.size() != transformation_systems_.size())
//    {
//        ROS_ERROR("The dimension of the goal vector %i does not coincide with the number of transformation systems %i.", static_cast<int>(goal.size()), static_cast<int>(transformation_systems_.size()));
//        params_.is_setup_ = false;
//        return params_.is_setup_;
//    }
//    return setup(Eigen::VectorXd::Map(&start[0], start.size()), Eigen::VectorXd::Map(&goal[0], goal.size()), movement_duration, sampling_frequency);
//}

bool DynamicMovementPrimitive::setup(const VectorXd &start, const VectorXd &goal, const double movement_duration, const double sampling_frequency)
{
    if (!initialized_)
    {
        ROS_ERROR("DMP unit is not initialized.");
        params_.is_setup_ = false;
        return params_.is_setup_;
    }

    if (!params_.is_learned_)
    {
        ROS_ERROR("DMP unit is not learned.");
        params_.is_setup_ = false;
        return params_.is_setup_;
    }

    if (start.size() != params_.num_transformation_systems_)
    {
        ROS_ERROR("Cannot set start of the DMP, the size is %i, but should be %i.", start.size(), params_.num_transformation_systems_);
        params_.is_setup_ = false;
        return params_.is_setup_;
    }

    if (goal.size() != params_.num_transformation_systems_)
    {
        ROS_ERROR("Cannot set goal of the DMP, the size is %i, but should be %i.", goal.size(), params_.num_transformation_systems_);
        params_.is_setup_ = false;
        return params_.is_setup_;
    }

    // reset canonical system
    resetCanonicalState();

    if (movement_duration > 0)
    {
        params_.tau_ = movement_duration;

        if (sampling_frequency <= 0)
        {
            ROS_ERROR("Sampling frequency %f [Hz] of the trajectory is not valid.", sampling_frequency);
            params_.is_setup_ = false;
            return params_.is_setup_;
        }
        params_.delta_t_ = static_cast<double> (1.0) / static_cast<double> (sampling_frequency);
    }
    else
    {
        params_.tau_ = params_.initial_tau_;
        params_.delta_t_ = params_.initial_delta_t_;
    }

    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        // set internal variables to zero
        transformation_systems_[i].reset();

        // set start and goal
        transformation_systems_[i].setStart(start(i));
        transformation_systems_[i].setGoal(goal(i));

        // set current state to start state (position and velocity)
        transformation_systems_[i].setState(start(i), 0.0);
    }

    // if dmp_debug is initialized, then debug information will be stored in debug_directory_name.
    if (!dmp_debug_.initialize(item_id_, std::string("debug_run"), params_.num_transformation_systems_, NUM_DEBUG_TRANSFORMATION_SYSTEM_VALUES,
                               NUM_DEBUG_CANONICAL_SYSTEM_VALUES, sampling_frequency))
    {
        ROS_WARN("Debugging object remains uninitialized and will not be used.");
    }

    // reset the sample counter
    params_.num_samples_ = 0;

    // start is set
    params_.is_start_set_ = true;

    params_.is_setup_ = true;
    return params_.is_setup_;
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::getCurrentPosition(VectorXd &current_desired_position, const int start_index, const int end_index)
{
    if ((!params_.is_setup_) || (start_index < 0) || (end_index > params_.num_transformation_systems_) || (end_index <= start_index))
    {
        return false;
    }
    if (current_desired_position.size() != end_index - start_index)
    {
        ROS_ERROR("Provided vector has wrong size (%i), required size is (%i). Cannot get current position.", current_desired_position.size(), end_index-start_index);
        return false;
    }
    for (int i = start_index; i < end_index; i++)
    {
        current_desired_position(i) = transformation_systems_[i].y_;
    }
    return true;
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::changeGoal(const VectorXd &goal, const int start_index, const int end_index)
{
    if ((!params_.is_setup_) || (start_index < 0) || (end_index > params_.num_transformation_systems_) || (end_index <= start_index))
    {
        return false;
    }
    if (goal.size() != end_index - start_index)
    {
        ROS_ERROR("Provided vector has wrong size (%i), required size is (%i). Cannot change goal position.", goal.size(), end_index-start_index);
        return false;
    }
    for (int i = start_index; i < end_index; i++)
    {
        transformation_systems_[i].setGoal(goal(i - start_index));
    }
    return true;
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::changeGoal(const double new_goal, const int index)
{
    if ((!params_.is_setup_) || (index < 0) || (index > params_.num_transformation_systems_))
    {
        return false;
    }
    transformation_systems_[index].setGoal(new_goal);
    return true;
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::changeStart(const VectorXd &start)
{
    if (!params_.is_setup_)
    {
        ROS_ERROR("DMP is not setup");
        return false;
    }
    if (start.size() != params_.num_transformation_systems_)
    {
        ROS_ERROR("Start vector has wrong size (%i), it should be %i.", start.size(), params_.num_transformation_systems_);
        return false;
    }
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        transformation_systems_[i].setStart(start(i));
        // set current state to start state (position and velocity)
        transformation_systems_[i].setState(start(i), 0.0);
    }
    params_.is_start_set_ = true;
    return true;
}

bool DynamicMovementPrimitive::getThetas(std::vector<VectorXd>& thetas)
{
    ROS_ASSERT(initialized_);

    thetas.clear();
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        int num_rfs;
        if (!transformation_systems_[i].lwr_model_.getNumRFS(num_rfs))
        {
            ROS_ERROR("Could not get number of receptive fields.");
            return false;
        }

        VectorXd theta_vector = VectorXd::Zero(num_rfs);
        if (!transformation_systems_[i].lwr_model_.getThetas(theta_vector))
        {
            ROS_ERROR("Could not retrieve thetas from transformation system %i.",i);
            return false;
        }
        thetas.push_back(theta_vector);
    }
    return true;
}

bool DynamicMovementPrimitive::setThetas(const std::vector<VectorXd>& thetas)
{
    ROS_ASSERT(initialized_);
    ROS_ASSERT(static_cast<int>(thetas.size()) == params_.num_transformation_systems_);
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        if (!transformation_systems_[i].lwr_model_.setThetas(thetas[i]))
        {
            ROS_ERROR("Could not set thetas of transformation system %i.",i);
            return false;
        }
    }
    return true;
}

bool DynamicMovementPrimitive::getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths, std::vector<Eigen::VectorXd>& centers)
{
    ROS_ASSERT(initialized_);

    widths.clear();
    centers.clear();

    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        int num_rfs;
        if (!transformation_systems_[i].lwr_model_.getNumRFS(num_rfs))
        {
            ROS_ERROR("Could not get number of receptive fields.");
            return false;
        }

        VectorXd centers_vector = VectorXd::Zero(num_rfs);
        VectorXd widths_vector = VectorXd::Zero(num_rfs);
        if (!transformation_systems_[i].lwr_model_.getWidthsAndCenters(widths_vector , centers_vector))
        {
            ROS_ERROR("Could not retrieve thetas from transformation system %i.",i);
            return false;
        }
        widths.push_back(widths_vector);
        centers.push_back(centers_vector);
    }
    return true;
}

bool DynamicMovementPrimitive::getWidthsAndCenters(const int trans_system_index, VectorXd& widths, VectorXd& centers)
{
    ROS_ASSERT(initialized_);

    int num_rfs;
    ROS_ASSERT_FUNC(!getNumRFS(trans_system_index, num_rfs));

    ROS_ASSERT(widths.size() == num_rfs);
    ROS_ASSERT(centers.size() == num_rfs);

    if (!transformation_systems_[trans_system_index].lwr_model_.getWidthsAndCenters(widths, centers))
    {
        ROS_ERROR("Could not get widths and centers of transformation system %i.", trans_system_index);
        return false;
    }
    return true;
}

bool DynamicMovementPrimitive::getBasisFunctions(const int num_time_steps, std::vector<MatrixXd>& basis_functions)
{
    ROS_ASSERT(initialized_);

    basis_functions.clear();
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        int num_rfs;
        if (!getNumRFS(i, num_rfs))
        {
            return false;
        }
        MatrixXd basis_function_matrix = MatrixXd::Zero(num_time_steps, num_rfs);
        VectorXd x_input_vector = VectorXd::Zero(num_time_steps);
        double dx = static_cast<double> (1.0) / static_cast<double> (num_time_steps - 1);
        x_input_vector(0) = 0.0;
        for (int j = 1; j < num_time_steps; j++)
        {
            x_input_vector(j) = x_input_vector(j - 1) + dx;
        }
        if (!transformation_systems_[i].lwr_model_.generateBasisFunctionMatrix(x_input_vector, basis_function_matrix))
        {
            ROS_ERROR("LWR basis function generation failed!");
            return false;
        }
        basis_functions.push_back(basis_function_matrix);
    }
    return true;
}

bool DynamicMovementPrimitive::getCanonicalSystem(const int num_time_steps, VectorXd& can_system_vector)
{
    ROS_ASSERT(can_system_vector.size() == num_time_steps);
    ROS_ASSERT(num_time_steps > 0);

    double dt = params_.tau_ / static_cast<double> (num_time_steps - 1);
    double time = 0;

    can_system_vector(0) = 1;
    for (int j = 1; j < num_time_steps; j++)
    {
        integrateCanonicalSystem(can_system_vector(j), time);
        time += dt;
    }
    return true;
}

bool DynamicMovementPrimitive::getNumRFS(const int trans_id, int& num_rfs)
{
    ROS_ASSERT(initialized_);

    if ((trans_id < 0) || (trans_id >= params_.num_transformation_systems_))
    {
        ROS_ERROR("Could not get number of receptive fields, the transformation system id (%i) is invalid.", trans_id);
        return false;
    }

    return transformation_systems_[trans_id].lwr_model_.getNumRFS(num_rfs);
}

bool DynamicMovementPrimitive::getNumRFS(std::vector<int>& num_rfs)
{
    num_rfs.clear();
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        int tmp_num_rfs;
        if (!getNumRFS(i, tmp_num_rfs))
        {
            return false;
        }
        num_rfs.push_back(tmp_num_rfs);
    }
    return true;
}

bool DynamicMovementPrimitive::setDuration(const double movement_duration, const int sampling_frequency)
{
    if (!params_.is_setup_)
    {
        ROS_ERROR("DMP need to be setup first.");
        return false;
    }

    if (sampling_frequency <= 0)
    {
        ROS_ERROR("Sampling frequency %i [Hz] is not valid.", sampling_frequency);
        return false;
    }

    // TODO: cleanup this mess

    if (movement_duration <= 0.5)
    {
        ROS_ERROR("Movement duration (%f) is too small.", movement_duration);
        return false;
    }

    params_.tau_ = movement_duration;
    params_.delta_t_ = static_cast<double> (1.0) / static_cast<double> (sampling_frequency);
    return true;
}

bool DynamicMovementPrimitive::propagateFull(Trajectory& trajectory, const double sampling_duration, const int num_samples)
{
    if ((!params_.is_learned_) || (!params_.is_setup_) || (!params_.is_start_set_))
    {
        ROS_ERROR_COND(!params_.is_learned_,"DMP is not learned from demonstration.");
        ROS_ERROR_COND(!params_.is_setup_,"DMP with id %i is not setup. Need to set start, goal, and duration first.", item_id_);
        return false;
    }

    ROS_ASSERT(trajectory.getMaxDimension() >= params_.num_transformation_systems_ * POS_VEL_ACC);
    ROS_ASSERT(trajectory.getMaxLength() > num_samples);

    double special_sampling_frequency = static_cast<double> (num_samples) / (sampling_duration);
    if (!trajectory.setSamplingFrequency(special_sampling_frequency))
    {
        ROS_ERROR("Could not set sampling frequency.");
        return false;
    }

    VectorXd desired_coordinates = VectorXd::Zero(params_.num_transformation_systems_ * POS_VEL_ACC);
    bool movement_finished = false;
    while (!movement_finished)
    {
        if (!propagateStep(desired_coordinates, movement_finished, sampling_duration, num_samples))
        {
            ROS_ERROR("Could not propagate dmp.");
            return false;
        }

        if (!trajectory.add(desired_coordinates))
        {
            ROS_ERROR("Could not add point to trajectory.");
            return false;
        }
    }

    return true;
}

// REAL-TIME REQUIREMENTS
bool DynamicMovementPrimitive::propagateStep(VectorXd &desired_coordinates, bool &movement_finished)
{
    return propagateStep(desired_coordinates, movement_finished, params_.tau_, static_cast<int> (params_.tau_ / params_.delta_t_));
}
// REAL-TIME REQUIREMENTS
// TODO: eliminate all the ROS_ERROR outputs...
bool DynamicMovementPrimitive::propagateStep(VectorXd &desired_coordinates, bool &movement_finished, const double sampling_duration, const int num_samples)
{
    if ((!params_.is_learned_) || (!params_.is_setup_) || (!params_.is_start_set_))
    {
        ROS_ERROR_COND(!params_.is_learned_,"DMP is not learned from demonstration.");
        ROS_ERROR_COND(!params_.is_setup_,"DMP with id %i is not setup. Need to set start, goal, and duration first.", item_id_);
        movement_finished = true;
        return false;
    }

    if (desired_coordinates.size() != params_.num_transformation_systems_ * POS_VEL_ACC)
    {
        ROS_ERROR("Number of desired coordinates (%i) is not correct, it should be %i. TODO: REMOVE ME !!", desired_coordinates.size(), params_.num_transformation_systems_ * POS_VEL_ACC);
        movement_finished = true;
        return false;
    }

    if (num_samples <= 0)
    {
        ROS_ERROR("Number of samples (%i) is not valid. TODO: REMOVE ME !!", num_samples);
    }
    double dt_total = sampling_duration / static_cast<double> (num_samples);
    double dt_threshold = static_cast<double> (1.0) / DEFAULT_SAMPLING_FREQUENCY;
    int num_iteration = ceil(dt_total / dt_threshold);

    // integrate the system, make sure that all internal variables are set properly
    if (!integrate(dt_total, num_iteration))
    {
        ROS_ERROR("Problem while integrating the dynamical system.");
        movement_finished = true;
        return false;
    }
    params_.num_samples_++;

    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        desired_coordinates(i * POS_VEL_ACC + _POS_ - 1) = transformation_systems_[i].y_;
        desired_coordinates(i * POS_VEL_ACC + _VEL_ - 1) = transformation_systems_[i].yd_;
        desired_coordinates(i * POS_VEL_ACC + _ACC_ - 1) = transformation_systems_[i].ydd_;
    }

    if (dmp_debug_.isInitialized())
    {
        int index = 0;
        debug_trajectory_point_(index) = canonical_system_.x;
        index++;
        debug_trajectory_point_(index) = canonical_system_.time;
        index++;
        debug_trajectory_point_(index) = 0;
        index++;
        for (int i = NUM_DEBUG_CANONICAL_SYSTEM_VALUES; i < params_.num_transformation_systems_ + NUM_DEBUG_CANONICAL_SYSTEM_VALUES; ++i)
        {
            debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].y_;
            index++;
            debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].yd_;
            index++;
            debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].ydd_;
            index++;
            debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].goal_;
            index++;
            debug_trajectory_point_(index) = transformation_systems_[i - NUM_DEBUG_CANONICAL_SYSTEM_VALUES].f_;
            index++;
            debug_trajectory_point_(index) = 0;
            index++;
        }
        if (!dmp_debug_.add(debug_trajectory_point_))
        {
            ROS_ERROR("Could not add trajectory point to (run) debug object. TODO: REMOVE ME !!");
            params_.is_learned_ = false;
            return params_.is_learned_;
        }
    }

    // check whether movement has finished...
    if (params_.num_samples_ >= num_samples)
    {
        params_.is_setup_ = false;
        params_.is_start_set_ = false;
        movement_finished = true;
        return true;
    }
    else
    {
        movement_finished = false;
    }
    return true;
}

std::string DynamicMovementPrimitive::getFileName(const int trial_id)
{
    policy_improvement_utilities::appendTrailingSlash(library_directory_name_);
    std::string file_name = library_directory_name_ + std::string(DYNAMIC_MOVEMENT_PRIMITIVE_FILE_NAME + std::string("_") + policy_improvement_utilities::getString(item_id_));
    policy_improvement_utilities::appendTrailingSlash(file_name);
    file_name.append(std::string(DYNAMIC_MOVEMENT_PRIMITIVE_FILE_NAME) + std::string("_") + policy_improvement_utilities::getString(item_id_));
    file_name.append(std::string("_trial_") + policy_improvement_utilities::getString(trial_id) + std::string(".bag"));
    return file_name;
}

bool DynamicMovementPrimitive::writeToDisc(const int trial_id)
{
    std::string abs_bagfile_name = getFileName(trial_id);
    return writeToDisc(abs_bagfile_name);
}

bool DynamicMovementPrimitive::writeToDisc(const std::string& abs_bagfile_name)
{
    ROS_ASSERT(initialized_);
    try
    {
        rosbag::Bag bag(abs_bagfile_name, rosbag::bagmode::Write);
        dmp_motion_generation::DynamicMovementPrimitive dynamic_movement_primitive;
        if (!writeToMessage(dynamic_movement_primitive))
        {
            ROS_ERROR("Could not store DMP into a message.");
            return false;
        }
        bag.write(DYNAMIC_MOVEMENT_PRIMITIVE_TOPIC_NAME, ros::Time::now(), dynamic_movement_primitive);
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s: %s", abs_bagfile_name.c_str(), ex.what());
        return false;
    }
    return true;
}

bool DynamicMovementPrimitive::writeDebugTrajectory()
{
    if (dmp_debug_.isInitialized())
    {
        return dmp_debug_.writeToCLMCFile(params_.version_);
    }
    return true;
}

bool DynamicMovementPrimitive::writeDebugTrajectory(const std::string& filename)
{
    if (dmp_debug_.isInitialized())
    {
        return dmp_debug_.writeToCLMCFile(filename);
    }
    return true;
}

bool DynamicMovementPrimitive::readFromDisc(const std::string& library_directory_name, const int dmp_id, const int trial_id)
{
    library_directory_name_ = library_directory_name;
    item_id_ = dmp_id;
    std::string abs_bagfile_name = getFileName(trial_id);
    return readFromDisc(abs_bagfile_name);
}

bool DynamicMovementPrimitive::readFromDisc(const std::string& abs_bagfile_name)
{
    try
    {
        rosbag::Bag bag(abs_bagfile_name, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(DYNAMIC_MOVEMENT_PRIMITIVE_TOPIC_NAME));
        int message_counter = 0;
        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
        {
            if (message_counter > 1)
            {
                ROS_ERROR("Bagfile should only contain a single DMP.");
                return false;
            }
            message_counter++;

            dmp_motion_generation::DynamicMovementPrimitive::ConstPtr dynamic_movement_primitive = msg.instantiate<dmp_motion_generation::DynamicMovementPrimitive> ();
            ROS_ASSERT(dynamic_movement_primitive != NULL);
            if (!initFromMessage(*dynamic_movement_primitive))
            {
                ROS_ERROR("Could not get and initialize the DMP.");
                return false;
            }
        }
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s: %s", abs_bagfile_name.c_str(), ex.what());
        return false;
    }
    return true;
}

void DynamicMovementPrimitive::print()
{
    ROS_INFO_STREAM(getInfoString());
}

std::string DynamicMovementPrimitive::getInfoString()
{
    std::string info("");
    std::stringstream ss;

    ss << item_id_;
    info.append(std::string("id: ") + ss.str());
    ss.str("");
    ss.clear();

    info.append(std::string("\t"));
    info.append(std::string("name: ") + item_name_);

    info.append(std::string("\n\t"));
    info.append(std::string("description: ") + description_);

    info.append(std::string("\n\t"));
    info.append(std::string("initialized: "));
    if (initialized_)
    {
        info.append(std::string("true "));
    }
    else
    {
        info.append(std::string("false"));
    }

    info.append(std::string("  learned: "));
    if (params_.is_learned_)
    {
        info.append(std::string("true "));
    }
    else
    {
        info.append(std::string("false"));
    }

    info.append(std::string("  setup: "));
    if (params_.is_setup_)
    {
        info.append(std::string("true "));
    }
    else
    {
        info.append(std::string("false"));
    }

    info.append(std::string("  is start set: "));
    if (params_.is_start_set_)
    {
        info.append(std::string("true "));
    }
    else
    {
        info.append(std::string("false"));
    }

    info.append(std::string("\n") + params_.getInfoString());
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        info.append(std::string("\n\t"));
        info.append(transformation_systems_[i].getInfoString());
    }

    return info;
}

bool DynamicMovementPrimitive::integrateAndFit()
{

    // double log_x = -log(canonical_system_.x) /* * params_.tau_ */ / params_.alpha_x_;
    // trajectory_target_function_input_.push_back(log_x);

    trajectory_target_function_input_.push_back(canonical_system_.x);

    switch (params_.version_)
    {
        case Parameters::ICRA2009:
        {
            for (int i = 0; i < params_.num_transformation_systems_; i++)
            {

                transformation_systems_[i].ft_ = ((transformation_systems_[i].tdd_ * pow(params_.tau_, 2) + params_.d_gain_ * transformation_systems_[i].td_
                        * params_.tau_) / params_.k_gain_) - (transformation_systems_[i].goal_ - transformation_systems_[i].t_)
                        + (transformation_systems_[i].goal_ - transformation_systems_[i].y0_) * canonical_system_.x;

                transformation_systems_[i].ft_ /= canonical_system_.x;

                // the nonlinearity is computed by LWR (later)
                transformation_systems_[i].trajectory_target_.push_back(transformation_systems_[i].ft_);

                // transformation state derivatives (make use of target knowledge)
                transformation_systems_[i].zd_ = (params_.k_gain_ * (transformation_systems_[i].goal_ - transformation_systems_[i].y_) - params_.d_gain_
                        * transformation_systems_[i].z_ - params_.k_gain_ * (transformation_systems_[i].goal_ - transformation_systems_[i].y0_)
                        * canonical_system_.x + params_.k_gain_ * transformation_systems_[i].ft_) * (static_cast<double> (1.0) / params_.tau_);

                // integrate all systems
                transformation_systems_[i].z_ += transformation_systems_[i].zd_ * params_.delta_t_;
                transformation_systems_[i].y_ += transformation_systems_[i].yd_ * params_.delta_t_;
            }
            break;
        }
        case Parameters::NIPS2003:
        {
            //            double amp = 1.0;
            //            for (int i = 0; i < params_.num_transformation_systems_; i++)
            //            {
            //
            //                if (fabs(transformation_systems_[i].delta_goal_fit_ - NOT_ASSIGNED) < 0.0000001)
            //                {
            //                    transformation_systems_[i].scale_ = 1.0; // this is the first time of fitting
            //                    transformation_systems_[i].delta_goal_fit_ = transformation_systems_[i].goal_ - transformation_systems_[i].y0_;
            //                }
            //                amp = transformation_systems_[i].scale_;
            //
            //                transformation_systems_[i].ft_ = ((transformation_systems_[i].tdd_ / pow(params_.tau_, 2)) - params_.alpha_z_ * (params_.beta_z_
            //                        * (transformation_systems_[i].s_ - transformation_systems_[i].t_) - (transformation_systems_[i].td_ / params_.tau_))) / amp;
            //
            //                // the nonlinearity is computed by LWR (later)
            //                transformation_systems_[i].trajectory_target_.push_back(transformation_systems_[i].ft_);
            //
            //                // transformation state derivatives (make use of target knowledge)
            //                transformation_systems_[i].zd_ = (params_.alpha_z_ * (params_.beta_z_ * (transformation_systems_[i].s_ - transformation_systems_[i].t_)
            //                        - (transformation_systems_[i].td_ / params_.tau_)) + (amp * transformation_systems_[i].ft_)) * params_.tau_;
            //
            //                transformation_systems_[i].yd_ = transformation_systems_[i].td_ * params_.tau_;
            //                transformation_systems_[i].sd_ = params_.alpha_s_ * (transformation_systems_[i].goal_ - transformation_systems_[i].s_) * params_.tau_;
            //                transformation_systems_[i].ydd_ = transformation_systems_[i].zd_ * params_.tau_;
            //
            //                // integrate all systems
            //                transformation_systems_[i].z_ += transformation_systems_[i].zd_ * params_.delta_t_;
            //                transformation_systems_[i].y_ += transformation_systems_[i].yd_ * params_.delta_t_;
            //
            //                transformation_systems_[i].s_ += transformation_systems_[i].sd_ * params_.delta_t_;
            //
            //            }

            break;
        }
        default:
            ROS_ERROR("DMP version (%i) not valid.", params_.version_);
            return false;
    }

    // canonical system
    integrateCanonicalSystem(canonical_system_.x, canonical_system_.time);
    canonical_system_.time += params_.delta_t_;

    return true;
}

bool DynamicMovementPrimitive::learnTransformationTarget()
{

    Eigen::Map<VectorXd> input = VectorXd::Map(&trajectory_target_function_input_[0], trajectory_target_function_input_.size());

    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        if (trajectory_target_function_input_.size() != transformation_systems_[i].trajectory_target_.size())
        {
            ROS_ERROR("Traget trajectory of transformation system %i has different size than input trajectory.", i);
            return false;
        }
        Eigen::Map<VectorXd> target = VectorXd::Map(&transformation_systems_[i].trajectory_target_[0], transformation_systems_[i].trajectory_target_.size());
        if (!transformation_systems_[i].lwr_model_.learnWeights(input, target))
        {
            ROS_ERROR("Could not learn weights of transformation system %i.", i);
            return false;
        }
    }

    // update debug data
    if (dmp_debug_.isInitialized())
    {
        int length_index = 0;
        for (std::vector<double>::iterator vi = trajectory_target_function_input_.begin(); vi != trajectory_target_function_input_.end(); vi++)
        {
            for (int i = 0; i < params_.num_transformation_systems_; i++)
            {
                double prediction = 0;
                if (!transformation_systems_[i].lwr_model_.predict(*vi, prediction))
                {
                    ROS_ERROR("Could not predict output.");
                    return false;
                }
                int dimension_index = NUM_DEBUG_CANONICAL_SYSTEM_VALUES + 4 + i * 6; // 6 is the index into f
                if (!dmp_debug_.update(length_index, dimension_index, prediction))
                {
                    ROS_ERROR("Could not update debug trajectory.");
                    return false;
                }
            }
            length_index++;
        }
    }

    // compute mean squared error
    for (int i = 0; i < params_.num_transformation_systems_; ++i)
    {
        transformation_systems_[i].computeMSE();
    }

    return true;
}

//inline bool DynamicMovementPrimitive::computeAcceleration(double& acceleration, const int index, const double nonlinear_term)
//{
//    switch (params_.version_)
//    {
//        case Parameters::NIPS2003:
//            acceleration = params_.k_gain_ * (transformation_systems_[index].goal_ - transformation_systems_[index].y_) - (params_.d_gain_
//                    * transformation_systems_[index].yd_) + (transformation_systems_[index].goal_ - transformation_systems_[index].y0_) * nonlinear_term;
//            break;
//        case Parameters::ICRA2009:
//            acceleration = (params_.k_gain_ * (transformation_systems_[index].goal_ - transformation_systems_[index].y_) - params_.d_gain_
//                    * transformation_systems_[index].z_ - params_.k_gain_ * (transformation_systems_[index].goal_ - transformation_systems_[index].y0_)
//                    * canonical_system_.x + params_.k_gain_ * nonlinear_term) * (static_cast<double> (1.0) / params_.tau_);
//            break;
//        default:
//            ROS_ERROR("DMP version (%i) is invalid.", params_.version_);
//            return false;
//    }
//    return true;
//}
//inline bool DynamicMovementPrimitive::computeTarget(double& target, const int index)
//{
//    switch (params_.version_)
//    {
//        case Parameters::NIPS2003:
//            //            target = (-params_.k_gain_ * (transformation_systems_[index].goal_ - transformation_systems_[index].t_) + (params_.d_gain_
//            //                    * transformation_systems_[index].td_) + (transformation_systems_[index].tdd_ * params_.tau_)) / (transformation_systems_[index].goal_
//            //                    - transformation_systems_[index].y0_);
//            target = (-params_.k_gain_ * (transformation_systems_[index].goal_ - transformation_systems_[index].t_) + (params_.d_gain_
//                    * transformation_systems_[index].td_) + (transformation_systems_[index].tdd_ * params_.tau_)) / (transformation_systems_[index].goal_
//                    - transformation_systems_[index].y0_);
//            break;
//        case Parameters::ICRA2009:
//            target = ((transformation_systems_[index].tdd_ * pow(params_.tau_, 2) + params_.d_gain_ * transformation_systems_[index].td_ * params_.tau_)
//                    / params_.k_gain_) - (transformation_systems_[index].goal_ - transformation_systems_[index].t_) + (transformation_systems_[index].goal_
//                    - transformation_systems_[index].y0_) * canonical_system_.x;
//            //            target = ((transformation_systems_[index].tdd_ * params_.tau_) + (params_.d_gain_ * transformation_systems_[index].td_)) / params_.k_gain_
//            //                    - (transformation_systems_[index].goal_ - transformation_systems_[index].y_) + (transformation_systems_[index].goal_
//            //                    - transformation_systems_[index].y0_) * canonical_system_.x;
//            break;
//        default:
//            ROS_ERROR("DMP version (%i) is invalid.", params_.version_);
//            return false;
//    }
//    return true;
//}

inline bool DynamicMovementPrimitive::integrate(const double dt_total, const int num_iteration)
{

    double dt = dt_total / static_cast<double> (num_iteration);

    // double log_x = -log(canonical_system_.x) /* * params_.tau_*/ / params_.alpha_x_;

    for (int n = 0; n < num_iteration; n++)
    {

        for (int i = 0; i < params_.num_transformation_systems_; i++)
        {
            // compute nonlinearity using LWR
            double prediction = 0;
            if (!transformation_systems_[i].lwr_model_.predict(canonical_system_.x, prediction))
            {
                ROS_ERROR("Could not predict output.");
                return false;
            }

            switch (params_.version_)
            {
                case Parameters::ICRA2009:

                    transformation_systems_[i].f_ = prediction * canonical_system_.x;

                    transformation_systems_[i].zd_ = (params_.k_gain_ * (transformation_systems_[i].goal_ - transformation_systems_[i].y_) - params_.d_gain_
                            * transformation_systems_[i].z_ - params_.k_gain_ * (transformation_systems_[i].goal_ - transformation_systems_[i].y0_)
                            * canonical_system_.x + params_.k_gain_ * transformation_systems_[i].f_) * (static_cast<double> (1.0) / params_.tau_);

                    transformation_systems_[i].yd_ = transformation_systems_[i].z_ * (static_cast<double> (1.0) / params_.tau_);
                    transformation_systems_[i].ydd_ = transformation_systems_[i].zd_ * (static_cast<double> (1.0) / params_.tau_);

                    transformation_systems_[i].z_ += transformation_systems_[i].zd_ * dt; //* params_.delta_t_;
                    transformation_systems_[i].y_ += transformation_systems_[i].yd_ * dt; //* params_.delta_t_;

                    break;

                case Parameters::NIPS2003:
                    //
                    //                    transformation_systems_[i].f_ = prediction * transformation_systems_[i].scale_ * params_.tau_;
                    //
                    //                    transformation_systems_[i].zd_ = (params_.alpha_z_ * (params_.beta_z_ * (transformation_systems_[i].s_ - transformation_systems_[i].y_)
                    //                            - transformation_systems_[i].z_) + transformation_systems_[i].scale_ * prediction) * params_.tau_;
                    //
                    //                    transformation_systems_[i].yd_ = transformation_systems_[i].z_ * params_.tau_;
                    //                    transformation_systems_[i].sd_ = params_.alpha_s_ * (transformation_systems_[i].goal_ - transformation_systems_[i].s_) * params_.tau_;
                    //
                    //                    transformation_systems_[i].ydd_ = transformation_systems_[i].zd_ * params_.tau_;
                    //
                    //                    transformation_systems_[i].z_ += transformation_systems_[i].zd_ * dt;
                    //                    transformation_systems_[i].y_ += transformation_systems_[i].yd_ * dt;
                    //                    transformation_systems_[i].s_ += transformation_systems_[i].sd_ * dt;

                    break;

                default:
                    ROS_ERROR("DMP version (%i) not valid.", params_.version_);
                    return false;
            }

        }

        // canonical system
        integrateCanonicalSystem(canonical_system_.x, canonical_system_.time);
        canonical_system_.time += dt; //+= params_.delta_t_;
    }

    return true;
}

//inline bool DynamicMovementPrimitive::integrate(const VectorXd &current_position, const double sampling_duration, const int num_samples)
//{
//
//    double log_x = -log(canonical_system_.x) / params_.alpha_x_;
//    for (int i = 0; i < params_.num_transformation_systems_; i++)
//    {
//        // compute nonlinearity using LWR
//        double prediction = 0;
//        if (!transformation_systems_[i].lwr_model_.predict(log_x, prediction))
//        {
//            ROS_ERROR("Could not predict output.");
//            return false;
//        }
//        transformation_systems_[i].f_ = prediction;
//
//        // transformation systems
//        transformation_systems_[i].zd_ = (params_.k_gain_ * (transformation_systems_[i].goal_ - transformation_systems_[i].y_) - params_.d_gain_
//                * transformation_systems_[i].z_ - params_.k_gain_ * (transformation_systems_[i].goal_ - transformation_systems_[i].y0_) * canonical_system_.x
//                + params_.k_gain_ * transformation_systems_[i].f_) * (static_cast<double> (1.0) / params_.tau_);
//
//        transformation_systems_[i].yd_ = (transformation_systems_[i].z_ * (static_cast<double> (1.0) / params_.tau_));
//        transformation_systems_[i].ydd_ = transformation_systems_[i].zd_ * (static_cast<double> (1.0) / params_.tau_);
//
//        transformation_systems_[i].z_ += transformation_systems_[i].zd_ * params_.delta_t_;
//        transformation_systems_[i].y_ += transformation_systems_[i].yd_ * params_.delta_t_;
//
//    }
//
//    // canonical system
//    integrateCanonicalSystem(canonical_system_.x, canonical_system_.time);
//    canonical_system_.time += params_.delta_t_;
//
//    return true;
//}

inline void DynamicMovementPrimitive::integrateCanonicalSystem(double& canonical_system_x, const double canonical_system_time) const
{
    canonical_system_x = exp(-(params_.alpha_x_ / params_.tau_) * canonical_system_time);
}

//void DMPMotionUnit::computeForceFieldTerm(const double* obstacle_position, const double* obstacle_velocity, double* repulsive_force)
//{
//
//	double obstacle_to_current_position[3];
//  for(int i=0; i<3; i++) {
//  	obstacle_to_current_position[i] = transformation_systems_[i].y_ - obstacle_position[i];
//	}
//
//  double norm_x;
//  norm_x = sqrt(pow(obstacle_to_current_position[0],2)+pow(obstacle_to_current_position[1],2)+pow(obstacle_to_current_position[2],2));
//
//	double force_direction[3];
//	for(i=0; i<3; i++) {
//		force_direction[i] = obstacle_to_current_position[i] / norm_x;
//	}
//
//	for(i=0; i<3; i++) {
//		force_direction[i] *= -1.0;
//	}
//
//	for(i=0; i<3; i++) {
//		// m_current_rep_force[i] += force_direction[i] * (static_cast<double>(1)/pow(norm_x,2) * 0.0001);
//		repulsive_force[i] = force_direction[i] * (static_cast<double>(1)/pow(norm_x,2) * 0.0001);
//	}
//
//}

bool DynamicMovementPrimitive::isIncreasing(int transformation_system_index, bool &is_increasing)
{
    if ((transformation_system_index >= 0) && (transformation_system_index < params_.num_transformation_systems_) && params_.is_learned_)
    {
        return (transformation_systems_[transformation_system_index].initial_y0_ >= transformation_systems_[transformation_system_index].initial_goal_);
    }
    return false;
}

bool DynamicMovementPrimitive::getCanonicalSystemState(double &canonical_system_value, double &canonical_system_time)
{
    ROS_ASSERT(initialized_);
    canonical_system_value = canonical_system_.x;
    canonical_system_time = canonical_system_.time;
    return true;
}

bool DynamicMovementPrimitive::getCanonicalSystemValue(const double canonical_system_input, double& canonical_system_value)
{
    canonical_system_value = -log(canonical_system_input) / params_.alpha_x_;
    return true;
}

// WARNING: need to run in real-time
void DynamicMovementPrimitive::computeObstAvoidanceTerm(const VectorXd &obstacle_position, const VectorXd &obstacle_velocity, VectorXd &repulsive_force)
{
    // WARNING: not tested yet, handle with care !!!

    // position
    Vector3d obstacle_to_current_position;
    for (int i = 0; i < N_CART; i++)
    {
        obstacle_to_current_position(i) = transformation_systems_[i].y_ - obstacle_position(i);
    }

    double norm_x = obstacle_to_current_position.norm();

    obstacle_to_current_position = obstacle_to_current_position / norm_x;

    Vector3d negative_obstacle_to_current_position = -obstacle_to_current_position;

    Vector3d obstacle_to_current_velocity;
    for (int i = 0; i < N_CART; i++)
    {
        obstacle_to_current_velocity(i) = transformation_systems_[i].yd_ - obstacle_velocity(i);
    }

    Vector3d normalized_obstacle_to_current_velocity = obstacle_to_current_velocity;
    normalized_obstacle_to_current_velocity.normalize();

    Vector3d rotation_axis = normalized_obstacle_to_current_velocity.cross(negative_obstacle_to_current_position);
    rotation_axis.normalize();

    Matrix3d rotation_matrix;
    MathHelper::getRotationMatrix(rotation_matrix, rotation_axis, -static_cast<double> (M_PI / 2.0));

    Vector3d negative_normalized_obstacle_to_current_velocity = -normalized_obstacle_to_current_velocity;

    double phi = acos(negative_normalized_obstacle_to_current_velocity.dot(obstacle_to_current_position));

    double gamma_ = 1000.0;
    double delta_ = 30.0;

    double dphi = gamma_ * phi * exp(-(delta_ / static_cast<double> (M_PI)) * phi);

    Vector3d obstacle_avoidance_velocity_change;
    obstacle_avoidance_velocity_change = (rotation_matrix * obstacle_to_current_velocity) * dphi;

    repulsive_force = obstacle_avoidance_velocity_change;
}

double DynamicMovementPrimitive::getProgress() const
{
    double progress;
    if (canonical_system_.x < 1.0e-8)
    {
        progress = 1.0;
    }
    else if (params_.alpha_x_ > 1.0e-8)
    {
        progress = -log(canonical_system_.x) / params_.alpha_x_;
    }
    else
    {
        progress = 0.0;
    }
    return progress;
}

}
