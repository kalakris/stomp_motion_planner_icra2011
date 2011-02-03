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

// system includes

// ros includes
#include <ros/ros.h>
#include <ros/package.h>

#include <boost/filesystem.hpp>

#include <policy_improvement_utilities/param_server.h>

#include <Eigen/Eigen>
// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

// local includes
#include <dmp_motion_generation/dmp_debug.h>

namespace dmp
{

static const char* TRAJECTORY_PREFIX = ".traj";

DMPDebug::DMPDebug(ros::NodeHandle& node_handle) :
    initialized_(false), debug_directory_name_(std::string("")), file_name_trunk_(std::string("tmp")), iteration_counter_(0), dmp_id_(0),
    trajectory_node_handle_(node_handle, "trajectory"), dmp_node_handle_(node_handle, "dmp"), debug_trajectory_(trajectory_node_handle_)
{
}

DMPDebug::~DMPDebug()
{
}

bool DMPDebug::initialize(const int dmp_id, const std::string& variable_names_key_word, const int num_transformation_systems,
                          const int num_debug_traces_per_trans_sys, const int num_extra_debug_traces, const double sampling_frequency)
{
    // ROS_WARN_COND(initialized_, "Debug object already initialized. Reinitializing with new parameters.");
    initialized_ = false;

    if ((num_transformation_systems <= 0) || (sampling_frequency <= 0.0))
    {
        ROS_WARN_COND(num_transformation_systems <= 0, "Number of transformation system is %i.", num_transformation_systems);
        ROS_WARN_COND(sampling_frequency <= 0, "Sampling frequency is %f.", sampling_frequency);
        ROS_WARN("No debug information will be stored...");
        initialized_ = false;
        return initialized_;
    }

    // sets the debug directory name and sets initialized to false if the parameter does not exist on the param server
    if (!setDebugDirectory())
    {
        ROS_WARN("Could not set debug directory name. Initialization failed, not storing debug information.");
        initialized_ = false;
        return initialized_;
    }

    boost::filesystem::create_directories(debug_directory_name_);

    dmp_id_ = dmp_id;
    file_name_trunk_.assign(variable_names_key_word);

    int trajectory_dimension = num_extra_debug_traces + num_debug_traces_per_trans_sys * num_transformation_systems;
    initialized_ = debug_trajectory_.initialize(variable_names_key_word, trajectory_dimension, sampling_frequency, num_debug_traces_per_trans_sys,
                                                num_extra_debug_traces, dmp::MAX_TRAJECTORY_LENGTH, dmp::MAX_TRAJECTORY_DIMENSION,
                                                Trajectory::eDEBUG_DATA_TRACE);

    return initialized_;
}

bool DMPDebug::setDebugDirectory()
{

    std::string package_name;
    if (!policy_improvement_utilities::read(dmp_node_handle_, std::string("package_name"), package_name))
    {
        initialized_ = false;
        return initialized_;
    }
    std::string package_path = ros::package::getPath(package_name);
    policy_improvement_utilities::appendTrailingSlash(package_path);

    std::string library_base_directory_name;
    if (!policy_improvement_utilities::read(dmp_node_handle_, std::string("library_base_directory_name"), library_base_directory_name))
    {
        initialized_ = false;
        return initialized_;
    }
    policy_improvement_utilities::appendTrailingSlash(library_base_directory_name);

    std::string sub_directory_name;
    if (!policy_improvement_utilities::read(dmp_node_handle_, std::string("debug_directory_name"), sub_directory_name))
    {
        initialized_ = false;
        return initialized_;
    }

    policy_improvement_utilities::appendTrailingSlash(sub_directory_name);
    debug_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    return true;
}

void DMPDebug::clear()
{
    if (initialized_)
    {
        debug_trajectory_.clear();
    }
}

// REAL-TIME REQUIREMENTS
bool DMPDebug::add(const VectorXd &debug_trajectory_point)
{
    if (initialized_)
    {
        if (debug_trajectory_point.size() != debug_trajectory_.getDimension())
        {
            return false;
        }
        debug_trajectory_.add(debug_trajectory_point);
    }
    return true;
}

bool DMPDebug::update(const int length_index, const int dimension_index, const double value)
{
    if (!initialized_)
    {
        ROS_ERROR("Debug trajectory is not initialized.");
        return false;
    }

    return debug_trajectory_.update(length_index, dimension_index, value);
}

bool DMPDebug::writeToCLMCFile(Parameters::Version version)
{
    if (initialized_)
    {
        std::string version_string;
        if(version == Parameters::ICRA2009)
        {
            version_string.assign("icra_2009");
        }
        else
        {
            version_string.assign("nips_2003");
        }

        std::stringstream ss_dmp_id;
        ss_dmp_id << dmp_id_;
        std::stringstream ss_iteration_counter;
        ss_iteration_counter << iteration_counter_;
        std::string file_name = debug_directory_name_ + file_name_trunk_ + std::string("_") + version_string + std::string("_") + ss_dmp_id.str() + std::string("_")
                + ss_iteration_counter.str() + std::string(TRAJECTORY_PREFIX);

        if (debug_trajectory_.writeToCLMCFile(file_name))
        {
            iteration_counter_++;
            clear();
        }
        else
        {
            ROS_ERROR_STREAM("Could not write clmc file " << file_name << ".");
            return false;
        }
    }
    else
    {
        ROS_WARN("Skipping writing to file since debug object is uninitialized.");
    }
    return true;
}

bool DMPDebug::writeToCLMCFile(const std::string& file_name)
{
    if (initialized_)
    {
        if (debug_trajectory_.writeToCLMCFile(file_name))
        {
            clear();
        }
        else
        {
            ROS_ERROR_STREAM("Could not write clmc file " << file_name << ".");
            return false;
        }
    }
    else
    {
        ROS_WARN("Skipping writing to file since debug object is uninitialized.");
    }
    return true;
}


}
