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
#include <sstream>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>

#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/assert.h>

// local includes
#include <dmp_motion_generation/parameters.h>

namespace dmp
{

Parameters::Parameters(ros::NodeHandle& node_handle) :
    is_learned_(false), is_setup_(false), is_start_set_(false), num_transformation_systems_(0), tau_(0), initial_tau_(0), delta_t_(0), initial_delta_t_(0),
            library_directory_name_(std::string("")), teaching_duration_(-1), execution_duration_(-1),
            can_sys_cutoff_(0), alpha_x_(-1), k_gain_(-1), d_gain_(-1), /* alpha_z_(0.0), beta_z_(0.0),
            alpha_s_(0.0), */ num_samples_(0), node_handle_(node_handle, "dmp")
{
}

Parameters::~Parameters()
{
}

bool Parameters::initialize(const Parameters::Version version)
{

    std::string package_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("package_name"), package_name));
    std::string package_path = ros::package::getPath(package_name);
    policy_improvement_utilities::appendTrailingSlash(package_path);

    std::string library_base_directory_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("library_base_directory_name"), library_base_directory_name));
    policy_improvement_utilities::appendTrailingSlash(library_base_directory_name);

    std::string sub_directory_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("library_directory_name"), sub_directory_name));
    library_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    policy_improvement_utilities::appendTrailingSlash(library_directory_name_);

    // ROS_INFO("DMP Library directory is set to %s.", library_directory_name_.c_str());

    // TODO: think about whether teaching_duration should be read from file or "always" set
    // according to the length of the trajectory of which the dmp is learned
    // node_handle_.param("teaching_duration", teaching_duration_, 0.0);
    teaching_duration_ = 0.0;

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("execution_duration"), execution_duration_));
    if (execution_duration_ <= 0)
    {
        ROS_ERROR("Execution duration is invalid (%.1f sec).", execution_duration_);
        return false;
    }

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("can_sys_cutoff"), can_sys_cutoff_));
    if (can_sys_cutoff_ <= 0)
    {
        ROS_ERROR("Canonical system cutoff frequency is invalid (%.1f).", can_sys_cutoff_);
        return false;
    }

    switch (version)
    {
        case Parameters::ICRA2009:
            ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, std::string("k_gain_icra2009"), k_gain_));
            if (k_gain_ <= 0)
            {
                ROS_ERROR("Time constant K (icra2009) is invalid (%f).", k_gain_);
                return false;
            }
            d_gain_ = k_gain_ / 4.0;
            break;
        case Parameters::NIPS2003:
//            node_handle_.param("alpha_z_nips2003", alpha_z_, 0.0);
//            if (alpha_z_ <= 0)
//            {
//                ROS_ERROR("Time constant alpha_z (nips2003) is %f.", alpha_z_);
//                return false;
//            }
//            beta_z_ = alpha_z_ / 4.0;
//            alpha_s_ = alpha_z_ / 2.0;
            break;
        default:
            ROS_ERROR("DMP version is invalid (%i).", version);
            return false;
    }

    num_samples_ = 0;
    return true;
}

void Parameters::print()
{
    ROS_INFO_STREAM(getInfoString());
}

std::string Parameters::getInfoString()
{
    std::string info("");
    std::stringstream ss;

    int precision = 3;
    ss.precision(precision);
    ss << std::fixed;

    ss << teaching_duration_;
    info.append(std::string("\t"));
    info.append(std::string("teaching duration: ") + ss.str());
    ss.str("");
    ss.clear();

    ss << execution_duration_;
    info.append(std::string("  execution duration: ") + ss.str());
    ss.str("");
    ss.clear();

    info.append(std::string("\n\t"));
    info.append(std::string("canonical system parameters>      "));

    info.append(std::string("\n\t"));
    info.append(std::string("canonical system parameters>      "));

    ss << alpha_x_;
    info.append(std::string("  alpha_x: ") + ss.str());
    ss.str("");
    ss.clear();

    ss << can_sys_cutoff_;
    info.append(std::string("  canonical system cutoff: ") + ss.str());
    ss.str("");
    ss.clear();

    info.append(std::string("\n\t"));
    info.append(std::string("transformation system parameters> "));

    ss << can_sys_cutoff_;
    info.append(std::string("  canonical system cutoff: ") + ss.str());
    ss.str("");
    ss.clear();

    ss << k_gain_;
    info.append(std::string("  K: ") + ss.str());
    ss.str("");
    ss.clear();

    ss << d_gain_;
    info.append(std::string("  D: ") + ss.str());
    ss.str("");
    ss.clear();

    return info;
}

}
