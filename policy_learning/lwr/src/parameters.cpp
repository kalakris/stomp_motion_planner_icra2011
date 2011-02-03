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

 \file    lwr_parameters.cpp

 \author  Peter Pastor
 \date    May 28, 2010

 **********************************************************************/

// system includes
#include <sstream>

// ros includes
#include <ros/ros.h>

#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>

// local includes
#include <lwr/parameters.h>

namespace lwr
{

Parameters::Parameters(ros::NodeHandle& node_handle) :
    initialized_(false), node_handle_(node_handle)
{
}

Parameters::~Parameters()
{
}

bool Parameters::initialize()
{
    return initFromNodeHandle();
}

bool Parameters::initFromNodeHandle()
{

    ros::NodeHandle lwr_node_handle(node_handle_, "lwr");

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(lwr_node_handle, "rfs_width_boundary", rfs_width_boundary_));
    if ((rfs_width_boundary_ < 0 || rfs_width_boundary_ > 1.0))
    {
        ROS_ERROR("Wrong value for width_boundary parameter (%f) ...should be between 0 and 1.",rfs_width_boundary_);
        initialized_ = false;
        return initialized_;
    }
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(lwr_node_handle, "num_rfs", num_rfs_));

    ros::NodeHandle dmp_node_handle(node_handle_, "dmp");
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(dmp_node_handle, "can_sys_cutoff", can_sys_cutoff_));

    return (initialized_ = true);
}

}

