/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan */

#include <policy_library/policy_factory.h>
#include <policy_library/dmp_policy.h>
#include <policy_library/single_parameter_policy.h>
#include <policy_library/mixed_policy.h>

namespace library
{

PolicyFactory::PolicyFactory(ros::NodeHandle& node_handle) :
    node_handle_(node_handle)
{
}

PolicyFactory::~PolicyFactory()
{
}

bool PolicyFactory::createPolicyInstanceByName(const std::string& class_name, boost::shared_ptr<LibraryItem>& policy)
{
    if (class_name == "DMPPolicy")
    {
        policy = boost::shared_ptr<Policy>(new DMPPolicy());
    }
    else if (class_name == "SingleParameterPolicy")
    {
        policy = boost::shared_ptr<Policy>(new SingleParameterPolicy());
    }
    else if (class_name == "MixedPolicy")
    {
        policy = boost::shared_ptr<Policy>(new MixedPolicy());
    }
    else
    {
        ROS_ERROR("PolicyFactory doesn't know about class %s.", class_name.c_str());
        return false;
    }
    return true;
}

// TODO: UGLY HACK!!!!!! DMPs should eventually not be stored directly in a policy_library

bool PolicyFactory::createPolicyInstanceByName(const std::string& class_name, boost::shared_ptr<dmp::DynamicMovementPrimitive>& policy)
{
    policy = boost::shared_ptr<dmp::DynamicMovementPrimitive>(new dmp::DynamicMovementPrimitive(node_handle_));
    return true;
}

}
