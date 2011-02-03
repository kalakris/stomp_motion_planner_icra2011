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
#include <errno.h>
#include <sstream>
#include <assert.h>

// ros includes
#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

// local includes
#include <dmp_motion_generation/transformation_system.h>
#include <dmp_motion_generation/constants.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

namespace dmp
{

static const char* transformation_system_file_name = "trans_";
static const char* transformation_system_topic_name = "transformation_system";

// Default Constructor. Note, everything need to be initialized since it is used in a vector inside DMPMotionUnit.
TransformationSystem::TransformationSystem() :
    initialized_(false), trans_id_(-1), version_id_(0), z_(0), zd_(0), y_(0), yd_(0), ydd_(0), t_(0), td_(0), tdd_(0), y0_(0), initial_y0_(0), goal_(0),
            initial_goal_(0), f_(0), ft_(0), /*scale_(0.0), delta_goal_fit_(0.0), s_(0.0), sd_(0.0),*/ mse_(0), mean_ft_(0), num_mse_data_points_(0)
{
}

TransformationSystem::~TransformationSystem()
{
}

/*!
 * @param index
 * @param name
 */
bool TransformationSystem::initialize(std::string library_directory_name, int dmp_id, int trans_id, lwr::Parameters lwr_params)
{
    if (initialized_)
    {
        ROS_WARN("Transformation system already initialized. Reinitializing with new parameters.");
    }

    trans_id_ = trans_id;

    if (!lwr_params.isInitialized())
    {
        ROS_ERROR("LWP parameters are not initialized.");
        initialized_ = false;
        return initialized_;
    }

    if (!lwr_model_.initialize(library_directory_name, lwr_params.getNumRFS(), lwr_params.getRFSWidthBoundary(), true, lwr_params.getCanSysCutOff()))
    {
        ROS_ERROR("Could not initialize LWR model.");
        initialized_ = false;
        return initialized_;
    }

    trajectory_target_.clear();

    initialized_ = true;
    return initialized_;
}

bool TransformationSystem::writeToDisc(std::string directory_name)
{
    if (!initialized_)
    {
        ROS_ERROR("Transformation system is not initialized, not writing to file.");
        return false;
    }

    if (directory_name.compare(directory_name.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        directory_name.append("/");
    }

    // create file name
    std::stringstream ss;
    ss << trans_id_;
    std::string abs_bagfile_name = directory_name + std::string(transformation_system_file_name + ss.str());
    try
    {
        rosbag::Bag bag(abs_bagfile_name + std::string(".bag"), rosbag::bagmode::Write);
        dmp_motion_generation::TransformationSystem transformation_system;
        if (!writeToMessage(transformation_system))
        {
            ROS_ERROR("Could not get transformation system.");
            return false;
        }
        bag.write(transformation_system_topic_name, ros::Time::now(), transformation_system);
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s: %s", abs_bagfile_name.c_str(), ex.what());
        return false;
    }

    if (!lwr_model_.writeToDisc(abs_bagfile_name + std::string("_")))
    {
        ROS_ERROR("Could not write LWR model to file.");
        return false;
    }

    return true;
}

bool TransformationSystem::readFromDisc(std::string directory_name, int trans_id)
{

    trans_id_ = trans_id;

    // create file name
    if (directory_name.compare(directory_name.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        directory_name.append("/");
    }
    std::stringstream ss;
    ss << trans_id_;
    std::string abs_bagfile_name = directory_name + std::string(transformation_system_file_name) + ss.str();
    try
    {
        rosbag::Bag bag(abs_bagfile_name + std::string(".bag"), rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(transformation_system_topic_name));
        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
        {
            dmp_motion_generation::TransformationSystem::ConstPtr transformation_system = msg.instantiate<dmp_motion_generation::TransformationSystem> ();
            assert(transformation_system != NULL);
            if (!initFromMessage(*transformation_system))
            {
                ROS_ERROR("Could not read bag file and set transformation system.");
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

    // initialized values to compute normalized mean squared error
    resetMSE();

    // TODO: do some checks on the values read from file
    return true;
}

bool TransformationSystem::initFromMessage(const dmp_motion_generation::TransformationSystem& transformation_system_msg)
{
    initialized_ = transformation_system_msg.initialized;
    trans_id_ = transformation_system_msg.trans_id;
    version_id_ = transformation_system_msg.version_id;

    initial_y0_ = transformation_system_msg.initial_start;
    initial_goal_ = transformation_system_msg.initial_goal;
    y0_ = transformation_system_msg.start;
    goal_ = transformation_system_msg.goal;

//    scale_ = transformation_system_msg.scale;
//    delta_goal_fit_ = transformation_system_msg.delta_goal_fit;

//    s_ = 1.0;
//    sd_ = 0.0;

    y_ = transformation_system_msg.current_position;
    z_ = transformation_system_msg.current_velocity;

    return lwr_model_.initFromMessage(transformation_system_msg.lwr_model);
}
bool TransformationSystem::writeToMessage(dmp_motion_generation::TransformationSystem& transformation_system_msg)
{
    transformation_system_msg.initialized = initialized_;
    transformation_system_msg.trans_id = trans_id_;
    transformation_system_msg.version_id = version_id_;

    transformation_system_msg.initial_start = initial_y0_;
    transformation_system_msg.initial_goal = initial_goal_;
    transformation_system_msg.start = y0_;
    transformation_system_msg.goal = goal_;

//    transformation_system_msg.scale = scale_;
//    transformation_system_msg.delta_goal_fit = delta_goal_fit_;

    transformation_system_msg.current_position = y_;
    transformation_system_msg.current_velocity = z_;

    return lwr_model_.writeToMessage(transformation_system_msg.lwr_model);
}

void TransformationSystem::print()
{
    ROS_INFO_STREAM(getInfoString());
}

std::string TransformationSystem::getInfoString()
{
    std::string info("");
    std::stringstream ss;

    int precision = 4;
    ss.precision(precision);
    ss << std::fixed;

    ss << trans_id_;
    info.append(std::string("index: ") + ss.str());
    ss.str("");
    ss.clear();

    info.append(std::string("  initialized: "));
    if (initialized_)
    {
        info.append(std::string("true "));
    }
    else
    {
        info.append(std::string("false"));
    }

    if (initial_y0_ < 0)
    {
        ss.precision(precision - 1);
    }
    ss << initial_y0_;
    info.append(std::string("  initial start state: ") + ss.str());
    ss.str("");
    ss.clear();
    ss.precision(precision);

    if (initial_goal_ < 0)
    {
        ss.precision(precision - 1);
    }
    ss << initial_goal_;
    info.append(std::string("  initial goal state: ") + ss.str());
    ss.str("");
    ss.clear();
    ss.precision(precision);

    info.append(std::string("\n") + lwr_model_.getInfoString());
    return info;
}

}
