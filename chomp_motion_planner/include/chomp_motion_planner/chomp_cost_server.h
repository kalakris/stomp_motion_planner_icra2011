/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef CHOMP_COST_SERVER_H_
#define CHOMP_COST_SERVER_H_

#include <ros/ros.h>
#include <chomp_motion_planner/GetChompCollisionCost.h>
#include <chomp_motion_planner/chomp_robot_model.h>
#include <chomp_motion_planner/chomp_collision_space.h>
#include <boost/thread/mutex.hpp>
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include <chomp_motion_planner/GetStateCost.h>

namespace chomp
{

/**
 * This node advertises a service that can respond to queries about collision cost
 * for a given robot configuration.
 */
class ChompCostServer
{
public:
  ChompCostServer();
  virtual ~ChompCostServer();

  bool init(bool advertise_service);
  int run();
  bool getChompCollisionCost(chomp_motion_planner::GetChompCollisionCost::Request& request, chomp_motion_planner::GetChompCollisionCost::Response& response);
  bool getStateCost(chomp_motion_planner::GetStateCost::Request& request, chomp_motion_planner::GetStateCost::Response& response);

  void mechanismStateCallback(const sensor_msgs::JointStateConstPtr& mech_state);

private:

  ros::NodeHandle node_;
  ros::ServiceServer get_chomp_collision_cost_server_, get_state_cost_server_;
  ros::Subscriber mechanism_state_subscriber_;
  ros::Publisher vis_marker_array_publisher_;

  ChompRobotModel chomp_robot_model_;
  ChompCollisionSpace chomp_collision_space_;
  boost::mutex mechanism_state_mutex_;

  sensor_msgs::JointState mechanism_state_;

  void fillDefaultJointArray(KDL::JntArray& jnt_array);
};

}

#endif /* CHOMP_COST_SERVER_H_ */
