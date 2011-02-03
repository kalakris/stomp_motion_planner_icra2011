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

#include <chomp_motion_planner/chomp_cost_server.h>

//#define ANIMATE 1

namespace chomp
{
  ChompCostServer::ChompCostServer()
  {
  }

  ChompCostServer::~ChompCostServer()
  {
  }

  bool ChompCostServer::init(bool advertise_service)
  {
    // build the robot model
    if (!chomp_robot_model_.init())
      return false;

    // initialize the collision space
    if (!chomp_collision_space_.init(chomp_robot_model_.getMaxRadiusClearance()))
      return false;

    // initialize the visualization publisher:
    vis_marker_array_publisher_ = node_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );

    // listen to mechanism_state
    mechanism_state_subscriber_ = node_.subscribe(std::string("/mechanism_state"), 1, &ChompCostServer::mechanismStateCallback, this);

    // advertise the cost service
    if (advertise_service)
    {
      get_chomp_collision_cost_server_ = node_.advertiseService("get_chomp_collision_cost", &ChompCostServer::getChompCollisionCost, this);
      get_state_cost_server_ = node_.advertiseService("get_state_cost", &ChompCostServer::getStateCost, this);
    }
    return true;
  }

  bool ChompCostServer::getChompCollisionCost(chomp_motion_planner::GetChompCollisionCost::Request& request, chomp_motion_planner::GetChompCollisionCost::Response& response)
  {
    if(request.state.joint_state.name.size() != request.state.joint_state.position.size())
    {
      ROS_ERROR("Number of joint states does not match number of joint positions");
      return false;
    }

    for (unsigned int i=0; i<request.state.joint_state.name.size(); ++i)
    {
      ROS_DEBUG("%s = %f", request.state.joint_state.name[i].c_str(), request.state.joint_state.position[i]);
    }
    KDL::JntArray joint_array(chomp_robot_model_.getNumKDLJoints());

    fillDefaultJointArray(joint_array);
    chomp_robot_model_.jointStateToArray(request.state.joint_state, joint_array);

    std::vector<KDL::Vector> joint_axis;
    std::vector<KDL::Vector> joint_pos;
    std::vector<KDL::Frame> segment_frames;
    chomp_robot_model_.getForwardKinematicsSolver()->JntToCart(joint_array, joint_pos, joint_axis, segment_frames);

    int num_links = request.links.size();
    response.costs.resize(num_links);
    response.gradient.resize(num_links);

    std::vector<ChompCollisionPoint> points;
    KDL::Vector position;
    Eigen::Vector3d potential_gradient;
    Eigen::Vector3d position_eigen;
    double potential;
    int num_collision_points=0;

    // for each link, get its collision points and accumulate the cost:
    for (unsigned int l=0; l<request.links.size(); ++l)
    {
      response.costs[l] = 0.0;
      chomp_robot_model_.getLinkCollisionPoints(request.links[l], points);
      //ROS_INFO("Link %s has %d collision points", request.links[l].c_str(), points.size());
      for (unsigned int i=0; i<points.size(); i++)
      {
        ++num_collision_points;
        points[i].getTransformedPosition(segment_frames, position);

        position_eigen = Eigen::Map<Eigen::Vector3d>(position.data);

        //bool colliding =
        chomp_collision_space_.getCollisionPointPotentialGradient(points[i],
                                                                  position_eigen, potential, potential_gradient);
        response.costs[l] += potential * points[i].getVolume();
        //ROS_INFO("\tPos=%f,%f,%f  cost = %f", position_eigen(0), position_eigen(1), position_eigen(2), response.costs[l]*1e8);
      }
    }
#ifdef ANIMATE
    visualization_msgs::MarkerArray msg;
    msg.markers.resize(num_collision_points);
    int i=0;
    for (unsigned int l=0; l<request.links.size(); ++l)
    {
      chomp_robot_model_.getLinkCollisionPoints(request.links[l], points);
      for (unsigned int p=0; p<points.size(); p++)
      {
        points[p].getTransformedPosition(segment_frames, position);
        msg.markers[i].header.frame_id = chomp_robot_model_.getReferenceFrame();
        msg.markers[i].header.stamp = ros::Time::now();
        msg.markers[i].ns = "chomp_collisions";
        msg.markers[i].id = i;
        msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        msg.markers[i].action = visualization_msgs::Marker::ADD;
        msg.markers[i].pose.position.x = position.x();
        msg.markers[i].pose.position.y = position.y();
        msg.markers[i].pose.position.z = position.z();
        msg.markers[i].pose.orientation.x = 0.0;
        msg.markers[i].pose.orientation.y = 0.0;
        msg.markers[i].pose.orientation.z = 0.0;
        msg.markers[i].pose.orientation.w = 1.0;
        double scale = points[p].getRadius()*2;
        msg.markers[i].scale.x = scale;
        msg.markers[i].scale.y = scale;
        msg.markers[i].scale.z = scale;
        msg.markers[i].color.a = 0.9;
        msg.markers[i].color.r = 0.5;
        msg.markers[i].color.g = 1.0;
        msg.markers[i].color.b = 0.3;
        ++i;
      }
    }
    vis_marker_array_publisher_.publish(msg);
    ros::WallDuration(0.1).sleep();
#endif
    return true;
  }



  bool ChompCostServer::getStateCost(chomp_motion_planner::GetStateCost::Request& request, chomp_motion_planner::GetStateCost::Response& response)
  {
    chomp_motion_planner::GetChompCollisionCost::Request  chomp_request;
    chomp_motion_planner::GetChompCollisionCost::Response chomp_response;

    chomp_request.links = request.link_names;
    chomp_request.state = request.robot_state;
    bool success = getChompCollisionCost(chomp_request,chomp_response);
    
    if(success)
    {
      response.valid = chomp_response.in_collision;
      response.costs = chomp_response.costs;
    }
    return success;
  }



  void ChompCostServer::mechanismStateCallback(const sensor_msgs::JointStateConstPtr& mech_state)
  {
    if (mechanism_state_mutex_.try_lock())
    {
      mechanism_state_ = *mech_state;
      mechanism_state_mutex_.unlock();
    }
  }

  void ChompCostServer::fillDefaultJointArray(KDL::JntArray& jnt_array)
  {
    mechanism_state_mutex_.lock();

    int num_joints = mechanism_state_.name.size();
    for (int i=0; i<num_joints; i++)
    {
      std::string& name = mechanism_state_.name[i];
      int num = chomp_robot_model_.urdfNameToKdlNumber(name);
      if (num>=0)
        jnt_array(num) = mechanism_state_.position[i];
    }
    mechanism_state_mutex_.unlock();
  }

  int ChompCostServer::run()
  {
    ros::spin();
    return 0;
  }

} // namespace chomp

using namespace chomp;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chomp_cost_server");
  ChompCostServer chomp_cost_server;

  if (chomp_cost_server.init(true))
    return chomp_cost_server.run();
  else
    return 1;
}
