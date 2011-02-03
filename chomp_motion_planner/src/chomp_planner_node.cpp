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

#include <chomp_motion_planner/chomp_planner_node.h>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <chomp_motion_planner/chomp_utils.h>
#include <chomp_motion_planner/chomp_parameters.h>
#include <chomp_motion_planner/chomp_optimizer.h>
#include <kdl/jntarray.hpp>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <spline_smoother/cubic_trajectory.h>

#include <map>
#include <vector>
#include <string>

using namespace std;

namespace chomp
{

ChompPlannerNode::ChompPlannerNode(ros::NodeHandle node_handle) : node_handle_(node_handle)
                                                                  //filter_constraints_chain_("motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request")
{

}

bool ChompPlannerNode::init()
{
  // load in some default parameters
  node_handle_.param("reference_frame", reference_frame_, std::string("base_link"));
  node_handle_.param("trajectory_duration", trajectory_duration_, 3.0);
  node_handle_.param("trajectory_discretization", trajectory_discretization_, 0.03);

  //filter_constraints_chain_.configure("filter_chain",node_handle_);

  collision_models_ = new planning_environment::CollisionModels("robot_description");

  if(!collision_models_->loadedModels()) {
    ROS_ERROR("Collision models could not load models");
    return false;
  }

  monitor_ = new planning_environment::CollisionSpaceMonitor(collision_models_, &tf_, reference_frame_);

  filter_trajectory_client_ = node_handle_.serviceClient<motion_planning_msgs::FilterJointTrajectoryWithConstraints>("trajectory_filter/filter_trajectory_with_constraints",true);    
  
  ros::service::waitForService("trajectory_filter/filter_trajectory_with_constraints");

  // build the robot model
  if (!chomp_robot_model_.init(monitor_, reference_frame_))
    return false;

  // load chomp parameters:
  chomp_parameters_.initFromNodeHandle();

  double max_radius_clearance = chomp_robot_model_.getMaxRadiusClearance();

  // initialize the collision space
  if (!chomp_collision_space_.init(monitor_,max_radius_clearance, reference_frame_))
    return false;

  // initialize the visualization publisher:
  vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
  vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  // advertise the planning service
  plan_kinematic_path_service_ = root_handle_.advertiseService("chomp_planner_longrange/plan_path", &ChompPlannerNode::planKinematicPath, this);

  filter_joint_trajectory_service_ = root_handle_.advertiseService("chomp_planner_longrange/filter_trajectory", &ChompPlannerNode::filterJointTrajectory, this);

  ROS_INFO("Initalized CHOMP planning service...");

  return true;
}

ChompPlannerNode::~ChompPlannerNode()
{
  delete collision_models_;
  delete monitor_;
}

int ChompPlannerNode::run()
{
  ros::spin();
  return 0;
}

bool ChompPlannerNode::planKinematicPath(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res)
{
  if (!(req.motion_plan_request.goal_constraints.position_constraints.empty() && req.motion_plan_request.goal_constraints.orientation_constraints.empty()))
  {
    ROS_ERROR("CHOMP cannot handle pose contraints yet.");
    return false;
  }

  sensor_msgs::JointState joint_goal_chomp = motion_planning_msgs::jointConstraintsToJointState(req.motion_plan_request.goal_constraints.joint_constraints);
  ROS_INFO("Chomp goal");

  if(joint_goal_chomp.name.size() != joint_goal_chomp.position.size())
  {
    ROS_ERROR("Invalid chomp goal");
    return false;
  }

  for(unsigned int i=0; i<joint_goal_chomp.name.size(); i++)
  {
    ROS_INFO("%s %f",joint_goal_chomp.name[i].c_str(),joint_goal_chomp.position[i]);
  }

  ros::WallTime start_time = ros::WallTime::now();
  ROS_INFO("Received planning request...");

  // get the planning group:
  const ChompRobotModel::ChompPlanningGroup* group = chomp_robot_model_.getPlanningGroup(req.motion_plan_request.group_name);

  if (group==NULL)
  {
    ROS_ERROR("Could not load planning group %s", req.motion_plan_request.group_name.c_str());
    return false;
  }

  ChompTrajectory trajectory(&chomp_robot_model_, trajectory_duration_, trajectory_discretization_);

  // set the start state:
  chomp_robot_model_.jointStateToArray(req.motion_plan_request.start_state.joint_state, trajectory.getTrajectoryPoint(0));

  //configure the distance field for the start state
  chomp_collision_space_.setStartState(*group, req.motion_plan_request.start_state);

  //updating collision points for the potential for new attached objects
  //note that this assume that setStartState has been run by chomp_collision_space_
  chomp_robot_model_.generateAttachedObjectCollisionPoints(&(req.motion_plan_request.start_state));
  chomp_robot_model_.populatePlanningGroupCollisionPoints();

  // set the goal state equal to start state, and override the joints specified in the goal
  // joint constraints
  int goal_index = trajectory.getNumPoints()-1;
  trajectory.getTrajectoryPoint(goal_index) = trajectory.getTrajectoryPoint(0);
  chomp_robot_model_.jointStateToArray(motion_planning_msgs::jointConstraintsToJointState(req.motion_plan_request.goal_constraints.joint_constraints), trajectory.getTrajectoryPoint(goal_index));

  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (int i=0; i<group->num_joints_; i++)
  {
    if (group->chomp_joints_[i].wrap_around_)
    {
      int kdl_index = group->chomp_joints_[i].kdl_joint_index_;
      double start = trajectory(0, kdl_index);
      double end = trajectory(goal_index, kdl_index);
      trajectory(goal_index, kdl_index) = start + angles::shortest_angular_distance(start, end);
    }
  }

  // fill in an initial quintic spline trajectory
  trajectory.fillInMinJerk();

  // set the max planning time:
  chomp_parameters_.setPlanningTimeLimit(req.motion_plan_request.allowed_planning_time.toSec());

  // optimize!
  ChompOptimizer optimizer(&trajectory, &chomp_robot_model_, group, &chomp_parameters_,
      vis_marker_array_publisher_, vis_marker_publisher_, &chomp_collision_space_);
  optimizer.optimize();

  // assume that the trajectory is now optimized, fill in the output structure:

  std::vector<double> velocity_limits(group->num_joints_, std::numeric_limits<double>::max());

  // fill in joint names:
  res.trajectory.joint_trajectory.joint_names.resize(group->num_joints_);
  for (int i=0; i<group->num_joints_; i++)
  {
    res.trajectory.joint_trajectory.joint_names[i] = group->chomp_joints_[i].joint_name_;
    // try to retrieve the joint limits:
    if (joint_velocity_limits_.find(res.trajectory.joint_trajectory.joint_names[i])==joint_velocity_limits_.end())
    {
      node_handle_.param("joint_velocity_limits/"+res.trajectory.joint_trajectory.joint_names[i], joint_velocity_limits_[res.trajectory.joint_trajectory.joint_names[i]], std::numeric_limits<double>::max());
    }
    velocity_limits[i] = joint_velocity_limits_[res.trajectory.joint_trajectory.joint_names[i]];
  }

  res.trajectory.joint_trajectory.header = req.motion_plan_request.start_state.joint_state.header; // @TODO this is probably a hack

  // fill in the entire trajectory
  res.trajectory.joint_trajectory.points.resize(trajectory.getNumPoints());
  for (int i=0; i<=goal_index; i++)
  {
    res.trajectory.joint_trajectory.points[i].positions.resize(group->num_joints_);
    for (int j=0; j<group->num_joints_; j++)
    {
      int kdl_joint_index = chomp_robot_model_.urdfNameToKdlNumber(res.trajectory.joint_trajectory.joint_names[j]);
      res.trajectory.joint_trajectory.points[i].positions[j] = trajectory(i, kdl_joint_index);
    }
    if (i==0)
      res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
    else
    {
      double duration = trajectory.getDiscretization();
      // check with all the joints if this duration is ok, else push it up
      for (int j=0; j<group->num_joints_; j++)
      {
        double d = fabs(res.trajectory.joint_trajectory.points[i].positions[j] - res.trajectory.joint_trajectory.points[i-1].positions[j]) / velocity_limits[j];
        if (d > duration)
          duration = d;
      }
      res.trajectory.joint_trajectory.points[i].time_from_start = res.trajectory.joint_trajectory.points[i-1].time_from_start + ros::Duration(duration);
    }
  }

  ROS_INFO("Serviced planning request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.joint_trajectory.points[goal_index].time_from_start.toSec());
  return true;
}

bool ChompPlannerNode::filterJointTrajectory(motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request &req, motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response &res)
{
  ros::WallTime start_time = ros::WallTime::now();
  ROS_INFO_STREAM("Received filtering request with trajectory size " << req.trajectory.points.size());

  //create a spline from the trajectory
  spline_smoother::CubicTrajectory trajectory_solver;
  spline_smoother::SplineTrajectory spline;

  getLimits(req.trajectory, req.limits);

  for (unsigned int i=0; i< req.trajectory.points.size(); i++)
  {
    req.trajectory.points[i].velocities.resize(req.trajectory.joint_names.size());
  }

  bool success = trajectory_solver.parameterize(req.trajectory,req.limits,spline);  

  double smoother_time;
  spline_smoother::getTotalTime(spline, smoother_time);
  
  ROS_INFO_STREAM("Total time is " << smoother_time);

  unsigned int NUM_POINTS=100;

  double t = 0.0;
  std::vector<double> times(NUM_POINTS);
  for(unsigned int i = 0; i < NUM_POINTS; i++,t += smoother_time/(1.0*NUM_POINTS)) {
    times[i] = t;
  }

  trajectory_msgs::JointTrajectory jtraj;
  spline_smoother::sampleSplineTrajectory(spline, times, jtraj);

  double planner_time = req.trajectory.points.back().time_from_start.toSec();
  
  t = 0.0;
  for(unsigned int i = 0; i < jtraj.points.size(); i++, t += planner_time/(1.0*NUM_POINTS)) {
    jtraj.points[i].time_from_start = ros::Duration(t);
  }

  ROS_INFO_STREAM("Sampled trajectory has " << jtraj.points.size() << " points with " << jtraj.points[0].positions.size() << " joints");

  // get the filter group - will need to figure out
  const ChompRobotModel::ChompPlanningGroup* group = chomp_robot_model_.getPlanningGroup("right_arm");

  if (group==NULL)
  {
    ROS_ERROR("Could not load planning group %s", "right_arm");
    return false;
  }

  ChompTrajectory trajectory(&chomp_robot_model_, group, jtraj);

  //configure the distance field - this should just use current state
  motion_planning_msgs::RobotState robot_state;
  robot_state.joint_state.position.clear();
  robot_state.joint_state.name.clear();
  planning_models::KinematicState sp(*(monitor_->getRobotState()));
  // boost::scoped_ptr<planning_models::KinematicState> sp(new planning_models::KinematicState(getKinematicModel()));
  // fill in robot state with current one
  std::vector<const planning_models::KinematicModel::Joint*> joints;
  monitor_->getKinematicModel()->getJoints(joints);
	    
  robot_state.joint_state.name.resize(joints.size());
  robot_state.joint_state.position.resize(joints.size());
  robot_state.joint_state.header.frame_id = reference_frame_;
  robot_state.joint_state.header.stamp = monitor_->lastJointStateUpdate();

  for (unsigned int i = 0 ; i < joints.size() ; ++i)
  {
    robot_state.joint_state.name[i] = joints[i]->name;
    std::vector<double> tmp;
    sp.copyParamsJoint(tmp, joints[i]->name);
    if(!tmp.empty())
      robot_state.joint_state.position[i] = tmp[0];
  }

  // set the start state:
  chomp_robot_model_.jointStateToArray(robot_state.joint_state, trajectory.getTrajectoryPoint(0));

  chomp_collision_space_.setStartState(*group, robot_state);

  //updating collision points for the potential for new attached objects
  //note that this assume that setStartState has been run by chomp_collision_space_

  chomp_robot_model_.generateAttachedObjectCollisionPoints(&(robot_state));
  chomp_robot_model_.populatePlanningGroupCollisionPoints();

  // set the goal state equal to start state, and override the joints specified in the goal
  // joint constraints
  int goal_index = trajectory.getNumPoints()-1;
  trajectory.getTrajectoryPoint(goal_index) = trajectory.getTrajectoryPoint(0);

  sensor_msgs::JointState goal_state = motion_planning_msgs::createJointState(req.trajectory.joint_names, jtraj.points.back().positions);

  chomp_robot_model_.jointStateToArray(goal_state, trajectory.getTrajectoryPoint(goal_index));
  
  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (int i=0; i<group->num_joints_; i++)
  {
    if (group->chomp_joints_[i].wrap_around_)
    {
      int kdl_index = group->chomp_joints_[i].kdl_joint_index_;
      double start = trajectory(0, kdl_index);
      double end = trajectory(goal_index, kdl_index);
      trajectory(goal_index, kdl_index) = start + angles::shortest_angular_distance(start, end);
    }
  }

  // set the max planning time:
  chomp_parameters_.setPlanningTimeLimit(req.allowed_time.toSec());
  
  ROS_INFO("Calling optimizer");

  // optimize!
  ChompOptimizer optimizer(&trajectory, &chomp_robot_model_, group, &chomp_parameters_,
      vis_marker_array_publisher_, vis_marker_publisher_, &chomp_collision_space_);
  optimizer.optimize();
  
  // assume that the trajectory is now optimized, fill in the output structure:

  std::vector<double> velocity_limits(group->num_joints_, std::numeric_limits<double>::max());

  // fill in joint names:
  res.trajectory.joint_names.resize(group->num_joints_);
  for (int i=0; i<group->num_joints_; i++)
  {
    res.trajectory.joint_names[i] = group->chomp_joints_[i].joint_name_;
    velocity_limits[i] = joint_limits_[res.trajectory.joint_names[i]].max_velocity;
  }
  
  res.trajectory.header.stamp = ros::Time::now();
  res.trajectory.header.frame_id = reference_frame_;

  // fill in the entire trajectory
  res.trajectory.points.resize(trajectory.getNumPoints());
  for (int i=0; i< trajectory.getNumPoints(); i++)
  {
    res.trajectory.points[i].positions.resize(group->num_joints_);
    res.trajectory.points[i].velocities.resize(group->num_joints_);
    for (int j=0; j<group->num_joints_; j++)
    {
      int kdl_joint_index = chomp_robot_model_.urdfNameToKdlNumber(res.trajectory.joint_names[j]);
      res.trajectory.points[i].positions[j] = trajectory(i, kdl_joint_index);
    }
    if (i==0)
      res.trajectory.points[i].time_from_start = ros::Duration(0.0);
    else
    {
      double duration = trajectory.getDiscretization();
      // check with all the joints if this duration is ok, else push it up
      for (int j=0; j<group->num_joints_; j++)
      {
        double d = fabs(res.trajectory.points[i].positions[j] - res.trajectory.points[i-1].positions[j]) / velocity_limits[j];
        if (d > duration)
          duration = d;
      }
      res.trajectory.points[i].time_from_start = res.trajectory.points[i-1].time_from_start + ros::Duration(duration);
    }
  }

//   motion_planning_msgs::FilterJointTrajectoryWithConstraints::Request  next_req;
//   motion_planning_msgs::FilterJointTrajectoryWithConstraints::Response next_res;
  
//   next_req = req;
//   next_req.trajectory = res.trajectory;
  
//   ROS_INFO("Trying to make call");

//   if(filter_trajectory_client_.call(next_req, next_res)) {
//     ROS_INFO("Filter call ok");
//   } else {
//     ROS_INFO("Filter call not ok");
//   }

//   res = next_res;

  // for every point in time:
  for (unsigned int i=1; i<res.trajectory.points.size()-1; ++i)
  {
    double dt1 = (res.trajectory.points[i].time_from_start - res.trajectory.points[i-1].time_from_start).toSec();
    double dt2 = (res.trajectory.points[i+1].time_from_start - res.trajectory.points[i].time_from_start).toSec();

    // for every (joint) trajectory
    for (int j=0; j<group->num_joints_; ++j)
    {
      double dx1 = res.trajectory.points[i].positions[j] - res.trajectory.points[i-1].positions[j];
      double dx2 = res.trajectory.points[i+1].positions[j] - res.trajectory.points[i].positions[j];

      double v1 = dx1/dt1;
      double v2 = dx2/dt2;

      res.trajectory.points[i].velocities[j] = 0.5*(v1 + v2);
    }
  }

  ROS_INFO("Serviced filter request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.points.back().time_from_start.toSec());
  return true;

}

void ChompPlannerNode::getLimits(const trajectory_msgs::JointTrajectory& trajectory, 
                                 std::vector<motion_planning_msgs::JointLimits>& limits_out)
{
  int num_joints = trajectory.joint_names.size();
  limits_out.resize(num_joints);
  for (int i=0; i<num_joints; ++i)
  {
    std::map<std::string, motion_planning_msgs::JointLimits>::const_iterator limit_it = joint_limits_.find(trajectory.joint_names[i]);
    motion_planning_msgs::JointLimits limits;
    if (limit_it == joint_limits_.end())
    {
      // load the limits from the param server
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/min_position", limits.min_position, -std::numeric_limits<double>::max());
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_position", limits.max_position, std::numeric_limits<double>::max());
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_velocity", limits.max_velocity, std::numeric_limits<double>::max());
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_acceleration", limits.max_acceleration, std::numeric_limits<double>::max());
      bool boolean;
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_position_limits", boolean, false);
      limits.has_position_limits = boolean?1:0;
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_velocity_limits", boolean, false);
      limits.has_velocity_limits = boolean?1:0;
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_acceleration_limits", boolean, false);
      limits.has_acceleration_limits = boolean?1:0;
      joint_limits_.insert(make_pair(trajectory.joint_names[i], limits));
    }
    else
    {
      limits = limit_it->second;
    }
    limits_out[i] = limits;
  }
}

} // namespace chomp

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chomp_planner_node");

  ros::AsyncSpinner spinner(1); // Use 1 thread
  spinner.start();

  ros::NodeHandle node_handle("~");
  chomp::ChompPlannerNode chomp_planner_node(node_handle);
  if (!chomp_planner_node.init())
    return 1;
  ros::waitForShutdown();
  //return chomp_planner_node.run();
}
