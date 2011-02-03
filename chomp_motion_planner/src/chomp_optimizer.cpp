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

#include <chomp_motion_planner/chomp_optimizer.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <chomp_motion_planner/chomp_utils.h>
#include <Eigen/LU>

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

namespace chomp
{

ChompOptimizer::ChompOptimizer(ChompTrajectory *trajectory, const ChompRobotModel *robot_model,
    const ChompRobotModel::ChompPlanningGroup *planning_group, const ChompParameters *parameters,
    const ros::Publisher& vis_marker_array_publisher,
    const ros::Publisher& vis_marker_publisher,
    ChompCollisionSpace *collision_space):
      full_trajectory_(trajectory),
      robot_model_(robot_model),
      planning_group_(planning_group),
      parameters_(parameters),
      collision_space_(collision_space),
      group_trajectory_(*full_trajectory_, planning_group_, DIFF_RULE_LENGTH),
      kdl_joint_array_(robot_model_->getKDLTree()->getNrOfJoints()),
      vis_marker_array_pub_(vis_marker_array_publisher),
      vis_marker_pub_(vis_marker_publisher)
{
  initialize();
}

void ChompOptimizer::initialize()
{

  // init some variables:
  num_vars_free_ = group_trajectory_.getNumFreePoints();
  num_vars_all_ = group_trajectory_.getNumPoints();
  num_joints_ = group_trajectory_.getNumJoints();

  free_vars_start_ = group_trajectory_.getStartIndex();
  free_vars_end_ = group_trajectory_.getEndIndex();

  //ROS_INFO_STREAM("Setting free vars start to " << free_vars_start_ << " end " << free_vars_end_);

  // set up joint index:
  group_joint_to_kdl_joint_index_.resize(num_joints_);
  for (int i=0; i<num_joints_; ++i)
    group_joint_to_kdl_joint_index_[i] = planning_group_->chomp_joints_[i].kdl_joint_index_;

  num_collision_points_ = planning_group_->collision_points_.size();

  // set up the joint costs:
  joint_costs_.reserve(num_joints_);

  double max_cost_scale = 0.0;
  ros::NodeHandle nh("~");
  for (int i=0; i<num_joints_; i++)
  {
    double joint_cost = 1.0;
    std::string joint_name = planning_group_->chomp_joints_[i].joint_name_;
    nh.param("joint_costs/"+joint_name, joint_cost, 1.0);
    std::vector<double> derivative_costs(3);
    derivative_costs[0] = joint_cost*parameters_->getSmoothnessCostVelocity();
    derivative_costs[1] = joint_cost*parameters_->getSmoothnessCostAcceleration();
    derivative_costs[2] = joint_cost*parameters_->getSmoothnessCostJerk();
    

    joint_costs_.push_back(ChompCost(group_trajectory_, i, derivative_costs, parameters_->getRidgeFactor()));
    double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
    if (max_cost_scale < cost_scale)
      max_cost_scale = cost_scale;
  }

  // scale the smoothness costs
  for (int i=0; i<num_joints_; i++)
  {
    joint_costs_[i].scale(max_cost_scale);
  }

  // allocate memory for matrices:
  smoothness_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  collision_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  final_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  smoothness_derivative_ = Eigen::VectorXd::Zero(num_vars_all_);
  jacobian_ = Eigen::MatrixXd::Zero(3, num_joints_);
  jacobian_pseudo_inverse_ = Eigen::MatrixXd::Zero(num_joints_, 3);
  jacobian_jacobian_tranpose_ = Eigen::MatrixXd::Zero(3, 3);
  random_state_ = Eigen::VectorXd::Zero(num_joints_);
  joint_state_velocities_ = Eigen::VectorXd::Zero(num_joints_);

  group_trajectory_backup_ = group_trajectory_.getTrajectory();
  best_group_trajectory_ = group_trajectory_.getTrajectory();

  joint_axis_.resize(num_vars_all_, std::vector<KDL::Vector>(robot_model_->getKDLTree()->getNrOfJoints()));
  joint_pos_.resize(num_vars_all_, std::vector<KDL::Vector>(robot_model_->getKDLTree()->getNrOfJoints()));
  segment_frames_.resize(num_vars_all_, std::vector<KDL::Frame>(robot_model_->getKDLTree()->getNrOfSegments()));
  collision_point_pos_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));
  collision_point_vel_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));
  collision_point_acc_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));

  collision_point_potential_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
  collision_point_vel_mag_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
  collision_point_potential_gradient_.resize(num_vars_all_, std::vector<Eigen::Vector3d>(num_collision_points_));

  // create the eigen maps:
  kdlVecVecToEigenVecVec(joint_axis_, joint_axis_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(joint_pos_, joint_pos_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(collision_point_pos_, collision_point_pos_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(collision_point_vel_, collision_point_vel_eigen_, 3, 1);
  kdlVecVecToEigenVecVec(collision_point_acc_, collision_point_acc_eigen_, 3, 1);

  collision_free_iteration_ = 0;
  is_collision_free_ = false;
  state_is_in_collision_.resize(num_vars_all_);
  point_is_in_collision_.resize(num_vars_all_, std::vector<int>(num_collision_points_));

  last_improvement_iteration_ = -1;

  // HMC initialization:
  momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  random_momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  random_joint_momentum_ = Eigen::VectorXd::Zero(num_vars_free_);
  multivariate_gaussian_.clear();
  stochasticity_factor_ = 1.0;
  for (int i=0; i<num_joints_; i++)
  {
    multivariate_gaussian_.push_back(MultivariateGaussian(Eigen::VectorXd::Zero(num_vars_free_), joint_costs_[i].getQuadraticCostInverse()));
  }

  // animation init:
  animate_endeffector_segment_number_ = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(parameters_->getAnimateEndeffectorSegment());

}

ChompOptimizer::~ChompOptimizer()
{
}

void ChompOptimizer::optimize()
{
  collision_space_->lock();

  animatePath();

  ros::WallTime start_time = ros::WallTime::now();
  // iterate
  for (iteration_=0; iteration_<parameters_->getMaxIterations(); iteration_++)
  {
    performForwardKinematics();
    double cost = getTrajectoryCost();

    if (iteration_==0)
    {
      best_group_trajectory_ = group_trajectory_.getTrajectory();
      best_group_trajectory_cost_ = cost;
    }
    else
    {
      if (cost < best_group_trajectory_cost_)
      {
        best_group_trajectory_ = group_trajectory_.getTrajectory();
        best_group_trajectory_cost_ = cost;
        last_improvement_iteration_ = iteration_;
      }
    }

    calculateSmoothnessIncrements();
    calculateCollisionIncrements();
    calculateTotalIncrements();

    if (!parameters_->getUseHamiltonianMonteCarlo())
    {
      // non-stochastic version:
      addIncrementsToTrajectory();
    }
    else
    {
      // hamiltonian monte carlo updates:
      getRandomMomentum();
      updateMomentum();
      updatePositionFromMomentum();
      stochasticity_factor_ *= parameters_->getHmcAnnealingFactor();
    }

    handleJointLimits();
    updateFullTrajectory();
    if (iteration_%10==0)
      ROS_DEBUG("Trajectory cost: %f (s=%f, c=%f)", getTrajectoryCost(), getSmoothnessCost(), getCollisionCost());
    if (collision_free_iteration_ >= parameters_->getMaxIterationsAfterCollisionFree())
    {
      iteration_++;
      break;
    }

    if (!is_collision_free_ && parameters_->getAddRandomness())
    {
      performForwardKinematics();
      double original_cost = getTrajectoryCost();
      group_trajectory_backup_ = group_trajectory_.getTrajectory();
      perturbTrajectory();
      handleJointLimits();
      updateFullTrajectory();
      performForwardKinematics();
      double new_cost = getTrajectoryCost();
      if (new_cost < original_cost)
      {
        //printf("Random jump improved cost from %.10f to %.10f!\n", original_cost, new_cost);
      }
      else
      {
        //printf("Random jump worsened cost from %.10f to %.10f!\n", original_cost, new_cost);
        group_trajectory_.getTrajectory() = group_trajectory_backup_;
        updateFullTrajectory();
      }

    }

    if (parameters_->getAnimateEndeffector())
    {
      animateEndeffector();
    }

    if (parameters_->getAnimatePath() && iteration_%25 == 0)
    {
      animatePath();
    }
    
  }
  ROS_INFO_STREAM("We think the path is collision free: " << is_collision_free_);

  if (parameters_->getAnimatePath())
  {
    animatePath();
  }

  group_trajectory_.getTrajectory() = best_group_trajectory_;
  updateFullTrajectory();
  ROS_INFO("Terminated after %d iterations, using path from iteration %d", iteration_, last_improvement_iteration_);
  ROS_INFO("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec());

  collision_space_->unlock();
  if (parameters_->getAnimatePath())
    animatePath();
}

void ChompOptimizer::calculateSmoothnessIncrements()
{
  for (int i=0; i<num_joints_; i++)
  {
    joint_costs_[i].getDerivative(group_trajectory_.getJointTrajectory(i), smoothness_derivative_);
    smoothness_increments_.col(i) = -smoothness_derivative_.segment(
        group_trajectory_.getStartIndex(), num_vars_free_);
  }
}

void ChompOptimizer::calculateCollisionIncrements()
{
  double potential;
  double vel_mag_sq;
  double vel_mag;
  Vector3d potential_gradient;
  Vector3d normalized_velocity;
  Matrix3d orthogonal_projector;
  Vector3d curvature_vector;
  Vector3d cartesian_gradient;

  collision_increments_.setZero(num_vars_free_, num_joints_);
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    for (int j=0; j<num_collision_points_; j++)
    {
      potential = collision_point_potential_[i][j];
      if (potential <= 1e-10)
        continue;

      potential_gradient = collision_point_potential_gradient_[i][j];

      vel_mag = collision_point_vel_mag_[i][j];
      vel_mag_sq = vel_mag*vel_mag;

      // all math from the CHOMP paper:

      normalized_velocity = collision_point_vel_eigen_[i][j] / vel_mag;
      orthogonal_projector = Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
      curvature_vector = (orthogonal_projector * collision_point_acc_eigen_[i][j]) / vel_mag_sq;
      cartesian_gradient = vel_mag*(orthogonal_projector*potential_gradient - potential*curvature_vector);

      // pass it through the jacobian transpose to get the increments
      planning_group_->collision_points_[j].getJacobian(joint_pos_eigen_[i], joint_axis_eigen_[i],
          collision_point_pos_eigen_[i][j], jacobian_, group_joint_to_kdl_joint_index_);
      if (parameters_->getUsePseudoInverse())
      {
        calculatePseudoInverse();
        collision_increments_.row(i-free_vars_start_).transpose() -=
            jacobian_pseudo_inverse_ * cartesian_gradient;
      }
      else
      {
        collision_increments_.row(i-free_vars_start_).transpose() -=
            jacobian_.transpose() * cartesian_gradient;
      }
      if (point_is_in_collision_[i][j])
        break;
    }
  }
  //cout << collision_increments_ << endl;
}

void ChompOptimizer::calculatePseudoInverse()
{
  jacobian_jacobian_tranpose_ = jacobian_*jacobian_.transpose() + Eigen::MatrixXd::Identity(3,3)*parameters_->getPseudoInverseRidgeFactor();
  jacobian_pseudo_inverse_ = jacobian_.transpose() * jacobian_jacobian_tranpose_.inverse();
}

void ChompOptimizer::calculateTotalIncrements()
{
  for (int i=0; i<num_joints_; i++)
  {
    final_increments_.col(i) = parameters_->getLearningRate() *
        (
            joint_costs_[i].getQuadraticCostInverse() *
            (
                parameters_->getSmoothnessCostWeight() * smoothness_increments_.col(i) +
                parameters_->getObstacleCostWeight() * collision_increments_.col(i)
            )
        );
  }

}

void ChompOptimizer::addIncrementsToTrajectory()
{
//  double scale = 1.0;
  for (int i=0; i<num_joints_; i++)
  {
    double scale = 1.0;
    double max = final_increments_.col(i).maxCoeff();
    double min = final_increments_.col(i).minCoeff();
    double max_scale = planning_group_->chomp_joints_[i].joint_update_limit_ / fabs(max);
    double min_scale = planning_group_->chomp_joints_[i].joint_update_limit_ / fabs(min);
    if (max_scale < scale)
      scale = max_scale;
    if (min_scale < scale)
      scale = min_scale;
    group_trajectory_.getFreeTrajectoryBlock().col(i) += scale * final_increments_.col(i);
  }
  //ROS_DEBUG("Scale: %f",scale);
  //group_trajectory_.getFreeTrajectoryBlock() += scale * final_increments_;
}

void ChompOptimizer::updateFullTrajectory()
{
  full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}

void ChompOptimizer::debugCost()
{
  double cost = 0.0;
  for (int i=0; i<num_joints_; i++)
    cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
  cout << "Cost = " << cost << endl;
}

double ChompOptimizer::getTrajectoryCost()
{
  return getSmoothnessCost() + getCollisionCost();
}

double ChompOptimizer::getSmoothnessCost()
{
  double smoothness_cost = 0.0;
  // joint costs:
  for (int i=0; i<num_joints_; i++)
    smoothness_cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));

  return parameters_->getSmoothnessCostWeight() * smoothness_cost;
}

double ChompOptimizer::getCollisionCost()
{
  double collision_cost = 0.0;

  double worst_collision_cost = 0.0;
  worst_collision_cost_state_ = -1;

  // collision costs:
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    double state_collision_cost = 0.0;
    for (int j=0; j<num_collision_points_; j++)
    {
      state_collision_cost += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
    }
    collision_cost += state_collision_cost;
    if (state_collision_cost > worst_collision_cost)
    {
      worst_collision_cost = state_collision_cost;
      worst_collision_cost_state_ = i;
    }
  }

  return parameters_->getObstacleCostWeight() * collision_cost;
}

void ChompOptimizer::handleJointLimits()
{
  for (int joint=0; joint<num_joints_; joint++)
  {
    if (!planning_group_->chomp_joints_[joint].has_joint_limits_)
      continue;

    double joint_max = planning_group_->chomp_joints_[joint].joint_limit_max_;
    double joint_min = planning_group_->chomp_joints_[joint].joint_limit_min_;

    int count = 0;

    bool violation = false;
    do
    {
      double max_abs_violation =  1e-6;
      double max_violation = 0.0;
      int max_violation_index = 0;
      violation = false;
      for (int i=free_vars_start_; i<=free_vars_end_; i++)
      {
        double amount = 0.0;
        double absolute_amount = 0.0;
        if (group_trajectory_(i, joint) > joint_max)
        {
          amount = joint_max - group_trajectory_(i, joint);
          absolute_amount = fabs(amount);
        }
        else if (group_trajectory_(i, joint) < joint_min)
        {
          amount = joint_min - group_trajectory_(i, joint);
          absolute_amount = fabs(amount);
        }
        if (absolute_amount > max_abs_violation)
        {
          max_abs_violation = absolute_amount;
          max_violation = amount;
          max_violation_index = i;
          violation = true;
        }
      }

      if (violation)
      {
        int free_var_index = max_violation_index - free_vars_start_;
        double multiplier = max_violation / joint_costs_[joint].getQuadraticCostInverse()(free_var_index,free_var_index);
        group_trajectory_.getFreeJointTrajectoryBlock(joint) +=
            multiplier * joint_costs_[joint].getQuadraticCostInverse().col(free_var_index);
      }
      if (++count > 10)
        break;
    }
    while(violation);
  }
}

void ChompOptimizer::performForwardKinematics()
{
  double invTime = 1.0 / group_trajectory_.getDiscretization();
  double invTimeSq = invTime*invTime;

  // calculate the forward kinematics for the fixed states only in the first iteration:
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (iteration_==0)
  {
    start = 0;
    end = num_vars_all_-1;
  }

  is_collision_free_ = true;

  // for each point in the trajectory
  for (int i=start; i<=end; ++i)
  {
    int full_traj_index = group_trajectory_.getFullTrajectoryIndex(i);
    full_trajectory_->getTrajectoryPointKDL(full_traj_index, kdl_joint_array_);

    //ROS_INFO_STREAM("Trajectory point " << i << " index " << full_traj_index);
//     for(unsigned int j = 0; j < kdl_joint_array_.rows(); j++) {
//       ROS_INFO_STREAM(j << " " << kdl_joint_array_(j));
//     }

    if (iteration_==0)
    {
      planning_group_->fk_solver_->JntToCartFull(kdl_joint_array_, joint_pos_[i], joint_axis_[i], segment_frames_[i]);
    }
    else
    {
      planning_group_->fk_solver_->JntToCartPartial(kdl_joint_array_, joint_pos_[i], joint_axis_[i], segment_frames_[i]);
    }

    //robot_model_->getForwardKinematicsSolver()->JntToCart(kdl_joint_array_, joint_pos_[i], joint_axis_[i], segment_frames_[i]);

    state_is_in_collision_[i] = false;

    // calculate the position of every collision point:
    for (int j=0; j<num_collision_points_; j++)
    {
      planning_group_->collision_points_[j].getTransformedPosition(segment_frames_[i], collision_point_pos_[i][j]);
      //int segment_number = planning_group_->collision_points_[j].getSegmentNumber();
      //collision_point_pos_[i][j] = segment_frames_[i][segment_number] * planning_group_->collision_points_[j].getPosition();

      bool colliding = collision_space_->getCollisionPointPotentialGradient(planning_group_->collision_points_[j],
          collision_point_pos_eigen_[i][j],
          collision_point_potential_[i][j],
          collision_point_potential_gradient_[i][j]);

      point_is_in_collision_[i][j] = colliding;

      if (colliding)
        state_is_in_collision_[i] = true;
    }

    if (state_is_in_collision_[i])
    {
      is_collision_free_ = false;
    }
  }

  // now, get the vel and acc for each collision point (using finite differencing)
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    for (int j=0; j<num_collision_points_; j++)
    {
      SetToZero(collision_point_vel_[i][j]);
      SetToZero(collision_point_acc_[i][j]);
      for (int k=-DIFF_RULE_LENGTH/2; k<=DIFF_RULE_LENGTH/2; k++)
      {
        collision_point_vel_[i][j] += (invTime * DIFF_RULES[0][k+DIFF_RULE_LENGTH/2]) *
            collision_point_pos_[i+k][j];
        collision_point_acc_[i][j] += (invTimeSq * DIFF_RULES[1][k+DIFF_RULE_LENGTH/2]) *
            collision_point_pos_[i+k][j];
      }
      // get the norm of the velocity:
      collision_point_vel_mag_[i][j] = collision_point_vel_eigen_[i][j].norm();
    }
  }

  if (is_collision_free_)
    collision_free_iteration_++;
  else
    collision_free_iteration_ = 0;

}

void ChompOptimizer::eigenMapTest()
{
  double foo_eigen;
  double foo_kdl;

  cout << "Eigen location: " << &(joint_axis_eigen_[free_vars_start_][0](0)) <<
          "  KDL location: " << &(joint_axis_[free_vars_start_][0](0)) << endl;

  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen==foo_kdl);

  joint_axis_eigen_[free_vars_start_][0](0) = 1.0;
  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_kdl == foo_eigen);

  joint_axis_[free_vars_start_][0](0) = 2.0;
  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen == foo_kdl);

  foo_eigen = joint_pos_eigen_[free_vars_start_][0](0);
  foo_kdl = joint_pos_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen==foo_kdl);

  foo_eigen = collision_point_pos_eigen_[free_vars_start_][5](0);
  foo_kdl = collision_point_pos_[free_vars_start_][5](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  ROS_ASSERT(foo_eigen==foo_kdl);
}

void ChompOptimizer::perturbTrajectory()
{
  //int mid_point = (free_vars_start_ + free_vars_end_) / 2;
  if (worst_collision_cost_state_ < 0)
    return;
  int mid_point = worst_collision_cost_state_;
  planning_group_->getRandomState(random_state_);

  // convert the state into an increment
  random_state_ -= group_trajectory_.getTrajectoryPoint(mid_point).transpose();

  // project the increment orthogonal to joint velocities
  group_trajectory_.getJointVelocities(mid_point, joint_state_velocities_);
  joint_state_velocities_.normalize();
  random_state_ = (Eigen::MatrixXd::Identity(num_joints_, num_joints_) - joint_state_velocities_*joint_state_velocities_.transpose()) * random_state_;

  int mp_free_vars_index = mid_point - free_vars_start_;
  for (int i=0; i<num_joints_; i++)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(i) +=
        joint_costs_[i].getQuadraticCostInverse().col(mp_free_vars_index) * random_state_(i);
  }
}

void ChompOptimizer::getRandomMomentum()
{
  if (is_collision_free_)
    random_momentum_.setZero(num_vars_free_, num_joints_);
  else
  for (int i=0; i<num_joints_; ++i)
  {
    multivariate_gaussian_[i].sample(random_joint_momentum_);
    random_momentum_.col(i) = stochasticity_factor_ * random_joint_momentum_;
  }
}

void ChompOptimizer::updateMomentum()
{
  //double alpha = 1.0 - parameters_->getHmcStochasticity();
  double eps = parameters_->getHmcDiscretization();
  if (iteration_ > 0)
    momentum_ = (momentum_ + eps*final_increments_);
  else
    momentum_ = random_momentum_;
  //momentum_ = alpha * (momentum_ + eps*final_increments_) + sqrt(1.0-alpha*alpha)*random_momentum_;
}

void ChompOptimizer::updatePositionFromMomentum()
{
  double eps = parameters_->getHmcDiscretization();
  group_trajectory_.getFreeTrajectoryBlock() += eps * momentum_;
}

void ChompOptimizer::animatePath()
{
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    visualizeState(i);
    //ros::WallDuration(group_trajectory_.getDiscretization()).sleep();
    ros::WallDuration(.05).sleep();
  }
}

void ChompOptimizer::animateEndeffector()
{
  visualization_msgs::Marker msg;
  msg.points.resize(num_vars_free_);
 // int joint_index = planning_group_->chomp_joints_[num_joints_-1].kdl_joint_index_;
  int sn = animate_endeffector_segment_number_;
  for (int i=0; i<num_vars_free_; ++i)
  {
    int j = i+free_vars_start_;
    msg.points[i].x = segment_frames_[j][sn].p.x();
    msg.points[i].y = segment_frames_[j][sn].p.y();
    msg.points[i].z = segment_frames_[j][sn].p.z();
  }
  msg.header.frame_id = robot_model_->getReferenceFrame();
  msg.header.stamp = ros::Time();
  msg.ns = "chomp_endeffector";
  msg.id = 0;
  msg.type = visualization_msgs::Marker::SPHERE_LIST;
  msg.action = visualization_msgs::Marker::ADD;
  double scale = 0.05;
  msg.scale.x = scale;
  msg.scale.y = scale;
  msg.scale.z = scale;
  msg.color.a = 0.6;
  msg.color.r = 0.5;
  msg.color.g = 1.0;
  msg.color.b = 0.3;
  vis_marker_pub_.publish(msg);
  ros::WallDuration(0.1).sleep();
}

void ChompOptimizer::visualizeState(int index)
{
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(num_collision_points_ + num_joints_);
  int num_arrows = 0;
  double potential_threshold = 1e-10;
  for (int i=0; i<num_collision_points_; i++)
  {
    msg.markers[i].header.frame_id = robot_model_->getReferenceFrame();
    msg.markers[i].header.stamp = ros::Time();
    msg.markers[i].ns = "chomp_collisions";
    msg.markers[i].id = i;
    msg.markers[i].type = visualization_msgs::Marker::SPHERE;
    msg.markers[i].action = visualization_msgs::Marker::ADD;
    msg.markers[i].pose.position.x = collision_point_pos_[index][i].x();
    msg.markers[i].pose.position.y = collision_point_pos_[index][i].y();
    msg.markers[i].pose.position.z = collision_point_pos_[index][i].z();
    msg.markers[i].pose.orientation.x = 0.0;
    msg.markers[i].pose.orientation.y = 0.0;
    msg.markers[i].pose.orientation.z = 0.0;
    msg.markers[i].pose.orientation.w = 1.0;
    double scale = planning_group_->collision_points_[i].getRadius()*2;
    msg.markers[i].scale.x = scale;
    msg.markers[i].scale.y = scale;
    msg.markers[i].scale.z = scale;
    msg.markers[i].color.a = 0.6;
    msg.markers[i].color.r = 0.5;
    msg.markers[i].color.g = 1.0;
    msg.markers[i].color.b = 0.3;
    if (collision_point_potential_[index][i] > potential_threshold)
      num_arrows++;
  }
  for (int j=0; j<num_joints_; ++j)
  {
    int i=num_collision_points_+j;
    int joint_index = planning_group_->chomp_joints_[j].kdl_joint_index_;
    msg.markers[i].header.frame_id = robot_model_->getReferenceFrame();
    msg.markers[i].header.stamp = ros::Time();
    msg.markers[i].ns = "chomp_collisions";
    msg.markers[i].id = i;
    msg.markers[i].type = visualization_msgs::Marker::ARROW;
    msg.markers[i].action = visualization_msgs::Marker::ADD;
    msg.markers[i].points.resize(2);
    double scale=0.2;
    double sx = joint_axis_[index][joint_index].x()*scale;
    double sy = joint_axis_[index][joint_index].y()*scale;
    double sz = joint_axis_[index][joint_index].z()*scale;
    msg.markers[i].points[0].x = joint_pos_[index][joint_index].x() - sx;
    msg.markers[i].points[0].y = joint_pos_[index][joint_index].y() - sy;
    msg.markers[i].points[0].z = joint_pos_[index][joint_index].z() - sz;
    msg.markers[i].points[1].x = joint_pos_[index][joint_index].x() + sx;
    msg.markers[i].points[1].y = joint_pos_[index][joint_index].y() + sy;
    msg.markers[i].points[1].z = joint_pos_[index][joint_index].z() + sz;
    msg.markers[i].scale.x = 0.03;
    msg.markers[i].scale.y = 0.06;
    msg.markers[i].color.a = 1.0;
    msg.markers[i].color.r = 1.0;
    msg.markers[i].color.g = 0.5;
    msg.markers[i].color.b = 0.5;
  }
  vis_marker_array_pub_.publish(msg);

  // publish arrows for distance field:
  msg.markers.resize(0);
  msg.markers.resize(num_collision_points_);
  for (int i=0; i<num_collision_points_; i++)
  {
      msg.markers[i].header.frame_id = robot_model_->getReferenceFrame();
      msg.markers[i].header.stamp = ros::Time();
      msg.markers[i].ns = "chomp_arrows";
      msg.markers[i].id = i;
      msg.markers[i].type = visualization_msgs::Marker::ARROW;
      msg.markers[i].action = visualization_msgs::Marker::ADD;
      msg.markers[i].points.resize(2);
      msg.markers[i].points[0].x = collision_point_pos_[index][i].x();
      msg.markers[i].points[0].y = collision_point_pos_[index][i].y();
      msg.markers[i].points[0].z = collision_point_pos_[index][i].z();
      msg.markers[i].points[1] = msg.markers[i].points[0];
      double scale = planning_group_->collision_points_[i].getRadius()*3;
      if (collision_point_potential_[index][i] <= potential_threshold)
        scale = 0.0;
      msg.markers[i].points[1].x -= scale*collision_point_potential_gradient_[index][i].x();
      msg.markers[i].points[1].y -= scale*collision_point_potential_gradient_[index][i].y();
      msg.markers[i].points[1].z -= scale*collision_point_potential_gradient_[index][i].z();
      msg.markers[i].scale.x = 0.01;
      msg.markers[i].scale.y = 0.03;
      msg.markers[i].color.a = 1.0;
      msg.markers[i].color.r = 0.2;
      msg.markers[i].color.g = 0.2;
      msg.markers[i].color.b = 1.0;
  }
  vis_marker_array_pub_.publish(msg);
}

} // namespace chomp
