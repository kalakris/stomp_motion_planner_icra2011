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

 \file    dmp_math_helper.cpp

 \author  Peter Pastor
 \date    Jun 6, 2010

 **********************************************************************/

// system includes
#include <math.h>

// ros includes
#include <ros/ros.h>

// local includes
#include <dmp_motion_generation/math_helper.h>
#include <dmp_motion_generation/constants.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

namespace dmp
{

bool MathHelper::calculate_min_jerk_next_step(const VectorXd &start, const VectorXd &goal, const double duration, const double delta_t, VectorXd &next)
{
    if ((duration <= 0) || (delta_t <= 0) || (delta_t > duration))
    {
        ROS_ERROR_COND(duration <= 0, "Duration is less or equal 0.");
        ROS_ERROR_COND(delta_t <= 0, "Delta t is less or equal 0.");
        ROS_ERROR_COND(delta_t > duration, "Delta t is greater than duration.");
        return false;
    }
    if ((start.size() != POS_VEL_ACC) || (goal.size() != POS_VEL_ACC) || (next.size() != POS_VEL_ACC))
    {
        ROS_ERROR_COND(start.size() != POS_VEL_ACC, "Start vector has wrong size (%i), should be (%i).", start.size(), POS_VEL_ACC);
        ROS_ERROR_COND(goal.size() != POS_VEL_ACC, "Goal vector has wrong size (%i), should be (%i).", goal.size(), POS_VEL_ACC);
        ROS_ERROR_COND(next.size() != POS_VEL_ACC, "Next vector has wrong size (%i), should be (%i).", next.size(), POS_VEL_ACC);
        return false;
    }

    const double t1 = delta_t;
    const double t2 = t1 * delta_t;
    const double t3 = t2 * delta_t;
    const double t4 = t3 * delta_t;
    const double t5 = t4 * delta_t;

    const double tau1 = duration;
    const double tau2 = tau1 * duration;
    const double tau3 = tau2 * duration;
    const double tau4 = tau3 * duration;
    const double tau5 = tau4 * duration;

    // calculate the constants
    const double dist = goal(_POS_-1) - start(_POS_-1);
    const double c1 = 6. * dist / tau5 + (goal(_ACC_-1) - start(_ACC_-1)) / (2. * tau3) - 3. * (start(_VEL_-1) + goal(_VEL_-1)) / tau4;
    const double c2 = -15. * dist / tau4 + (3. * start(_ACC_-1) - 2. * goal(_ACC_-1)) / (2. * tau2) + (8. * start(_VEL_-1) + 7. * goal(_VEL_-1)) / tau3;
    const double c3 = 10. * dist / tau3 + (goal(_ACC_-1) - 3. * start(_ACC_-1)) / (2. * duration) - (6. * start(_VEL_-1) + 4. * goal(_VEL_-1)) / tau2;
    const double c4 = start(_ACC_-1) / 2.;
    const double c5 = start(_VEL_-1);
    const double c6 = start(_POS_-1);

    next(_POS_ - 1) = c1 * t5 + c2 * t4 + c3 * t3 + c4 * t2 + c5 * t1 + c6;
    next(_VEL_ - 1) = 5. * c1 * t4 + 4 * c2 * t3 + 3 * c3 * t2 + 2 * c4 * t1 + c5;
    next(_ACC_ - 1) = 20. * c1 * t3 + 12. * c2 * t2 + 6. * c3 * t1 + 2. * c4;

    return true;
}

bool MathHelper::generateMinJerkTrajectory(const VectorXd &start, const VectorXd &goal, const double duration, const double delta_t, Trajectory &trajectory)
{

    int trajectory_dimension = trajectory.getDimension();

    if ((trajectory_dimension % POS_VEL_ACC) != 0)
    {
        ROS_ERROR("Trajectory dimension (%i) must be a multiple of 3 to contain position, velocity, and acceleration information for each trace.",
                  trajectory_dimension);
        return false;
    }
    trajectory_dimension /= POS_VEL_ACC;

    if ((trajectory_dimension != start.size()) || (trajectory_dimension != goal.size()))
    {
        ROS_ERROR("Trajectory dimension (%i), does not match start (%i) and goal (%i).", trajectory_dimension, start.size(), goal.size());
        return false;
    }
    trajectory.clear();

    int num_steps = static_cast<int> (duration / delta_t);

    VectorXd tmp_current = VectorXd::Zero(POS_VEL_ACC);
    VectorXd tmp_goal = VectorXd::Zero(POS_VEL_ACC);
    VectorXd tmp_next = VectorXd::Zero(POS_VEL_ACC);
    VectorXd next = VectorXd::Zero(trajectory_dimension * POS_VEL_ACC);

    // add first trajectory point
    for (int j = 0; j < trajectory_dimension; j++)
    {
        next(j * POS_VEL_ACC + 0) = start(j);
        next(j * POS_VEL_ACC + 1) = 0;
        next(j * POS_VEL_ACC + 2) = 0;
    }
    if (!trajectory.add(next))
    {
        ROS_ERROR("Could not add first trajectory point.");
        return false;
    }

    for (int i = 1; i < num_steps; i++)
    {
        for (int j = 0; j < trajectory_dimension; j++)
        {
            if (i == 1)
            {
                tmp_current(0) = start(j);
            }
            else
            {
                // update the current state
                for (int k = 0; k < POS_VEL_ACC; k++)
                {
                    tmp_current(k) = next(j * POS_VEL_ACC + k);
                }
            }
            tmp_goal(0) = goal(j);

            if (!MathHelper::calculate_min_jerk_next_step(tmp_current, tmp_goal, duration - ((i - 1) * delta_t), delta_t, tmp_next))
            {
                ROS_ERROR("Could not compute next minimum jerk trajectory point.");
                return false;
            }

            for (int k = 0; k < POS_VEL_ACC; k++)
            {
                next(j * POS_VEL_ACC + k) = tmp_next(k);
            }
            // next.block(j*POS_VEL_ACC, 0, POS_VEL_ACC, 0) = tmp_next;
        }
        if (!trajectory.add(next))
        {
            ROS_ERROR("Could not add next trajectory point.");
            return false;
        }
    }
    return true;
}

void MathHelper::getQuaternionFromRPY(const double roll, const double pitch, const double yaw, double &qx, double &qy, double &qz, double &qw)
{
  qw = cos(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0)+sin(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
  qx = sin(roll/2.0)*cos(pitch/2.0)*cos(yaw/2.0)-cos(roll/2.0)*sin(pitch/2.0)*sin(yaw/2.0);
  qy = cos(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0)+sin(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0);
  qz = cos(roll/2.0)*cos(pitch/2.0)*sin(yaw/2.0)-sin(roll/2.0)*sin(pitch/2.0)*cos(yaw/2.0);
}

void MathHelper::computeAngularVelocityError(const VectorXd& quat1, const VectorXd& quat2, VectorXd& angular_velocity_error)
{
    angular_velocity_error(0) = quat1(0)*quat2(1) - quat2(0)*quat1(1) - (quat1(2)*quat2(3) - quat1(3)*quat2(2));
    angular_velocity_error(1) = quat1(0)*quat2(2) - quat2(0)*quat1(2) - (quat1(3)*quat2(1) - quat1(1)*quat2(3));
    angular_velocity_error(2) = quat1(0)*quat2(3) - quat2(0)*quat1(3) - (quat1(1)*quat2(2) - quat1(2)*quat2(1));
}

}
