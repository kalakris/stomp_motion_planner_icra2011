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

 \file    test_dmp.cpp

 \author  Ludovic Righetti and Peter Pastor
 \date    May 26, 2010

 **********************************************************************/

// ros includes
#include <ros/ros.h>

// local includes
#include <policy_improvement_test/simple_dmp.h>

namespace test_dmp {

TestDMP::TestDMP() : initialized_(false)
{
}

TestDMP::~TestDMP()
{
    // TODO Auto-generated destructor stub
}

bool TestDMP::initialize(int num_rfs, int num_dimensions, int num_steps)
{
    num_rfs_ = num_rfs;
    num_steps_ = num_steps;
    num_dimensions_ = num_dimensions;

    centers_ = VectorXd::Zero(num_rfs);

    //policies parameter
    dt_ = 0.001;

    //periodic parameters
    rcp_alpha_ = 2;
    rcp_beta_ = 8;
    rcp_goal_ = 0;
    rcp_omega_ = 4;

    //discrete parameters
    dcp_tau_ = 1;
    dcp_k_ = 25;
    dcp_d_ = 2 * sqrt(25);

    initialized_ = true;
    return initialized_;
}

void TestDMP::iteratePeriodicPolicy(double dt, double initial_positions, double initial_velocities, const MatrixXd &theta_matrix,
                                    MatrixXd &basis_function_matrix, VectorXd &time, MatrixXd &positions, MatrixXd &velocities, VectorXd &canonical_system_s)
{

    for (int i = 0; i < num_rfs_; i++)
    {
        centers_(i) = 2 * 3.14 / num_rfs_ * i;
    }

    VectorXd c_gauss(num_rfs_);

    double c_dy, c_dz, c_dphi;
    double c_y = initial_positions;
    double c_z = initial_velocities / rcp_omega_;
    double c_phi = 0;

    double c_t = 0;

    int innerLoop = (int)(dt / dt_);

    for (int j = 0; j < num_rfs_; j++)
    {
        c_gauss(j) = exp(2.5 * num_rfs_ * (cos(c_phi - centers_(j)) - 1));
    }

    for (int i = 0; i < num_steps_; i++)
    {

        basis_function_matrix.block(i, 0, 1, num_rfs_) = c_gauss.transpose();
        positions(i) = c_y;
        velocities(i) = c_z;
        canonical_system_s(i) = c_phi;
        time(i) = c_t;

        VectorXd tmpTheta(num_rfs_);
        tmpTheta = (theta_matrix.block(i, 0, 1, num_rfs_)).transpose();

        for (int j = 0; j < innerLoop; j++)
        {
            for (int k = 0; k < num_rfs_; k++)
            {
                c_gauss(k) = exp(2.5 * num_rfs_ * (cos(c_phi - centers_(k)) - 1));
            }

            c_dphi = rcp_omega_;
            c_dy = rcp_omega_ * c_z;

            c_dz = rcp_omega_ * (rcp_alpha_ * (rcp_beta_ * (rcp_goal_ - c_y) - c_z) + c_gauss.dot(tmpTheta) / c_gauss.sum());

            c_y += c_dy * dt_;
            c_z += c_dz * dt_;
            c_phi += c_dphi * dt_;
            c_t += dt_;
        }
    }
}

bool TestDMP::iterateDiscretePolicy(double dt, const VectorXd &initial_positions, const VectorXd &initial_velocities,
                                    const VectorXd &goal_positions, const MatrixXd &theta_matrix,
                                    const MatrixXd &noise_matrix, MatrixXd &basis_function_matrix,
                                    VectorXd &time, MatrixXd &positions, MatrixXd &velocities,
                                    VectorXd &canonical_system_s)
{

    if(!initialized_)
    {
        ROS_ERROR("Policy is not initialized.");
        return initialized_;
    }

    for (int i = 0; i < num_rfs_; i++)
    {
        centers_(i) = (double)i / (double)num_rfs_;
    }

    VectorXd c_gauss(num_rfs_);

    VectorXd c_dy(num_dimensions_);
    VectorXd c_dz(num_dimensions_);

    VectorXd c_y(num_dimensions_);
    c_y = initial_positions;
    VectorXd c_y0(num_dimensions_);
    c_y0 = initial_positions;

    VectorXd c_z(num_dimensions_);
    c_z = initial_velocities * dcp_tau_;

    VectorXd c_goal(num_dimensions_);
    c_goal = goal_positions;

    double c_ds;
    double c_s = 1;
    double c_t = 0;

    int num_iterations = (int)(dt / dt_); // TODO: check what this actually means

    for (int j = 0; j < num_rfs_; j++)
    {
        c_gauss(j) = exp(-num_rfs_ * (c_s - centers_(j)) * (c_s - centers_(j)));
    }

    MatrixXd tmp_theta_matrix(num_rfs_, num_dimensions_);
    int max_activ_index;
    for (int i = 0; i < num_steps_; i++)
    {
        basis_function_matrix.block(i, 0, 1, num_rfs_) = c_gauss.transpose();
        positions.block(i, 0, 1, num_dimensions_) = c_y.transpose();
        velocities.block(i, 0, 1, num_dimensions_) = c_z.transpose();

        canonical_system_s(i) = c_s;
        time(i) = c_t;

        for (int j = 0; j < num_iterations; j++)
        {
            for (int k = 0; k < num_rfs_; k++)
            {
                c_gauss(k) = exp(-num_rfs_ * (c_s - centers_(k)) * (c_s - centers_(k)));
            }
            c_gauss = c_gauss / c_gauss.sum();

            c_gauss.maxCoeff(&max_activ_index);
            tmp_theta_matrix = theta_matrix;
            tmp_theta_matrix.block(max_activ_index, 0, 1, num_dimensions_) = noise_matrix.block(max_activ_index, 0, 1, num_dimensions_);

            c_ds = -1 / dcp_tau_ * 5 * c_s;
            c_dy = 1 / dcp_tau_ * c_z;

            c_dz = (double)1.0 / dcp_tau_ * ( (dcp_k_ * (c_goal - c_y))  - (dcp_d_ * c_z) - (dcp_k_ * (c_goal - c_y0) * c_s) + (dcp_k_ * (c_gauss.transpose() * tmp_theta_matrix).transpose()));
            // c_dz = 1 / dcp_tau_ * (dcp_k_ * ( -dcp_d_ * (c_y.cwise() - dcp_goal_) - c_z) + (c_gauss.transpose() * tmp_theta_matrix).transpose());

            // ROS_INFO_STREAM("goal = " << goal_positions);

            c_y += c_dy * dt_;
            c_z += c_dz * dt_;
            c_s += c_ds * dt_;
            c_t += dt_;
        }
    }

    return true;
}

}
