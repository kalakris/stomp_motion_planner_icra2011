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

 \file    test_dmp.h

 \author  Ludovic Righetti and Peter Pastor
 \date    May 26, 2010

 **********************************************************************/

#ifndef SIMPLEDMP_H_
#define SIMPLEDMP_H_

#include <cmath>
#include <cstdlib>

#include <Eigen/Eigen>

USING_PART_OF_NAMESPACE_EIGEN

namespace test_dmp {

class TestDMP
{

public:

    /*!
     * @return
     */
    TestDMP();
    ~TestDMP();

    /*!
     * @param num_rfs
     * @param num_dimensions
     * @param num_steps
     * @return
     */
    bool initialize(int num_rfs, int num_dimensions, int num_steps);

    /*!
     * @param dt
     * @param initial_positions
     * @param initial_velocities
     * @param theta_matrix
     * @param basis_function_matrix
     * @param time
     * @param positions
     * @param velocities
     * @param canonical_system_s
     */
    void iteratePeriodicPolicy(double dt, double initial_positions, double initial_velocities, const MatrixXd &theta_matrix, MatrixXd &basis_function_matrix,
                               VectorXd &time, MatrixXd &positions, MatrixXd &velocities, VectorXd &canonical_system_s);

    /*!
     * @param dt
     * @param initial_positions
     * @param initial_velocities
     * @param goal_positions
     * @param theta_matrix
     * @param noise_matrix
     * @param basis_function_matrix
     * @param time
     * @param positions
     * @param velocities
     * @param canonical_system_s
     * @return
     */
    bool iterateDiscretePolicy(double dt, const VectorXd &initial_positions, const VectorXd &initial_velocities, const VectorXd &goal_positions, const MatrixXd &theta_matrix,
                               const MatrixXd &noise_matrix, MatrixXd &basis_function_matrix, VectorXd &time, MatrixXd &positions, MatrixXd &velocities,
                               VectorXd &canonical_system_s);

private:

    bool initialized_;

    int num_rfs_;
    int num_steps_;
    int num_dimensions_;

    double dt_;

    VectorXd centers_;

    //for the periodic policy
    double rcp_alpha_;
    double rcp_beta_;
    double rcp_goal_;
    double rcp_omega_;

    //for the discrete policy
    double dcp_tau_;
    double dcp_k_;
    double dcp_d_;
};

}

#endif /* SIMPLEDMP_H_ */
