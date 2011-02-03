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

 \file    dmp_policy.cpp

 \author  Peter Pastor
 \date    Jun 10, 2010

 **********************************************************************/

// system includes
#include <assert.h>

// ros includes
#include <Eigen/Array>
// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

// local includes
#include <policy_library/dmp_policy.h>

namespace library 
{

DMPPolicy::DMPPolicy() :
    initialized_(false)
{
}

DMPPolicy::~DMPPolicy()
{
}

bool DMPPolicy::initialize(boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp)
{

    if (!dmp->isInitialized())
    {
        ROS_ERROR("DMP is not initialized.");
        return false;
    }
    dmp_ = dmp;
    return (initialized_ = true);
}

bool DMPPolicy::setNumTimeSteps(const int num_time_steps)
{
    if (!initialized_)
    {
        return false;
    }
    num_time_steps_ = num_time_steps;
    return true;
}

bool DMPPolicy::getNumTimeSteps(int& num_time_steps)
{
    num_time_steps = num_time_steps_;
    return true;
}


bool DMPPolicy::getNumDimensions(int& num_dimensions)
{
    if (!initialized_)
    {
        return false;
    }
    num_dimensions = dmp_->getNumTransformationSystems();
    return true;
}

bool DMPPolicy::getNumParameters(std::vector<int>& num_params)
{
    if (!initialized_)
    {
        return false;
    }
    num_params.clear();
    return dmp_->getNumRFS(num_params);
}

bool DMPPolicy::getBasisFunctions(std::vector<MatrixXd>& basis_functions)
{
    if (!initialized_)
    {
        return false;
    }

    std::vector<MatrixXd> pure_basis_functions;
    if(!dmp_->getBasisFunctions(num_time_steps_, pure_basis_functions))
    {
        ROS_ERROR("Could not get basis function matrix.");
        return false;
    }

    VectorXd can_system_vector = VectorXd::Zero(num_time_steps_);
    if(!dmp_->getCanonicalSystem(num_time_steps_, can_system_vector))
    {
        ROS_ERROR("Could not evaluate the canonical system.");
        return false;
    }

    std::vector<int> num_thetas;
    if (!getNumParameters(num_thetas))
    {
        ROS_ERROR("Could not get number of parameters.");
        return false;
    }

    basis_functions.clear();
    for (int d=0; d<dmp_->getNumTransformationSystems(); d++)
    {
        MatrixXd basis_function_matrix = MatrixXd::Zero(num_time_steps_, num_thetas[d]);
        for (int j=0; j<num_thetas[d]; j++)
        {
            basis_function_matrix.col(j) = pure_basis_functions[d].col(j).cwise() * can_system_vector;
        }
        basis_functions.push_back(basis_function_matrix);
    }

    return true;
}


bool DMPPolicy::getControlCosts(std::vector<MatrixXd>& control_costs)
{
    if (!initialized_)
    {
        return false;
    }
    control_costs.clear();
    std::vector<int> num_thetas;
    if (!getNumParameters(num_thetas))
    {
        ROS_ERROR("Could not get number of parameters.");
        return false;
    }
    for (std::vector<int>::iterator it = num_thetas.begin(); it != num_thetas.end(); it++)
    {
        MatrixXd idendity_control_cost_matrix = MatrixXd::Identity(*it, *it);
        control_costs.push_back(idendity_control_cost_matrix);
    }
    return true;
}

bool DMPPolicy::updateParameters(const std::vector<MatrixXd>& updates)
{
    if (!initialized_)
    {
        return false;
    }

    std::vector<VectorXd> theta_vectors;
    if (!dmp_->getThetas(theta_vectors))
    {
        ROS_ERROR("Could not get parameter vector.");
        return false;
    }
    assert(theta_vectors.size() == updates.size());

    std::vector<MatrixXd> basis_functions;
    if (!dmp_->getBasisFunctions(num_time_steps_, basis_functions))
    {
        ROS_ERROR("Could not get basis function matrix.");
        return false;
    }
    assert(basis_functions.size() == updates.size());

    for (int d = 0; d < static_cast<int> (updates.size()); d++)
    {
        int num_rfs = basis_functions[d].cols();

        assert(updates[d].rows() == static_cast<int>(num_time_steps_));
        assert(updates[d].cols() == static_cast<int>(theta_vectors[d].size()));
        assert(updates[d].rows() == basis_functions[d].rows());
        assert(updates[d].cols() == basis_functions[d].cols());

        for (int j = 0; j < num_rfs; j++)
        {
            double sum_time_weight_times_basis_function_weight = 0;
            double sum_time_weight_times_basis_function_weight_times_update = 0;
            for (int i = 0; i < num_time_steps_; i++)
            {
                double time_weight = num_time_steps_ - i;
                sum_time_weight_times_basis_function_weight += (time_weight * basis_functions[d](i,j));
                sum_time_weight_times_basis_function_weight_times_update += ((time_weight * basis_functions[d](i,j)) * updates[d](i,j));
            }

            // update the theta vector
            theta_vectors[d](j) += sum_time_weight_times_basis_function_weight_times_update / sum_time_weight_times_basis_function_weight;
        }
    }

    if (!dmp_->setThetas(theta_vectors))
    {
        ROS_ERROR("Could not set parameter vector.");
        return false;
    }

    return true;
}

bool DMPPolicy::getParameters(std::vector<VectorXd>& parameters)
{
    if (!initialized_)
    {
        return false;
    }

    parameters.clear();
    return dmp_->getThetas(parameters);
}

bool DMPPolicy::setParameters(const std::vector<VectorXd>& parameters)
{
    if (!initialized_)
    {
        return false;
    }
    return dmp_->setThetas(parameters);
}

bool DMPPolicy::getDMP(boost::shared_ptr<dmp::DynamicMovementPrimitive>& dmp)
{
    if (!initialized_)
        return false;
    dmp = dmp_;
    return true;
}

std::string DMPPolicy::getInfoString()
{
    return dmp_->getInfoString();
}

std::string DMPPolicy::getFileName(const int trial_id)
{
    return dmp_->getFileName(trial_id);
}

bool DMPPolicy::readFromDisc(const std::string directory_name, const int dmp_id, const int trial_id)
{
    return dmp_->readFromDisc(directory_name, dmp_id, trial_id);
}

bool DMPPolicy::readFromDisc(const std::string abs_bagfile_name)
{
    return dmp_->readFromDisc(abs_bagfile_name);
}

bool DMPPolicy::writeToDisc(const int trial_id)
{
    return dmp_->writeToDisc(trial_id);
}

bool DMPPolicy::writeToDisc(const std::string abs_bagfile_name)
{
    return dmp_->writeToDisc(abs_bagfile_name);
}

std::string DMPPolicy::getClassName()
{
    return "DMPPolicy";
}

}
