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

 \file    dmp_policy.h

 \author  Peter Pastor
 \date    Jun 10, 2010

 **********************************************************************/

#ifndef DMP_POLICY_H_
#define DMP_POLICY_H_

// system includes

// ros includes
#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>

#include <dmp_motion_generation/dynamic_movement_primitive.h>

// local includes
#include <policy_library/policy.h>

namespace library
{

class DMPPolicy : public Policy
{

public:

    DMPPolicy();
    virtual ~DMPPolicy();

    /*!
     * Initializes the DMPPolicy object with a pointer to a already initialized dmp
     * @param dmp
     * @return true on success, false on failure
     */
    bool initialize(boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp);

    /**
     * Retrieves the underlying DMP from the policy
     * @param dmp
     * @return true on success, false on failure
     */
    bool getDMP(boost::shared_ptr<dmp::DynamicMovementPrimitive>& dmp);

    /**
     * Sets the number of time steps used in reinforcement learning
     * @param num_time_steps
     * @return true on success, false on failure
     */
    bool setNumTimeSteps(const int num_time_steps);

    /**
     * Gets the number of time steps used in reinforcement learning
     * @param num_time_steps
     * @return true on success, fase on failure
     */
    bool getNumTimeSteps(int& num_time_steps);

    /**
     * Gets the number of dimensions
     * @param num_dimensions (output) number of dimensions
     * @return true on success, false on failure
     */
    bool getNumDimensions(int& num_dimensions);

    /**
     * Gets the number of policy parameters per dimension
     *
     * @param num_params (output) vector of number of parameters per dimension
     * @return true on success, false on failure
     */
    bool getNumParameters(std::vector<int>& num_params);

    /**
     * Gets the basis functions that multiply the policy parameters in the dynamical system
     * @param basis_function_matrix_array (output) Array of "num_time_steps x num_parameters" matrices, per dimension
     * @return true on success, false on failure
     */
    bool getBasisFunctions(std::vector<Eigen::MatrixXd>& basis_functions);

    /**
     * Gets the positive semi-definite matrix of the quadratic control cost
     * The weight of this control cost is provided by the task
     *
     * @param control_cost_matrix (output) Array of square, positive semi-definite matrix: num_params x num_params
     * @return true on success, false on failure
     */
    bool getControlCosts(std::vector<Eigen::MatrixXd>& control_costs);

    /**
     * Update the policy parameters based on the updates per timestep
     * @param updates (input) parameter updates per time-step, num_time_steps x num_parameters
     * @return true on success, false on failure
     */
    bool updateParameters(const std::vector<Eigen::MatrixXd>& updates);

    /**
     * Get the policy parameters per dimension
     * @param parameters (output) array of parameter vectors
     * @return true on success, false on failure
     */
    bool getParameters(std::vector<Eigen::VectorXd>& parameters);

    /**
     * Set the policy parameters per dimension
     * @param parameters (input) array of parameter vectors
     * @return true on success, false on failure
     */
    bool setParameters(const std::vector<Eigen::VectorXd>& parameters);

    /**
     * Returns a string containing some information about the policy, mainly used for debugging and displaying information on a GUI.
     * @return string containing information about the DMP.
     */
    std::string getInfoString();

    /**
     * @return
     */
    std::string getFileName(const int trial_id);

    /**
     * Reads a DMP with id dmp_id from disc stored in directory directory_name
     * @param directory_name (input) the name of the directory containing the DMP.
     * @param dmp_id (intput) the ID of the DMP.
     * @return true on success, false on failure
     */
    bool readFromDisc(const std::string directory_name, const int dmp_id, const int trial_id = 0);
    bool readFromDisc(const std::string abs_bagfile_name);

    /**
     * Writes the DMP to disc into the directory given by item_name_
     * @return true on success, false on failure
     */
    bool writeToDisc(const int trial_id = 0);
    bool writeToDisc(const std::string abs_bagfile_name);

    /**
     * Returns the name of the class.
     * @return
     */
    std::string getClassName();

private:

    /*!
     */
    bool initialized_;
    int num_time_steps_;

    /*!
     */
    boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp_;

};

}

#endif /* DMP_POLICY_H_ */
