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

#ifndef LWR_H_
#define LWR_H_

// system includes
#include <vector>
#include <string>

// ros includes
#include <ros/ros.h>

#include <Eigen/Eigen>

// local includes
#include <lwr/Model.h>

namespace lwr
{

class LocallyWeightedRegression
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     *
     * @return
     */
    LocallyWeightedRegression();
    ~LocallyWeightedRegression();

    /*! Initialized the LWR model
     *
     * @param num_rfs
     * @param activation
     * @return True on success, false on failure
     */
    bool initialize(const std::string directory_name, const int num_rfs, const double activation, const bool exponentially_spaced, const double can_sys_cutoff);

    /*! Initializes the LWR model using the parameters from the paramserver in the namespace parameter_namespace
     *
     * @param parameter_namespace
     * @return
     */
    bool initialize(const std::string parameter_namespace);

    /*! initialized LWR model from message
     * @param model
     * @return
     */
    bool initFromMessage(const lwr::Model& model);

    /*! Writes LWR model to message
     * @param model
     * @return
     */
    bool writeToMessage(lwr::Model& model);

    /*!
     * @param x
     * @param y
     * @return True on success, false on failure
     */
    bool learnWeights(const Eigen::VectorXd& x_input_vector, const Eigen::VectorXd& y_target_vector);

    /*!
     *
     * @param x
     * @param y
     * @return True on success, false on failure
     */
    bool predict(const double x_query, double& y_prediction);

    /*!
     * @param x_input_vector
     * @param basis_function_matrix
     * @return True on success, false on failure
     */
    bool generateBasisFunctionMatrix(const Eigen::VectorXd& x_input_vector, Eigen::MatrixXd& basis_function_matrix);

    /*!
     * Gets the theta vector
     * @param thetas
     * @return True on success, false on failure
     */
    bool getThetas(Eigen::VectorXd& thetas);

    /*!
     * Sets the theta vector
     * @param thetas
     * @return True on success, false on failure
     */
    bool setThetas(const Eigen::VectorXd& thetas);

    /*!
     * updates the theta vectors (thetas += delta_thetas)
     * @param delta_thetas
     * @return True on success, false on failure
     */
    bool updateThetas(const Eigen::VectorXd& delta_thetas);

    /*!
     * @param widths
     * @param centers
     * @return True on success, false on failure
     */
    bool getWidthsAndCenters(Eigen::VectorXd& widths, Eigen::VectorXd& centers);

     /*! Writes the LWR model to file
     *
     * @param directory_name (input) Directory name in which the LWR model is stored.
     * @return True on success, false on failure
     */
    bool writeToDisc(const std::string directory_name);

    /*!
     * @param directory_name
     * @param item_id
     * @return
     */
    bool initializeFromDisc(const std::string directory_name);

    /*! Returns a string containing relevant information of the LWR model
     * @return
     */
    std::string getInfoString();

    /*! Gets the number of receptive fields
     * @return
     */
    bool getNumRFS(int& num_rfs);

private:

    /*! Evaluates the kernel
     */
    double getKernel(const double x_input, const int center_index);

    /*! Indicates whether the LWR model is initialized
     */
    bool initialized_;

    /*! Number of receptive fields used in this LWR model
     *
     */
    int num_rfs_;

    /*! determines whether gaussians are exponentially spaced (true) or equally spaced (false)
     *  exponential scale is used to deal with the nonlinearity of the
     *  phase variable of the canonical system of a DMP
     */
    bool exponentially_spaced_;

    /*! Centers of the receptive fields
     *
     */
    Eigen::VectorXd centers_;


    /*! Bandwidth used for each local model
     *
     */
    Eigen::VectorXd widths_;

    /*! Slopes of the local linear approximations
     *
     */
    Eigen::VectorXd thetas_;

    /*! Offsets of the local linear approximations. (currently not implemented)
     *
     */
    Eigen::VectorXd offsets_;
};

}

#endif /* LWR_H_ */
