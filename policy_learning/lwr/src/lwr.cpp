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

// system includes
#include <iostream>
#include <fstream>
#include <assert.h>

// ros includes
#include <boost/filesystem.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

// local includes
#include <lwr/lwr.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

namespace lwr
{

static const char* lwr_model_file_name = "lwr_model.bag";
static const char* lwr_model_topic_file_name = "lwr_model";

LocallyWeightedRegression::LocallyWeightedRegression() :
    initialized_(false), num_rfs_(0)
{
}

LocallyWeightedRegression::~LocallyWeightedRegression()
{
}

bool LocallyWeightedRegression::initialize(const std::string directory_name, const int num_rfs, const double activation, const bool exponentially_spaced, const double can_sys_cutoff)
{

    ROS_WARN_COND(initialized_, "LWR model already initialized. Re-initializing with new parameters.");

    if (num_rfs <= 0)
    {
        ROS_ERROR("Number of receptive fields (%i) is invalid.", num_rfs);
        initialized_ = false;
        return initialized_;
    }
    num_rfs_ = num_rfs;
    centers_ = VectorXd::Zero(num_rfs_);
    thetas_ = VectorXd::Zero(num_rfs_);
    widths_ = VectorXd::Zero(num_rfs_);
    offsets_ = VectorXd::Zero(num_rfs_);

    if(exponentially_spaced)
    {
        double last_input_x = 1.0;
        double alpha_x = -log(can_sys_cutoff);
        for (int i = 0; i < num_rfs_; ++i)
        {
            double t = (i+1) * (1. / static_cast<double> (num_rfs_ - 1)) * 1.0; // 1.0 is the default duration
            double input_x = exp(-alpha_x * t);

            // widths_(i) = -2. * log(activation) / pow(input_x - last_input_x, 2);
            widths_(i) = pow(input_x - last_input_x, 2) / -log(activation);

//            ROS_ERROR(">> width  : %i %f", i, widths_(i));
//            ROS_ERROR(">> input_x: %i %f", i, input_x);

            centers_(i) = last_input_x;
            last_input_x = input_x;
        }
    }
    else
    {
        double diff;
        if (num_rfs_ == 1)
        {
            centers_(0) = 0.5;
            diff = 0.5;
        }
        else
        {
            for (int i = 0; i < num_rfs_; i++)
            {
                centers_(i) = static_cast<double> (i) / static_cast<double> (num_rfs - 1);
            }
            diff = static_cast<double> (1.0) / static_cast<double> (num_rfs - 1);
        }
        double width = -pow(diff / static_cast<double> (2.0), 2) / log(activation);
        for (int i = 0; i < num_rfs_; i++)
        {
            widths_(i) = width;
        }
    }


    initialized_ = true;
    return initialized_;
}

bool LocallyWeightedRegression::predict(const double x_query, double &y_prediction)
{
    if (!initialized_)
    {
        ROS_ERROR("LWR model not initialized.\n");
        return initialized_;
    }

    double sx = 0;
    double sxtd = 0;
    for (int i = 0; i < num_rfs_; i++)
    {
        double psi = getKernel(x_query, i);
        sxtd += psi * thetas_(i) * x_query;
        sx += psi;
    }

    y_prediction = sxtd / sx;
    return true;
}

bool LocallyWeightedRegression::generateBasisFunctionMatrix(const VectorXd &x_input_vector, MatrixXd &basis_function_matrix)
{
    if (x_input_vector.size() == 0)
    {
        ROS_ERROR("Cannot compute psi for an empty vector.");
        return false;
    }
    assert(basis_function_matrix.rows() == x_input_vector.size());
    assert(basis_function_matrix.cols() == centers_.size());
    for (int i = 0; i < x_input_vector.size(); i++)
    {
        for (int j = 0; j < centers_.size(); j++)
        {
            basis_function_matrix(i, j) = getKernel(x_input_vector[i], j);
        }
    }
    return true;
}

bool LocallyWeightedRegression::getThetas(VectorXd &thetas)
{
    if (!initialized_)
    {
        ROS_ERROR("LWR model not initialized.\n");
        return initialized_;
    }

    assert(thetas.cols() == thetas_.cols());
    assert(thetas.rows() == thetas_.rows());

    thetas = thetas_;
    return true;
}

bool LocallyWeightedRegression::setThetas(const VectorXd &thetas)
{
    if (!initialized_)
    {
        ROS_ERROR("LWR model not initialized.\n");
        return initialized_;
    }
    assert(thetas.cols() == thetas_.cols());
    assert(thetas.rows() == thetas_.rows());
    thetas_ = thetas;
    return true;
}

bool LocallyWeightedRegression::updateThetas(const VectorXd &delta_thetas)
{
    if (!initialized_)
    {
        ROS_ERROR("LWR model not initialized.\n");
        return initialized_;
    }
    assert(delta_thetas.cols() == thetas_.cols());
    assert(delta_thetas.rows() == thetas_.rows());
    thetas_ += delta_thetas;
    return true;
}

bool LocallyWeightedRegression::getWidthsAndCenters(VectorXd &widths, VectorXd &centers)
{
    if (!initialized_)
    {
        ROS_ERROR("LWR model not initialized.\n");
        return initialized_;
    }
    widths = widths_;
    centers = centers_;
    return true;
}

double LocallyWeightedRegression::getKernel(const double x_input, const int center_index)
{
    if (!initialized_)
    {
        ROS_ERROR("LWR model not initialized.\n");
        return initialized_;
    }
    return exp(-(static_cast<double> (1.0) / widths_(center_index)) * pow(x_input - centers_(center_index), 2));
}

bool LocallyWeightedRegression::learnWeights(const VectorXd &x_input_vector, const VectorXd &y_target_vector)
{
    if (!initialized_)
    {
        ROS_ERROR("LWR model is not initialized.");
        return initialized_;
    }

    if(x_input_vector.size() != y_target_vector.size())
    {
        ROS_ERROR("Input (%i) and target (%i) vector have different sizes.", x_input_vector.size(), y_target_vector.size());
        return false;
    }

    MatrixXd basis_function_matrix = MatrixXd::Zero(x_input_vector.size(), centers_.size());
    if (!generateBasisFunctionMatrix(x_input_vector, basis_function_matrix))
    {
        ROS_ERROR("Could not generate basis function matrix.");
        return false;
    }

    MatrixXd tmp_matrix_a = MatrixXd::Zero(x_input_vector.size(), num_rfs_);
    tmp_matrix_a = x_input_vector.cwise().square() * MatrixXd::Ones(1, num_rfs_);
    tmp_matrix_a = tmp_matrix_a.cwise() * basis_function_matrix;

    VectorXd tmp_matrix_sx = VectorXd::Zero(num_rfs_, 1);
    tmp_matrix_sx = tmp_matrix_a.colwise().sum();

    MatrixXd tmp_matrix_b = MatrixXd::Zero(x_input_vector.size(), num_rfs_);
    tmp_matrix_b = x_input_vector.cwise() * y_target_vector * MatrixXd::Ones(1, num_rfs_);
    tmp_matrix_b = tmp_matrix_b.cwise() * basis_function_matrix;

    VectorXd tmp_matrix_sxtd = VectorXd::Zero(num_rfs_, 1);
    tmp_matrix_sxtd = tmp_matrix_b.colwise().sum();

    double ridge_regression = 0.0000000001;
    thetas_ = tmp_matrix_sxtd.cwise() / (tmp_matrix_sx.cwise() + ridge_regression);

    return true;
}


bool LocallyWeightedRegression::writeToMessage(lwr::Model& lwr_model)
{
    lwr_model.initialized = initialized_;
    lwr_model.num_rfs = num_rfs_;
    lwr_model.widths.clear();
    lwr_model.centers.clear();
    lwr_model.thetas.clear();
    lwr_model.offsets.clear();
    for (int i = 0; i < num_rfs_; i++)
    {
        lwr_model.widths.push_back(widths_(i));
        lwr_model.centers.push_back(centers_(i));
        lwr_model.thetas.push_back(thetas_(i));
        lwr_model.offsets.push_back(offsets_(i));
    }
    return true;
}
bool LocallyWeightedRegression::initFromMessage(const lwr::Model& lwr_model)
{
    initialized_ = lwr_model.initialized;
    num_rfs_ = lwr_model.num_rfs;
    widths_ = VectorXd::Zero(num_rfs_);
    centers_ = VectorXd::Zero(num_rfs_);
    thetas_ = VectorXd::Zero(num_rfs_);
    offsets_ = VectorXd::Zero(num_rfs_);
    for (int i = 0; i < num_rfs_; i++)
    {
        widths_(i) = lwr_model.widths[i];
        centers_(i) = lwr_model.centers[i];
        thetas_(i) = lwr_model.thetas[i];
        offsets_(i) = lwr_model.offsets[i];
    }
    return true;
}

bool LocallyWeightedRegression::writeToDisc(const std::string directory_name)
{
    if (!initialized_)
    {
        ROS_ERROR("LWR model not initialized.\n");
        return initialized_;
    }

    assert(centers_.size() == num_rfs_);
    assert(thetas_.size() == num_rfs_);

    std::string abs_bagfile_name = directory_name + std::string(lwr_model_file_name);
    try
    {
        rosbag::Bag bag(abs_bagfile_name, rosbag::bagmode::Write);
        lwr::Model lwr_model;
        if(!writeToMessage(lwr_model))
        {
            ROS_ERROR("Could not get LWR model.");
            return false;
        }
        bag.write(lwr_model_topic_file_name, ros::Time::now(), lwr_model);
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s: %s", abs_bagfile_name.c_str(), ex.what());
        return false;
    }
    return true;
}

bool LocallyWeightedRegression::initializeFromDisc(const std::string directory_name)
{

    if (initialized_)
    {
        ROS_WARN("LWR model already initialized, re-initializing it.");
    }

    std::string abs_bagfile_name = directory_name + std::string(lwr_model_file_name);
    try
    {
        rosbag::Bag bag(abs_bagfile_name, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(lwr_model_topic_file_name));
        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
        {
            lwr::Model::ConstPtr lwr_model = msg.instantiate<lwr::Model> ();
            assert(transformation_system != NULL);
            if(!initFromMessage(*lwr_model))
            {
                ROS_ERROR("Could not set LWR model.");
                initialized_ = false;
                return initialized_;
            }
        }
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s: %s", abs_bagfile_name.c_str(), ex.what());
        initialized_ = false;
        return initialized_;
    }

    initialized_ = true;
    return initialized_;
}

std::string LocallyWeightedRegression::getInfoString()
{
    std::string info;
    info.clear();

    // TODO: use utility function to convert from int to string

    std::stringstream ss;
    ss.clear();

    int precision = 4;
    ss.precision(precision);

    if (initialized_)
    {
        info.append(std::string("initialized:true "));
    }
    else
    {
        info.append(std::string("initialized:false"));
    }
    info.append(std::string(" \tnum_rfs:"));
    ss << num_rfs_;
    info.append(ss.str());

    ss.str("");
    ss.clear();
    info.append(std::string("\nwidths: "));
    for (int i = 0; i < widths_.size(); i++)
    {
        ss << widths_[i];
        info.append(std::string("\t") + ss.str() + std::string(" "));
        ss.str("");
        ss.clear();
    }

    ss.str("");
    ss.clear();
    info.append(std::string("\nthetas: "));
    for (int i = 0; i < thetas_.size(); i++)
    {
        ss << thetas_[i];
        info.append(std::string("\t") + ss.str() + std::string(" "));
        ss.str("");
        ss.clear();
    }

    ss.str("");
    ss.clear();
    info.append(std::string("\ncenters:"));
    for (int i = 0; i < centers_.size(); i++)
    {
        ss << centers_[i];
        info.append(std::string("\t") + ss.str() + std::string(" "));
        ss.str("");
        ss.clear();
    }

    return info;
}

bool LocallyWeightedRegression::getNumRFS(int &num_rfs)
{
    if (!initialized_)
    {
        ROS_ERROR("LWR is not initialized, not returning number of receptive fields.");
        return false;
    }
    num_rfs = num_rfs_;
    return true;
}

}
