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
#include <time.h>
#include <iostream>
#include <fstream>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>

#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/assert.h>

#include <Eigen/Eigen>
USING_PART_OF_NAMESPACE_EIGEN

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

//local includes
#include <lwr/lwr.h>

using namespace lwr;

class LWRTest
{

public:

    LWRTest();
    ~LWRTest();

    bool initialize();

    bool runLWRTest();

private:

    bool initialized_;

    std::string data_directory_name_;
    std::string library_directory_name_;

    double rfs_width_boundary_;
    int num_rfs_;

    double can_sys_cutoff_;

    int num_data_learn_;
    int num_data_query_;
    double mse_prediction_error_threshold_;
    double mse_rescaling_error_threshold_;

    double testFunction(const double test_x);

};

LWRTest::LWRTest() :
    initialized_(false)
{
}

LWRTest::~LWRTest()
{
}

bool LWRTest::initialize()
{
    ros::NodeHandle node_handle(std::string("/lwr_test/test_parameters"));

    // get package name and then get the package path
    std::string package_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("package_name"), package_name));

    std::string package_path = ros::package::getPath(package_name);
    policy_improvement_utilities::appendTrailingSlash(package_path);

    std::string sub_directory_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("library_directory_name"), sub_directory_name));

    library_directory_name_.assign(package_path + sub_directory_name);
    policy_improvement_utilities::appendTrailingSlash(library_directory_name_);
    ROS_INFO("Library directory is set to %s.", library_directory_name_.c_str());

    sub_directory_name.clear();
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("data_directory_name"), sub_directory_name));
    data_directory_name_.assign(package_path + sub_directory_name);
    policy_improvement_utilities::appendTrailingSlash(data_directory_name_);
    ROS_INFO("Data directory is set to %s.", data_directory_name_.c_str());

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("can_sys_cutoff"), can_sys_cutoff_));

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("rfs_width_boundary"), rfs_width_boundary_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("num_rfs"), num_rfs_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("num_data_learn"), num_data_learn_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("num_data_query"), num_data_query_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("mse_prediction_error_threshold"), mse_prediction_error_threshold_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("mse_rescaling_error_threshold"), mse_rescaling_error_threshold_));

    // create directory if it doesn't exist:
    boost::filesystem::create_directories(library_directory_name_);
    boost::filesystem::create_directories(data_directory_name_);

    initialized_ = true;
    return initialized_;
}

bool LWRTest::runLWRTest()
{

    LocallyWeightedRegression lwr;
    if (!lwr.initialize(library_directory_name_, num_rfs_, rfs_width_boundary_, true, can_sys_cutoff_))
    {
        ROS_ERROR("Could not initialize LWR model.");
        return false;
    }

    // initialize random seed
    srand(time(NULL));

    // generate input vector
        VectorXd test_x = VectorXd::Zero(num_data_learn_);
        test_x(0) = 0;
        double dx = static_cast<double> (1.0) / (test_x.size() - 1);
        for (int i = 1; i < test_x.size(); i++)
        {
            test_x(i) = test_x(i - 1) + dx;
        }

//    VectorXd test_x = VectorXd::Zero(num_data_learn_);
//    double x = 1;
//    double xd;
//    double dt = 1.0 / num_data_learn_;
//    double alpha_x = -log(0.001);
//    for (int i = test_x.size() - 1; i >= 0; --i)
//    {
//        xd = -alpha_x * x;
//        x = x + dt * xd;
//        test_x(i) = x;
//    }

    // generate target
    VectorXd test_y = VectorXd::Zero(test_x.size());
    for (int i = 1; i < test_x.size(); i++)
    {
        // test_y(i) = -pow(test_x(i) - 0.5, 2) + 0.2 + ((rand() % 1000 + 1) * 0.00005);
        test_y(i) = testFunction(test_x(i));
    }

    // learn parameters
    if (!lwr.learnWeights(test_x, test_y))
    {
        ROS_ERROR("Could not learn weights.");
        return false;
    }

    VectorXd test_xq = VectorXd::Zero(num_data_query_);
    test_xq(0) = 0;
    dx = static_cast<double> (1.0) / (test_xq.size() - 1);
    for (int i = 1; i < test_xq.size(); i++)
    {
        test_xq(i) = test_xq(i - 1) + dx;
    }

//    VectorXd test_xq = VectorXd::Zero(num_data_query_);
//    x = 1;
//    dt = 1.0 / num_data_query_;
//    for (int i = test_xq.size() - 1; i >= 0; --i)
//    {
//        xd = -alpha_x * x;
//        x = x + dt * xd;
//        test_xq(i) = x;
//    }

    // get predictions
    VectorXd test_yp = VectorXd::Zero(test_xq.size());
    for (int i = 0; i < test_xq.size(); i++)
    {
        if (!lwr.predict(test_xq(i), test_yp(i)))
        {
            ROS_ERROR("Could not predict from LWR model.");
            return false;
        }
    }

    // compute mean squared error
    double mse = 0;
    for (int i = 0; i < test_xq.size(); i++)
    {
        mse += pow(testFunction(test_xq(i)) - test_yp(i), 2);
    }
    if (mse > mse_prediction_error_threshold_)
    {
        ROS_ERROR("MSE of the prediciton (%f) is larger than the threshold (%f).", mse, mse_prediction_error_threshold_);
        return false;
    }

    std::ofstream outfile;
    outfile.open(std::string(data_directory_name_ + std::string("test_x.txt")).c_str());
    outfile << test_x;
    outfile.close();

    outfile.open(std::string(data_directory_name_ + std::string("test_y.txt")).c_str());
    outfile << test_y;
    outfile.close();

    outfile.open(std::string(data_directory_name_ + std::string("test_xq.txt")).c_str());
    outfile << test_xq;
    outfile.close();

    outfile.open(std::string(data_directory_name_ + std::string("test_yp.txt")).c_str());
    outfile << test_yp;
    outfile.close();

    MatrixXd basis_function_matrix;
    basis_function_matrix = MatrixXd::Zero(test_x.size(), num_rfs_);
    if (!lwr.generateBasisFunctionMatrix(test_x, basis_function_matrix))
    {
        ROS_ERROR("Could not get basis function matrix");
        return false;
    }
    outfile.open(std::string(data_directory_name_ + std::string("basis_function_matrix.txt")).c_str());
    outfile << basis_function_matrix;
    outfile.close();

    // write LWR model to disc
    if (!lwr.writeToDisc(library_directory_name_))
    {
        ROS_ERROR("Could not write LWR model to file.");
        return false;
    }

    ROS_INFO_STREAM(lwr.getInfoString());

    // create new LWR model and read previous one from disc
    LocallyWeightedRegression lwr_copy;
    if (!lwr_copy.initializeFromDisc(library_directory_name_))
    {
        ROS_ERROR("Could not read LWR model from file.");
        return false;
    }

    // testing re-initializing
    if (!lwr_copy.initializeFromDisc(library_directory_name_))
    {
        ROS_ERROR("Could not read LWR model from file.");
        return false;
    }

    // get predictions
    VectorXd test_yp_copy = VectorXd::Zero(test_xq.size());
    for (int i = 0; i < test_xq.size(); i++)
    {
        if (!lwr_copy.predict(test_xq(i), test_yp_copy(i)))
        {
            ROS_ERROR("Could not predict from LWR model.");
            return false;
        }
    }

    outfile.open(std::string(data_directory_name_ + std::string("test_yp_copy.txt")).c_str());
    outfile << test_yp_copy;
    outfile.close();

    ROS_INFO_STREAM(lwr_copy.getInfoString());

    // compute mean squared error
    mse = 0;
    for (int i = 0; i < test_xq.size(); i++)
    {
        mse += pow(test_yp(i) - test_yp_copy(i), 2);
    }
    if (mse > mse_rescaling_error_threshold_)
    {
        ROS_ERROR("MSE of the prediciton (%f) is larger than the threshold (%f).", mse, mse_rescaling_error_threshold_);
        return false;
    }

    // test assignemnt operator
    LocallyWeightedRegression lwr_another_copy;
    lwr_another_copy = lwr_copy;
    VectorXd test_yp_another_copy = VectorXd::Zero(test_xq.size());
    for (int i = 0; i < test_xq.size(); i++)
    {
        if (!lwr_another_copy.predict(test_xq(i), test_yp_another_copy(i)))
        {
            ROS_ERROR("Could not predict from the second copy of LWR model.");
            return false;
        }
    }
    // compute mean squared error
    mse = 0;
    for (int i = 0; i < test_xq.size(); i++)
    {
        mse += pow(test_yp(i) - test_yp_another_copy(i), 2);
    }
    if (mse > mse_rescaling_error_threshold_)
    {
        ROS_ERROR("MSE of the prediciton (%f) is larger than the threshold (%f).", mse, mse_rescaling_error_threshold_);
        return false;
    }

    // checking whether we made a deep copy and not just using the same object
    VectorXd thetas = VectorXd::Zero(num_rfs_);
    if(!lwr_another_copy.getThetas(thetas))
    {
        ROS_ERROR("Could not obtain thetas.");
        return false;
    }
    for(int i=0; i<num_rfs_; i++)
    {
        thetas(i) = i;
    }
    if(!lwr_another_copy.setThetas(thetas))
    {
        ROS_ERROR("Could not set thetas.");
        return false;
    }

    test_yp_copy.setZero();
    test_yp_another_copy.setZero();
    for (int i = 0; i < test_xq.size(); i++)
    {
        if (!lwr_copy.predict(test_xq(i), test_yp_copy(i)))
        {
            ROS_ERROR("Could not predict from LWR model.");
            return false;
        }
        if (!lwr_another_copy.predict(test_xq(i), test_yp_another_copy(i)))
        {
            ROS_ERROR("Could not predict from the second copy of LWR model.");
            return false;
        }
    }

    mse = 0;
    for (int i = 0; i < test_xq.size(); i++)
    {
        mse += pow(test_yp_copy(i) - test_yp_another_copy(i), 2);
    }
    if (mse < mse_rescaling_error_threshold_)
    {
        ROS_ERROR("MSE of the prediciton (%f) is less than the threshold (%f) --> Copy constructor does not make a deep copy.", mse, mse_rescaling_error_threshold_);
        return false;
    }

    return true;
}

double LWRTest::testFunction(const double test_x)
{
    return -pow(test_x - 0.5, 2);
}

TEST(lwr_tests, run_lwr_test)
{
    LWRTest lwr_test;
    EXPECT_TRUE(lwr_test.initialize());
    EXPECT_TRUE(lwr_test.runLWRTest());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lwr_tests");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
