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

 \file    dmp_motion_generation_test.cpp

 \author  Peter Pastor
 \date    Jun 4, 2010

 **********************************************************************/

// system includes
#include <string>
#include <sstream>
#include <ctime>
#include <cstdlib>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <gtest/gtest.h>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <policy_library/policy_library.h>
#include <policy_improvement_utilities/assert.h>
#include <policy_improvement_utilities/param_server.h>


#include <Eigen/Eigen>
USING_PART_OF_NAMESPACE_EIGEN

#include <sensor_msgs/JointState.h>

#include <dmp_motion_generation/dynamic_movement_primitive.h>
#include <dmp_motion_generation/parameters.h>
#include <dmp_motion_generation/math_helper.h>

const int TEST_DMP_ID_1 = 1;
const int TEST_DMP_ID_2 = 2;
const int TEST_DMP_ID_3 = 3;
const int TEST_DMP_ID_4 = 4;
const int TEST_DMP_ID_5 = 5;
const int TEST_DMP_ID_6 = 6;
const int TEST_DMP_ID_7 = 7;
const int TEST_DMP_ID_8 = 8;
const int TEST_DMP_ID_9 = 9;
const int TEST_DMP_ID_10 = 10;
const int TEST_DMP_ID_11 = 11;
const int TEST_DMP_ID_12 = 12;

// TODO: add ROS_ASSERT_FUNC and friends
// TODO: add test that covers the setup function only setting goal + changeStart()

class DMPMotionGenerationTest
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DMPMotionGenerationTest();
    ~DMPMotionGenerationTest();

    bool initialize();

    /*!
     * @return
     */
    bool runLearnFromThetas();
    bool runLearnFromBagFile();
    bool runLearnFromMinJerk(const dmp::Parameters::Version dmp_version);
    bool runAddToDmpLib();
    bool runReadFromDmpLib();
    bool runOverwriteDmpInLib();
    bool runGeneralizeDmpGoal();
    bool runCopyConstructorTest();
    bool runTrajectoryCopyConstructorTest();
    bool runPropagateStepVsFull();
    bool runZeroWeigths(const dmp::Parameters::Version dmp_version);
    bool checkVersionResult();
    bool runChangeThetas();

private:

    bool initialized_;

    std::string library_directory_name_;
    std::string data_directory_name_;
    std::string debug_directory_name_;
    std::string dmp_test_namespace_;

    double duration_;
    double sampling_frequency_;
    std::string bag_file_name_;

    std::string test_trajectory_result_filename_1_;
    std::string test_trajectory_result_filename_2_;
    std::string test_trajectory_result_filename_3_;

    std::vector<std::string> variable_names_;
    std::vector<ros::Time> time_stamps_;

    std::vector<std::string> pr2_r_arm_joint_names_;
    std::vector<std::string> pr2_l_arm_joint_names_;

    std::vector<std::string> pr2_r_arm_trajectory_variable_names_;
    std::vector<std::string> pr2_l_arm_trajectory_variable_names_;

    int num_rfs_;
    int num_transformation_systems_;

    std::vector<VectorXd> thetas_;
    VectorXd start_;
    VectorXd goal_;

    double min_jerk_mse_error_threshold_;
    double generalization_mse_error_threshold_;
    double zero_weight_version_threshold_;

    ros::NodeHandle node_handle_;

};

DMPMotionGenerationTest::DMPMotionGenerationTest() :
    initialized_(false), dmp_test_namespace_(std::string("/dmp_motion_generation_test")), node_handle_(dmp_test_namespace_)
{
}

DMPMotionGenerationTest::~DMPMotionGenerationTest()
{
}

bool DMPMotionGenerationTest::initialize()
{

    initialized_ = false;
    ros::NodeHandle node_handle(dmp_test_namespace_ + std::string("/dmp"));

    // get package name and then get the package path
    std::string package_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("package_name"), package_name));

    std::string package_path = ros::package::getPath(package_name);
    policy_improvement_utilities::appendTrailingSlash(package_path);

    std::string library_base_directory_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("library_base_directory_name"), library_base_directory_name));
    policy_improvement_utilities::appendTrailingSlash(library_base_directory_name);

    std::string sub_directory_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("library_directory_name"), sub_directory_name));
    library_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    policy_improvement_utilities::appendTrailingSlash(library_directory_name_);
    // ROS_INFO("Library directory is set to %s.", library_directory_name_.c_str());

    sub_directory_name.clear();
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("data_directory_name"), sub_directory_name));
    data_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    policy_improvement_utilities::appendTrailingSlash(data_directory_name_);
    // ROS_INFO("Data directory is set to %s.", data_directory_name_.c_str());

    sub_directory_name.clear();
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("debug_directory_name"), sub_directory_name));
    debug_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    policy_improvement_utilities::appendTrailingSlash(debug_directory_name_);
    // ROS_INFO("Debug directory is set to %s.", debug_directory_name_.c_str());

    node_handle = ros::NodeHandle(dmp_test_namespace_ + std::string("/test_parameters"));

    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("duration"), duration_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("sampling_frequency"), sampling_frequency_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("min_jerk_mse_error_threshold"), min_jerk_mse_error_threshold_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("generalization_mse_error_threshold"), generalization_mse_error_threshold_));
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("zero_weight_version_threshold"), zero_weight_version_threshold_));

    std::string bag_file_name;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("bag_file_name"), bag_file_name));
    bag_file_name_.assign(data_directory_name_ + bag_file_name);

    std::string test_trajectory_result_filename;
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("test_trajectory_result_filename_1"), test_trajectory_result_filename));
    test_trajectory_result_filename_1_.assign(data_directory_name_ + test_trajectory_result_filename);
    test_trajectory_result_filename.clear();
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("test_trajectory_result_filename_2"), test_trajectory_result_filename));
    test_trajectory_result_filename_2_.assign(data_directory_name_ + test_trajectory_result_filename);
    test_trajectory_result_filename.clear();
    ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle, std::string("test_trajectory_result_filename_3"), test_trajectory_result_filename));
    test_trajectory_result_filename_3_.assign(data_directory_name_ + test_trajectory_result_filename);

    XmlRpc::XmlRpcValue dmps;
    if (!node_handle.getParam("dmps", dmps))
    {
        ROS_ERROR("Could not retrive parameter struct >>dmps<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }

    if (dmps.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Could not retrive parameter struct >>dmps<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        ROS_ERROR("The parameter need to be a map of dmps.");
        initialized_ = false;
        return initialized_;
    }

    num_transformation_systems_ = dmps.size();
    start_ = VectorXd::Zero(num_transformation_systems_);
    goal_ = VectorXd::Zero(num_transformation_systems_);
    variable_names_.clear();
    std::vector<VectorXd> theta_vectors;
    for (int i = 0; i < dmps.size(); i++)
    {
        XmlRpc::XmlRpcValue dmp = dmps[i];
        if (dmp.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR("DMP must be specified as maps, but they are XmlRpcType: %d", dmp.getType());
            initialized_ = false;
            return initialized_;
        }

        if (!dmp.hasMember("name"))
        {
            ROS_ERROR("DMP does not have specified >>name<<.");
            initialized_ = false;
            return initialized_;
        }
        variable_names_.push_back(dmp["name"]);

        if (!dmp.hasMember("thetas"))
        {
            ROS_ERROR("DMP does not have specified >>thetas<<.");
            initialized_ = false;
            return initialized_;
        }
        XmlRpc::XmlRpcValue thetas = dmp["thetas"];
        if (thetas.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Could not retrive parameter >>thetas<< from param server in the namespace %s. They are not specified as a list, instead they are %d.",
                    node_handle.getNamespace().c_str(), thetas.getType());
            initialized_ = false;
            return initialized_;
        }
        num_rfs_ = thetas.size();
        VectorXd theta_vector = VectorXd::Zero(num_rfs_);
        for (int j = 0; j < thetas.size(); j++)
        {
            if (thetas[j].getType() != XmlRpc::XmlRpcValue::TypeDouble)
            {
                ROS_ERROR("Could not retrive parameter >>thetas<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
                ROS_ERROR("They are not specified a list of doubles, instead they are type %d, but should be type %d.", thetas[j].getType(),
                        XmlRpc::XmlRpcValue::TypeDouble);
                initialized_ = false;
                return initialized_;
            }
            theta_vector(j) = static_cast<double> (thetas[j]);
        }
        theta_vectors.push_back(theta_vector);

        // Get the start
        if (!dmp.hasMember("start"))
        {
            ROS_ERROR("DMP does not have specified >>start<<.");
            initialized_ = false;
            return initialized_;
        }
        start_(i) = dmp["start"];
        // Get the goal
        if (!dmp.hasMember("goal"))
        {
            ROS_ERROR("DMP does not have specified >>goal<<.");
            initialized_ = false;
            return initialized_;
        }
        goal_(i) = dmp["goal"];
        // Get the name
    }

    VectorXd theta_vector = VectorXd::Zero(num_rfs_);
    for (int i = 0; i < num_transformation_systems_; i++)
    {
        thetas_.push_back(theta_vector);
    }

    std::string arm_joint_names;
    if (!node_handle.getParam(std::string("r_arm_joint_names"), arm_joint_names))
    {
        ROS_ERROR("Could not retrive parameter >>r_arm_joint_names<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        return false;
    }
    std::stringstream ss_names(arm_joint_names);
    std::string name;
    while (ss_names >> name)
    {
        pr2_r_arm_joint_names_.push_back(name);
    }
    arm_joint_names.clear();
    if (!node_handle.getParam(std::string("l_arm_joint_names"), arm_joint_names))
    {
        ROS_ERROR("Could not retrive parameter >>l_arm_joint_names<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        return false;
    }

    ss_names.clear();
    ss_names.str();
    ss_names << arm_joint_names;
    name.clear();
    while (ss_names >> name)
    {
        pr2_l_arm_joint_names_.push_back(name);
    }

    pr2_r_arm_trajectory_variable_names_.clear();
    arm_joint_names.clear();
    if (!node_handle.getParam(std::string("pr2_r_arm_joint_names"), arm_joint_names))
    {
        ROS_ERROR("Could not retrive parameter >>pr2_r_arm_joint_names<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        return false;
    }
    ss_names.clear();
    ss_names.str();
    ss_names << arm_joint_names;
    name.clear();
    while (ss_names >> name)
    {
        pr2_r_arm_trajectory_variable_names_.push_back(name);
    }

    pr2_l_arm_trajectory_variable_names_.clear();
    arm_joint_names.clear();
    if (!node_handle.getParam(std::string("pr2_l_arm_joint_names"), arm_joint_names))
    {
        ROS_ERROR("Could not retrive parameter >>pr2_l_arm_joint_names<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        return false;
    }
    ss_names.clear();
    ss_names.str();
    ss_names << arm_joint_names;
    name.clear();
    while (ss_names >> name)
    {
        pr2_l_arm_trajectory_variable_names_.push_back(name);
    }

    initialized_ = true;
    return initialized_;
}

bool DMPMotionGenerationTest::runLearnFromBagFile()
{
    if (!initialized_)
    {
        ROS_ERROR("DMP test program is not initialized.");
        return false;
    }

    dmp::Trajectory test_trajectory_1(node_handle_);
    if (!test_trajectory_1.initialize(pr2_r_arm_trajectory_variable_names_, sampling_frequency_, true))
    {
        ROS_ERROR("Could not initialize trajectory");
        return false;
    }

    const int num_joints = static_cast<int> (pr2_r_arm_joint_names_.size());
    VectorXd joint_positions = VectorXd::Zero(num_joints * dmp::POS_VEL_ACC);
    time_stamps_.clear();

    try
    {
        rosbag::Bag bag(bag_file_name_, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery("/joint_states"));
        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
        {
            sensor_msgs::JointState::ConstPtr joint_state = msg.instantiate<sensor_msgs::JointState> ();
            if (joint_state != NULL)
            {
                int index = 0;
                int num_joints_found = 0;
                for (std::vector<std::string>::const_iterator vsi = joint_state->name.begin(); vsi != joint_state->name.end(); vsi++)
                {
                    for (int i = 0; i < num_joints; i++)
                    {
                        if (vsi->compare(pr2_r_arm_joint_names_[i]) == 0)
                        {
                            joint_positions(i * dmp::POS_VEL_ACC) = joint_state->position[index];
                            num_joints_found++;
                        }

                    }
                    index++;
                }
                if (num_joints_found == num_joints)
                {
                    if (!test_trajectory_1.add(joint_positions))
                    {
                        ROS_ERROR("Could not add trajectory point.");
                        return false;
                    }
                    time_stamps_.push_back(joint_state->header.stamp);
                }
                else
                {
                    ROS_ERROR("Number of joints is %i, but there have been %i matches.", num_joints, num_joints_found);
                    return false;
                }
            }
            else
            {
                ROS_ERROR("Could not read bag file.");
                return false;
            }
        }
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Could not open bag file %s", bag_file_name_.c_str());
        return false;
    }

    if (!test_trajectory_1.writeToCLMCFile(data_directory_name_ + std::string("test_joint_demo.traj")))
    {
        ROS_ERROR("Could not write rescaled trajectory.");
        return false;
    }

    if (!test_trajectory_1.resample(time_stamps_, dmp::ADDITIONAL_TIME_IN_SEC_FOR_SMOOTHING))
    {
        ROS_ERROR("Could not resample trajectory.");
        return false;
    }

    if (!test_trajectory_1.writeToCLMCFile(data_directory_name_ + std::string("test_joint_demo_rescaled.traj")))
    {
        ROS_ERROR("Could not write rescaled trajectory.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_1(new dmp::DynamicMovementPrimitive(node_handle_));
    if (!test_dmp_1->initialize(num_joints, TEST_DMP_ID_1))
    {
        ROS_ERROR("Could not initialize test_dmp.");
        return false;
    }

    if (!test_dmp_1->learnFromTrajectory(test_trajectory_1))
    {
        ROS_ERROR("Could not learn test_dmp_1 from trajectory.");
        return false;
    }
    test_trajectory_1.clear();

    dmp::DMPDebug debug_desired_1(node_handle_);
    if (!debug_desired_1.initialize(test_dmp_1->getID(), std::string("pr2_r_arm"), num_joints, dmp::POS_VEL_ACC, 0, sampling_frequency_))
    {
        ROS_ERROR("Could not initialize debug object.");
        return false;
    }

    // setup for reproduction
    if (!test_dmp_1->setup(sampling_frequency_))
    {
        ROS_ERROR("Could not setup test test_dmp_1.");
        return false;
    }

    VectorXd desired_coordinates = VectorXd::Zero(num_joints * dmp::POS_VEL_ACC);
    bool movement_finished = false;
    while (!movement_finished)
    {
        if (!test_dmp_1->propagateStep(desired_coordinates, movement_finished))
        {
            ROS_ERROR("Could not run test_dmp_1.");
            return false;
        }

        if (!debug_desired_1.add(desired_coordinates))
        {
            ROS_ERROR("Could not add point to trajectory.");
            return false;
        }
    }

    std::string dmp_version;
    assert(test_dmp_1->getVersion(dmp_version));

    if (!debug_desired_1.writeToCLMCFile(dmp_version))
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    if (!test_dmp_1->writeDebugTrajectory())
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    boost::shared_ptr<library::PolicyLibrary<dmp::DynamicMovementPrimitive> > test_lib(new library::PolicyLibrary<dmp::DynamicMovementPrimitive>(node_handle_));
    if (!test_lib->initialize(library_directory_name_))
    {
        ROS_ERROR("Could not initialize test library.");
        return false;
    }

    if (!test_lib->add(test_dmp_1))
    {
        ROS_ERROR("Could not add test dmp %i to test library.", test_dmp_1->getID());
        return false;
    }

    return true;
}

bool DMPMotionGenerationTest::runLearnFromMinJerk(const dmp::Parameters::Version dmp_version)
{
    if (!initialized_)
    {
        ROS_ERROR("DMP test program is not initialized.");
        return false;
    }

    // TODO: check why there is a jump at the end...
    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_2(new dmp::DynamicMovementPrimitive(node_handle_));
    EXPECT_TRUE(test_dmp_2->initialize(num_transformation_systems_, TEST_DMP_ID_2, dmp_version));

    dmp::Trajectory test_trajectory_2(node_handle_);
    EXPECT_TRUE(test_trajectory_2.initialize(variable_names_, sampling_frequency_));

    double delta_t = static_cast<double> (1.0) / sampling_frequency_;
    EXPECT_TRUE(dmp::MathHelper::generateMinJerkTrajectory(start_, goal_, duration_, delta_t, test_trajectory_2));

    std::string file_name;
    file_name.assign(data_directory_name_ + std::string("min_jerk_trajectory_") + dmp::MathHelper::getString(test_dmp_2->getID()) + std::string(".traj"));
    EXPECT_TRUE(test_trajectory_2.writeToCLMCFile(file_name));

    EXPECT_TRUE(test_dmp_2->learnFromTrajectory(test_trajectory_2));

    dmp::Trajectory desired_trajectory_2(node_handle_);
    EXPECT_TRUE(desired_trajectory_2.initialize(variable_names_, sampling_frequency_));

    // setup for reproduction
    EXPECT_TRUE(test_dmp_2->setup(sampling_frequency_));

    VectorXd desired_coordinates = VectorXd::Zero(num_transformation_systems_ * dmp::POS_VEL_ACC);
    bool movement_finished = false;
    while (!movement_finished)
    {
        EXPECT_TRUE(test_dmp_2->propagateStep(desired_coordinates, movement_finished));
        EXPECT_TRUE(desired_trajectory_2.add(desired_coordinates));
    }

    file_name.assign(data_directory_name_ + std::string("min_jerk_dmp_") + dmp::MathHelper::getString(test_dmp_2->getID()) + std::string(".traj"));
    EXPECT_TRUE(desired_trajectory_2.writeToCLMCFile(file_name));

    EXPECT_TRUE(test_dmp_2->writeDebugTrajectory());

    boost::shared_ptr<library::PolicyLibrary<dmp::DynamicMovementPrimitive> > test_lib(new library::PolicyLibrary<dmp::DynamicMovementPrimitive>(node_handle_));
    EXPECT_TRUE(test_lib->initialize(library_directory_name_));

    EXPECT_TRUE(test_lib->add(test_dmp_2));

    VectorXd initial_start = VectorXd::Zero(num_transformation_systems_);
    EXPECT_TRUE(test_dmp_2->getInitialStart(initial_start));
    VectorXd initial_goal = VectorXd::Zero(num_transformation_systems_);
    EXPECT_TRUE(test_dmp_2->getInitialGoal(initial_goal));

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_3(new dmp::DynamicMovementPrimitive(node_handle_));
    EXPECT_TRUE(test_dmp_3->initialize(num_transformation_systems_, TEST_DMP_ID_3, dmp_version));

    EXPECT_TRUE(test_dmp_3->learnFromMinJerk(initial_start, initial_goal, duration_, delta_t));

    // setup for reproduction
    EXPECT_TRUE(test_dmp_3->setup(sampling_frequency_));

    dmp::Trajectory desired_trajectory_3(node_handle_);
    EXPECT_TRUE(desired_trajectory_3.initialize(variable_names_, sampling_frequency_));

    movement_finished = false;
    while (!movement_finished)
    {
        EXPECT_TRUE(test_dmp_3->propagateStep(desired_coordinates, movement_finished));
        EXPECT_TRUE(desired_trajectory_3.add(desired_coordinates));
    }

    file_name.assign(data_directory_name_ + std::string("desired_") + dmp::MathHelper::getString(test_dmp_3->getID()) + std::string(".traj"));
    EXPECT_TRUE(desired_trajectory_3.writeToCLMCFile(file_name));

    EXPECT_TRUE(test_dmp_3->writeDebugTrajectory());

    EXPECT_TRUE(test_lib->add(test_dmp_3));

    std::vector<VectorXd> theta_vectors;
    EXPECT_TRUE(test_dmp_2->getThetas(theta_vectors));

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_4(new dmp::DynamicMovementPrimitive(node_handle_));
    EXPECT_TRUE(test_dmp_4->initialize(num_transformation_systems_, TEST_DMP_ID_4, dmp_version));

    EXPECT_TRUE(test_dmp_4->learnFromThetas(theta_vectors, initial_start, initial_goal, sampling_frequency_, duration_));

    EXPECT_TRUE(test_lib->add(test_dmp_4));

    // setup for reproduction
    EXPECT_TRUE(test_dmp_4->setup(sampling_frequency_));

    dmp::Trajectory desired_trajectory_4(node_handle_);
    EXPECT_TRUE(desired_trajectory_4.initialize(variable_names_, sampling_frequency_));

    movement_finished = false;
    while (!movement_finished)
    {
        EXPECT_TRUE(test_dmp_4->propagateStep(desired_coordinates, movement_finished));
        EXPECT_TRUE(desired_trajectory_4.add(desired_coordinates));
    }

    file_name.assign(data_directory_name_ + std::string("desired_") + dmp::MathHelper::getString(test_dmp_4->getID()) + std::string(".traj"));
    EXPECT_TRUE(desired_trajectory_4.writeToCLMCFile(file_name));

    EXPECT_TRUE(test_dmp_4->writeDebugTrajectory());

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_5(new dmp::DynamicMovementPrimitive(node_handle_));
    EXPECT_TRUE(test_dmp_5->initialize(num_transformation_systems_, TEST_DMP_ID_5, dmp_version));

    EXPECT_TRUE(test_dmp_5->learnFromThetas(thetas_, start_, goal_, sampling_frequency_, duration_));

    EXPECT_TRUE(test_dmp_5->setup(start_, goal_, duration_, sampling_frequency_));

    dmp::Trajectory desired_trajectory_5(node_handle_);
    EXPECT_TRUE(desired_trajectory_5.initialize(variable_names_, sampling_frequency_));

    movement_finished = false;
    while (!movement_finished)
    {
        EXPECT_TRUE(test_dmp_5->propagateStep(desired_coordinates, movement_finished));
        EXPECT_TRUE(desired_trajectory_5.add(desired_coordinates));
    }

    file_name.assign(data_directory_name_ + std::string("desired_") + dmp::MathHelper::getString(test_dmp_5->getID()) + std::string(".traj"));
    EXPECT_TRUE(desired_trajectory_5.writeToCLMCFile(file_name));
    EXPECT_TRUE(test_dmp_5->writeDebugTrajectory());

    EXPECT_TRUE(test_lib->add(test_dmp_5));

    // do the actual test
    VectorXd mse_vector = VectorXd::Zero(num_transformation_systems_);
    EXPECT_TRUE(test_trajectory_2.computeMSE(desired_trajectory_2, mse_vector));
    for (int i = 0; i < num_transformation_systems_; i++)
    {
        if (mse_vector(i) > min_jerk_mse_error_threshold_)
        {
            ROS_ERROR(
                    "Mean squared error of transformation system %i (between actual and desired of dmp 3) is %f and therefore larger than the threshold (%f)",
                    i, mse_vector(i), min_jerk_mse_error_threshold_);
            return false;
        }
    }

    EXPECT_TRUE(desired_trajectory_2.computeMSE(desired_trajectory_3, mse_vector));
    for (int i = 0; i < num_transformation_systems_; i++)
    {
        if (mse_vector(i) > min_jerk_mse_error_threshold_)
        {
            ROS_ERROR("Mean squared error between trajectory 3 and 4 is %f and therefore larger than the threshold (%f)", mse_vector(i),
                    min_jerk_mse_error_threshold_);
            return false;
        }
    }

    EXPECT_TRUE(desired_trajectory_2.computeMSE(desired_trajectory_4, mse_vector));
    for (int i = 0; i < num_transformation_systems_; i++)
    {
        if (mse_vector(i) > min_jerk_mse_error_threshold_)
        {
            ROS_ERROR("Mean squared error between trajectory 3 and 5 is %f and therefore larger than the threshold (%f)", mse_vector(i),
                    min_jerk_mse_error_threshold_);
            return false;
        }
    }

    return true;
}

bool DMPMotionGenerationTest::runAddToDmpLib()
{
    if (!initialized_)
    {
        ROS_ERROR("DMP test program is not initialized.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_6(new dmp::DynamicMovementPrimitive(node_handle_));
    if (!test_dmp_6->initialize(num_transformation_systems_, TEST_DMP_ID_6))
    {
        ROS_ERROR("Could not initialize test_dmp_6.");
        return false;
    }

    if (!test_dmp_6->learnFromThetas(thetas_, start_, goal_, sampling_frequency_, duration_))
    {
        ROS_ERROR("Could not set theta matrix.");
        return false;
    }

    boost::shared_ptr<library::PolicyLibrary<dmp::DynamicMovementPrimitive> > test_lib(new library::PolicyLibrary<dmp::DynamicMovementPrimitive>(node_handle_));
    if (!test_lib->initialize(library_directory_name_))
    {
        ROS_ERROR("Could not initialize test library.");
        return false;
    }

    if (!test_lib->add(test_dmp_6))
    {
        ROS_ERROR("Could not add test_dmp_6 to test library.");
        return false;
    }

    return true;
}

bool DMPMotionGenerationTest::runReadFromDmpLib()
{
    if (!initialized_)
    {
        ROS_ERROR("DMP test program is not initialized.");
        return false;
    }

    boost::shared_ptr<library::PolicyLibrary<dmp::DynamicMovementPrimitive> > test_lib(new library::PolicyLibrary<dmp::DynamicMovementPrimitive>(node_handle_));
    if (!test_lib->initialize(library_directory_name_))
    {
        ROS_ERROR("Could not initialize test library.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_6;
    if (!test_lib->getItem(TEST_DMP_ID_6, test_dmp_6))
    {
        ROS_ERROR("Could not get test_dmp_6 from library.");
        return false;
    }

    if (!test_dmp_6->setup(start_, goal_, duration_, sampling_frequency_))
    {
        ROS_ERROR("Could not setup test_dmp_6.");
        return false;
    }

    dmp::Trajectory desired_trajectory_6(node_handle_);
    if (!desired_trajectory_6.initialize(variable_names_, sampling_frequency_))
    {
        ROS_ERROR("Could not initialize trajectory");
        return false;
    }

    VectorXd desired_coordinates = VectorXd::Zero(num_transformation_systems_ * dmp::POS_VEL_ACC);
    bool movement_finished = false;
    while (!movement_finished)
    {
        if (!test_dmp_6->propagateStep(desired_coordinates, movement_finished))
        {
            ROS_ERROR("Could not run test_dmp_6.");
            return false;
        }

        if (!desired_trajectory_6.add(desired_coordinates))
        {
            ROS_ERROR("Could not add point to trajectory.");
            return false;
        }
    }

    std::string file_name;
    std::stringstream ss;
    ss << test_dmp_6->getID();
    file_name.assign(data_directory_name_ + std::string("desired_") + ss.str() + std::string(".traj"));
    if (!desired_trajectory_6.writeToCLMCFile(file_name))
    {
        ROS_ERROR("Could not write desired_trajectory_6.");
        return false;
    }

    if (!test_dmp_6->writeDebugTrajectory())
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    return true;
}

bool DMPMotionGenerationTest::runOverwriteDmpInLib()
{
    if (!initialized_)
    {
        ROS_ERROR("DMP test program is not initialized.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_7(new dmp::DynamicMovementPrimitive(node_handle_));
    if (!test_dmp_7->initialize(num_transformation_systems_, TEST_DMP_ID_7))
    {
        ROS_ERROR("Could not initialize test_dmp_7.");
        return false;
    }

    if (!test_dmp_7->learnFromThetas(thetas_, start_, goal_, sampling_frequency_, duration_))
    {
        ROS_ERROR("Could not set theta matrix.");
        return false;
    }

    boost::shared_ptr<library::PolicyLibrary<dmp::DynamicMovementPrimitive> > test_lib(new library::PolicyLibrary<dmp::DynamicMovementPrimitive>(node_handle_));
    if (!test_lib->initialize(library_directory_name_))
    {
        ROS_ERROR("Could not initialize test library.");
        return false;
    }

    if (!test_lib->add(test_dmp_7))
    {
        ROS_ERROR("Could not add test_dmp_7 to test library.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_7_copy(new dmp::DynamicMovementPrimitive(node_handle_));
    if (!test_dmp_7_copy->initialize(num_transformation_systems_, TEST_DMP_ID_7))
    {
        ROS_ERROR("Could not initialize test_dmp_7_copy.");
        return false;
    }

    if (!test_dmp_7_copy->learnFromThetas(thetas_, start_, goal_, sampling_frequency_, duration_))
    {
        ROS_ERROR("Could not set theta matrix.");
        return false;
    }

    if (!test_lib->add(test_dmp_7_copy))
    {
        ROS_ERROR("Could not add test_dmp_7_copy to test library.");
        return false;
    }
    return true;
}

bool DMPMotionGenerationTest::runGeneralizeDmpGoal()
{
    if (!initialized_)
    {
        ROS_ERROR("DMP test program is not initialized.");
        return false;
    }

    boost::shared_ptr<library::PolicyLibrary<dmp::DynamicMovementPrimitive> > test_lib(new library::PolicyLibrary<dmp::DynamicMovementPrimitive>(node_handle_));
    if (!test_lib->initialize(library_directory_name_))
    {
        ROS_ERROR("Could not initialize test library.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_2_copy;

    if (!test_lib->getItem(TEST_DMP_ID_2, test_dmp_2_copy))
    {
        ROS_ERROR("Could not get test_dmp_2 from library.");
        return false;
    }

    std::vector<VectorXd> theta_vectors;
    if (!test_dmp_2_copy->getThetas(theta_vectors))
    {
        ROS_ERROR("Could not get theta matrix from test_dmp_2_copy.");
        return false;
    }

    std::vector<VectorXd> modified_theta_vectors;
    for (int i = 0; i < static_cast<int> (theta_vectors.size()); i++)
    {
        // replicate theta of first transformation system
        modified_theta_vectors.push_back(theta_vectors[0]);
    }

    if (!test_dmp_2_copy->setThetas(modified_theta_vectors))
    {
        ROS_ERROR("Could not set modified theta matrix from test_dmp_2_copy.");
        return false;
    }

    dmp::Trajectory test_trajectory_result_1(node_handle_);
    if (!test_trajectory_result_1.readFromCLMCFile(test_trajectory_result_filename_1_))
    {
        ROS_ERROR("Could not read test_trajectory_result_1 from file %s.", test_trajectory_result_filename_1_.c_str());
        return false;
    }

    double new_duration = static_cast<double> (test_trajectory_result_1.getLength()) * (static_cast<double> (1.0)
            / test_trajectory_result_1.getSamplingFrequency());
    if (!test_dmp_2_copy->setup(start_, goal_, new_duration, sampling_frequency_))
    {
        ROS_ERROR("Could not setup test_dmp_2_copy.");
        return false;
    }

    dmp::Trajectory desired_trajectory_2_copy(node_handle_);
    if (!desired_trajectory_2_copy.initialize(variable_names_, sampling_frequency_))
    {
        ROS_ERROR("Could not initialize trajectory");
        return false;
    }

    VectorXd desired_coordinates = VectorXd::Zero(num_transformation_systems_ * dmp::POS_VEL_ACC);
    bool movement_finished = false;
    while (!movement_finished)
    {
        if (!test_dmp_2_copy->propagateStep(desired_coordinates, movement_finished))
        {
            ROS_ERROR("Could not run test_dmp_2_copy.");
            return false;
        }

        if (!desired_trajectory_2_copy.add(desired_coordinates))
        {
            ROS_ERROR("Could not add point to trajectory.");
            return false;
        }
    }

    std::string file_name;
    std::stringstream ss;
    // ss << test_dmp_2_copy->getID();
    ss << 7;
    file_name.assign(data_directory_name_ + std::string("desired_") + ss.str() + std::string(".traj"));
    if (!desired_trajectory_2_copy.writeToCLMCFile(file_name))
    {
        ROS_ERROR("Could not write desired_trajectory_2_copy.");
        return false;
    }

    if (!test_dmp_2_copy->writeDebugTrajectory())
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    VectorXd mse_vector = VectorXd::Zero(num_transformation_systems_);
    if (!desired_trajectory_2_copy.computeMSE(test_trajectory_result_1, mse_vector))
    {
        ROS_ERROR("Could not compute MSE between trajectory read from file and generated trajectory.");
        return false;
    }
    for (int i = 0; i < num_transformation_systems_; i++)
    {
        if (mse_vector(i) > generalization_mse_error_threshold_)
        {
            ROS_ERROR(
                    "Mean squared error between the trajectory read from file and the generated trajectory is %f and therefore larger than the threshold (%f)",
                    mse_vector(i), generalization_mse_error_threshold_);
            return false;
        }
    }

    return true;
}

bool DMPMotionGenerationTest::runCopyConstructorTest()
{
    if (!initialized_)
    {
        ROS_ERROR("DMP test program is not initialized.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_8(new dmp::DynamicMovementPrimitive(node_handle_));
    if (!test_dmp_8->initialize(num_transformation_systems_, TEST_DMP_ID_8))
    {
        ROS_ERROR("Could not initialize test_dmp_8.");
        return false;
    }

    if (!test_dmp_8->learnFromThetas(thetas_, start_, goal_, sampling_frequency_, duration_))
    {
        ROS_ERROR("Could not set theta matrix.");
        return false;
    }

    boost::shared_ptr<library::PolicyLibrary<dmp::DynamicMovementPrimitive> > test_lib(new library::PolicyLibrary<dmp::DynamicMovementPrimitive>(node_handle_));
    if (!test_lib->initialize(library_directory_name_))
    {
        ROS_ERROR("Could not initialize test library.");
        return false;
    }

    // setup for reproduction
    if (!test_dmp_8->setup(sampling_frequency_))
    {
        ROS_ERROR("Could not setup test test_dmp_8.");
        return false;
    }

    dmp::Trajectory desired_trajectory_8(node_handle_);
    if (!desired_trajectory_8.initialize(variable_names_, sampling_frequency_))
    {
        ROS_ERROR("Could not initialize trajectory.");
        return false;
    }

    VectorXd desired_coordinates = VectorXd::Zero(num_transformation_systems_ * dmp::POS_VEL_ACC);
    bool movement_finished = false;
    while (!movement_finished)
    {
        if (!test_dmp_8->propagateStep(desired_coordinates, movement_finished))
        {
            ROS_ERROR("Could not run test_dmp_8.");
            return false;
        }

        if (!desired_trajectory_8.add(desired_coordinates))
        {
            ROS_ERROR("Could not add point to trajectory.");
            return false;
        }
    }

    std::string file_name;
    std::stringstream ss;
    ss << test_dmp_8->getID();
    file_name.assign(data_directory_name_ + std::string("desired_") + ss.str() + std::string(".traj"));
    if (!desired_trajectory_8.writeToCLMCFile(file_name))
    {
        ROS_ERROR("Could not write desired_trajectory_8.");
        return false;
    }

    if (!test_dmp_8->writeDebugTrajectory())
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    if (!test_lib->add(test_dmp_8))
    {
        ROS_ERROR("Could not add test_dmp_8 to test library.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_9(new dmp::DynamicMovementPrimitive(*test_dmp_8));
    if (!test_dmp_9->setID(TEST_DMP_ID_9))
    {
        ROS_ERROR("Could not set dmp if of test_dmp_9.");
        return false;
    }

    // setup for reproduction
    if (!test_dmp_9->setup(sampling_frequency_))
    {
        ROS_ERROR("Could not setup test test_dmp_9.");
        return false;
    }

    dmp::Trajectory desired_trajectory_9(node_handle_);
    if (!desired_trajectory_9.initialize(variable_names_, sampling_frequency_))
    {
        ROS_ERROR("Could not initialize trajectory.");
        return false;
    }

    movement_finished = false;
    while (!movement_finished)
    {
        if (!test_dmp_9->propagateStep(desired_coordinates, movement_finished))
        {
            ROS_ERROR("Could not run test_dmp_9.");
            return false;
        }

        if (!desired_trajectory_9.add(desired_coordinates))
        {
            ROS_ERROR("Could not add point to trajectory.");
            return false;
        }
    }

    ss.clear();
    ss.str("");
    ss << test_dmp_9->getID();
    file_name.assign(data_directory_name_ + std::string("desired_") + ss.str() + std::string(".traj"));
    if (!desired_trajectory_9.writeToCLMCFile(file_name))
    {
        ROS_ERROR("Could not write desired_trajectory_9.");
        return false;
    }

    if (!test_dmp_9->writeDebugTrajectory())
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    if (!test_lib->add(test_dmp_9))
    {
        ROS_ERROR("Could not add test_dmp_9 to test library.");
        return false;
    }

    // do the actual test
    VectorXd mse_vector = VectorXd::Zero(num_transformation_systems_);
    if (!desired_trajectory_8.computeMSE(desired_trajectory_9, mse_vector))
    {
        ROS_ERROR("Could not compute MSE between generated trajectories.");
        return false;
    }
    for (int i = 0; i < num_transformation_systems_; i++)
    {
        if (mse_vector(i) > min_jerk_mse_error_threshold_)
        {
            ROS_ERROR(
                    "Mean squared error of transformation system %i (between desired trajectories of dmp 8 and 9) is %f and therefore larger than the threshold (%f)",
                    i, mse_vector(i), min_jerk_mse_error_threshold_);
            return false;
        }
    }

    return true;
}

bool DMPMotionGenerationTest::runTrajectoryCopyConstructorTest()
{
    if (!initialized_)
    {
        ROS_ERROR("DMP test program is not initialized.");
        return false;
    }

    dmp::Trajectory test_trajectory_1(node_handle_);
    if (!test_trajectory_1.initialize(dmp::POS_VEL_ACC, 1000.0))
    {
        ROS_ERROR("Could not initialize test_trajectory_1.");
        return false;
    }

    VectorXd test_trajectory_point = VectorXd::Zero(dmp::POS_VEL_ACC);
    test_trajectory_point(0) = 0;
    test_trajectory_point(1) = 1;
    test_trajectory_point(2) = 2;
    if (!test_trajectory_1.add(test_trajectory_point))
    {
        ROS_ERROR("Could not add trajectory point to test_trajectory_1.");
        return false;
    }

    dmp::Trajectory test_trajectory_2(node_handle_);
    test_trajectory_2 = test_trajectory_1;

    for (int i = 0; i < dmp::POS_VEL_ACC; i++)
    {
        if (!test_trajectory_1.update(0, i, 3 + i))
        {
            ROS_ERROR("Could not update test_trajectory_1.");
            return false;
        }
    }

    VectorXd mse = VectorXd::Zero(1);
    if (!test_trajectory_1.computeMSE(test_trajectory_2, mse))
    {
        ROS_ERROR("Could not compute mse.");
        return false;
    }

    if (mse(0) != static_cast<double> (9.0))
    {
        ROS_ERROR_STREAM("MSE between test_trajectory_1 and test_trajectory_2 is not 9.");
        return false;
    }

    return true;
}

bool DMPMotionGenerationTest::runPropagateStepVsFull()
{
    if (!initialized_)
    {
        ROS_ERROR("DMP test program is not initialized.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_10(new dmp::DynamicMovementPrimitive(node_handle_));
    if (!test_dmp_10->initialize(num_transformation_systems_, TEST_DMP_ID_10))
    {
        ROS_ERROR("Could not initialize test_dmp_10.");
        return false;
    }

    if (!test_dmp_10->learnFromThetas(thetas_, start_, goal_, sampling_frequency_, duration_))
    {
        ROS_ERROR("Could not set theta matrix.");
        return false;
    }

    if (!test_dmp_10->setup(start_, goal_, duration_, sampling_frequency_))
    {
        ROS_ERROR("Could not setup test_dmp_10.");
        return false;
    }

    dmp::Trajectory desired_full_trajectory_10(node_handle_);
    if (!desired_full_trajectory_10.initialize(variable_names_, sampling_frequency_))
    {
        ROS_ERROR("Could not initialize trajectory");
        return false;
    }
    if (!test_dmp_10->propagateFull(desired_full_trajectory_10, duration_, static_cast<int> (sampling_frequency_ * duration_)))
    {
        ROS_ERROR("Could not propagate the full dmp.");
        return false;
    }
    if (!test_dmp_10->writeDebugTrajectory())
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    if (!test_dmp_10->setup(start_, goal_, duration_, sampling_frequency_))
    {
        ROS_ERROR("Could not setup test_dmp_10.");
        return false;
    }
    dmp::Trajectory desired_step_trajectory_10(node_handle_);
    if (!desired_step_trajectory_10.initialize(variable_names_, sampling_frequency_))
    {
        ROS_ERROR("Could not initialize trajectory");
        return false;
    }
    VectorXd desired_coordinates = VectorXd::Zero(num_transformation_systems_ * dmp::POS_VEL_ACC);
    bool movement_finished = false;
    while (!movement_finished)
    {
        if (!test_dmp_10->propagateStep(desired_coordinates, movement_finished))
        {
            ROS_ERROR("Could not run test_dmp_10.");
            return false;
        }

        if (!desired_step_trajectory_10.add(desired_coordinates))
        {
            ROS_ERROR("Could not add point to trajectory.");
            return false;
        }
    }

    if (desired_full_trajectory_10.getLength() != desired_step_trajectory_10.getLength())
    {
        ROS_ERROR("Trajectory sizes do not match (full: %i, step: %i).", desired_full_trajectory_10.getLength(), desired_step_trajectory_10.getLength());
        return false;
    }

    std::string file_name;
    std::stringstream ss;
    ss << test_dmp_10->getID();
    file_name.assign(data_directory_name_ + std::string("desired_full_") + ss.str() + std::string(".traj"));
    if (!desired_full_trajectory_10.writeToCLMCFile(file_name))
    {
        ROS_ERROR("Could not write desired_full_trajectory_10.");
        return false;
    }
    file_name.assign(data_directory_name_ + std::string("desired_step_") + ss.str() + std::string(".traj"));
    if (!desired_step_trajectory_10.writeToCLMCFile(file_name))
    {
        ROS_ERROR("Could not write desired_step_trajectory_10.");
        return false;
    }

    if (!test_dmp_10->writeDebugTrajectory())
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    if (!test_dmp_10->setup(start_, goal_, duration_, sampling_frequency_))
    {
        ROS_ERROR("Could not setup test_dmp_10.");
        return false;
    }

    int num_samples = 200;
    double sampling_duration = 3.0;

    dmp::Trajectory desired_full_specific_trajectory_10(node_handle_);
    if (!desired_full_specific_trajectory_10.initialize(variable_names_, sampling_frequency_))
    {
        ROS_ERROR("Could not initialize trajectory");
        return false;
    }
    if (!test_dmp_10->propagateFull(desired_full_specific_trajectory_10, sampling_duration, num_samples))
    {
        ROS_ERROR("Could not propagate the full dmp.");
        return false;
    }

    if (desired_full_specific_trajectory_10.getLength() != num_samples)
    {
        ROS_ERROR("Number of samples (%i) does not match trajectory length (%i).", desired_full_specific_trajectory_10.getLength(), num_samples);
        return false;
    }
    file_name.assign(data_directory_name_ + std::string("desired_full_specific_") + ss.str() + std::string(".traj"));
    if (!desired_full_specific_trajectory_10.writeToCLMCFile(file_name))
    {
        ROS_ERROR("Could not write desired_full_specific_trajectory_10.");
        return false;
    }

    return true;
}

bool DMPMotionGenerationTest::runZeroWeigths(const dmp::Parameters::Version dmp_version)
{
    if (!initialized_)
    {
        ROS_ERROR("DMP test program is not initialized.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_11(new dmp::DynamicMovementPrimitive(node_handle_));
    if (!test_dmp_11->initialize(num_transformation_systems_, TEST_DMP_ID_11, dmp_version))
    {
        ROS_ERROR("Could not initialize test_dmp_11.");
        return false;
    }

    std::vector<VectorXd> theta_vectors;
    for (int i = 0; i < num_transformation_systems_; ++i)
    {
        theta_vectors.push_back(VectorXd::Zero(num_rfs_));
    }

    if (!test_dmp_11->learnFromThetas(theta_vectors, start_, goal_, sampling_frequency_, duration_))
    {
        ROS_ERROR("Could not learn dmp 11 from thetas.");
        return false;
    }

    dmp::Trajectory desired_trajectory_11(node_handle_);
    if (!desired_trajectory_11.initialize(variable_names_, sampling_frequency_))
    {
        ROS_ERROR("Could not initialize trajectory");
        return false;
    }

    if (!test_dmp_11->setup(start_, goal_, duration_, sampling_frequency_))
    {
        ROS_ERROR("Could not setup test_dmp_11.");
        return false;
    }

    int num_samples = 3000;
    double sampling_duration = 3.0;

    if (!test_dmp_11->propagateFull(desired_trajectory_11, sampling_duration, num_samples))
    {
        ROS_ERROR("Could not propagate the full dmp.");
        return false;
    }

    std::string file_name, version_string;
    std::stringstream ss;
    ss << test_dmp_11->getID();
    if (!test_dmp_11->getVersion(version_string))
    {
        ROS_ERROR("Could not obtain version string from test_dmp_11.");
        return false;
    }
    file_name.assign(data_directory_name_ + std::string("desired_") + version_string + std::string("_") + ss.str() + std::string(".traj"));
    if (!desired_trajectory_11.writeToCLMCFile(file_name))
    {
        ROS_ERROR("Could not write desired_full_specific_trajectory_11.");
        return false;
    }

    if (!test_dmp_11->writeDebugTrajectory())
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    return true;
}
bool DMPMotionGenerationTest::checkVersionResult()
{

    std::string file_name;
    std::stringstream ss;
    ss << TEST_DMP_ID_11;
    dmp::Trajectory trajectory_icra2009(node_handle_);
    file_name.assign(data_directory_name_ + std::string("desired_icra2009_") + ss.str() + std::string(".traj"));
    if (!trajectory_icra2009.readFromCLMCFile(file_name))
    {
        ROS_ERROR("Could not read trajectory_icra2009 from file %s.", file_name.c_str());
        return false;
    }
    dmp::Trajectory trajectory_nips2003(node_handle_);
    file_name.assign(data_directory_name_ + std::string("desired_nips2003_") + ss.str() + std::string(".traj"));
    if (!trajectory_nips2003.readFromCLMCFile(file_name))
    {
        ROS_ERROR("Could not read trajectory_nips2003 from file %s.", file_name.c_str());
        return false;
    }

    VectorXd mse_vector = VectorXd::Zero(num_transformation_systems_);
    if (!trajectory_icra2009.computeMSE(trajectory_nips2003, mse_vector))
    {
        ROS_ERROR("Could not compute MSE between generated trajectories.");
        return false;
    }
    for (int i = 0; i < num_transformation_systems_; i++)
    {
        if (mse_vector(i) > zero_weight_version_threshold_)
        {
            ROS_ERROR("Mean squared error of transformation system %i is %f and therefore larger than the threshold (%f)", i, mse_vector(i), zero_weight_version_threshold_);
            return false;
        }
    }
    return true;
}

bool DMPMotionGenerationTest::runChangeThetas()
{

    int num_transformation_systems = 1;

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_12(new dmp::DynamicMovementPrimitive(node_handle_));
    EXPECT_TRUE(test_dmp_12->initialize(num_transformation_systems, TEST_DMP_ID_12, dmp::Parameters::ICRA2009));

    boost::shared_ptr<library::PolicyLibrary<dmp::DynamicMovementPrimitive> > test_lib(new library::PolicyLibrary<dmp::DynamicMovementPrimitive>(node_handle_));
    EXPECT_TRUE(test_lib->initialize(library_directory_name_));

    VectorXd initial_start = VectorXd::Zero(num_transformation_systems);
    VectorXd initial_goal = VectorXd::Zero(num_transformation_systems);

    // initial_goal(0) = 1;

    double delta_t = static_cast<double> (1.0) / sampling_frequency_;
    EXPECT_TRUE(test_dmp_12->learnFromMinJerk(initial_start, initial_goal, duration_, delta_t));

    std::vector<std::string> variable_names;
    variable_names.push_back("test_name");

    dmp::Trajectory desired_trajectory_12(node_handle_);
    EXPECT_TRUE(desired_trajectory_12.initialize(variable_names, sampling_frequency_));

    double sampling_duration = 2*duration_;
    int num_samples = sampling_duration * sampling_frequency_;
    std::string file_name;
    std::vector<VectorXd> theta_vectors;

    int max = 10000;
    srand ( time(NULL) );
    for (int i=0; i<10; i++)
    {

        EXPECT_TRUE(test_lib->add(test_dmp_12));

        // setup for reproduction
        EXPECT_TRUE(test_dmp_12->setup(sampling_frequency_));

        desired_trajectory_12.clear();
        EXPECT_TRUE(test_dmp_12->propagateFull(desired_trajectory_12, sampling_duration, num_samples));

        file_name.assign(data_directory_name_ + std::string("desired_") + dmp::MathHelper::getString(test_dmp_12->getID())
                         + std::string("_iter_") + dmp::MathHelper::getString(i) + std::string(".traj"));

        EXPECT_TRUE(desired_trajectory_12.writeToCLMCFile(file_name));

        EXPECT_TRUE(test_dmp_12->writeDebugTrajectory());

        EXPECT_TRUE(test_dmp_12->getThetas(theta_vectors));

        std::vector<VectorXd> widths;
        std::vector<VectorXd> centers;

        EXPECT_TRUE(test_dmp_12->getWidthsAndCenters(widths, centers));

        for (std::vector<VectorXd>::iterator it = centers.begin(); it != centers.end(); ++it)
        {
            for (int n=0; n<it->size(); n++)
            {
                double can_x;
                if(n==0)
                {
                    EXPECT_TRUE(test_dmp_12->getCanonicalSystemValue(0.001, can_x));
                }
                else
                {
                    EXPECT_TRUE(test_dmp_12->getCanonicalSystemValue((*it)(n), can_x));
                }
                // ROS_INFO("x = %f", can_x);
            }
        }

        for (std::vector<VectorXd>::iterator it = theta_vectors.begin(); it != theta_vectors.end(); ++it)
        {
            for (int n=0; n<it->size(); n++)
            {
                double noise = (rand() % max - (max/2)) / (double)max;
                (*it)(n) += noise;
            }
        }

        EXPECT_TRUE(test_dmp_12->setThetas(theta_vectors));

    }

    return true;
}


// TODO: test more stuff...

TEST(dmp_test, LEARN_DMP_FROM_BAG_FILE)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runLearnFromBagFile());
}
TEST(dmp_test, LEARN_DMPS_FROM_MIN_JERK_ICRA2009)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runLearnFromMinJerk(dmp::Parameters::ICRA2009));
}
//TEST(dmp_test, LEARN_DMPS_FROM_MIN_JERK_NIPS2003)
//{
//    DMPMotionGenerationTest dmp_test;
//    EXPECT_TRUE(dmp_test.initialize());
//    EXPECT_TRUE(dmp_test.run_learn_from_min_jerk(dmp::Parameters::NIPS2003));
//}
TEST(dmp_test, ADD_DMP_TO_LIB)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runAddToDmpLib());
}
TEST(dmp_test, READ_DMP_FROM_LIB)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runReadFromDmpLib());
}
TEST(dmp_test, OVERWRITE_DMP_IN_LIB)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runOverwriteDmpInLib());
}
TEST(dmp_test, GENERALIZE_DMP_GOAL)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runGeneralizeDmpGoal());
}
TEST(dmp_test, COPY_CONSTRUCTOR_TEST)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runCopyConstructorTest());
}
TEST(dmp_test, TRAJECTORY_COPY_CONSTRUCTOR_TEST)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runTrajectoryCopyConstructorTest());
}
TEST(dmp_test, PROPAGATE_STEP_VS_FULL)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runPropagateStepVsFull());
}
TEST(dmp_test, ZERO_WEIGHTS_ICRA2009)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runZeroWeigths(dmp::Parameters::ICRA2009));
}
//TEST(dmp_test, ZERO_WEIGHTS_NIPS2003)
//{
//    DMPMotionGenerationTest dmp_test;
//    EXPECT_TRUE(dmp_test.initialize());
//    EXPECT_TRUE(dmp_test.run_zero_weigths(dmp::Parameters::NIPS2003));
//}
//TEST(dmp_test, COMPARE_ZERO_WEIGHTS)
//{
//    DMPMotionGenerationTest dmp_test;
//    EXPECT_TRUE(dmp_test.initialize());
//    EXPECT_TRUE(dmp_test.check_version_result());
//}
TEST(dmp_test, CHANGE_THETAS_TEST)
{
    DMPMotionGenerationTest dmp_test;
    EXPECT_TRUE(dmp_test.initialize());
    EXPECT_TRUE(dmp_test.runChangeThetas());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmp_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
