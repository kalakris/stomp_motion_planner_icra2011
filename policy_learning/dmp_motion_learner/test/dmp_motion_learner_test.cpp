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

 \file    dmp_motion_learner_test.cpp

 \author  Peter Pastor
 \date    Jun 23, 2010

 **********************************************************************/

// system includes
#include <string>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#include <sensor_msgs/JointState.h>

#include <angles/angles.h>

#include <Eigen/Eigen>
USING_PART_OF_NAMESPACE_EIGEN

#include <dmp_motion_generation/dynamic_movement_primitive.h>

// local includes
#include <dmp_motion_learner/LearnJointSpaceDMP.h>
#include <dmp_motion_learner/LearnCartesianSpaceDMP.h>

const int TEST_DMP_ID_1 = 1;
const int TEST_DMP_ID_2 = 2;

class DMPMotionLearnerTest
{

public:

    DMPMotionLearnerTest(ros::NodeHandle& node_handle);
    ~DMPMotionLearnerTest();

    bool initialize();

    bool run_learn_joint_space_dmp();
    bool run_learn_cartesian_space_dmp();

private:

    bool initialized_;

    ros::NodeHandle node_handle_;

    std::string data_directory_name_;

    std::string bag_file_name_;

    double mse_error_threshold_;

    ros::ServiceClient learn_joint_space_dmp_service_client_;
    ros::ServiceClient learn_cartesian_space_dmp_service_client_;

};

DMPMotionLearnerTest::DMPMotionLearnerTest(ros::NodeHandle& node_handle) :
    initialized_(false), node_handle_(node_handle)
{
}

DMPMotionLearnerTest::~DMPMotionLearnerTest()
{
}

bool DMPMotionLearnerTest::initialize()
{

    ros::NodeHandle dmp_node_handle(node_handle_, "dmp");

    // get package name and then get the package path
    std::string package_name;
    if (!dmp_node_handle.getParam(std::string("package_name"), package_name))
    {
        ROS_ERROR("Could not retrive parameter >>package_name<< from param server in the namespace %s.", dmp_node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }
    std::string package_path = ros::package::getPath(package_name);
    if (package_path.compare(package_path.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        package_path.append("/");
    }

    std::string library_base_directory_name;
    if (!dmp_node_handle.getParam(std::string("library_base_directory_name"), library_base_directory_name))
    {
        ROS_ERROR("Could not retrive parameter >>library_base_directory_name<< from param server in the namespace %s.", dmp_node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }
    if (library_base_directory_name.compare(library_base_directory_name.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        library_base_directory_name.append("/");
    }

    std::string sub_directory_name;
    if (!dmp_node_handle.getParam(std::string("data_directory_name"), sub_directory_name))
    {
        ROS_ERROR("Could not retrive parameter >>data_directory_name<< from param server in the namespace %s.", dmp_node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }
    data_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    if (data_directory_name_.compare(data_directory_name_.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        data_directory_name_.append("/");
    }

    ros::NodeHandle test_parameters_node_handle(node_handle_, "test_parameters");

    std::string bag_file_name;
    if (!test_parameters_node_handle.getParam(std::string("bag_file_name"), bag_file_name))
    {
        ROS_ERROR("Could not retrive parameter >>bag_file_name<< from param server in the namespace %s.", test_parameters_node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }
    bag_file_name_.assign(data_directory_name_ + bag_file_name);

    if (!test_parameters_node_handle.getParam(std::string("mse_error_threshold"), mse_error_threshold_))
    {
        ROS_ERROR("Could not retrive parameter >>mse_error_threshold<< from param server in the namespace %s.", test_parameters_node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }

    learn_joint_space_dmp_service_client_
            = node_handle_.serviceClient<dmp_motion_learner::LearnJointSpaceDMP> ("/dmp_motion_learner/learn_joint_space_dmp_from_bag_file");
    learn_cartesian_space_dmp_service_client_
            = node_handle_.serviceClient<dmp_motion_learner::LearnCartesianSpaceDMP> ("/dmp_motion_learner/learn_cartesian_and_joint_space_dmp_from_bag_file");

    initialized_ = true;
    return initialized_;
}

bool DMPMotionLearnerTest::run_learn_joint_space_dmp()
{
    if (!initialized_)
    {
        ROS_ERROR("DMPMotionLearnerTest is not initialized.");
        return false;
    }

    dmp_motion_learner::LearnJointSpaceDMP::Request learn_joint_space_dmp_request;
    dmp_motion_learner::LearnJointSpaceDMP::Response learn_joint_space_dmp_response;

    learn_joint_space_dmp_request.joint_names.push_back("r_shoulder_pan_joint");
    learn_joint_space_dmp_request.joint_names.push_back("r_shoulder_lift_joint");
    learn_joint_space_dmp_request.joint_names.push_back("r_upper_arm_roll_joint");
    learn_joint_space_dmp_request.joint_names.push_back("r_elbow_flex_joint");
    learn_joint_space_dmp_request.joint_names.push_back("r_forearm_roll_joint");
    learn_joint_space_dmp_request.joint_names.push_back("r_wrist_flex_joint");
    learn_joint_space_dmp_request.joint_names.push_back("r_wrist_roll_joint");
    learn_joint_space_dmp_request.bag_file_name.assign(bag_file_name_);
    learn_joint_space_dmp_request.data_directory_name.assign(data_directory_name_);
    learn_joint_space_dmp_request.dmp_id = TEST_DMP_ID_1;

    if (!learn_joint_space_dmp_service_client_.call(learn_joint_space_dmp_request, learn_joint_space_dmp_response))
    {
        ROS_ERROR("Learn joint space DMP from bag file service call failed.");
        return false;
    }

    if(learn_joint_space_dmp_response.return_code != dmp_motion_learner::LearnJointSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL)
    {
        ROS_ERROR("Learn joint space DMP from bag file service call was not successfull.");
        return false;
    }

    boost::shared_ptr<dmp::DynamicMovementPrimitive> test_dmp_1(new dmp::DynamicMovementPrimitive(node_handle_));
    if(!test_dmp_1->initFromMessage(learn_joint_space_dmp_response.dmp))
    {
        ROS_ERROR("Could not set DMP from retrieved message.");
        return false;
    }

    double duration;
    if(!test_dmp_1->getInitialDuration(duration))
    {
        ROS_ERROR("Could not get initial duration.");
        return false;
    }

    if (!test_dmp_1->setup(dmp::DEFAULT_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Could not setup test_dmp_1.");
        return false;
    }

    dmp::Trajectory test_trajectory_1(node_handle_);
    if (!test_trajectory_1.initialize(learn_joint_space_dmp_request.joint_names, dmp::DEFAULT_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Could not initialize trajectory");
        return false;
    }
    if (!test_dmp_1->propagateFull(test_trajectory_1, duration, static_cast<int> (dmp::DEFAULT_SAMPLING_FREQUENCY * duration)))
    {
        ROS_ERROR("Could not propagate the full dmp.");
        return false;
    }
    if (!test_dmp_1->writeDebugTrajectory())
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    const int num_joints = static_cast<int> (learn_joint_space_dmp_request.joint_names.size());
    VectorXd joint_positions = VectorXd::Zero(num_joints* dmp::POS_VEL_ACC);
    dmp::Trajectory bag_trajectory(node_handle_);
    if (!bag_trajectory.initialize(learn_joint_space_dmp_request.joint_names, dmp::DEFAULT_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Could not initialize trajectory.");
        return false;
    }

    try
    {
        rosbag::Bag bag(learn_joint_space_dmp_request.bag_file_name, rosbag::bagmode::Read);
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
                        if (vsi->compare(learn_joint_space_dmp_request.joint_names[i]) == 0)
                        {
                            if ((i == dmp::CONTINOUS_FOREARM_ROLL_JOINT) || (i == dmp::CONTINOUS_WRIST_ROLL_JOINT))
                            {
                                joint_positions(i * dmp::POS_VEL_ACC) = angles::normalize_angle(joint_state->position[index]);
                            }
                            else
                            {
                                joint_positions(i * dmp::POS_VEL_ACC) = joint_state->position[index];
                            }
                            num_joints_found++;
                        }

                    }
                    index++;
                }
                if (num_joints_found == num_joints)
                {
                    if (!bag_trajectory.add(joint_positions))
                    {
                        ROS_ERROR("Could not add trajectory point.");
                        return false;
                    }
                }
                else
                {
                    ROS_ERROR("Number of joints is %i, but there have been %i matches.", num_joints, num_joints_found);
                    return false;
                }
            }
            else
            {
                ROS_ERROR("Could not read bag file %s.", learn_joint_space_dmp_request.bag_file_name.c_str());
                return false;
            }
        }
        bag.close();
    }
    catch (rosbag::BagIOException ex)
    {
        ROS_ERROR("Problem when reading from bag file named %s.", learn_joint_space_dmp_request.bag_file_name.c_str());
        return false;
    }

    VectorXd mse_vector = VectorXd::Zero(num_joints);
    if(!bag_trajectory.computeMSE(test_trajectory_1, mse_vector))
    {
        ROS_ERROR("Could not compute MSE between bag file trajectory and generated trajectory.");
        return false;
    }

    for (int i=0; i<mse_vector.size(); ++i)
    {
        if(mse_vector(i) > mse_error_threshold_)
        {
            ROS_ERROR("MSE of dimension %i is %f and therefore higher then the allowed mse error threshold %f.", i, mse_vector(i), mse_error_threshold_);
            return false;
        }
    }

    return true;
}
bool DMPMotionLearnerTest::run_learn_cartesian_space_dmp()
{
    if (!initialized_)
    {
        ROS_ERROR("DMPMotionLearnerTest is not initialized.");
        return false;
    }

    dmp_motion_learner::LearnCartesianSpaceDMP::Request learn_cartesian_space_dmp_request;
    dmp_motion_learner::LearnCartesianSpaceDMP::Response learn_cartesian_space_dmp_response;

    learn_cartesian_space_dmp_request.bag_file_name.assign(bag_file_name_);
    learn_cartesian_space_dmp_request.data_directory_name.assign(data_directory_name_);
    // learn_cartesian_space_dmp_request.parameter_namespace.assign(std::string("/dmp_motion_learner/"));
    learn_cartesian_space_dmp_request.dmp_id = TEST_DMP_ID_2;

    if (!learn_cartesian_space_dmp_service_client_.call(learn_cartesian_space_dmp_request, learn_cartesian_space_dmp_response))
    {
        ROS_ERROR("Learn cartesian space DMP from bag file service call failed.");
        return false;
    }

    if(learn_cartesian_space_dmp_response.return_code != dmp_motion_learner::LearnCartesianSpaceDMP::Response::SERVICE_CALL_SUCCESSFUL)
    {
        ROS_ERROR("Learn cartesian space DMP from bag file service call was not successfull.");
        return false;
    }

    dmp::DynamicMovementPrimitive test_dmp_2(node_handle_);
    if(!test_dmp_2.initFromMessage(learn_cartesian_space_dmp_response.dmp))
    {
        ROS_ERROR("Could not set DMP from retrieved message.");
        return false;
    }

    double duration;
    if(!test_dmp_2.getInitialDuration(duration))
    {
        ROS_ERROR("Could not get initial duration.");
        return false;
    }

    if (!test_dmp_2.setup(dmp::DEFAULT_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Could not setup test_dmp_2.");
        return false;
    }

    dmp::Trajectory test_trajectory_2(node_handle_);
    std::vector<std::string> variable_names;
    variable_names.push_back("r_arm_x");
    variable_names.push_back("r_arm_y");
    variable_names.push_back("r_arm_z");
    variable_names.push_back("r_arm_q0");
    variable_names.push_back("r_arm_q1");
    variable_names.push_back("r_arm_q2");
    variable_names.push_back("r_arm_q3");
    variable_names.push_back("r_shoulder_pan_joint");
    variable_names.push_back("r_shoulder_lift_joint");
    variable_names.push_back("r_upper_arm_roll_joint");
    variable_names.push_back("r_elbow_flex_joint");
    variable_names.push_back("r_forearm_roll_joint");
    variable_names.push_back("r_wrist_flex_joint");
    variable_names.push_back("r_wrist_roll_joint");

    if (!test_trajectory_2.initialize(variable_names, dmp::DEFAULT_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Could not initialize trajectory");
        return false;
    }
    if (!test_dmp_2.propagateFull(test_trajectory_2, duration, static_cast<int> (dmp::DEFAULT_SAMPLING_FREQUENCY * duration)))
    {
        ROS_ERROR("Could not propagate the full dmp.");
        return false;
    }
    if (!test_dmp_2.writeDebugTrajectory())
    {
        ROS_ERROR("Could not write debug trajectory.");
        return false;
    }

    //    const int num_joints = static_cast<int> (learn_cartesian_space_dmp_request.joint_names.size());
//    VectorXd joint_positions = VectorXd::Zero(num_joints* dmp::POS_VEL_ACC);
//    dmp::Trajectory bag_trajectory(node_handle_);
//    if (!bag_trajectory.initialize(learn_cartesian_space_dmp_request.joint_names, dmp::DEFAULT_SAMPLING_FREQUENCY))
//    {
//        ROS_ERROR("Could not initialize trajectory.");
//        return false;
//    }
//
//    try
//    {
//        rosbag::Bag bag(learn_cartesian_space_dmp_request.bag_file_name, rosbag::bagmode::Read);
//        rosbag::View view(bag, rosbag::TopicQuery("/joint_states"));
//
//        BOOST_FOREACH(rosbag::MessageInstance const msg, view)
//        {
//            sensor_msgs::JointState::ConstPtr joint_state = msg.instantiate<sensor_msgs::JointState> ();
//            if (joint_state != NULL)
//            {
//                int index = 0;
//                int num_joints_found = 0;
//                for (std::vector<std::string>::const_iterator vsi = joint_state->name.begin(); vsi != joint_state->name.end(); vsi++)
//                {
//                    for (int i = 0; i < num_joints; i++)
//                    {
//                        if (vsi->compare(learn_cartesian_space_dmp_request.joint_names[i]) == 0)
//                        {
//                            if ((i == dmp::CONTINOUS_FOREARM_ROLL_JOINT) || (i == dmp::CONTINOUS_WRIST_ROLL_JOINT))
//                            {
//                                joint_positions(i * dmp::POS_VEL_ACC) = angles::normalize_angle(joint_state->position[index]);
//                            }
//                            else
//                            {
//                                joint_positions(i * dmp::POS_VEL_ACC) = joint_state->position[index];
//                            }
//                            num_joints_found++;
//                        }
//
//                    }
//                    index++;
//                }
//                if (num_joints_found == num_joints)
//                {
//                    if (!bag_trajectory.add(joint_positions))
//                    {
//                        ROS_ERROR("Could not add trajectory point.");
//                        return false;
//                    }
//                }
//                else
//                {
//                    ROS_ERROR("Number of joints is %i, but there have been %i matches.", num_joints, num_joints_found);
//                    return false;
//                }
//            }
//            else
//            {
//                ROS_ERROR("Could not read bag file %s.", learn_cartesian_space_dmp_request.bag_file_name.c_str());
//                return false;
//            }
//        }
//        bag.close();
//    }
//    catch (rosbag::BagIOException ex)
//    {
//        ROS_ERROR("Problem when reading from bag file named %s.", learn_cartesian_space_dmp_request.bag_file_name.c_str());
//        return false;
//    }
//
//    VectorXd mse_vector = VectorXd::Zero(num_joints);
//    if(!bag_trajectory.computeMSE(test_trajectory_2, mse_vector))
//    {
//        ROS_ERROR("Could not compute MSE between bag file trajectory and generated trajectory.");
//        return false;
//    }
//
//    for (int i=0; i<mse_vector.size(); ++i)
//    {
//        if(mse_vector(i) > mse_error_threshold_)
//        {
//            ROS_ERROR("MSE of dimension %i is %f and therefore higher then the allowed mse error threshold %f.", i, mse_vector(i), mse_error_threshold_);
//            return false;
//        }
//    }

    return true;
}

TEST(dmp_learner_test, LEARN_JOINT_SPACE_DMP_TEST)
{
    ros::NodeHandle node_handle("/dmp_motion_learner");
    DMPMotionLearnerTest dmp_learner_test(node_handle);
    EXPECT_TRUE(dmp_learner_test.initialize());
    EXPECT_TRUE(dmp_learner_test.run_learn_joint_space_dmp());
}
TEST(dmp_learner_test, LEARN_CARTESIAN_SPACE_DMP_TEST)
{
    ros::NodeHandle node_handle("/dmp_motion_learner");
    DMPMotionLearnerTest dmp_learner_test(node_handle);
    EXPECT_TRUE(dmp_learner_test.initialize());
    EXPECT_TRUE(dmp_learner_test.run_learn_cartesian_space_dmp());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmp_learner_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
