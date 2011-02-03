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

 \file    test_policy_improvement.cpp

 \author  Ludovic Righetti and Peter Pastor
 \date    May 26, 2010

 **********************************************************************/

// system includes
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

#include <boost/filesystem.hpp>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>

#include <Eigen/Eigen>
USING_PART_OF_NAMESPACE_EIGEN

#include <gtest/gtest.h>

#include "XmlRpcValue.h"

#include <dmp_motion_generation/parameters.h>
#include <dmp_motion_generation/constants.h>
#include <dmp_motion_generation/dynamic_movement_primitive.h>
#include <dmp_motion_generation/trajectory.h>

#include <policy_improvement/policy_improvement.h>
#include <policy_improvement/PI2TrialResult.h>
#include <policy_improvement/PI2TrialResults.h>
#include <policy_library/dmp_policy.h>

// local includes
#include <policy_improvement_test/WaypointCost.h>

using namespace pi2;
using namespace library;

class PolicyImprovementTest
{

public:

    PolicyImprovementTest();
    ~PolicyImprovementTest();


    /*!
     * @return
     */
    bool initialize();

    /*!
     * @return
     */
    bool run_pi2();

private:

    bool initialized_;

    int num_trials_;
    int num_rollouts_;
    int num_rfs_;
    int num_reused_trials_;
    int num_dimensions_;

    double variance_;

    /*!
     */
    policy_improvement_test::WaypointCost waypoint_cost_definition_;

    /*!
     */
    ros::NodeHandle node_handle_;

    std::string pi2_test_namespace_;

    std::string learn_directory_name_;
    std::string data_directory_name_;
    std::string debug_directory_name_;
    std::string library_directory_name_;

    boost::shared_ptr<DMPPolicy> policy_;

    VectorXd goal_positions_;
    VectorXd start_positions_;
    std::vector<std::string> variable_names_;

    double movement_duration_;
    double sampling_duration_;
    int num_time_steps_;
    double sampling_frequency_;

    /**
     * Performs a rollout for the given parameters, outputs cost
     * @param parameters
     * @return
     */
    bool rollout(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, dmp::Trajectory& trajectory);

    bool computeCost(dmp::Trajectory& trajectory, Eigen::VectorXd& costs);

};

PolicyImprovementTest::PolicyImprovementTest() :
    initialized_(false), pi2_test_namespace_(std::string("/policy_improvement_test")),
    movement_duration_(2.0),
    sampling_duration_(2.0),
    num_time_steps_(100),
    sampling_frequency_(double(sampling_duration_) / double(num_time_steps_))
{
}

PolicyImprovementTest::~PolicyImprovementTest()
{
}

bool PolicyImprovementTest::initialize()
{
    ros::NodeHandle node_handle;

    node_handle = ros::NodeHandle(pi2_test_namespace_ + std::string("/test_parameters/dmp"));

    // get package name and then get the package path
    std::string package_name;
    if (!node_handle.getParam(std::string("package_name"), package_name))
    {
        ROS_ERROR("Could not retrieve parameter >>package_name<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }
    std::string package_path = ros::package::getPath(package_name);
    if (package_path.compare(package_path.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        package_path.append("/");
    }

    std::string library_base_directory_name;
    if (!node_handle.getParam(std::string("library_base_directory_name"), library_base_directory_name))
    {
        ROS_ERROR("Could not retrieve parameter >>library_base_directory_name<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }
    if (library_base_directory_name.compare(library_base_directory_name.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        library_base_directory_name.append("/");
    }

    std::string sub_directory_name;
    if (!node_handle.getParam(std::string("library_directory_name"), sub_directory_name))
    {
        ROS_ERROR("Could not retrieve parameter >>library_directory_name<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }
    library_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    if (library_directory_name_.compare(library_directory_name_.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        library_directory_name_.append("/");
    }
    ROS_INFO("Library directory is set to %s.", library_directory_name_.c_str());

    sub_directory_name.clear();
    if (!node_handle.getParam(std::string("data_directory_name"), sub_directory_name))
    {
        ROS_ERROR("Could not retrive parameter >>data_directory_name<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }
    data_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    if (data_directory_name_.compare(data_directory_name_.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        data_directory_name_.append("/");
    }
    ROS_INFO("Data directory is set to %s.", data_directory_name_.c_str());

    sub_directory_name.clear();
    if (!node_handle.getParam(std::string("debug_directory_name"), sub_directory_name))
    {
        ROS_ERROR("Could not retrieve parameter >>debug_directory_name<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }
    debug_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    if (debug_directory_name_.compare(debug_directory_name_.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        debug_directory_name_.append("/");
    }
    ROS_INFO("Debug directory is set to %s.", debug_directory_name_.c_str());

    node_handle = ros::NodeHandle(pi2_test_namespace_ + std::string("/test_parameters"));
    node_handle_ = node_handle;

    sub_directory_name.clear();
    if (!node_handle.getParam(std::string("learn_directory_name"), sub_directory_name))
    {
        ROS_ERROR("Could not retrive parameter >>learn_directory_name<< from param server in the namespace %s.", node_handle.getNamespace().c_str());
        initialized_ = false;
        return initialized_;
    }
    learn_directory_name_.assign(package_path + library_base_directory_name + sub_directory_name);
    if (learn_directory_name_.compare(learn_directory_name_.size() - 1, 1, "/") != 0) // the directory name ends NOT with a slash
    {
        learn_directory_name_.append("/");
    }
    ROS_INFO("Data directory is set to %s.", learn_directory_name_.c_str());

    node_handle.param("num_time_steps", num_time_steps_, 100);
    node_handle.param("num_trials", num_trials_, 50);
    node_handle.param("num_rollouts", num_rollouts_, 10);
    node_handle.param("num_rfs", num_rfs_, 20);
    node_handle.param("num_reused_trials", num_reused_trials_, 0);
    node_handle.param("variance", variance_, 0.001 );
    node_handle.param("final_cost_penalty", waypoint_cost_definition_.final_cost_penalty, 100000.0);
    node_handle.param("instant_velocity_cost_penalty", waypoint_cost_definition_.instant_velocity_cost_penalty, 0.0);
    node_handle.param("waypoint_cost_penalty", waypoint_cost_definition_.waypoint_cost_penalty, 100000.0);

    XmlRpc::XmlRpcValue waypoints;
    if (!node_handle.getParam("waypoints", waypoints))
    {
        ROS_ERROR("No waypoints are provided.");
        initialized_ = false;
        return initialized_;
    }

    if (waypoints.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Expecting a list for waypoints");
        initialized_ = false;
        return initialized_;
    }

    num_dimensions_ = waypoints.size();

    goal_positions_ = VectorXd::Zero(num_dimensions_);
    start_positions_ = VectorXd::Zero(num_dimensions_);

    for (int i = 0; i < waypoints.size(); i++)
    {
        XmlRpc::XmlRpcValue waypoint = waypoints[i];
        if (waypoint.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR("Waypoint must be specified as maps, but they are XmlRpcType: %d", waypoint.getType());
            initialized_ = false;
            return initialized_;
        }

        // Get the timing
        if (!waypoint.hasMember("timing"))
        {
            ROS_ERROR("Waypoint does not have specified >>timing<<.");
            initialized_ = false;
            return initialized_;
        }
        waypoint_cost_definition_.waypoint_timings.push_back(waypoint["timing"]);
        // Get the waypoint
        if (!waypoint.hasMember("waypoint"))
        {
            ROS_ERROR("Waypoint does not have specified >>waypoint<<.");
            initialized_ = false;
            return initialized_;
        }
        waypoint_cost_definition_.waypoints.push_back(waypoint["waypoint"]);
        // Get the start
        if (!waypoint.hasMember("start"))
        {
            ROS_ERROR("waypoint does not have specified >>start<<.");
            initialized_ = false;
            return initialized_;
        }
        waypoint_cost_definition_.starts.push_back(waypoint["start"]);
        start_positions_(i) = waypoint["start"];

        // Get the goal
        if (!waypoint.hasMember("goal"))
        {
            ROS_ERROR("waypoint does not have specified >>goal<<.");
            initialized_ = false;
            return initialized_;
        }
        waypoint_cost_definition_.goals.push_back(waypoint["goal"]);
        goal_positions_(i) = waypoint["goal"];
    }

    for (int i=0; i<num_dimensions_; ++i)
    {
        char tmp_name[10] = "d1";
        tmp_name[2] = i;
        variable_names_.push_back(tmp_name);
    }

    initialized_ = true;
    return initialized_;

}

bool PolicyImprovementTest::computeCost(dmp::Trajectory& trajectory, Eigen::VectorXd& costs)
{
    costs = VectorXd::Zero(num_time_steps_);
    for (int i=0; i<num_time_steps_; ++i)
    {
        for (int d=0; d<num_dimensions_; ++d)
        {
            if (i == (int)(waypoint_cost_definition_.waypoint_timings[d] * num_time_steps_))
            {
                double c = (trajectory.getTrajectoryPosition(i, d) - waypoint_cost_definition_.waypoints[d]);
                c*=c;
                costs(i) += waypoint_cost_definition_.waypoint_cost_penalty * c;
                //ROS_ERROR("\t\tdim %d - %lf", d, waypoint_cost_definition_.waypoint_cost_penalty * c);
            }
            if (i==num_time_steps_-1)
            {
                double c = (trajectory.getTrajectoryPosition(i, d) - waypoint_cost_definition_.goals[d]);
                c*=c;
                costs(i) += waypoint_cost_definition_.final_cost_penalty * c;
            }
        }
    }
    return true;
}

/*bool PolicyImprovementTest::computeCost(const VectorXd &time, const MatrixXd &positions, const MatrixXd &velocities, VectorXd &instant_reward_vector,
                                        double &final_reward)
{

    if (!initialized_)
    {
        ROS_ERROR("Policy improvement test program is not initialized.");
        return initialized_;
    }

    int size = time.size();

    double final_cost_sum = 0;
    for (int d = 0; d < num_dimensions_; d++)
    {
        final_cost_sum += pow(positions(size - 1, d) - waypoint_cost_definition_.goals[d], 2);
    }
    final_reward = waypoint_cost_definition_.final_cost_penalty * final_cost_sum;

    for (int i = 0; i < size; i++)
    {
        double instant_velocity_cost_sum = 0;
        for (int d = 0; d < num_dimensions_; d++)
        {
            instant_velocity_cost_sum += pow(velocities(i, d), 2);
        }
        instant_reward_vector(i) = waypoint_cost_definition_.instant_velocity_cost_penalty * instant_velocity_cost_sum;

        for (int d = 0; d < num_dimensions_; d++)
        {
            if (i == (int)(waypoint_cost_definition_.waypoint_timings[d] * size))
            {
                instant_reward_vector(i) += waypoint_cost_definition_.waypoint_cost_penalty * pow(positions(i, d) - waypoint_cost_definition_.waypoints[d], 2);
            }
        }
    }

    return true;
}*/

bool PolicyImprovementTest::run_pi2()
{
    EXPECT_TRUE(initialized_);

    std::string debug_dir = "debug/";
    boost::filesystem::create_directories(debug_dir);

    double dt = 0.001;
    ROS_INFO("Setting dt to %f sec.", dt);

    //ros::NodeHandle node_handle(node_handle_,"test_parameters");
    boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp(new dmp::DynamicMovementPrimitive(node_handle_));

    EXPECT_TRUE(dmp->initialize(num_dimensions_, 1));
    VectorXd start = VectorXd::Zero(num_dimensions_);
    VectorXd goal = VectorXd::Zero(num_dimensions_);

    EXPECT_TRUE(dmp->learnFromMinJerk(start_positions_, goal_positions_, movement_duration_, dt));
    EXPECT_TRUE(dmp->setup(1.0/dt));

    policy_.reset(new DMPPolicy());
    EXPECT_TRUE(policy_->initialize(dmp));

    PolicyImprovement policy_improvement;
    EXPECT_TRUE(policy_improvement.initialize(num_rollouts_, num_time_steps_, num_reused_trials_, 0, policy_));

    //return true;
    double control_cost_weight=0.00001;
    double noise_decay=0.999;
    std::vector<std::vector<Eigen::VectorXd> > rollouts;
    std::vector<Eigen::MatrixXd> parameter_updates;
    std::vector<Eigen::VectorXd> parameters;
    Eigen::MatrixXd rollout_costs = MatrixXd::Zero(num_rollouts_, num_time_steps_);
    Eigen::VectorXd rollout_cost = VectorXd::Zero(num_time_steps_);
    std::vector<double> noise_vector;
    for (int i=0; i<num_dimensions_; ++i)
    {
        noise_vector.push_back(variance_);
    }

    // the PI^2 loop
    double original_cost = 0.0;
    double cost = 0.0;

    dmp::Trajectory trajectory(node_handle_);

    for (int i=0; i<num_trials_; ++i)
    {
        EXPECT_TRUE(policy_->getParameters(parameters));

        policy_->getDMP(dmp);
        //dmp->writeDebugTrajectory();

        // get the noise-less cost:
        EXPECT_TRUE(rollout(parameters, rollout_cost, trajectory));
        double cost = rollout_cost.sum();
        if (i==0)
            original_cost = cost;
        ROS_INFO("iteration %d, cost: %lf", i+1, cost);

        // write the noise-less traj to disk
        char filename[20];
        snprintf(filename, 20, "iter%03d.traj", i);
        std::string file_name_str = debug_dir+filename;
        trajectory.writeToCLMCFile(file_name_str);

        EXPECT_TRUE(policy_improvement.getRollouts(rollouts, noise_vector));
        //printf("size = %d", rollouts.size());
        for (int r=0; r<int(rollouts.size()); ++r)
        {
            rollout(rollouts[r], rollout_cost, trajectory);
            rollout_costs.row(r) = rollout_cost.transpose();
        }

        std::vector<double> all_costs;
        EXPECT_TRUE(policy_improvement.setRolloutCosts(rollout_costs, control_cost_weight, all_costs));
        EXPECT_TRUE(policy_improvement.improvePolicy(parameter_updates));
        EXPECT_TRUE(policy_->updateParameters(parameter_updates));
        for (int i=0; i<num_dimensions_; ++i)
            noise_vector[i] *= noise_decay;
    }

    EXPECT_TRUE(cost < 0.1*original_cost);

    return true;
}



bool PolicyImprovementTest::rollout(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, dmp::Trajectory& trajectory_out)
{
    VectorXd initial_velocities = VectorXd::Zero(num_dimensions_);


    boost::shared_ptr<dmp::DynamicMovementPrimitive> dmp_ptr;
    policy_->getDMP(dmp_ptr);

    dmp::Trajectory trajectory(node_handle_);

    if (!trajectory.initialize(num_dimensions_*3, sampling_frequency_, num_time_steps_+1, num_dimensions_*3, dmp::Trajectory::eDEBUG_DATA_TRACE))
    {
        ROS_ERROR("Could not initialize trajectory.");
        return false;
    }

    dmp::DynamicMovementPrimitive new_dmp = *dmp_ptr;
    new_dmp.setThetas(parameters);

    new_dmp.propagateFull(trajectory, sampling_duration_, num_time_steps_);

    computeCost(trajectory, costs);

    trajectory_out = trajectory;

    return true;
}

TEST(policy_improvement_test, run_pi2)
{
    PolicyImprovementTest pi2_test;
    EXPECT_TRUE(pi2_test.initialize());
    EXPECT_TRUE(pi2_test.run_pi2());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "policy_improvement_test");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
