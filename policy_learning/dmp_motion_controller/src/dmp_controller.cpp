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
#include <sstream>

// ros includes
#include <ros/package.h>

#include <dmp_motion_generation/constants.h>
#include <dmp_motion_generation/math_helper.h>

#include <policy_improvement_utilities/assert.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <Eigen/Eigen>
// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

// local includes
#include <dmp_motion_controller/dmp_controller.h>
#include <dmp_motion_controller/Goal.h>

namespace dmp_controller
{

const int SIZE_RUN_QUEUE = 100;

DMPController::DMPController() :
    num_dofs_(0), skipped_updates_(0), first_control_cycle_(true)
{
    int mutex_return_code = pthread_mutex_init(&dmp_cmd_lock_, NULL);
    if (mutex_return_code != 0)
    {
        ROS_ERROR("Could not initialize mutex (%i : %s).", mutex_return_code, strerror(mutex_return_code));
    }
    mutex_return_code = pthread_mutex_unlock(&dmp_cmd_lock_);
    if (mutex_return_code != 0)
    {
        ROS_ERROR("Could not unlock mutex (%i : %s).", mutex_return_code, strerror(mutex_return_code));
    }
}

DMPController::~DMPController()
{
}

bool DMPController::initialize(ros::NodeHandle& node_handle, const std::vector<int>& num_dofs, RepresentationType dmp_representation_type)
{
    for (int i=0; i<static_cast<int>(num_dofs.size()); ++i)
    {
        ROS_INFO("Initializing dmp controller with %i variables.", num_dofs[i]);
    }

    node_handle_ = node_handle;
    num_dofs_ = num_dofs;

    // advertise services
    add_to_execute_dmp_queue_service_server_ = node_handle_.advertiseService("add_to_execute_dmp_queue", &DMPController::addToExecuteDMPQueue, this);
    write_trajectories_service_server_ = node_handle_.advertiseService("write_trajectories", &DMPController::writeTrajectories, this);
    // get_controller_status_service_server_ = node_handle_.advertiseService("get_controller_status", &DMPController::getControllerStatus, this);

    ROS_ASSERT_FUNC(setControllerVariables(dmp_representation_type, node_handle_.getNamespace()));

    rosrt::init();
    dmp_motion_controller::DMPExecutionStatistics dmp_execution_statistics_msg;
    dmp_execution_statistics_publisher_.reset(new rosrt::Publisher<dmp_motion_controller::DMPExecutionStatistics>(node_handle_.advertise
            < dmp_motion_controller::DMPExecutionStatistics>(std::string("dmp_execution_statistics"), 1), SIZE_RUN_QUEUE, dmp_execution_statistics_msg));

    // pre-allocate memory
    run_buffer_.set_capacity(SIZE_RUN_QUEUE);
    completed_buffer_.set_capacity(SIZE_RUN_QUEUE);

    first_control_cycle_ = true;
    return true;
}

bool DMPController::setControllerVariables(RepresentationType dmp_representation_type, const std::string controller_namespace)
{

    std::string variable_names_key_word = controller_namespace;

    switch(dmp_representation_type)
    {
        case DMPController::UNINITIALIZED:
        {
            ROS_ERROR("Controller type is not set.");
            return false;
            break;
        }
        case DMPController::JOINT:
        {
            if(variable_names_key_word.compare(0,6,std::string("/l_arm")) == 0)
            {
                variable_names_key_word_.assign("pr2_l_arm_joint_names");
            }
            else if(variable_names_key_word.compare(0,6,std::string("/r_arm")) == 0)
            {
                variable_names_key_word_.assign("pr2_r_arm_joint_names");
            }
            else
            {
                ROS_ERROR("Invalid namespace: %s", controller_namespace.c_str());
                return false;
            }
            break;
        }
        case DMPController::CARTESIAN_AND_JOINT:
        {
            if(variable_names_key_word.compare(0,6,std::string("/l_arm")) == 0)
            {
                variable_names_key_word_.assign("cart_and_joint_left_arm");
            }
            else if(variable_names_key_word.compare(0,6,std::string("/r_arm")) == 0)
            {
                variable_names_key_word_.assign("cart_and_joint_right_arm");
            }
            else
            {
                ROS_ERROR("Invalid namespace: %s", controller_namespace.c_str());
                return false;
            }
            break;
        }
    }
    return true;
}

bool DMPController::addToQueue(std::vector<DMPStruct> dmp_structs)
{
    // ROS_INFO("Inserting %i dmp(s) into run dmp queue.", static_cast<int> (dmp_structs.size()));
    std::vector<DMPStruct>::iterator struct_it;
    if (pthread_mutex_lock(&dmp_cmd_lock_) == 0)
    {
        for(struct_it = dmp_structs.begin(); struct_it != dmp_structs.end(); ++struct_it)
        {
            run_buffer_.push_back(*struct_it);
        }
        pthread_mutex_unlock(&dmp_cmd_lock_);
    }
    return true;
}

bool DMPController::addToExecuteDMPQueue(dmp_motion_controller::AddToExecuteDMPQueue::Request& request,
                                         dmp_motion_controller::AddToExecuteDMPQueue::Response& response)
{

    std::vector<DMPStruct> dmp_structs;

    if(!DMPControllerCommon::setDMPStructFromRequest(node_handle_, request, response, num_dofs_, variable_names_key_word_, dmp_structs))
    {
        response.info.assign(std::string("Setting dmp structs failed."));
        response.return_code = dmp_motion_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_FAILED;
        ROS_ERROR("Setting dmp structs failed.");
        return true;
    }

    addToQueue(dmp_structs);

    response.info.assign(std::string("Starting dmp with ids "));
    std::vector<DMPStruct>::iterator struct_it;
    for(struct_it = dmp_structs.begin(); struct_it != dmp_structs.end(); ++struct_it)
    {
        response.info.append(dmp::MathHelper::getString(struct_it->dmp->getID()));
        if(struct_it+1 != dmp_structs.end())
        {
            response.info.append(std::string(", "));
        }
    }
    response.info.append(std::string("."));
    response.return_code = dmp_motion_controller::AddToExecuteDMPQueue::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
}

bool DMPController::writeTrajectories(dmp_motion_controller::WriteTrajectories::Request& request,
                                      dmp_motion_controller::WriteTrajectories::Response& response)
{

    double start_time = ros::Time::now().toSec();
    double wait_limit = 20.0;
    bool wait_limit_exceeded = true;

    while (start_time + wait_limit > ros::Time::now().toSec())
    {
        if (pthread_mutex_lock(&dmp_cmd_lock_) == 0)
        {
            // dmps are currently executed
            if (!run_buffer_.empty())
            {
                pthread_mutex_unlock(&dmp_cmd_lock_);
            }
            // there are no dmps that have not been written out yet.
            else if (completed_buffer_.empty())
            {
                pthread_mutex_unlock(&dmp_cmd_lock_);
                // ROS_ERROR("There is no debug information to be written out.");
                response.info.assign(std::string("There is no debug information to be written out."));
                response.return_code = dmp_motion_controller::WriteTrajectories::Response::SERVICE_CALL_FAILED;
                return true;
            }
            else
            {
                // keep mutex locked and unlock after writing out trajectories
                wait_limit_exceeded = false;
                break;
            }
        }
        ros::Duration(0.5).sleep();
    }

    if (wait_limit_exceeded)
    {
        response.info.assign(std::string("Maximum wait time ") + policy_improvement_utilities::getString(wait_limit) + std::string("s exceeded."));
        response.return_code = dmp_motion_controller::WriteTrajectories::Response::SERVICE_CALL_FAILED;
        return true;
    }

    std::stringstream ss;
    ss.precision(3);
    ss << std::fixed;
    std::string response_info;
    response_info.assign(std::string(""));

    if(completed_buffer_.empty())
    {
        ROS_WARN("Completed buffer is empty.");
        response.info.assign(std::string("Completed buffer is empty."));
        response.return_code = dmp_motion_controller::WriteTrajectories::Response::SERVICE_CALL_FAILED;
    }

    boost::circular_buffer<DMPStruct>::iterator it;
    for(it = completed_buffer_.begin(); it != completed_buffer_.end(); ++it)
    {
        std::string debug_run_filename = request.filename + std::string("_run.traj");
        if (!it->dmp->writeDebugTrajectory(debug_run_filename))
        {
            ROS_WARN("Could not write debug trajectory of dmp %i.", it->dmp->getID());
            response_info.append(std::string("Could not write debug trajectory of dmp ") + dmp::MathHelper::getString(it->dmp->getID()) + std::string(".\n"));
        }

        if( (!it->debug_object_desired->isInitialized()) || (!it->debug_object_actual->isInitialized()) )
        {
            ROS_WARN("Debug objects for dmp with id %i were not initialized, cannot compute MSE.",it->dmp->getID());
            response_info.append(std::string("Debug objects for dmp with id ") + dmp::MathHelper::getString(it->dmp->getID())
                    + std::string(" were not initialized, cannot compute MSE.\n"));
        }
        else
        {
            ROS_ASSERT(it->type >= 0);
            ROS_ASSERT(it->type < static_cast<int>(num_dofs_.size()));

            ROS_INFO("Computing MSE for %i dofs of type %i.", num_dofs_[it->type], it->type);

            VectorXd mse_vector = VectorXd::Zero(num_dofs_[it->type]);
            if (!it->debug_object_actual->computeMSE(*(it->debug_object_desired), mse_vector))
            {
                ROS_WARN("Could not compute MSE.");
                response.info.assign(std::string("Could not compute MSE.\n"));
            }
            else
            {
                response_info.append(std::string("MSE for dmp with id ") + dmp::MathHelper::getString(it->dmp->getID()) + std::string(" is "));
                for (int j = 0; j < num_dofs_[it->type]; ++j)
                {
                    ss << mse_vector(j);
                    response_info.append(ss.str() + std::string(" "));
                    ss.str("");
                    ss.clear();
                }
            }
            // Warning: writing out the debug trajectories will also clear them
            std::string debug_actual_filename = request.filename + std::string("_actual.traj");
            if (!it->debug_object_actual->writeToCLMCFile(debug_actual_filename))
            {
                ROS_WARN("Could not write debug object (actual) to file %s.", debug_actual_filename.c_str());
                response_info.append(std::string("Could not write debug object (actual) to file.\n"));
            }
            std::string debug_desired_filename = request.filename + std::string("_desired.traj");
            if (!it->debug_object_desired->writeToCLMCFile(debug_desired_filename))
            {
                ROS_WARN("Could not write debug object (desired) to file%s.", debug_actual_filename.c_str());
                response_info.append(std::string("Could not write debug object (desired) to file.\n"));
            }
        }
        response_info.append(std::string(".\n"));
    }

    completed_buffer_.clear();

    pthread_mutex_unlock(&dmp_cmd_lock_);

    response.info.assign(response_info);
    response.return_code = dmp_motion_controller::WriteTrajectories::Response::SERVICE_CALL_SUCCESSFUL;
    return true;
}

//bool DMPController::getControllerStatus(dmp_motion_controller::ControllerStatus::Request& request, dmp_motion_controller::ControllerStatus::Response& response)
//{
//    if (pthread_mutex_lock(&dmp_cmd_lock_) == 0)
//    {
//        if (run_queue_index_ < run_queue_size_)
//        {
//            pthread_mutex_unlock(&dmp_cmd_lock_);
//            std::stringstream ss;
//            ss.precision(3);
//            ss << std::fixed;
//            ss << run_queue_size_ - run_queue_index_;
//            response.info.assign(std::string("Controller is executing ") + ss.str() + std::string(" DMPs."));
//            response.return_code = dmp_motion_controller::ControllerStatus::Response::SERVICE_CALL_SUCCESSFUL
//                    | dmp_motion_controller::ControllerStatus::Response::EXECUTING_DMP;
//            return true;
//        }
//        else if (run_queue_size_ == 0)
//        {
//            pthread_mutex_unlock(&dmp_cmd_lock_);
//            response.info.assign(std::string("Controller is ready and DMP execution queue is empty."));
//            response.return_code = dmp_motion_controller::ControllerStatus::Response::SERVICE_CALL_SUCCESSFUL
//                    | dmp_motion_controller::ControllerStatus::Response::DMP_QUEUE_EMPTY;
//            return true;
//        }
//        else
//        {
//            pthread_mutex_unlock(&dmp_cmd_lock_);
//            std::stringstream ss;
//            ss.precision(3);
//            ss << std::fixed;
//            ss << run_queue_index_;
//            response.info.assign(std::string("Controller is ready and DMP execution queue has ") + ss.str() + std::string(" DMPs."));
//            response.return_code = dmp_motion_controller::ControllerStatus::Response::SERVICE_CALL_SUCCESSFUL;
//            return true;
//        }
//    }
//    response.return_code = dmp_motion_controller::ControllerStatus::Response::SERVICE_CALL_FAILED;
//    response.info.assign(std::string("Could not get lock of the DMP execution queue."));
//    return true;
//}

// REAL-TIME REQUIREMENTS
void DMPController::publishStatistics()
{

    dmp_motion_controller::DMPExecutionStatisticsPtr dmp_execution_statistics_msg;
    dmp_execution_statistics_msg = dmp_execution_statistics_publisher_->allocate();
    if(dmp_execution_statistics_msg)
    {
        dmp_execution_statistics_msg->dmp_id = run_buffer_.front().dmp->getID();
        dmp_execution_statistics_msg->start_time = start_time_;
        dmp_execution_statistics_msg->end_time = end_time_;
        dmp_execution_statistics_publisher_->publish(dmp_execution_statistics_msg);
    }
    else
    {
        ROS_ERROR("Could not publish execution statistics.");
    }
}

}
