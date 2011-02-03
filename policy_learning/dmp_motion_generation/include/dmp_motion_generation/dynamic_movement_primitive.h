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

#ifndef DYNAMIC_MOVEMENT_PRIMITIVE_H_
#define DYNAMIC_MOVEMENT_PRIMITIVE_H_

// system includes
#include <string>

// ros includes
#include <Eigen/Eigen>

#include <policy_improvement_utilities/param_server.h>
#include <lwr/parameters.h>

// local includes
#include <dmp_motion_generation/constants.h>
#include <dmp_motion_generation/parameters.h>
#include <dmp_motion_generation/trajectory.h>
#include <dmp_motion_generation/dmp_debug.h>
#include <dmp_motion_generation/transformation_system.h>

#include <dmp_motion_generation/DynamicMovementPrimitive.h>

namespace dmp
{

// forward declaration
class TransformationSystem;

// TODO: make this class a base class and derive different versions !!!

/*!
 *  \class DynamicMovementPrimitve represents the interface to learn a movement, save it to disc, load it from disc,
 *  reproduce the movement, generalize to new target, ...
 */
class DynamicMovementPrimitive
{

public:

    /*! constructor
     */
    DynamicMovementPrimitive(ros::NodeHandle& node_handle);

    /*! destructor
     */
    ~DynamicMovementPrimitive();

    /*! Initializes the DMP. This is necessary for all future use of the DMP.
     *
     * @param num_transformation_systems (input) Number of transformation systems (dimensions) of the DMP.
     * @param dmp_id (input) ID of the DMP
     * @param dmp_parameter_namespace (input) namespace in which the DMP parameters live on the param server
     * @param lwr_parameter_namespace (input) namespace in which the LWR parameters live on the param server
     * @return true if initialization was successful, false if it failed
     */
    bool initialize(int num_transformation_systems, int dmp_id, Parameters::Version version = Parameters::ICRA2009);

    // TODO: think about which of these are actually used and should be kept
    bool initialize(int num_transformation_systems, Parameters dmp_params, int dmp_id, Parameters::Version version = Parameters::ICRA2009);
    bool initialize(int num_transformation_systems, lwr::Parameters lwr_params, int dmp_id, Parameters::Version version = Parameters::ICRA2009);

    /*!
     * Initializes the DMP. This is necessary for all future use of the DMP.
     * @param num_transformation_systems (input) Number of transformation systems (dimensions) of the DMP.
     * @param dmp_params (input) parameter object that contains the neccessary parameters to initialize the DMP
     * @param lwr_params (input) parameter object that contains the neccessary parameters to initialize the LWR
     * @param dmp_id (input) ID of the DMP
     * @param version
     * @return true if initialization was successful, false if it failed
     */
    bool initialize(int num_transformation_systems, Parameters dmp_params, lwr::Parameters lwr_params, int dmp_id, Parameters::Version version =
            Parameters::ICRA2009);

    /*! Indicates whether the DMP is initialized
     *
     * @return true if DMP is initialized, false if not
     */
    bool isInitialized() const;

    /*!
     *
     * @param parameter_namespace
     * @return
     */
    bool reInitializeParams();

    /*! Initializes a DMP from a message
     *
     * @param dmp_msg (input)
     * @return true if initialization was successful, false if it failed
     */
    bool initFromMessage(const dmp_motion_generation::DynamicMovementPrimitive& dmp_msg);

    /*! Writes the DMP into a message
     *
     * @param dmp_msg (output)
     * @return true if initialization was successful, false if it failed
     */
    bool writeToMessage(dmp_motion_generation::DynamicMovementPrimitive& dmp_msg);

    /*!
     * @param trajectory
     * @return
     */
    bool learnFromTrajectory(const Trajectory& trajectory);

    /*!
     * @param trajectory
     * @return
     */
    bool learnFromMinJerk(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const double duration, const double delta_t);

    /*!
     * @param theta_matrix
     * @return
     */
    bool learnFromThetas(const std::vector<Eigen::VectorXd>& thetas, const Eigen::VectorXd& initial_start, const Eigen::VectorXd& initial_goal,
                         const double sampling_frequency, const double initial_duration);

    /*!
     */
    void print();

    /*!
     */
    // TODO: change this to not use verbosity stuff...
    std::string getInfoString();

    /*!
     * @return
     */
    bool getInitialDuration(double& initial_duration);

    /*!
     * @param initial_start
     * @return
     */
    bool getInitialStart(Eigen::VectorXd& initial_start);
    /*!
     * @param initial_goal
     * @return
     */
    bool getInitialGoal(Eigen::VectorXd& initial_goal);
    /*!
     * @param goal
     * @return
     */
    bool getGoal(Eigen::VectorXd& initial_goal);

    /*!
     * Gets the parameters of each transformation system
     * @param thetas
     * @return
     */
    bool getThetas(std::vector<Eigen::VectorXd>& thetas);

    /*!
     * Sets the parameters of each transformation system
     * @param thetas
     * @return
     */
    bool setThetas(const std::vector<Eigen::VectorXd>& thetas);

    /*!
     * Gets the widths and centers of each transformation system
     * @param thetas
     * @return
     */
    bool getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths, std::vector<Eigen::VectorXd>& centers);

    /*!
     * @param widths
     * @param centers
     * @return
     */
    bool getWidthsAndCenters(const int trans_system_index, Eigen::VectorXd& widths, Eigen::VectorXd& centers);

    /*!
     * Gets the number of receptive field used by transformation system with id trans_id
     * @param trans_id
     * @param num_rfs
     * @return
     */
    bool getNumRFS(const int trans_id, int& num_rfs);

    /*!
     * Gets the number of receptive fields for each transformation system
     * @param num_rfs
     * @return
     */
    bool getNumRFS(std::vector<int>& num_rfs);

    /*!
     *
     * @param num_time_steps
     * @param basis_functions
     * @return
     */
    bool getBasisFunctions(const int num_time_steps, std::vector<Eigen::MatrixXd>& basis_functions);

    /*! set start and goal to initial start and initial goal
     * @param sampling_frequency
     * @return
     */
    bool setup(const double sampling_frequency);
    /*!
     * @param goal
     * @param sampling_frequency
     * @return
     */
    // bool setup(const std::vector<double>& goal, const double sampling_frequency);
    bool setup(const Eigen::VectorXd& goal, const double movement_duration = -1.0, const double sampling_frequency = -1.0);

    /*!
     * @param start
     * @param goal
     * @param movement_duration
     * @param sampling_frequency
     * @return
     */
    bool setup(const Eigen::VectorXd& start, const Eigen::VectorXd& goal, const double movement_duration = -1.0, const double sampling_frequency = -1.0);

    /*!
     *
     * @param start
     * @param goal
     * @param movement_duration
     * @param sampling_frequency
     * @return
     */
    //    bool
    //    setup(const std::vector<double>& start, const std::vector<double>& goal, const double movement_duration = -1.0, const double sampling_frequency = -1.0);

    /*! REAL-TIME REQUIREMENTS
     * @param goal
     * @param start_index
     * @param end_index
     * @return
     */
    bool changeGoal(const Eigen::VectorXd& goal, const int start_index, const int end_index);
    bool changeGoal(const double new_goal, const int index);

    /*! REAL-TIME REQUIREMENTS
     *
     * @param start
     * @return
     */

    bool changeStart(const Eigen::VectorXd& start);
    /*!
     * @return
     */
    bool isSetup() const;
    bool isStartSet() const;
    void unsetStart();

    /*!
     * @param movement_duration
     * @param sampling_frequency
     * @return
     */
    bool setDuration(const double movement_duration, const int sampling_frequency);

    /*! REAL-TIME REQUIREMENTS
     *  Propagates the DMP and generates an entire rollout of size num_samples. The duration of the DMP need to be
     *  set previously using one of the setup functions. The sampling duration and the number of samples specified
     *  determine the length of the trajectory and its sampling frequecy. Note, the sampling frequency of the
     *  trajectory may change.
     *
     * @param trajectory
     * @param sampling_duration
     * @param num_samples
     * @return
     */
    bool propagateFull(Trajectory& trajectory, const double sampling_duration, const int num_samples);

    /*! REAL-TIME REQUIREMENTS
     *
     * @param desired_coordinates
     * @param finished
     * @param current_position
     * @param movement_duration
     * @return
     */
    bool propagateStep(Eigen::VectorXd& desired_coordinates, bool& movement_finished);

    /*! REAL-TIME REQUIREMENTS
     *
     * @param desired_coordinates
     * @param movement_finished
     * @param sampling_duration
     * @param num_samples
     * @return
     */
    bool propagateStep(Eigen::VectorXd& desired_coordinates, bool& movement_finished, const double sampling_duration, const int num_samples);

    /*! writes the data trace within dmp_debug to file. This has to happen outside the run() function since otherwise would violate the real-time constraints.
     *
     */
    bool writeDebugTrajectory();
    bool writeDebugTrajectory(const std::string& filename);

    /*!
     * @param current_desired_position
     * @param start_index
     * @param end_index
     * @return
     */
    bool getCurrentPosition(Eigen::VectorXd& current_desired_position, const int start_index, const int end_index);

    /*!
     * @return
     */
    double getProgress() const;

    /*!
     * @return
     */
    int getNumTransformationSystems() const;

    /*!
     * @param transformation_system_index
     * @param is_increasing
     * @return
     */
    bool isIncreasing(int transformation_system_index, bool& is_increasing);

    /*!
    *
    * @param canonical_system_value
    * @param canonical_system_time
    * @return
    */
    bool getCanonicalSystemState(double& canonical_system_value, double& canonical_system_time);

    /*!
    *
    * @param canonical_system_input
    * @param canonical_system_value
    * @return
    */
    bool getCanonicalSystemValue(const double canonical_system_input, double& canonical_system_value);

    /*!
     * Evaluates the canonical system using its duration (tau) and alpha_x at num_time_steps time steps going from 0 to 1.
     * @param num_time_steps
     * @param can_system_vector
     * @return
     */
    bool getCanonicalSystem(const int num_time_steps, Eigen::VectorXd& can_system_vector);

    /*! Reads a DMP from the directory provided by directory_name and the dmp_id
     * @param directory_name
     * @param item_id
     * @return
     */
    bool readFromDisc(const std::string& abs_bagfile_name);
    bool readFromDisc(const std::string& library_directory_name, const int dmp_id, const int trial_id = 0);

    /*! Writes the DMP to the directory
     * @return
     */
    bool writeToDisc(const int trial_id = 0);
    bool writeToDisc(const std::string& abs_bagfile_name);

    /*!
     * @return
     */
    Parameters::Version getVersion() const;
    /*!
     *
     * @param version
     * @return
     */
    bool getVersion(std::string& version) const;

    // ###################################################################################################
    // TODO: (MAJOR HACK!!) these functions/variables copied over from policy_library/library::LibraryItem
    // ###################################################################################################
    /*!
     *
     * @param directory_name
     * @param item_id
     * @param item_name
     * @param is_new_dmp
     * @return
     */
    bool initializeBase(const std::string directory_name, const int item_id, const std::string item_name, const bool is_new_dmp = true);

    /*!
     */
    int getID() const;
    bool setID(const int id);

    /*!
     */
    std::string getName() const;
    std::string getFileName(const int trial_id);

    /*!
     * @param description
     */
    void setDescription(const std::string& description);
    std::string getDescription() const;

    /*!
     * @return
     */
    std::string getClassName();


private:

    /*!
     * the structure that contains the states of the canonical system
     */
    typedef struct
    {
        double x;
        double time;
    } DMPCanonical;

    /*!
     */
    bool integrateAndFit();

    /*!
     *
     * @return
     */
    bool learnTransformationTarget();

    /*!
     *
     * @return returns false if the LWR model could not come up with a prediction for various reasons, otherwise true.
     */
    bool integrate(const double dt_total, const int num_iteration);

    //    /*!
    //     *
    //     * @param current_position
    //     * @return
    //     */
    //    bool integrate(const Eigen::VectorXd& current_position, const double sampling_duration, const int num_samples);

    /*!
     * Evaluates the canonical system at the given time step (canonical_system_time) and writes it to canonical_system_x
     * @param canonical_system_x (output)
     * @param canonical_system_time (input)
     * @return
     */
    void integrateCanonicalSystem(double& canonical_system_x, const double canonical_system_time) const;

    //    /*! Reads the version enum and computes the desired acceleration using the current state of the internal variables
    //     *
    //     * @param acceleration
    //     * @return
    //     */
    //    bool computeAcceleration(double& acceleration, const int index, const double nonlinear_term);
    //
    //    /*! Reads the version enum and computes the desired acceleration using the current state of the internal variables
    //     *
    //     * @param acceleration
    //     * @return
    //     */
    //    bool computeTarget(double& target, const int index);

    /*!
     *
     * @param obstacle_position
     * @param obstacle_velocity
     * @param repulsive_force
     */
    void computeObstAvoidanceTerm(const Eigen::VectorXd& obstacle_position, const Eigen::VectorXd& obstacle_velocity, Eigen::VectorXd& repulsive_force);

    /*!
     */
    void resetCanonicalState();

    /*!
     */
    void initialize();

    // #################################################################################################
    // TODO: (MAJOR HACK) these functions/variables copied over from policy_library/library::LibraryItem
    bool initialized_;
    int item_id_;
    std::string item_name_;
    std::string library_directory_name_;
    std::string description_;
    std::string initial_item_name_;
    // #################################################################################################

    /*!
     */
    ros::NodeHandle node_handle_;

    /*!
     * parameters that are read from file and (usually) kept the same for all DMPs
     */
    Parameters params_;

    /*!
     * struct that contains the variables of the canonical system (the phase variable x and time)
     */
    DMPCanonical canonical_system_;

    /*!
     * vector of num_transformation_systems_ instances of the class DMPTransformationSystem that contain relevant
     * information for each dimension of the movement system.
     */
    std::vector<TransformationSystem> transformation_systems_;

    /*!
     */
    std::vector<double> trajectory_target_function_input_;

    /*!
     */
    DMPDebug dmp_debug_;

    /*!
     */
    Eigen::VectorXd debug_trajectory_point_;

};

// inline functions follow
inline bool DynamicMovementPrimitive::isInitialized() const
{
    return initialized_;
}
inline bool DynamicMovementPrimitive::isSetup() const
{
    return params_.is_setup_;
}
inline bool DynamicMovementPrimitive::isStartSet() const
{
    return params_.is_start_set_;
}
inline void DynamicMovementPrimitive::unsetStart()
{
    params_.is_start_set_ = false;
}

inline void DynamicMovementPrimitive::resetCanonicalState()
{
    canonical_system_.x = 1.0;
    canonical_system_.time = 0.0;
    params_.num_samples_ = 0;
}
inline bool DynamicMovementPrimitive::getInitialDuration(double& initial_duration)
{
    if (!initialized_)
    {
        return false;
    }
    initial_duration = params_.initial_tau_;
    return true;
}
inline int DynamicMovementPrimitive::getNumTransformationSystems() const
{
    return params_.num_transformation_systems_;
}

// #################################################################################################
// TODO: (MAJOR HACK) these functions/variables copied over from policy_library/library::LibraryItem
// #################################################################################################
inline int DynamicMovementPrimitive::getID() const
{
    return item_id_;
}
inline bool DynamicMovementPrimitive::setID(const int item_id)
{
    item_id_ = item_id;
    return true;
    // HACK...
    // return initializeBase(library_directory_name_, item_id_, initial_item_name_, false);
}
inline std::string DynamicMovementPrimitive::getName() const
{
    return item_name_;
}
inline void DynamicMovementPrimitive::setDescription(const std::string& description)
{
    description_.assign(description);
}
inline std::string DynamicMovementPrimitive::getDescription() const
{
    return description_;
}

inline std::string DynamicMovementPrimitive::getClassName()
{
    return "DynamicMovementPrimitive";
}

inline Parameters::Version DynamicMovementPrimitive::getVersion() const
{
    return params_.version_;
}
inline bool DynamicMovementPrimitive::getVersion(std::string& version) const
{
    switch (params_.version_)
    {
        case Parameters::ICRA2009:
            version.assign(std::string("icra2009"));
            break;

        case dmp::Parameters::NIPS2003:
            version.assign(std::string("nips2003"));
            break;

        default:
            ROS_ERROR("DMP version (%i) is invalid.", params_.version_);
            return false;
    }
    return true;
}

// #################################################################################################
// TODO: (MAJOR HACK) these functions/variables copied over from policy_library/library::LibraryItem
// #################################################################################################
inline bool DynamicMovementPrimitive::initializeBase(const std::string library_directory_name, const int item_id, const std::string item_name,
                                                     const bool is_new_dmp)
{
    // ROS_WARN_COND(initialized_, "DMP is already initialized. Re-initializing it.");

    if (library_directory_name.empty())
    {
        ROS_ERROR("Library directory name is empty.");
        initialized_ = false;
        return initialized_;
    }
    if (item_id < 0)
    {
        ROS_ERROR("Item id is %i and therefore not valid.", item_id);
        initialized_ = false;
        return initialized_;
    }

    initial_item_name_.assign(item_name);
    library_directory_name_.assign(library_directory_name);
    item_id_ = item_id;

    item_name_.assign(library_directory_name_ + item_name + std::string("_") + policy_improvement_utilities::getString(item_id_));
    // ROS_INFO("Initializing item %s with id %i.",item_name_.c_str(), item_id_);

    return (initialized_ = true);
}

}

#endif /* DYNAMIC_MOVEMENT_PRIMITIVE_H_ */
