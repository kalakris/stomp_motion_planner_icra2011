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

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

// system includes
#include <vector>
#include <string>

// ros includes
#include <ros/ros.h>
#include <Eigen/Eigen>

// local includes
#include <dmp_motion_generation/parameters.h>
#include <dmp_motion_generation/constants.h>

namespace dmp
{

static const int MAX_TRAJECTORY_LENGTH = 10000;
static const int MAX_TRAJECTORY_DIMENSION = 180;

static const int ABSOLUTE_MAX_TRAJECTORY_LENGTH = 20 * MAX_TRAJECTORY_LENGTH;
static const int ABSOLUTE_MAX_TRAJECTORY_DIMENSION = 20 * MAX_TRAJECTORY_DIMENSION;

class Trajectory
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum eTrajectoryType
    {
        eUNINITIALIZED, eR_ARM_JOINT_SPACE, eR_ARM_CARTESIAN_SPACE, eL_ARM_JOINT_SPACE, eL_ARM_CARTESIAN_SPACE, eR_ARM_CARTESIAN_AND_JOINT_SPACE,
        eDEBUG_DATA_TRACE
    };

    /*! constructor
     * Trajectory is a class that stores a trajectory that contains trajectory_dimension.
     * After instantiating a trajectory object, one need to initialize it with
     * the appropriate sampling_frquency, then one can add(trajectory_points).
     */
    Trajectory(ros::NodeHandle& node_handle);

    /*! destructor
     *
     */
    ~Trajectory();

    /*! initializes the object with variable names and units read from the paramserver.
     * If the trajectory is already initialized, the trajectory will be re-initialized with the given arguments.
     * @param sampling_frequency
     * @return
     */
    bool initialize(const std::string& variable_names_key_word, const int trajectory_dimension,
                    const double sampling_frequency, const int num_debug_traces_per_trans_sys, const int num_extra_debug_traces,
                    const int max_trajectory_length = MAX_TRAJECTORY_LENGTH, const int max_trajectory_dimension = MAX_TRAJECTORY_DIMENSION,
                    eTrajectoryType type = eUNINITIALIZED);

    /*! initializes the trajectory with the variable names provided as argument. The trajectory dimension depends on the argument containsVelAcc. If the
     * argument is set to false (default) the variable names will be treated as position only variables and velocity (_d) and acceleration (_dd) names
     * will be added.
     * It also sets the sampling frequency and type.
     *
     * @param variable_names
     * @param sampling_frequency
     * @param type
     * @return Returns true on success, else false.
     */
    bool initialize(const std::vector<std::string>& variable_names, const double sampling_frequency, bool containsVelAcc = false,
                    const int max_trajectory_length = MAX_TRAJECTORY_LENGTH, const int max_trajectory_dimension = MAX_TRAJECTORY_DIMENSION,
                    eTrajectoryType type = eUNINITIALIZED);

    /*! initializes the trajectory and sets the dimension of the trajectory and its sampling frequency accordingly. The
     *  variable names and units are set to default names and units.
     *
     * @param trajectory_dimension
     * @param sampling_frequency
     * @param type
     * @return
     */
    bool initialize(const int trajectory_dimension, const double sampling_frequency, const int max_trajectory_length = MAX_TRAJECTORY_LENGTH,
                    const int max_trajectory_dimension = MAX_TRAJECTORY_DIMENSION, eTrajectoryType type = eUNINITIALIZED);

    /*!
     *
     * @return
     */
    bool isInitialized() const;

    /*!
     * @param variable_names
     * @return
     */
    bool setVariableNames(const std::vector<std::string>& variable_names);

    /*!
     *
     * @param file_name
     * @return
     */
    bool readFromCLMCFile(std::string file_name);

    /*!
     * @param file_name
     * @return
     */
    bool writeToCLMCFile(std::string file_name);

    /*! adds the trajectory_point (of size trajectory_dimension) to the trajectory.
     *
     * The procedure checks whether sampling frequency is specified correctly and also updates the trajectory_duration
     * REAL-TIME REQUIREMENTS
     * @param trajectory_point
     * @param computeVelocityAndAcceleration
     */
    bool add(const Eigen::VectorXd& trajectory_point);

    /*!
     * @param dimension_index
     * @param length_index
     * @param value
     * @return
     */
    bool update(const int length_index, const int dimension_index, const double value);

    /*!
     *
     * @param trajectory_index
     * @param trajectory_point
     */
    bool getTrajectoryPoint(const int trajectory_index, Eigen::VectorXd& trajectory_point);

    /*!
     */
    void print();
    void clear();

    /*!
     */
    int getDimension() const;
    int getLength() const;

    /*!
     */
    int getMaxDimension() const;
    int getMaxLength() const;

    /*!
     */
    bool setSamplingFrequency(double sample_frequency);
    double getSamplingFrequency() const;

    /*!
     */
    eTrajectoryType getType() const;
    void setType(eTrajectoryType type);

    /*!
     *
     * @param trajectory_start
     * @return
     */
    bool getStartPosition(Eigen::VectorXd& trajectory_start) const;
    bool getEndPosition(Eigen::VectorXd& trajectory_end) const;

    /*!
     * @param trajectory_index
     * @param trajecotry_dimension
     * @return
     */
    double getTrajectoryValue(int length_index, int trajecotry_dimension) const;

    /*!
     * @param trajectory_index
     * @param trajectory_trace
     * @return
     */
    double getTrajectoryPosition(int trajectory_index, int trajectory_trace) const;
    double getTrajectoryVelocity(int trajectory_index, int trajectory_trace) const;
    double getTrajectoryAcceleration(int trajectory_index, int trajectory_trace) const;

    /*!
     * @return
     */
    std::vector<std::string> getPositionVariableNames();

    /*!
     * @param time_stamps
     * @param additional_boundary_duration
     * @return
     */
    bool resample(std::vector<ros::Time> time_stamps, double additional_boundary_duration = 0.0, const std::vector<int>& quaternion_indices = std::vector<int>());

    /*!
     */
    void computeDerivatives();

    /*!
     * @param other_trajectory
     * @return
     */
    bool computeMSE(const Trajectory& other_trajectory, Eigen::VectorXd& mse_vector) const;

private:

    /*!
     */
    bool initialized_;

    /*!
     */
    int trajectory_length_;
    int trajectory_dimension_;

    /*!
     *
     */
    int max_trajectory_length_;
    int max_trajectory_dimension_;

    /*!
     */
    eTrajectoryType trajectory_type_;

    /*!
     */
    double sampling_frequency_;

    /*!
     * mainly for debugging purpose (since it is actually redundant) to check for consistency
     * between trajectory_length, sampling_frequency and trajectory_duration
     */
    double trajectory_duration_;

    /*!
     */
    std::vector<std::string> variable_names_;
    std::vector<std::string> variable_units_;

    /*! This matrix actually contains the data of the trajectory.
     */
    Eigen::MatrixXd trajectory_data_;

    /*!
     */
    ros::NodeHandle node_handle_;

    /*!
     * @param variable_names_key_word
     * @param parameter_namespace
     * @param num_debug_traces_per_trans_sys
     * @param num_extra_debug_traces
     */
    void initializeFromNodeHandle(const std::string& variable_names_key_word, const int num_debug_traces_per_trans_sys, const int num_extra_debug_traces);

    /*!
     * @param dimension_index
     * @return
     */
    bool isWithinDimensionBoundaries(const int dimension_index) const;
    /*!
     *
     * @param length_index
     * @return
     */
    bool isWithinLengthBoundaries(const int length_index) const;

};

// inline functions follow

inline bool Trajectory::isInitialized() const
{
    return initialized_;
}

inline int Trajectory::getDimension() const
{
    return trajectory_dimension_;
}
inline int Trajectory::getLength() const
{
    return trajectory_length_;
}

inline bool Trajectory::setSamplingFrequency(double sampling_frequency)
{
    if (sampling_frequency <= 0.0)
    {
        ROS_ERROR("Sampling frequency is %.1f and therefore invalid.",sampling_frequency);
        return false;
    }
    sampling_frequency_ = sampling_frequency;
    trajectory_duration_ = static_cast<double> (trajectory_length_) / sampling_frequency_;
    return true;
}
inline double Trajectory::getSamplingFrequency() const
{
    return sampling_frequency_;
}

inline Trajectory::eTrajectoryType Trajectory::getType() const
{
    return trajectory_type_;
}

inline void Trajectory::setType(eTrajectoryType type)
{
    trajectory_type_ = type;
}

inline bool Trajectory::getStartPosition(Eigen::VectorXd &trajectory_start) const
{
    if (trajectory_length_ == 0)
    {
        ROS_ERROR("Trajectory is empty (trajectory length is %i)",trajectory_length_);
        return false;
    }
    if (trajectory_dimension_ == 0)
    {
        ROS_ERROR("Trajectory is empty (trajectory dimension is %i)",trajectory_dimension_);
        return false;
    }

    if (trajectory_start.size() == trajectory_dimension_)
    {
        trajectory_start = trajectory_data_.row(trajectory_length_ - 1);
    }
    else if (trajectory_start.size() == trajectory_dimension_ / POS_VEL_ACC)
    {
        for (int i = 0; i < trajectory_dimension_ / POS_VEL_ACC; i++)
        {
            trajectory_start(i) = trajectory_data_(0, i * POS_VEL_ACC);
        }
    }
    else
    {
        return false;
    }
    return true;
}

inline bool Trajectory::getEndPosition(Eigen::VectorXd &trajectory_end) const
{
    if (trajectory_length_ == 0)
    {
        ROS_ERROR("Trajectory is empty (trajectory length is %i)",trajectory_length_);
        return false;
    }
    if (trajectory_dimension_ == 0)
    {
        ROS_ERROR("Trajectory is empty (trajectory dimension is %i)",trajectory_dimension_);
        return false;
    }

    if (trajectory_end.size() == trajectory_dimension_)
    {
        trajectory_end = trajectory_data_.row(trajectory_length_ - 1);
    }
    else if (trajectory_end.size() == trajectory_dimension_ / POS_VEL_ACC)
    {
        for (int i = 0; i < trajectory_dimension_ / POS_VEL_ACC; i++)
        {
            trajectory_end(i) = trajectory_data_(trajectory_length_ - 1, i * POS_VEL_ACC);
        }
    }
    else
    {
        return false;
    }
    return true;
}

inline double Trajectory::getTrajectoryPosition(int trajectory_index, int trajectory_trace) const
{
    // TODO: think about leaving out all the if and else...
    if ((trajectory_trace >= 0) && (trajectory_trace < trajectory_dimension_ / POS_VEL_ACC))
    {
        if ((trajectory_index >= 0) && (trajectory_index < trajectory_length_))
        {
            // TODO: think about changing _POS_ to 0 and _VEL_ to 1 and...
            return trajectory_data_(trajectory_index, (trajectory_trace * POS_VEL_ACC) + _POS_ - 1);
        }
        else
        {
            ROS_ERROR("Trajectory index (%i) not valid (trajectory length is %i), returning 0.0",trajectory_index, trajectory_length_);
            return 0.0;
        }
    }
    else
    {
        ROS_ERROR("Trajectory trace (%i) not valid (trajectory dimension is %i), returning 0.0",trajectory_trace, trajectory_dimension_);
        return 0.0;
    }
    return 0.0;
}
inline double Trajectory::getTrajectoryVelocity(int trajectory_index, int trajectory_trace) const
{
    if ((trajectory_trace >= 0) && (trajectory_trace < trajectory_dimension_ / POS_VEL_ACC))
    {
        if ((trajectory_index >= 0) && (trajectory_index < trajectory_length_))
        {
            return trajectory_data_(trajectory_index, (trajectory_trace * POS_VEL_ACC) + _VEL_ - 1);
        }
        else
        {
            ROS_ERROR("Trajectory index (%i) not valid (trajectory length is %i), returning 0.0",trajectory_index, trajectory_length_);
            return 0.0;
        }
    }
    else
    {
        ROS_ERROR("Trajectory trace (%i) not valid (trajectory dimension is %i), returning 0.0",trajectory_trace, trajectory_dimension_);
        return 0.0;
    }
    return 0.0;
}
inline double Trajectory::getTrajectoryAcceleration(int trajectory_index, int trajectory_trace) const
{
    if ((trajectory_trace >= 0) && (trajectory_trace < trajectory_dimension_ / POS_VEL_ACC))
    {
        if ((trajectory_index >= 0) && (trajectory_index < trajectory_length_))
        {
            return trajectory_data_(trajectory_index, (trajectory_trace * POS_VEL_ACC) + _ACC_ - 1);
        }
        else
        {
            ROS_ERROR("Trajectory index (%i) not valid (trajectory length is %i), returning 0.0",trajectory_index, trajectory_length_);
            return 0.0;
        }
    }
    else
    {
        ROS_ERROR("Trajectory trace (%i) not valid (trajectory dimension is %i), returning 0.0",trajectory_trace, trajectory_dimension_);
        return 0.0;
    }
    return 0.0;
}

inline double Trajectory::getTrajectoryValue(int length_index, int dimension_index) const
{
    if (isWithinLengthBoundaries(length_index))
    {
        if (isWithinDimensionBoundaries(dimension_index))
        {
            return trajectory_data_(length_index, dimension_index);
        }
        else
        {
            ROS_ERROR("Trajectory dimension %i is out of bound [0..%i], returning 0.0",dimension_index, trajectory_dimension_);
            return 0.0;
        }
    }
    else
    {
        ROS_ERROR("Trajectory index %i is out of bound [0..%i], returning 0.0",length_index, trajectory_length_);
        return 0.0;
    }
    return 0.0;
}

inline bool Trajectory::isWithinDimensionBoundaries(const int dimension_index) const
{
    return ((dimension_index >= 0) && (dimension_index < trajectory_dimension_));
}
inline bool Trajectory::isWithinLengthBoundaries(const int length_index) const
{
    return ((length_index >= 0) && (length_index < trajectory_length_));
}

}

#endif /* TRAJECTORY_H_ */
