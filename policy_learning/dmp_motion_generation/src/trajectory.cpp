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

// 32 bit word byte/word swap macros
#define LLSB(x) ((x) & 0xff)
#define LNLSB(x) (((x) >> 8) & 0xff)
#define LNMSB(x) (((x) >> 16) & 0xff)
#define LMSB(x) (((x) >> 24) & 0xff)
#define LONGSWAP(x) ((LLSB(x) << 24) | (LNLSB(x) << 16)| (LNMSB(x) << 8) | (LMSB(x)))

//system includes
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <errno.h>
#include <assert.h>

//ros includes
#include <bspline/BSpline.h>

#include <policy_improvement_utilities/param_server.h>
#include <policy_improvement_utilities/assert.h>

//local includes
#include <dmp_motion_generation/trajectory.h>
#include <dmp_motion_generation/parameters.h>

// import most common Eigen types
USING_PART_OF_NAMESPACE_EIGEN

namespace dmp
{

// TODO: come up with some reasonable default values
static const int MAX_TRAJECTORY_SAMPLING_FREQUENCY = 1100;

static const char* DEFAULT_VARIABLE_NAME = "name";
static const char* DEFAULT_VARIABLE_UNIT = "unit";

Trajectory::Trajectory(ros::NodeHandle& node_handle) :
    initialized_(false), trajectory_length_(0), trajectory_dimension_(0), max_trajectory_length_(0.0), max_trajectory_dimension_(0.0),
            sampling_frequency_(-1.0), trajectory_duration_(0.0), node_handle_(node_handle)
{
}

Trajectory::~Trajectory()
{
    // ROS_ERROR("Delete trajectory...");
    variable_names_.clear();
    variable_units_.clear();
}

bool Trajectory::initialize(const std::vector<std::string>& variable_names, const double sampling_frequency, bool containsVelAcc,
                            const int max_trajectory_length, const int max_trajectory_dimension, eTrajectoryType type)
{
    // ROS_WARN_COND(initialized_, "Trajectory is already initialized. Invalidating trajectory initialization and re-initializing it with given parameters.");
    initialized_ = false;

    int trajectory_dimension = static_cast<int> (variable_names.size());
    if (!containsVelAcc)
    {
        trajectory_dimension *= POS_VEL_ACC;
    }

    if (!initialize(trajectory_dimension, sampling_frequency, max_trajectory_length, max_trajectory_dimension, type))
    {
        ROS_ERROR("Could not initialize trajectory.");
        initialized_ = false;
        return initialized_;
    }

    if (!setVariableNames(variable_names))
    {
        ROS_ERROR("Could not set variable names.");
        initialized_ = false;
        return initialized_;
    }

    initialized_ = true;
    return initialized_;
}

bool Trajectory::initialize(const std::string& variable_names_key_word, const int trajectory_dimension, const double sampling_frequency,
                            const int num_debug_traces_per_trans_sys, const int num_extra_debug_traces, const int max_trajectory_length,
                            const int max_trajectory_dimension, eTrajectoryType type)
{
    // ROS_WARN_COND(initialized_, "Trajectory already initialized. Re-initialization with given parameters (%s, %i, %.1f).", variable_names_key_word.c_str(), trajectory_dimension, sampling_frequency);
    initialized_ = false;

    bool initialization_successfull = initialize(trajectory_dimension, sampling_frequency, max_trajectory_length, max_trajectory_dimension, type);
    if (initialization_successfull)
    {
        initializeFromNodeHandle(variable_names_key_word, num_debug_traces_per_trans_sys, num_extra_debug_traces);
    }
    return initialization_successfull;
}

bool Trajectory::initialize(const int trajectory_dimension, const double sampling_frequency, const int max_trajectory_length,
                            const int max_trajectory_dimension, eTrajectoryType type)
{

    ROS_WARN_COND(initialized_, "Trajectory is already initialized. Invalidating trajectory initialization and re-initializing it with given parameters.");
    initialized_ = false;

    // check whether trajectory_dimension is valid, and set if it is valid.
    if ((trajectory_dimension <= 0) || (trajectory_dimension > max_trajectory_dimension))
    {
        ROS_ERROR("Trajectory dimension %i is not valid. Should be within [0..%i].", trajectory_dimension, max_trajectory_dimension);
        initialized_ = false;
        return initialized_;
    }
    trajectory_dimension_ = trajectory_dimension;
    ROS_WARN_COND((trajectory_dimension_ % POS_VEL_ACC) != 0,"Trajectory dimension (%i) is not a multiple of 3 (for pos, vel, and acc), this may cause a problem", trajectory_dimension_);

    // check whether sampling frequency is valid, and set if it is valid.
    if ((sampling_frequency <= 0.0) || (sampling_frequency > MAX_TRAJECTORY_SAMPLING_FREQUENCY))
    {
        ROS_ERROR("Invalid sampling frequency %.1f. It must be within (0..%i]", sampling_frequency, MAX_TRAJECTORY_SAMPLING_FREQUENCY);
        initialized_ = false;
        return initialized_;
    }
    sampling_frequency_ = sampling_frequency;
    trajectory_duration_ = static_cast<double> (trajectory_length_) / sampling_frequency_;

    //    if (trajectory_duration_ != 0)
    //    {
    //        ROS_ERROR("Trajectory length (%i) is not zero. This should never happen.", trajectory_length_);
    //        initialized_ = false;
    //        return initialized_;
    //    }

    // set all variable names and units to default names and units,
    // initializeFromNodeHandle sets variable names from paramserver which is is called afterwards (if desired).
    variable_names_.clear();
    for (int i = 0; i < trajectory_dimension_; i++)
    {
        std::stringstream ss;
        ss << i;
        // valgrind says: 128 bytes in 2 blocks are still reachable in loss record 3 of 8 ??
        variable_names_.push_back(std::string(DEFAULT_VARIABLE_NAME) + std::string("_") + ss.str());
    }
    variable_units_.clear();
    for (int i = 0; i < trajectory_dimension_; i++)
    {
        std::stringstream ss;
        ss << i;
        variable_units_.push_back(std::string(DEFAULT_VARIABLE_UNIT) + std::string("_") + ss.str());
    }

    max_trajectory_length_ = max_trajectory_length;
    max_trajectory_dimension_ = max_trajectory_dimension;
    trajectory_data_ = MatrixXd::Zero(max_trajectory_length_, max_trajectory_dimension_);
    // ROS_WARN("Allocation memory for trajectory.");

    trajectory_type_ = type;

    clear();

    // everything is correctly initialized
    return (initialized_ = true);
}

void Trajectory::initializeFromNodeHandle(const std::string& variable_names_key_word, const int num_debug_traces_per_trans_sys,
                                          const int num_extra_debug_traces)
{

    // count the number of variable names
    int tmp_trajectory_dimension = 0;

    // temporary variable names and units
    std::vector<std::string> tmp_variable_names_vector;
    std::vector<std::string> tmp_variable_units_vector;

    if (variable_names_key_word.compare(0, 5, std::string("debug")) == 0)
    {
        // ROS_ERROR("Trajectory is initialized using %s .", variable_names_key_word.c_str());

        if (trajectory_type_ == eUNINITIALIZED)
        {
            trajectory_type_ = eDEBUG_DATA_TRACE;
        }
        else if (trajectory_type_ != eDEBUG_DATA_TRACE)
        {
            ROS_WARN("Trajectory is initialized using %s but is not of type eDEBUG_DATA_TRACE.", variable_names_key_word.c_str());
        }

        std::string tmp_group_names;
        ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, variable_names_key_word, tmp_group_names));

        std::vector<std::string> name_buffer;
        std::vector<std::string> unit_buffer;

        // split the group names based on whitespace
        std::stringstream ss_variable_group_names(tmp_group_names);

        std::string tmp_group_name;
        while (ss_variable_group_names >> tmp_group_name)
        {
            std::string tmp_variable_names;
            // ROS_ASSERT_FUNC(policy_improvement_utilities::read(node_handle_, tmp_group_name, tmp_variable_names));
            node_handle_.param(tmp_group_name, tmp_variable_names, std::string(""));

            // split the variable names based on whitespace
            std::stringstream ss_variable_names(tmp_variable_names);

            std::string tmp_variable_name;
            while (ss_variable_names >> tmp_variable_name)
            {
                std::string tmp_string;
                ros::NodeHandle variable_name_node_handle(node_handle_, tmp_variable_name);
                variable_name_node_handle.param(std::string("name"), tmp_string, std::string("empty"));

                name_buffer.push_back(tmp_string);
                variable_name_node_handle.param(std::string("unit"), tmp_string, std::string("empty"));
                unit_buffer.push_back(tmp_string);
                tmp_trajectory_dimension++;
            }
        }

        if (tmp_trajectory_dimension >= num_extra_debug_traces)
        {
            tmp_variable_names_vector.assign(name_buffer.begin(), name_buffer.begin() + num_extra_debug_traces);
            tmp_variable_units_vector.assign(unit_buffer.begin(), unit_buffer.begin() + num_extra_debug_traces);
        }

        if ((((trajectory_dimension_ - num_extra_debug_traces) % (tmp_trajectory_dimension - num_extra_debug_traces)) == 0) && !name_buffer.empty())
        {
            std::string tmp_string;
            tmp_trajectory_dimension = num_extra_debug_traces;
            int transformation_system_id = 0;
            while (tmp_trajectory_dimension < trajectory_dimension_)
            {
                std::stringstream ssi;
                ssi << transformation_system_id;

                int index = (num_extra_debug_traces - 1) + ((tmp_trajectory_dimension - num_extra_debug_traces) % num_debug_traces_per_trans_sys) + 1;
                tmp_string.assign(name_buffer[index] + std::string("_") + ssi.str());
                tmp_variable_names_vector.push_back(tmp_string);
                tmp_string.assign(unit_buffer[index] + std::string("_") + ssi.str());
                tmp_variable_units_vector.push_back(tmp_string);
                tmp_trajectory_dimension++;
                if (((tmp_trajectory_dimension - num_extra_debug_traces) % num_debug_traces_per_trans_sys) == 0)
                {
                    transformation_system_id++;
                }
            }
        }
    }
    else
    {
        std::string tmp_names;
        node_handle_.param(variable_names_key_word, tmp_names, std::string(""));

        if (variable_names_key_word.compare(0, 5, std::string("cart_r")) == 0)
        {
            trajectory_type_ = eR_ARM_CARTESIAN_SPACE;
        }
        else if (variable_names_key_word.compare(0, 5, std::string("cart_l")) == 0)
        {
            trajectory_type_ = eL_ARM_CARTESIAN_SPACE;
        }
        else if (variable_names_key_word.compare(0, 9, std::string("pr2_r_arm")) == 0)
        {
            trajectory_type_ = eR_ARM_JOINT_SPACE;
        }
        else if (variable_names_key_word.compare(0, 9, std::string("pr2_l_arm")) == 0)
        {
            trajectory_type_ = eL_ARM_JOINT_SPACE;
        }
        else if (variable_names_key_word.compare(0, 14, std::string("cart_and_joint")) == 0)
        {
            trajectory_type_ = eR_ARM_CARTESIAN_AND_JOINT_SPACE;
        }
        else if (trajectory_type_ != eUNINITIALIZED && trajectory_type_ != eDEBUG_DATA_TRACE)
        {
            ROS_WARN("Trajectory is initialized with type %i, but key is %s.", static_cast<int>(trajectory_type_), variable_names_key_word.c_str());
        }

        ROS_WARN_COND(tmp_names.empty(),"Could not retrieve variable_names_key_word >>%s<< from paramserver in namespace %s.", variable_names_key_word.c_str(), node_handle_.getNamespace().c_str());

        // split the variable names based on whitespace
        std::stringstream ss(tmp_names);

        // count the number of variable names
        tmp_trajectory_dimension = 0;
        while (ss >> tmp_names)
        {
            ros::NodeHandle variable_name_node_handle(node_handle_, tmp_names);
            std::string name;
            variable_name_node_handle.param(std::string("name"), name, std::string("empty"));
            tmp_variable_names_vector.push_back(name);
            variable_name_node_handle.param(std::string("unit"), name, std::string("empty"));
            tmp_variable_units_vector.push_back(name);
            tmp_trajectory_dimension++;
        }
    }

    if (tmp_trajectory_dimension == trajectory_dimension_)
    {
        variable_names_.clear();
        variable_names_ = tmp_variable_names_vector;
        variable_units_.clear();
        variable_units_ = tmp_variable_units_vector;
    }

}

bool Trajectory::setVariableNames(const std::vector<std::string>& variable_names)
{

    if (static_cast<int> (variable_names.size()) == trajectory_dimension_)
    {
        variable_names_.clear();
        variable_names_ = variable_names;
    }
    else if (static_cast<int> (variable_names.size()) * POS_VEL_ACC == trajectory_dimension_)
    {
        variable_names_.clear();
        for (std::vector<std::string>::const_iterator vsi = variable_names.begin(); vsi != variable_names.end(); vsi++)
        {
            variable_names_.push_back(*vsi);
            variable_names_.push_back(*vsi + std::string("_d"));
            variable_names_.push_back(*vsi + std::string("_dd"));
        }
    }
    else
    {
        ROS_ERROR("Number of variables %i and trajectory dimension %i do not fit either way.", static_cast<int> (variable_names.size()), trajectory_dimension_);
        return false;
    }
    variable_units_.clear();
    // For now, we ignore to set correct unit, they are not "really" used at this point.
    for (int i = 0; i < trajectory_dimension_; i++)
    {
        variable_units_.push_back(std::string("-"));
    }

    return true;
}

// REAL-TIME REQUIREMENTS
bool Trajectory::add(const VectorXd& trajectory_point)
{
    if (!initialized_ || (sampling_frequency_ <= 0.0))
    {
        ROS_ERROR("Sampling frequency (%f) is not set yet. Not adding trajectory point.", sampling_frequency_);
        return false;
    }

    if (trajectory_point.size() != trajectory_dimension_)
    {
        ROS_ERROR("Trajectory dimension %i is not the same as the dimension of the trajectory point %i.", trajectory_dimension_, trajectory_point.size());
        return false;
    }

    if (trajectory_length_ >= max_trajectory_length_)
    {
        ROS_ERROR("Maximum trajectory length (%i) reached ! TODO: remove me...", max_trajectory_length_);
        return false;
    }
    else
    {
        // VectorXd comes in as colum vector... so we have to transpose it
        trajectory_data_.block(trajectory_length_, 0, 1, trajectory_dimension_) = trajectory_point.transpose();
        trajectory_length_++;
        // update the duration of the trajectory. It has been checked that sampling_frequency is non zero
        trajectory_duration_ = static_cast<double> (trajectory_length_) / sampling_frequency_;
    }
    return true;
}

bool Trajectory::update(const int length_index, const int dimension_index, const double value)
{
    if (!initialized_ || (sampling_frequency_ < 0.0))
    {
        ROS_ERROR("Sampling frequency (%f) is not set yet. Not updating trajectory point.", sampling_frequency_);
        return false;
    }

    if (!isWithinDimensionBoundaries(dimension_index))
    {
        ROS_ERROR("Trajectory dimension index %i exceeds trajectory dimension %i. Trajectory has size (%i x %i).", dimension_index, trajectory_dimension_, trajectory_data_.rows(), trajectory_data_.cols());
        return false;
    }
    if (!isWithinLengthBoundaries(length_index))
    {
        ROS_ERROR("Trajectory length index %i exceeds trajectory length %i. Trajectory has size (%i x %i).", length_index, trajectory_length_, trajectory_data_.rows(), trajectory_data_.cols());
        return false;
    }

    trajectory_data_(length_index, dimension_index) = value;
    return true;
}

bool Trajectory::getTrajectoryPoint(const int length_index, VectorXd& trajectory_point)
{
    if ((!isWithinLengthBoundaries(length_index)) || (trajectory_point.size() != trajectory_dimension_))
    {
        return false;
    }
    trajectory_point = trajectory_data_.block(length_index, 0, 1, trajectory_dimension_).transpose();
    return true;
}

void Trajectory::print()
{
    std::string type;
    if (trajectory_type_ == eUNINITIALIZED)
    {
        type.assign("uninitialized");
    }
    else if (trajectory_type_ == eR_ARM_JOINT_SPACE)
    {
        type.assign("right arm in joint space");
    }
    else if (trajectory_type_ == eR_ARM_CARTESIAN_SPACE)
    {
        type.assign("right arm in cartesian space");
    }
    else if (trajectory_type_ == eL_ARM_JOINT_SPACE)
    {
        type.assign("left arm in joint space");
    }
    else if (trajectory_type_ == eL_ARM_CARTESIAN_SPACE)
    {
        type.assign("left arm in cartesian space");
    }
    else if (trajectory_type_ == eDEBUG_DATA_TRACE)
    {
        type.assign("debug data");
    }
    else
    {
        type.assign("invalid");
    }

    ROS_INFO("Type:%s \tsize:%i x %i", type.c_str(), trajectory_dimension_, trajectory_length_);
    for (std::vector<std::string>::iterator vsi = variable_names_.begin(); vsi != variable_names_.end(); vsi++)
    {
        ROS_INFO_STREAM(*vsi);
    }
}

void Trajectory::clear()
{
    trajectory_data_.setZero(max_trajectory_length_, max_trajectory_dimension_);
    trajectory_length_ = 0;

    // also update the duration of the trajectory;
    trajectory_duration_ = 0.0;
}

bool Trajectory::readFromCLMCFile(std::string file_name)
{

    FILE *fp;
    // open the file, and parse the parameters
    if ((fp = fopen(file_name.c_str(), "r")) == NULL)
    {
        ROS_ERROR("Cannot fopen file >%s<", file_name.c_str());
        return false;
    }

    int tmp_trajectory_type, tmp_num_rows, tmp_num_cols;
    double tmp_sampling_frequency;
    if (fscanf(fp, "%d %d %d %lf", &tmp_trajectory_type, &tmp_num_cols, &tmp_num_rows, &tmp_sampling_frequency) != 4)
    {
        ROS_ERROR("Could not read/parse header.");
        return false;
    }

    // checking whether values make sense
    if ((tmp_num_cols <= 0) || (tmp_num_rows <= 0))
    {
        ROS_ERROR("Values for number of columns >%i< and rows >%i< are negative", tmp_num_cols, tmp_num_rows);
        return false;
    }
    if ((tmp_num_cols > ABSOLUTE_MAX_TRAJECTORY_DIMENSION) || (tmp_num_rows > ABSOLUTE_MAX_TRAJECTORY_LENGTH))
    {
        ROS_ERROR("Values for number of columns >%i< and rows >%i< are out of bound (%i x %i).", tmp_num_cols, tmp_num_rows, ABSOLUTE_MAX_TRAJECTORY_DIMENSION,ABSOLUTE_MAX_TRAJECTORY_LENGTH);
        return false;
    }
    if (tmp_sampling_frequency <= 0)
    {
        ROS_ERROR("Read implausible sampling frequency >%f<", tmp_sampling_frequency);
        return false;
    }
    // TODO: check tmp_trajectory_type for correct values
    // if( tmp_trajectory_type )
    // {
    // }

    variable_names_.clear();
    variable_units_.clear();

    char tmp_name[MAX_VARNAME_LENGTH];
    char tmp_unit[MAX_VARNAME_LENGTH];
    for (int i = 0; i < tmp_num_cols; i++)
    {
        if (fscanf(fp, "%s %s", tmp_name, tmp_unit) != 2)
        {
            ROS_ERROR("Cannot read variable names and units");
            return false;
        }
        variable_names_.push_back(std::string(tmp_name));
        variable_units_.push_back(std::string(tmp_unit));
        // ROS_INFO("Read %s [%s]",variable_names_.back().c_str(), variable_units_.back().c_str());
    }

    // there are two extra blank chars at the end of the block and a line return which we must account for
    fgetc(fp);
    fgetc(fp);
    fgetc(fp);

    // read file into a buffer
    float trajectory_data[tmp_num_rows][tmp_num_cols];
    if (fread(&(trajectory_data[0][0]), sizeof(float), tmp_num_rows * tmp_num_cols, fp) != (unsigned)(tmp_num_rows * tmp_num_cols))
    {
        ROS_ERROR("Cannot read trajectory data");
        return false;
    }
    fclose(fp);

    // convert little-endian to big-endian
    int aux;
    for (int i = 0; i < tmp_num_rows; i++)
    {
        for (int j = 0; j < tmp_num_cols; j++)
        {
            aux = LONGSWAP(*((int *)&(trajectory_data[i][j])));
            trajectory_data[i][j] = *((float *)&aux);
        }
    }

    max_trajectory_dimension_ = tmp_num_cols;
    max_trajectory_length_ = tmp_num_rows;

    trajectory_dimension_ = tmp_num_cols;
    trajectory_length_ = tmp_num_rows;

    // first set the whole matrix to zero
    trajectory_data_.resize(max_trajectory_length_, max_trajectory_dimension_);
    trajectory_data_.setZero(max_trajectory_length_, max_trajectory_dimension_);

    for (int i = 0; i < max_trajectory_length_; i++)
    {
        for (int j = 0; j < max_trajectory_dimension_; j++)
        {
            trajectory_data_(i, j) = trajectory_data[i][j];
        }
    }

    trajectory_type_ = (eTrajectoryType)tmp_trajectory_type;
    if (tmp_sampling_frequency <= 0)
    {
        ROS_ERROR("Sampling frequency (%f) is invalid", tmp_sampling_frequency);
        return false;
    }
    sampling_frequency_ = tmp_sampling_frequency;
    trajectory_duration_ = static_cast<double> (trajectory_length_) / sampling_frequency_;
    return true;
}

bool Trajectory::writeToCLMCFile(std::string file_name)
{

    // ROS_INFO("Writing to file: %s", file_name.c_str());

    // open the file, and write all data
    FILE *fp;
    if ((fp = fopen(file_name.c_str(), "w")) == NULL)
    {
        ROS_ERROR("Cannot fopen file >%s< -> %s", file_name.c_str(), strerror(errno));
        return false;
    }

    // create a floating point buffer and copy all data over
    float buffer[trajectory_length_][trajectory_dimension_];

    for (int i = 0; i < trajectory_length_; i++)
    {
        for (int j = 0; j < trajectory_dimension_; j++)
        {
            buffer[i][j] = 0.0;
        }
    }
    for (int i = 0; i < trajectory_length_; i++)
    {
        for (int j = 0; j < trajectory_dimension_; j++)
        {
            buffer[i][j] = static_cast<float> (trajectory_data_(i, j));
        }
    }

    // write the the buffer size, the number of columns, the sampling time, the column names and units
    if (sampling_frequency_ == 0.0)
    {
        ROS_WARN("Sampling frequency is %f, it will not be possible to view the trace in mrdplot", sampling_frequency_);
    }
    fprintf(fp, "%d  %d  %d  %f\n", trajectory_type_, trajectory_dimension_, trajectory_length_, sampling_frequency_);

    for (int i = 0; i < trajectory_dimension_; i++)
    {

        if (variable_names_[i].empty())
        {
            ROS_WARN("Variable name is empty");
            char tmp[MAX_VARNAME_LENGTH];
            sprintf(tmp, "%s_%d", DEFAULT_VARIABLE_NAME, i);
            variable_names_[i].assign(tmp);
        }
        fprintf(fp, "%s  ", variable_names_[i].c_str());

        if (variable_units_[i].empty())
        {
            ROS_WARN("Variable unit is empty");
            char tmp[MAX_VARNAME_LENGTH];
            sprintf(tmp, "%s_%d", DEFAULT_VARIABLE_UNIT, i);
            variable_units_[i].assign(tmp);
        }
        fprintf(fp, "%s  ", variable_units_[i].c_str());

    }
    fprintf(fp, "\n");

    // convert little-endian to big-endian
    int aux;
    // ROS_WARN("Byteswap active...");
    for (int i = 0; i < trajectory_length_; ++i)
    {
        for (int j = 0; j < trajectory_dimension_; ++j)
        {
            aux = LONGSWAP(*((int *) &(buffer[i][j])));
            buffer[i][j] = *((float *)&aux);
        }
    }

    // TODO: valgrind says "Syscall param write(buf) points to uninitialised byte(s)" change this !!!
    if (fwrite(&(buffer[0][0]), sizeof(float), trajectory_length_ * trajectory_dimension_, fp) != (unsigned)trajectory_length_ * trajectory_dimension_)
    {
        ROS_ERROR("Cannot fwrite trajectory data");
        return false;
    }
    fclose(fp);

    return true;
}

bool Trajectory::resample(std::vector<ros::Time> time_stamps, double additional_boundary_duration, const std::vector<int>& quaternion_indices)
{

    if ((trajectory_dimension_ % POS_VEL_ACC) != 0)
    {
        ROS_ERROR("For now, only trajectories that are a multiple of %i can be resampled", POS_VEL_ACC);
        return false;
    }
    int trajectory_trace = trajectory_dimension_ / POS_VEL_ACC;

    if (time_stamps.size() != static_cast<unsigned int> (trajectory_length_))
    {
        ROS_ERROR("Trajectory contains %i data points but %i time stamps are provided.", trajectory_length_, static_cast<int> (time_stamps.size()));
        return false;
    }

    if (trajectory_length_ < MIN_NUM_DATA_POINTS)
    {
        ROS_ERROR("Trajectory length is %i and therefore to little (less then %i) for resampling.",trajectory_length_, MIN_NUM_DATA_POINTS);
        return false;
    }

    if ((static_cast<int> (quaternion_indices.size()) != 0) && (static_cast<int> (quaternion_indices.size()) != dmp::N_QUAT))
    {
        ROS_ERROR("Quaternion index invalid. There are %i index.", static_cast<int>(quaternion_indices.size()));
        return false;
    }

    // compute mean dt of the provided time stamps
    double mean_dt = 0.0;
    int num_dts = time_stamps.size() - 1;
    double dts[num_dts];
    for (int i = 0; i < num_dts; i++)
    {
        dts[i] = time_stamps[i + 1].toSec() - time_stamps[i].toSec();
        mean_dt += dts[i];
    }
    mean_dt /= static_cast<double> (time_stamps.size() - 1);

    // TODO:remove after debugging...
    ROS_ERROR_COND(mean_dt <= 0.0, "Mean dt is %f and therefore can't be right.", mean_dt);

    // since we will leave out the half of the additional_boundary_duration to avoid wired effects close to the boundaries after computing derivatives,
    // we add two times the boundary duration on both ends.
    int additional_points = 2.0 * additional_boundary_duration / mean_dt;

    // ROS_INFO("Mean dt is %f sec --> %i trajectory points are added.", mean_dt, additional_points);

    int tmp_trajectory_length = additional_points + trajectory_length_ + additional_points;

    // given the new duration and the desired number of trajectory points (new_trajectory_length) we compute the appropriate sampling frequency.
    double tmp_sampling_frequency = static_cast<double> (tmp_trajectory_length) / (trajectory_duration_ + 2.0 * additional_boundary_duration);

    double x_vector[tmp_trajectory_length];
    for (int i = 0; i < tmp_trajectory_length; i++)
    {
        if (i <= additional_points)
        {
            x_vector[i] = static_cast<double> (i) * mean_dt;
        }
        else if (i < additional_points + trajectory_length_)
        {
            x_vector[i] = x_vector[i - 1] + dts[i - additional_points - 1];
        }
        else
        {
            x_vector[i] = x_vector[i - 1] + mean_dt;
        }
    }

    double xs = static_cast<double> (1.0) / tmp_sampling_frequency;

    double y_vector[tmp_trajectory_length];
    double rescaled_trajectory_data[tmp_trajectory_length][trajectory_trace];
    for (int i = 0; i < trajectory_trace; i++)
    {
        for (int j = 0; j < tmp_trajectory_length; j++)
        {
            if (j < additional_points)
            {
                y_vector[j] = getTrajectoryPosition(0, i);
            }
            else if (j < additional_points + trajectory_length_)
            {
                y_vector[j] = getTrajectoryPosition(j - additional_points, i);
            }
            else
            {
                y_vector[j] = y_vector[j - 1];
            }
        }

        // TODO: think about reading this from file... or compute a reasonable value
        double wl = 0.2;

        // Create our bspline base on the X vector with a simple wavelength.
        BSpline<double>::Debug(0);
        int bc = BSplineBase<double>::BC_ZERO_SECOND;
        BSpline<double> b_spline(&(x_vector[0]), tmp_trajectory_length, &(y_vector[0]), wl, bc);

        if (b_spline.ok())
        {
            double x_val;
            // double xs = (b_spline.Xmax() - b_spline.Xmin()) / (static_cast<double> (new_trajectory_length));

            x_val = b_spline.Xmin();
            for (int s = 0; s < tmp_trajectory_length; s++)
            {
                rescaled_trajectory_data[s][i] = b_spline.evaluate(x_val);
                x_val += xs;
            }
        }
        else
        {
            ROS_ERROR("Could not rescale trajectory, splining failed.");
            return false;
        }
    }

    // first set the whole matrix to zero
    for (int i = 0; i < max_trajectory_length_; i++)
    {
        for (int j = 0; j < max_trajectory_dimension_; j++)
        {
            trajectory_data_(i, j) = 0.0;
        }
    }
    // fill in the new rescaled trajectory data
    for (int i = 0; i < tmp_trajectory_length; i++)
    {
        for (int j = 0; j < trajectory_trace; j++)
        {
            trajectory_data_(i, j * POS_VEL_ACC) = rescaled_trajectory_data[i][j];
        }
    }
    // set the new temporary trajectory length and temporary sampling frequency
    trajectory_length_ = tmp_trajectory_length;
    if (!setSamplingFrequency(tmp_sampling_frequency))
    {
        ROS_ERROR("Could not set sampling frequency.");
        return false;
    }

    if (static_cast<int> (quaternion_indices.size()) == dmp::N_QUAT)
    {
        // normalize quaternion
        double mag;
        double qx, qy, qz, qw;
        for (int i = 0; i < tmp_trajectory_length; i++)
        {
            qx = trajectory_data_(i, quaternion_indices[dmp::_QX_] * POS_VEL_ACC);
            qy = trajectory_data_(i, quaternion_indices[dmp::_QY_] * POS_VEL_ACC);
            qz = trajectory_data_(i, quaternion_indices[dmp::_QZ_] * POS_VEL_ACC);
            qw = trajectory_data_(i, quaternion_indices[dmp::_QW_] * POS_VEL_ACC);
            mag = sqrt(pow(qx, 2) + pow(qy, 2) + pow(qz, 2) + pow(qw, 2));
            trajectory_data_(i, quaternion_indices[dmp::_QX_] * POS_VEL_ACC) = qx / mag;
            trajectory_data_(i, quaternion_indices[dmp::_QY_] * POS_VEL_ACC) = qy / mag;
            trajectory_data_(i, quaternion_indices[dmp::_QZ_] * POS_VEL_ACC) = qz / mag;
            trajectory_data_(i, quaternion_indices[dmp::_QW_] * POS_VEL_ACC) = qw / mag;
        }
    }

    //TODO: if trajectory contains a quaternion it needs to be normalized after resampling...
    computeDerivatives();

    MatrixXd cutout_trajectory_data = trajectory_data_;
    trajectory_data_.setZero(max_trajectory_length_, max_trajectory_dimension_);

    // fill in the part that only contains the rescaled trajectory data
    // for (int i = additional_points; i < additional_points + new_trajectory_length; i++)
    for (int i = additional_points; i < additional_points + tmp_trajectory_length; i++)
    {
        for (int j = 0; j < trajectory_trace * POS_VEL_ACC; j++)
        {
            trajectory_data_(i - additional_points, j) = cutout_trajectory_data(i, j);
        }
    }
    // trajectory_length_ = new_trajectory_length;
    trajectory_length_ = tmp_trajectory_length - 2 * additional_points;
    // double new_sampling_frequency = static_cast<double> (new_trajectory_length) / trajectory_duration_;
    double new_sampling_frequency = static_cast<double> (trajectory_length_) / trajectory_duration_;

    if (!setSamplingFrequency(new_sampling_frequency))
    {
        ROS_ERROR("Could not set sampling frequency.");
        return false;
    }
    return true;
}

void Trajectory::computeDerivatives()
{

    if ((trajectory_dimension_ % POS_VEL_ACC) != 0)
    {
        ROS_ERROR("Cannot compute the derivatives for trajectories which dimension is not a multiple of %i.", POS_VEL_ACC);
        return;
    }
    int trajectory_trace = trajectory_dimension_ / POS_VEL_ACC;

    int add_pos_points = 4;
    int pos_length = add_pos_points + trajectory_length_ + add_pos_points;
    int vel_length = -2 + pos_length - 2;
    int acc_length = -2 + vel_length - 2;

    MatrixXd tmp_trajectory_pos = MatrixXd::Zero(pos_length, trajectory_trace);
    for (int i = 0; i < pos_length; i++)
    {
        for (int j = 0; j < trajectory_trace; j++)
        {
            if (i < add_pos_points)
            {
                tmp_trajectory_pos(i, j) = trajectory_data_(0, j * POS_VEL_ACC);
            }
            else if (i < add_pos_points + trajectory_length_)
            {
                tmp_trajectory_pos(i, j) = trajectory_data_(i - add_pos_points, j * POS_VEL_ACC);
            }
            else
            {
                tmp_trajectory_pos(i, j) = tmp_trajectory_pos(i - 1, j);
            }
        }
    }

    MatrixXd tmp_trajectory_vel = MatrixXd::Zero(vel_length, trajectory_trace);
    for (int i = 0; i < vel_length; i++)
    {
        for (int j = 0; j < trajectory_trace; j++)
        {
            tmp_trajectory_vel(i, j) = (tmp_trajectory_pos(i, j) - (static_cast<double> (8.0) * tmp_trajectory_pos(i + 1, j)) + (static_cast<double> (8.0)
                    * tmp_trajectory_pos(i + 3, j)) - tmp_trajectory_pos(i + 4, j)) / static_cast<double> (12.0);
            tmp_trajectory_vel(i, j) *= sampling_frequency_;
        }
    }

    MatrixXd tmp_trajectory_acc = MatrixXd::Zero(acc_length, trajectory_trace);

    for (int i = 0; i < acc_length; i++)
    {
        for (int j = 0; j < trajectory_trace; j++)
        {
            tmp_trajectory_acc(i, j) = (tmp_trajectory_vel(i, j) - (static_cast<double> (8.0) * tmp_trajectory_vel(i + 1, j)) + (static_cast<double> (8.0)
                    * tmp_trajectory_vel(i + 3, j)) - tmp_trajectory_vel(i + 4, j)) / static_cast<double> (12.0);
            tmp_trajectory_acc(i, j) *= sampling_frequency_;
        }
    }

    clear();

    VectorXd trajectory_point = VectorXd::Zero(trajectory_dimension_);
    for (int i = 0; i < acc_length; i++)
    {
        for (int j = 0; j < trajectory_trace; j++)
        {
            trajectory_point(j * POS_VEL_ACC + 0) = tmp_trajectory_pos(i + 4, j);
            trajectory_point(j * POS_VEL_ACC + 1) = tmp_trajectory_vel(i + 2, j);
            trajectory_point(j * POS_VEL_ACC + 2) = tmp_trajectory_acc(i + 0, j);
        }
        add(trajectory_point);
    }

}

std::vector<std::string> Trajectory::getPositionVariableNames()
{
    std::vector<std::string> position_names;
    for (unsigned int i = 0; i < variable_names_.size() / POS_VEL_ACC; i++)
    {
        std::string variable_name;
        variable_name.assign(variable_names_[i * POS_VEL_ACC]);
        position_names.push_back(variable_name);
    }
    return position_names;
}

bool Trajectory::computeMSE(const Trajectory& other_trajectory, VectorXd& mse_vector) const
{

    if (trajectory_dimension_ != other_trajectory.getDimension())
    {
        ROS_ERROR("Trajectories do not have same dimension (%i vs. %i).", trajectory_dimension_, other_trajectory.getDimension());
        return false;
    }

    int num_trajectory_points_used_for_computing_mse = trajectory_length_;
    int other_trajectory_length = other_trajectory.getLength();
    if (trajectory_length_ != other_trajectory_length)
    {
        ROS_WARN("Trajectory length are different: %i vs %i.", trajectory_length_, other_trajectory_length);
        if (fabs(trajectory_length_ - other_trajectory_length) > 10)
        {
            ROS_ERROR("Trajectory lengths differ more than 10 points (%i vs %i).", trajectory_length_, other_trajectory_length);
            return false;
        }

        if (trajectory_length_ > other_trajectory_length)
        {
            num_trajectory_points_used_for_computing_mse = other_trajectory_length;
        }
        else
        {
            num_trajectory_points_used_for_computing_mse = trajectory_length_;
        }
    }

    if ((trajectory_dimension_ % POS_VEL_ACC) != 0)
    {
        ROS_ERROR("Compute the MSE for trajectories which do not have a multiple of 3 dimension is not implemented yet. The trajectory contains %i dimensions.", trajectory_dimension_);
        return false;
    }

    int num_position_traces = trajectory_dimension_ / POS_VEL_ACC;

    if (mse_vector.size() != num_position_traces)
    {
        ROS_ERROR("MSE vector has wrong size (%i), it should have size %i.", mse_vector.size(), num_position_traces);
        return false;
    }

    for (int i = 0; i < num_trajectory_points_used_for_computing_mse; i++)
    {
        for (int j = 0; j < num_position_traces; j++)
        {
            mse_vector(j) += pow(trajectory_data_(i, j * POS_VEL_ACC) - other_trajectory.getTrajectoryPosition(i, j), 2);
        }
    }

    for (int j = 0; j < num_position_traces; j++)
    {
        mse_vector(j) /= num_trajectory_points_used_for_computing_mse;
    }
    return true;
}

int Trajectory::getMaxDimension() const
{
    assert(initialized_);
    return max_trajectory_dimension_;
}
int Trajectory::getMaxLength() const
{
    assert(initialized_);
    return max_trajectory_length_;
}

}
