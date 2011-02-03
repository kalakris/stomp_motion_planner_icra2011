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

#ifndef DMP_DEBUG_H_
#define DMP_DEBUG_H_

// system includes

// ros includes
#include <Eigen/Eigen>

// local includes
#include <dmp_motion_generation/trajectory.h>

// forward declaration
class Trajectory;

namespace dmp
{

class DMPDebug
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! constructor
     *
     * @return
     */
    DMPDebug(ros::NodeHandle& node_handle);

    /*! destructor
     *
     * @return
     */
    ~DMPDebug();


    bool initialize(const int dmp_id, const std::string& variable_names_key_word, const int num_transformation_systems,
                    const int num_debug_traces_per_trans_sys, const int num_extra_debug_traces, const double sampling_frequency);
//    bool initialize(const int dmp_id, std::vector<std::string>& variable_names, const int num_transformation_systems, const int num_debug_traces_per_trans_sys,
//                    const int num_extra_debug_traces, const double sampling_frequency, ros::NodeHandle& node_handle);

    /*!
     * @param dmp_id
     */
    void setDMPId(const int dmp_id);

    /*!
     * @param file_name_trunk
     */
    void setTrunkFileName(std::string file_name_trunk);

    /*!
     * @return
     */
    bool isInitialized();

    /*!
     */
    void clear();

    /*!
     * @return
     */
    bool writeToCLMCFile(Parameters::Version version = Parameters::ICRA2009);
    bool writeToCLMCFile(const std::string& file_name);

    /*! REAL-TIME REQUIREMENTS
     * @param debug_trajectory_point
     */
    bool add(const Eigen::VectorXd& debug_trajectory_point);

    /*!
     *
     * @param length_index
     * @param dimension_index
     * @param value
     * @return
     */
    bool update(const int length_index, const int dimension_index, const double value);

    /*!
     *
     * @param other_debug
     * @param mse_vector
     * @return
     */
    bool computeMSE(const DMPDebug& other_debug, Eigen::VectorXd& mse_vector);

private:

    /*!
     * @return
     */
    bool setDebugDirectory();

    /*!
     *
     * @param other_trajectory
     * @param mse_vector
     * @return
     */
    bool computeMSE(const Trajectory &other_trajectory, Eigen::VectorXd& mse_vector) const;

    /*!
     */
    bool initialized_;

    /*!
     */
    std::string debug_directory_name_;

    /*!
     */
    std::string file_name_trunk_;

    /*!
     */
    int iteration_counter_;

    /*!
     */
    int dmp_id_;

    /*!
     */
    ros::NodeHandle trajectory_node_handle_;
    ros::NodeHandle dmp_node_handle_;

    /*!
     */
    Trajectory debug_trajectory_;

};

// inline functions follow

inline bool DMPDebug::isInitialized()
{
    return initialized_;
}
inline void DMPDebug::setDMPId(const int dmp_id)
{
    dmp_id_ = dmp_id;
}
inline void DMPDebug::setTrunkFileName(std::string file_name_trunk)
{
    // ROS_INFO("DMPDebug::setTrunkFileName>> assigning %s.",file_name_trunk.c_str());
    file_name_trunk_.assign(file_name_trunk);
}
inline bool DMPDebug::computeMSE(const DMPDebug& other_debug, Eigen::VectorXd& mse_vector)
{
    if (initialized_)
    {
        return other_debug.computeMSE(debug_trajectory_, mse_vector);
    }
    return false;
}

inline bool DMPDebug::computeMSE(const Trajectory& trajectory, Eigen::VectorXd& mse_vector) const
{
    if (initialized_)
    {
        return debug_trajectory_.computeMSE(trajectory, mse_vector);
    }
    return false;
}

}

#endif /* DMP_DEBUG_H_ */
