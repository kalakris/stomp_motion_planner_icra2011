/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef DMP_IK_CONTROLLER_H_
#define DMP_IK_CONTROLLER_H_

// system includes
#include <boost/shared_ptr.hpp>

// ros includes
#include <rosrt/rosrt.h>
#include <pr2_controller_interface/controller.h>
// #include <trajectory_msgs/JointTrajectoryPoint.h>

#include <visualization_msgs/Marker.h>

#include <dmp_motion_generation/dmp_debug.h>

#include <Eigen/Eigen>

// local includes
#include <dmp_motion_controller/dmp_controller.h>
#include <dmp_motion_controller/dmp_gripper_controller.h>
#include <dmp_motion_controller/cartesian_twist_controller_ik_with_nullspace_optimization.h>
#include <dmp_motion_controller/Waypoint.h>

#include <dmp_motion_controller/AddToExecuteDMPQueue.h>

/*!
 */

namespace dmp_controller
{

/*! \class controller that uses cartesian space set points and sends them to the pose twist controller.
 */
class DMPIkController : public pr2_controller_interface::Controller
{

    typedef CartesianTwistControllerIkWithNullspaceOptimization ChildController;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! constructor
     */
    DMPIkController();

    /*! destructor
     */
    virtual ~DMPIkController();

    /*!
     * @param robot_state
     * @param node_handle
     * @return
     */
    bool init(pr2_mechanism_model::RobotState* robot_state, ros::NodeHandle& node_handle);

    /*!
     */
    void starting();

    /*!
     */
    void update();

    /*!
     */
    void stopping();

    /*!
     * @return
     */
    bool initXml(pr2_mechanism_model::RobotState* robot, TiXmlElement* config);

    /*!
     * @return
     */
    bool isQueueEmpty();

    /*!
     * @param dmp_structs
     * @return
     */
    bool addToQueue(std::vector<DMPStruct> dmp_structs);

    /*!
     * @return
     */
    std::string getVariableNamesKeyWord() const;
    std::vector<int> getNumDofs() const;

private:

    /*!
     * @param trajectory_point
     * @param movement_finished
     * @param execution_duration
     * @param num_samples
     * @return
     */
    bool transformCommand(Eigen::VectorXd& trajectory_point, bool& movement_finished, const double execution_duration, const int num_samples);

    /*!
     * @return
     */
    bool readParameters();

    /*!
     * @return
     */
    bool initRTPublisher();

    /*!
     * @param handle_namespace
     * @param controller_handle_namespace
     * @return
     */
    bool getArmRelatedVariables(const std::string& handle_namespace, std::string& controller_handle_namespace);

    /*!
     */
    bool initialized_;

    /*!
     */
    DMPController dmp_controller_;

    /*! robot structure
     */
    pr2_mechanism_model::RobotState* robot_state_;

    /*!
     */
    std::string root_name_;

    /*!
     */
    ros::NodeHandle node_handle_;

    /*!
     * @param next_waypoint
     */
    void setCommand(const dmp_motion_controller::Waypoint& next_waypoint);

    /*!
     * @param next_waypoint
     * @return
     */
    bool getCommand(dmp_motion_controller::Waypoint& next_waypoint);

    /*!
     * @return
     */
    // bool computeAndSetJointGoal();

    /*!
     * @param pose_measured_gripper_center
     * REAL-TIME REQUIREMENTS
     */
    // void setStartConfigurationFromCurrent(KDL::Frame& pose_measured_gripper_center);

    /*!
     * @param pose_measured_gripper_center
     * @param twist_measured_gripper_center
     * REAL-TIME REQUIREMENTS
     */
    void setTrajectoryPointFromCurrent(KDL::Frame& pose_measured_gripper_center, KDL::Twist twist_measured_gripper_center);

    /*! IK controller
     */
    boost::shared_ptr<ChildController> cart_controller_;

    /*!
     */
    dmp_motion_controller::Waypoint next_waypoint_;

    /*!
     */
    void visualize();

    /*!
     */
    int publishing_rate_;
    int publishing_counter_;
    int publisher_buffer_size_;

    /*!
     */
    Eigen::Vector3d actual_endeffector_linear_twist_;
    Eigen::Vector3d desired_endeffector_linear_twist_;

    /*!
     */
    boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > viz_marker_actual_arrow_publisher_;
    boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > viz_marker_desired_arrow_publisher_;
    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > pose_actual_publisher_;
    boost::shared_ptr<rosrt::Publisher<geometry_msgs::PoseStamped> > pose_desired_publisher_;

    int visualization_line_counter_;
    int visualization_line_rate_;
    int visualization_line_max_points_;
    int visualization_line_points_index_;
    boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > viz_marker_actual_line_publisher_;
    boost::shared_ptr<rosrt::Publisher<visualization_msgs::Marker> > viz_marker_desired_line_publisher_;

    /*!
     */
    DMPGripperController gripper_controller_;

    /*!
     */
    bool keep_restposture_fixed_for_testing_;

    /*!
     */
    bool last_frame_set_;
    KDL::Frame last_frame_;
    KDL::Frame current_frame_;
    KDL::Twist current_twist_;

    /*!
     */
    Eigen::Vector4d quat_;
    Eigen::VectorXd controller_trajectory_point_;
    /*!
     */
    std::vector<Eigen::VectorXd> dmp_trajectory_points_;
    std::vector<Eigen::VectorXd> dmp_pos_vel_acc_trajectory_points_;


    /*! quick hack... TODO: remove this later...
     */
    Eigen::VectorXd joint_pos_vel_acc_point_;

    /*!
     */
    pr2_tasks_transforms::TaskTransforms task_frame_transformer_;

};

inline std::string DMPIkController::getVariableNamesKeyWord() const
{
    return dmp_controller_.getVariableNamesKeyWord();
}
inline std::vector<int> DMPIkController::getNumDofs() const
{
    return dmp_controller_.getNumDofs();
}

inline bool DMPIkController::isQueueEmpty()
{
    return dmp_controller_.isQueueEmpty();
}
inline bool DMPIkController::addToQueue(std::vector<DMPStruct> dmp_structs)
{
    return dmp_controller_.addToQueue(dmp_structs);
}

}

#endif /* DMP_IK_CONTROLLER_H_ */
