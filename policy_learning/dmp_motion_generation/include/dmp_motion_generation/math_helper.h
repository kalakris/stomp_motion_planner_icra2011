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

  \file    dmp_math_helper.h

  \author  Peter Pastor
  \date    Jun 6, 2010

**********************************************************************/

#ifndef MATH_HELPER_H_
#define MATH_HELPER_H_

#include <string>
#include <sstream>

#include <Eigen/Eigen>

#include <dmp_motion_generation/trajectory.h>

namespace dmp {

class MathHelper
{

public:

    static bool generateMinJerkTrajectory(const Eigen::VectorXd &start, const Eigen::VectorXd &goal, const double duration, const double delta_t, Trajectory &trajectory);

    static void getQuaternionFromRPY(const double roll, const double pitch, const double yaw, double &qx, double &qy, double &qz, double &qw);

    static void computeAngularVelocityError(const Eigen::VectorXd& quat1, const Eigen::VectorXd& quat2, Eigen::VectorXd& angular_velocity_error);

    static std::string getString(const int number);

    static void getRotationMatrix(Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& rotation_axis, const double angle);

private:

    static bool calculate_min_jerk_next_step(const Eigen::VectorXd &start, const Eigen::VectorXd &goal, const double duration, const double delta_t, Eigen::VectorXd &next);

};

// inline functions
inline std::string MathHelper::getString(const int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

inline void MathHelper::getRotationMatrix(Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& rotation_axis, const double angle)
{

    double cos_of_angle = cos(angle);
    double sin_of_angle = sin(angle);
    double inverted_cos_of_angle = static_cast<double> (1.0) - cos_of_angle;

    rotation_matrix(0, 0) = rotation_axis(_XX_) * rotation_axis(_XX_) * inverted_cos_of_angle + cos_of_angle;
    rotation_matrix(0, 1) = rotation_axis(_XX_) * rotation_axis(_YY_) * inverted_cos_of_angle - rotation_axis(_ZZ_) * sin_of_angle;
    rotation_matrix(0, 2) = rotation_axis(_XX_) * rotation_axis(_ZZ_) * inverted_cos_of_angle + rotation_axis(_YY_) * sin_of_angle;

    rotation_matrix(1, 0) = rotation_axis(_YY_) * rotation_axis(_XX_) * inverted_cos_of_angle + rotation_axis(_ZZ_) * sin_of_angle;
    rotation_matrix(1, 1) = rotation_axis(_YY_) * rotation_axis(_YY_) * inverted_cos_of_angle + cos_of_angle;
    rotation_matrix(1, 2) = rotation_axis(_YY_) * rotation_axis(_ZZ_) * inverted_cos_of_angle - rotation_axis(_XX_) * sin_of_angle;

    rotation_matrix(2, 0) = rotation_axis(_ZZ_) * rotation_axis(_XX_) * inverted_cos_of_angle - rotation_axis(_YY_) * sin_of_angle;
    rotation_matrix(2, 1) = rotation_axis(_ZZ_) * rotation_axis(_YY_) * inverted_cos_of_angle + rotation_axis(_XX_) * sin_of_angle;
    rotation_matrix(2, 2) = rotation_axis(_ZZ_) * rotation_axis(_ZZ_) * inverted_cos_of_angle + cos_of_angle;

}

}

#endif /* MATH_HELPER_H_ */
