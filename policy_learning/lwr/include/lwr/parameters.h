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

 \file    lwr_parameters.h

 \author  Peter Pastor
 \date    May 28, 2010

 **********************************************************************/

#ifndef LWR_PARAMETERS_H_
#define LWR_PARAMETERS_H_

// system includes
#include <string>
#include <vector>

// ros includes
#include <ros/ros.h>

namespace lwr
{

class Parameters
{

public:

    /*!
     * @return
     */
	Parameters(ros::NodeHandle& node_handle);

	/*!
	 * @return
	 */
	virtual ~Parameters();

	/*!
	 * @param parameter_namespace
	 * @return
	 */
	bool initialize();
	bool isInitialized() const;

	/*!
	 * @return
	 */
	int getNumRFS() const;
    double getRFSWidthBoundary() const;
    double getCanSysCutOff() const;

private:

	/*!
	 *
	 */
	bool initFromNodeHandle();

	/*!
	 *
	 */
	bool initialized_;

	/*!
	 * Number of receptive fields
	 */
	int num_rfs_;

	/*!
	 *  This value specifies at which value to neighboring RFs will intersect
	 */
	double rfs_width_boundary_;

	/*!
	 */
	double can_sys_cutoff_;

	/*!
	 */
    ros::NodeHandle node_handle_;

};

// inline functions follow
inline bool Parameters::isInitialized() const
{
	return initialized_;
}
inline int Parameters::getNumRFS() const
{
	return num_rfs_;
}
inline double Parameters::getCanSysCutOff() const
{
    return can_sys_cutoff_;
}
inline double Parameters::getRFSWidthBoundary() const
{
    return rfs_width_boundary_;
}

}

#endif /* LWR_PARAMETERS_H_ */
