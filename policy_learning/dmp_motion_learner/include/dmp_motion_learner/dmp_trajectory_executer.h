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


#ifndef DMP_TRAJECTORY_EXECUTER_H_
#define DMP_TRAJECTORY_EXECUTER_H_

// system includes

// ros includes
#include <trajectory/trajectory.h>
#include <dmp_motion_generation/dmp_trajectory.h>

// local includes
#include <dmp_motion_learner/ExecuteMRDTrajectory.h>

namespace dmp {

class DMPTrajectoryExecuter {

public:

	DMPTrajectoryExecuter();
	~DMPTrajectoryExecuter();

	bool initialize();

	int run();

	bool executeCLMCTrajectory(dmp_motion_learner::ExecuteMRDTrajectory::Request &request, dmp_motion_learner::ExecuteMRDTrajectory::Response &response);

private:

	bool initialized_;

	ros::NodeHandle node_;
	ros::ServiceServer execute_mrd_trajectory_service_server_;

};



}

#endif /* DMP_MRD_TRAJECTORY_EXECUTER_H_ */
