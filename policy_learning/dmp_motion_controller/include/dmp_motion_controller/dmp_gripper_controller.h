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


#ifndef DMP_GRIPPER_CONTROL_H_
#define DMP_GRIPPER_CONTROL_H_

// system includes

// ros includes
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <rosrt/rosrt.h>
#include <dmp_motion_generation/parameters.h>

// local includes

namespace dmp_controller {

class DMPGripperController {

public:

	/*!
	 * @return
	 */
	DMPGripperController();
	/*!
	 * @return
	 */
	~DMPGripperController();

	/*!
	 * @return
	 */
	bool initialize(ros::NodeHandle& node_handle);

	/*!
	 * @param progress
	 */
	void command(double progress);

	/*!
	 */
	void reset();

private:

	/*!
	 */
	bool initialized_;

	/*!
	 */
	int task_type_;

	/*! gripper effort publisher
	 */
    boost::shared_ptr<rosrt::Publisher<std_msgs::Float64> > r_gripper_effort_publisher_;
    boost::shared_ptr<rosrt::Publisher<std_msgs::Float64> > l_gripper_effort_publisher_;

	/*!
	 */
	int command_published_;

};

}

#endif /* DMP_GRIPPER_CONTROL_H_ */

