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

/** \author Nate Koenig and Peter Pastor */

// system includes
#include <string>

// ros includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

// local includes

void spinThread(void)
{
	ros::spin();
}

class CloudFilter
{

public:

	/*!
	 * @param node_handle
	 * @return
	 */
	CloudFilter(ros::NodeHandle &node_handle);

	/*!
	 * @return
	 */
	bool initialize();

	/*!
	 * @param cloud
	 */
	void CloudCB(const sensor_msgs::PointCloudConstPtr& cloud);

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber cloud_subscriber_;
	ros::Publisher cloud_publisher_;
	tf::TransformListener tf_;
	std::string root_name_;

	double min_x_;
	double max_x_;
	double min_y_;
	double max_y_;
	double min_z_;
	double max_z_;

};

CloudFilter::CloudFilter(ros::NodeHandle &node_handle) :
	node_handle_(node_handle), tf_(node_handle_)
{
}

bool CloudFilter::initialize()
{

	// cloud_subscriber_ = node_handle_.subscribe(std::string("/wide_stereo_offset/points"), 1, &CloudFilter::CloudCB, this);
	cloud_subscriber_ = node_handle_.subscribe(std::string("/stereo/output/points"), 1, &CloudFilter::CloudCB, this);
	cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud> (std::string("/dmp_motion_controller/filtered_cloud"), 1);

	ros::NodeHandle controller_handle("/r_arm_cartesian_twist_controller_ik_with_nullspace_optimization");
	if(!controller_handle.getParam(std::string("root_name"), root_name_))
	{
		ROS_ERROR_STREAM("CloudFilter::initialize>> root name is empty.");
		return false;
	}

	ros::NodeHandle cloud_filter_handle("/cloud_filter_parameters");
	if(!cloud_filter_handle.getParam(std::string("min_x"), min_x_))
	{
		ROS_ERROR_STREAM("CloudFilter::initialize>> could not find parameter \"min_x\".");
		return false;
	}
	if(!cloud_filter_handle.getParam(std::string("max_x"), max_x_))
	{
		ROS_ERROR_STREAM("CloudFilter::initialize>> could not find parameter \"max_x\".");
		return false;
	}
	if(!cloud_filter_handle.getParam(std::string("min_y"), min_y_))
	{
		ROS_ERROR_STREAM("CloudFilter::initialize>> could not find parameter \"min_y\".");
		return false;
	}
	if(!cloud_filter_handle.getParam(std::string("max_y"), max_y_))
	{
		ROS_ERROR_STREAM("CloudFilter::initialize>> could not find parameter \"max_y\".");
		return false;
	}
	if(!cloud_filter_handle.getParam(std::string("min_z"), min_z_))
	{
		ROS_ERROR_STREAM("CloudFilter::initialize>> could not find parameter \"min_z\".");
		return false;
	}
	if(!cloud_filter_handle.getParam(std::string("max_z"), max_z_))
	{
		ROS_ERROR_STREAM("CloudFilter::initialize>> could not find parameter \"max_z\".");
		return false;
	}
	
	return true;
}

void CloudFilter::CloudCB(const sensor_msgs::PointCloudConstPtr& cloud)
{
	
	sensor_msgs::PointCloud tmp;
	sensor_msgs::PointCloud result;
	
	tmp = *cloud;

	result.channels.resize(tmp.channels.size());
	for (unsigned i = 0; i < tmp.channels.size(); i++)
	{
		result.channels[i].name = tmp.channels[i].name;
	}

	// Lets check if the requested transform exists in the tf tree,
	// if not we'll wait .5 seconds for it to show up
	// ros::Time current_time = ros::Time()::now();
	ros::Duration timeout(0.5);
	std::string error_msg;

	if (!tf_.waitForTransform(root_name_, tmp.header.frame_id, tmp.header.stamp, timeout, ros::Duration(0.01), &error_msg))
	{
	  // ROS_ERROR("CloudFilter::CloudCB>> could not find required transform from %s to %s. Reason: %s", root_name_.c_str(), tmp.header.frame_id.c_str(), error_msg.c_str());
	  return;
	}

	tf_.transformPointCloud(root_name_, tmp, tmp);

	result.header.frame_id = tmp.header.frame_id;
	result.header.stamp = tmp.header.stamp;
	result.header.seq = tmp.header.seq;
	
	// Remove all the points that are too close or too far from the robot
	for (unsigned int i = 0; i < tmp.points.size(); i++)
		{
			
			if ( (tmp.points[i].x > min_x_) && (tmp.points[i].x < max_x_)
				&& (tmp.points[i].y > min_y_) && (tmp.points[i].y < max_y_)
				&& (tmp.points[i].z > min_z_) && (tmp.points[i].z < max_z_) )
				{
					result.points.push_back(tmp.points[i]);
					for (unsigned int j = 0; j < tmp.channels.size(); j++)
						{
							result.channels[j].values.push_back(tmp.channels[j].values[i]);
						}
				}
		}
	
	cloud_publisher_.publish(result);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cloud_filter_node");
	ros::NodeHandle node_handle;

	// this is required for actions to work
	boost::thread th(&spinThread);

	CloudFilter cloud_filter(node_handle);

	if(cloud_filter.initialize())
	{
		ros::spin();
	}

	return 0;
}

