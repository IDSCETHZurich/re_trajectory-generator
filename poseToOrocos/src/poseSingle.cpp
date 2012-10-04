/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher odoPub = n.advertise<nav_msgs::Odometry>("odometryDsr", 2, true);

	geometry_msgs::Pose pose;
	nav_msgs::Odometry odo;

	// call with parameter: ``rosrun poseToOrocos poseSingle _pos:=1``
	ros::NodeHandle nh("~");
	int pos_choice = 0;
	nh.getParam("pos", pos_choice);

	switch (pos_choice) {
	  case 0: // arbitrary "high-up" position
	    // position in meters
	    pose.position.x = .0;
	    pose.position.y = .20;
	    pose.position.z = 0.99;
	    // rotation in quaternion
	    pose.orientation.x = -0.2;
	    pose.orientation.y = 0.0;
	    pose.orientation.z = 0.0;
	    pose.orientation.w = 0.9;
	    break;

	  case 1: // arbitrary "front" position
	    pose.position.x = .40;
	    pose.position.y = -.40;
	    pose.position.z = 0.70;
	    pose.orientation.x = 0.2;
	    pose.orientation.y = 0.0;
	    pose.orientation.z = 0.0;
	    pose.orientation.w = 0.9;
	    break;

	  case 2: // (low) position directly over base where plate is horizontal
	    pose.position.x = .0;
	    pose.position.y = .0;
	    pose.position.z = 0.80;
	    pose.orientation.x = 0.0;
	    pose.orientation.y = 0.0;
	    pose.orientation.z = 0.0;
	    pose.orientation.w = 0.9;
	    break;

	  case 3: // (high) position directly over base where plate is horizontal
	    pose.position.x = .0;
	    pose.position.y = .0;
	    pose.position.z = 1.15;
	    pose.orientation.x = 0.0;
	    pose.orientation.y = 0.0;
	    pose.orientation.z = 0.0;
	    pose.orientation.w = 0.9;
	    break;
	}

	odo.pose.pose = pose;
	odoPub.publish(odo);
	std::cout << "Finished Publishing first msg" << std::endl;
	ros::spin();


	return 0;
}

