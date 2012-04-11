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

	ros::Rate loop_rate(1);

	int count = 0, aux = 0;
	bool newposition = false;
	double x=0.0, y=0.0, z=0.0;

	geometry_msgs::Pose pose;
	nav_msgs::Odometry odo;
	while (ros::ok())
	{
		newposition = false;
		while (!newposition) {
		x = (0.7-(-0.7))*(double)rand()/(double)RAND_MAX + (-0.7);
		y = (0.7-(-0.7))*(double)rand()/(double)RAND_MAX + (-0.7);
		z = (0.9-( 0.4))*(double)rand()/(double)RAND_MAX + (0.4);
		if (x*x+y*y+z*z>=0.50 && x*x+y*y+z*z<=0.70) {
			newposition = true;
	}
	}

	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;

	pose.orientation.x = (1.0-(-1.0))*(double)rand()/(double)RAND_MAX + (-1.0);
	aux = sqrt(1.0 - pose.orientation.x*pose.orientation.x);
	pose.orientation.y = (aux-(-aux))*(double)rand()/(double)RAND_MAX + (-aux);
	aux = sqrt(1.0 - pose.orientation.x*pose.orientation.x - pose.orientation.y*pose.orientation.y);
	pose.orientation.z = (aux-(-aux))*(double)rand()/(double)RAND_MAX + (-aux);
	pose.orientation.w = sqrt(1.0 - pose.orientation.x*pose.orientation.x
							  - pose.orientation.y*pose.orientation.y
							  - pose.orientation.z*pose.orientation.z);

	odo.pose.pose = pose;
	odoPub.publish(odo);
	ros::spinOnce();
	loop_rate.sleep();
	++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%
