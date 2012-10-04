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
// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
// %EndTag(MSG_HEADER)%

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher posePub = n.advertise<sensor_msgs::JointState>("jntPosDsr", 2, true);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(1);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
// Set the random seed based on actual time
//  srand ( time(NULL) );
  sensor_msgs::JointState jntState;
  jntState.position.clear();

  // joints are ordered in the physical order of the robot, i.e. A1, A2, E1, A3, A4, A5, A6
  // angles are measured in RAD.
  double rad_per_angle = 3.141 / 180;
  // A2 is measured by the robot 90 degrees ahead of what we send.
  double A2_correction = -90;

  jntState.position = std::vector<double>(7,0.0);

  // call with parameter: ``rosrun poseToOrocos jntPosSingle _pos:=1``
  ros::NodeHandle nh("~");
  int pos_choice = 0;
  nh.getParam("pos", pos_choice);

  switch (pos_choice) {
    case 0:
      // position where plate is horizontal
      jntState.position[0] = -14 * rad_per_angle;
      jntState.position[1] = (-2.8 + A2_correction) * rad_per_angle;
      jntState.position[2] = -16 * rad_per_angle;
      jntState.position[3] = -66.8 * rad_per_angle;
      jntState.position[4] = 32.2 * rad_per_angle;
      jntState.position[5] = 30.2 * rad_per_angle;
      jntState.position[6] = 10 * rad_per_angle;
      break;
    case 1:
      // arbitrary "high-up" position
      jntState.position[0] = 60 * rad_per_angle;
      jntState.position[1] = (60 + A2_correction) * rad_per_angle;
      jntState.position[2] = 30 * rad_per_angle;
      jntState.position[3] = -30 * rad_per_angle;
      jntState.position[4] = 60 * rad_per_angle;
      jntState.position[5] = 30 * rad_per_angle;
      jntState.position[6] = 0 * rad_per_angle;
      break;
    case 2:
      // random position
      for (int i=0; i<7 ; i++) {
        jntState.position.push_back(-2 + 4.0*((double)rand()/(double)RAND_MAX));
      }
      break;
  }

  // %Tag(PUBLISH)%
      posePub.publish(jntState);
      std::cout << "Finished Publishing first msg" << std::endl;
      ros::spin();
  // %EndTag(PUBLISH)%


  return 0;
}
// %EndTag(FULLTEXT)%
