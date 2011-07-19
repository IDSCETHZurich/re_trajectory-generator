#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"


#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher posePub = n.advertise<geometry_msgs::PoseStamped>("poseStampedDsr", 2, true);

  ros::Rate loop_rate(1);

  int count = 0, aux = 0;
  bool newposition = false;
  double x=0.0, y=0.0, z=0.0;

  while (ros::ok())
  {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id="/frame_id_1";
    poseStamped.header.stamp = ros::Time::now();

    newposition = false;
    while (!newposition) {
    	x = (0.7-(-0.7))*(double)rand()/(double)RAND_MAX + (-0.7);
    	y = (0.7-(-0.7))*(double)rand()/(double)RAND_MAX + (-0.7);
    	z = (0.7-(0.2))*(double)rand()/(double)RAND_MAX + (0.2);
    	if (x*x+y*y+z*z>=0.50 && x*x+y*y+z*z<=0.70) {
    		newposition = true;
    	}
    }

    poseStamped.pose.position.x = x;
    poseStamped.pose.position.y = y;
    poseStamped.pose.position.z = z;

    poseStamped.pose.orientation.x = (1.0-(-1.0))*(double)rand()/(double)RAND_MAX + (-1.0);
    aux = sqrt(1.0 - poseStamped.pose.orientation.x*poseStamped.pose.orientation.x);
    poseStamped.pose.orientation.y = (aux-(-aux))*(double)rand()/(double)RAND_MAX + (-aux);
    aux = sqrt(1.0 - poseStamped.pose.orientation.x*poseStamped.pose.orientation.x
    								- poseStamped.pose.orientation.y*poseStamped.pose.orientation.y);
    poseStamped.pose.orientation.z = (aux-(-aux))*(double)rand()/(double)RAND_MAX + (-aux);
    poseStamped.pose.orientation.w = sqrt(1.0 - poseStamped.pose.orientation.x*poseStamped.pose.orientation.x
    							  - poseStamped.pose.orientation.y*poseStamped.pose.orientation.y
    							  - poseStamped.pose.orientation.z*poseStamped.pose.orientation.z);


    posePub.publish(poseStamped);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
