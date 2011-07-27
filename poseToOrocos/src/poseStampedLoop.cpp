#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Dense>

#include <sstream>
using namespace std;
using namespace Eigen;

# define PI 3.14159265

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

<<<<<<< HEAD
  ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("poseStampedDsr", 2, true);
  ros::Publisher posePub = n.advertise<geometry_msgs::Pose>("poseDsr", 2, true);

  ros::Rate loop_rate(0.2);
=======
  ros::Publisher posePub = n.advertise<geometry_msgs::Pose>("poseDsr", 2, true);
  ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("poseStampedDsr", 2, true);

  ros::Rate loop_rate(0.25);
>>>>>>> a92e5c5a39ea76383bd22ad60360fa1c63f40699

  int count = 0;
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

    Vector3d rotVec = Vector3d::Random();
    rotVec.normalize();

    double rotAng = -PI + 2*PI*(double)rand()/(double)RAND_MAX;
    rotVec *= sin(rotAng/2.0);

    poseStamped.pose.orientation.x = rotVec(0);
    poseStamped.pose.orientation.y = rotVec(1);
    poseStamped.pose.orientation.z = rotVec(2);
    poseStamped.pose.orientation.w = cos(rotAng/2.0);

    ROS_INFO("poseStamped orientation: %lf, %lf, %lf, %lf",
    		poseStamped.pose.orientation.x,poseStamped.pose.orientation.y,poseStamped.pose.orientation.z,poseStamped.pose.orientation.w);

<<<<<<< HEAD
    poseStampedPub.publish(poseStamped);
    posePub.publish(poseStamped.pose);
=======
    poseStampedPub.publish(poseStamped); //to rviz
    posePub.publish(poseStamped.pose); //to orocos
>>>>>>> a92e5c5a39ea76383bd22ad60360fa1c63f40699

    ros::spinOnce();


    loop_rate.sleep();
    ++count;
  }


  return 0;
}
