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

  ros::Publisher posePub = n.advertise<geometry_msgs::Pose>("poseDsr", 2, true);

  ros::Rate loop_rate(0.25);

  int count = 0;
  bool newposition = false;
  double x=0.0, y=0.0, z=0.0;

  while (ros::ok())
  {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id="/frame_id_1";
    poseStamped.header.stamp = ros::Time::now();


    if (count%2==0)
    	poseStamped.pose.position.z = 0.931;
    else
    	poseStamped.pose.position.z = 0.751;

    poseStamped.pose.position.x = 0.417;
    poseStamped.pose.position.y = 0.0;

    //Vector3d rotVec = Vector3d::Random();
    //rotVec.normalize();

    //double rotAng = -PI + 2*PI*(double)rand()/(double)RAND_MAX;
    //rotVec *= sin(rotAng/2.0);

    poseStamped.pose.orientation.x = 0;//rotVec(0);
    poseStamped.pose.orientation.y = 0;//rotVec(1);
    poseStamped.pose.orientation.z = 0;//rotVec(2);
    poseStamped.pose.orientation.w = 1;//cos(rotAng/2.0);

    ROS_INFO("poseStamped pose: %lf, %lf, %lf", poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z);
    //ROS_INFO("poseStamped orientation: %lf, %lf, %lf, %lf",
    //		poseStamped.pose.orientation.x,poseStamped.pose.orientation.y,poseStamped.pose.orientation.z,poseStamped.pose.orientation.w);

    posePub.publish(poseStamped.pose);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
