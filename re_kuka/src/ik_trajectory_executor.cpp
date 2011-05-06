#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <re_kuka/ExecuteCartesianIKTrajectory.h>
#include <vector>

#define MAX_JOINT_VEL 1.5  //in radians/sec

static const std::string ARM_IK_NAME = "/arm_kinematics/get_ik";

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::
                                      JointTrajectoryAction> TrajClient;

class IKTrajectoryExecutor{

private:
  ros::NodeHandle node;
  ros::ServiceClient ik_client;
  ros::ServiceServer service;
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  kinematics_msgs::GetPositionIK::Request  ik_request;
  kinematics_msgs::GetPositionIK::Response ik_response;  
  TrajClient *action_client;

public:
  IKTrajectoryExecutor(){

    //create a client function for the IK service
    ik_client = node.serviceClient<kinematics_msgs::GetPositionIK>
      (ARM_IK_NAME, true);    

    //wait for the various services to be ready
    ROS_INFO("Waiting for services to be ready");
    ros::service::waitForService(ARM_IK_NAME);
    ROS_INFO("Services ready");

    //tell the joint trajectory action client that we want 
    //to spin a thread by default
    action_client = new TrajClient("arm_controller/joint_trajectory_action", true);

    //wait for the action server to come up
    while(ros::ok() && !action_client->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the [/arm_controller/arm_joint_trajectory_action_node] action server to come up");
    }

    //register a service to input desired Cartesian trajectories
    service = node.advertiseService("execute_cartesian_ik_trajectory", 
        &IKTrajectoryExecutor::execute_cartesian_ik_trajectory, this);

    //have to specify the order of the joints we're sending in our 
    //joint trajectory goal, even if they're already on the param server
    goal.trajectory.joint_names.push_back("arm_1_joint");
    goal.trajectory.joint_names.push_back("arm_2_joint");
    goal.trajectory.joint_names.push_back("arm_3_joint");
    goal.trajectory.joint_names.push_back("arm_4_joint");
    goal.trajectory.joint_names.push_back("arm_5_joint");
    goal.trajectory.joint_names.push_back("arm_6_joint");
    goal.trajectory.joint_names.push_back("arm_7_joint");

  }

  ~IKTrajectoryExecutor(){
    delete action_client;
  }


  //run inverse kinematics on a PoseStamped (7-dof pose 
  //(position + quaternion orientation) + header specifying the 
  //frame of the pose)
  //tries to stay close to double start_angles[7]
  //returns the solution angles in double solution[7] 
  bool run_ik(geometry_msgs::PoseStamped pose, double start_angles[7], 
              double solution[7], std::string link_name){

    kinematics_msgs::GetPositionIK::Request  ik_request;
    kinematics_msgs::GetPositionIK::Response ik_response;  
 
    ik_request.timeout = ros::Duration(5.0);
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("arm_1_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("arm_2_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("arm_3_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("arm_4_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("arm_5_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("arm_6_joint");
    ik_request.ik_request.ik_seed_state.joint_state.name.push_back("arm_7_joint");

    ik_request.ik_request.ik_link_name = link_name; //arm_7_link

    ik_request.ik_request.pose_stamped = pose; //requested pose
    ik_request.ik_request.ik_seed_state.joint_state.position.resize(7);

    for(int i=0; i<7; i++) ik_request.ik_request.ik_seed_state.joint_state.position[i] = start_angles[i];

    ROS_INFO("request pose: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);

    bool ik_service_call = ik_client.call(ik_request,ik_response);
    if(!ik_service_call){
      ROS_ERROR("IK service call failed!");  
      return 0;
    }
    if(ik_response.error_code.val == ik_response.error_code.SUCCESS){
      for(int i=0; i<7; i++){
        solution[i] = ik_response.solution.joint_state.position[i];
      }
      ROS_INFO("solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", 
             solution[0],solution[1], solution[2],solution[3],
             solution[4],solution[5],solution[6]);
      ROS_INFO("IK service call succeeded");
      return 1;
    }
    ROS_INFO("IK service call error code: %d", ik_response.error_code.val);
    return 0;
  }


  //figure out where the arm is now  
  void get_current_joint_angles(double current_angles[7]){
    int i;

    //get a single message from the topic 'r_arm_controller/state'
    pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg = 
      ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState>
      ("arm_controller/state");
    
    //extract the joint angles from it
    for(i=0; i<7; i++){
      current_angles[i] = state_msg->actual.positions[i];
    }
  }


  //send a desired joint trajectory to the joint trajectory action
  //and wait for it to finish
  bool execute_joint_trajectory(std::vector<double *> joint_trajectory){
    int i, j; 
    int trajectorylength = joint_trajectory.size();

    //get the current joint angles
    double current_angles[7];    
    get_current_joint_angles(current_angles);

    //fill the goal message with the desired joint trajectory
    goal.trajectory.points.resize(trajectorylength+1);

    //set the first trajectory point to the current position
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].velocities.resize(7);
    for(j=0; j<7; j++){
      goal.trajectory.points[0].positions[j] = current_angles[j];
      goal.trajectory.points[0].velocities[j] = 0.0; 
    }

    //make the first trajectory point start 0.25 seconds from when we run
    goal.trajectory.points[0].time_from_start = ros::Duration(0.25);     

    //fill in the rest of the trajectory
    double time_from_start = 0.25;
    for(i=0; i<trajectorylength; i++){
      goal.trajectory.points[i+1].positions.resize(7);
      goal.trajectory.points[i+1].velocities.resize(7);

      //fill in the joint positions (velocities of 0 mean that the arm
      //will try to stop briefly at each waypoint)
      for(j=0; j<7; j++){
        goal.trajectory.points[i+1].positions[j] = joint_trajectory[i][j];
        goal.trajectory.points[i+1].velocities[j] = 0.0;
      }

      //compute a desired time for this trajectory point using a max 
      //joint velocity
      double max_joint_move = 0;
      for(j=0; j<7; j++){
        double joint_move = fabs(goal.trajectory.points[i+1].positions[j] 
                                 - goal.trajectory.points[i].positions[j]);
        if(joint_move > max_joint_move) max_joint_move = joint_move;
      }
      double seconds = max_joint_move/MAX_JOINT_VEL;
      ROS_INFO("max_joint_move: %0.3f, seconds: %0.3f", max_joint_move, seconds);
      time_from_start += seconds;
      goal.trajectory.points[i+1].time_from_start = 
        ros::Duration(time_from_start);
    }

    //when to start the trajectory
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.25);

    ROS_INFO("Sending goal to joint_trajectory_action");
    action_client->sendGoal(goal);

    action_client->waitForResult();

    //get the current joint angles for debugging
    get_current_joint_angles(current_angles);
    ROS_INFO("joint angles after trajectory: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n",current_angles[0],current_angles[1],current_angles[2],current_angles[3],current_angles[4],current_angles[5],current_angles[6]);

    if(action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, the arm finished the trajectory!");
      return 1;
    }
    ROS_INFO("The arm failed to execute the trajectory.");
    if(action_client->getState() == actionlib::SimpleClientGoalState::ABORTED)
	ROS_INFO("\n ABORTED \n ");
    return 0;
  }


  //service function for execute_cartesian_ik_trajectory
  bool execute_cartesian_ik_trajectory(
         re_kuka::ExecuteCartesianIKTrajectory::Request &req,
         re_kuka::ExecuteCartesianIKTrajectory::Response &res){

    int trajectory_length = req.poses.size();
    int i, j;
    
    //IK takes in Cartesian poses stamped with the frame they belong to
    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.header = req.header;
    stamped_pose.header.stamp = ros::Time::now();
    bool success;
    std::vector<double *> joint_trajectory;

    //get the current joint angles (to find ik solutions close to)
    double last_angles[7];    
    get_current_joint_angles(last_angles);

    //find IK solutions for each point along the trajectory 
    //and stick them into joint_trajectory
    for(i=0; i<trajectory_length; i++){
      
      stamped_pose.pose = req.poses[i];
      double *trajectory_point = new double[7];
      success = run_ik(stamped_pose, last_angles, trajectory_point, "/arm_7_link");
      joint_trajectory.push_back(trajectory_point);

      if(!success){
        ROS_ERROR("IK solution not found for trajectory point number %d!\n", i);
        return 0;
      }
      for(j=0; j<7; j++) last_angles[j] = trajectory_point[j];
    }        

    //run the resulting joint trajectory
    ROS_INFO("executing joint trajectory");
    success = execute_joint_trajectory(joint_trajectory);
    res.success = success;
    
    return success;
  }

};


int main(int argc, char** argv){

  //init ROS node
  ros::init(argc, argv, "cartesian_ik_trajectory_executor");
  
  sleep(5);

  IKTrajectoryExecutor ik_traj_exec = IKTrajectoryExecutor();

  ROS_INFO("Waiting for cartesian trajectories to execute");
  ros::spin();
  
  return 0;

}
