#include "CartesianGeneratorPos.hpp"
#include <ocl/Component.hpp>

ORO_CREATE_COMPONENT( trajectory_generators::CartesianGeneratorPos);

namespace trajectory_generators
{
    using namespace RTT;
    using namespace KDL;
    using namespace std;

    CartesianGeneratorPos::CartesianGeneratorPos(string name)
        : TaskContext(name,PreOperational)
    {
        //Creating TaskContext
    	is_moving = false;
    	toROS = true;

        //Adding InputPorts
        this->addEventPort("CartesianPoseInput",input_cartPosPort, boost::bind(&CartesianGeneratorPos::generateNewVelocityProfilesCartPosInput, this, _1));
        this->addEventPort("JointPositionInput",input_jntPosPort, boost::bind(&CartesianGeneratorPos::generateNewVelocityProfilesJntPosInput, this, _1));
        this->addPort("JointPositionMsr",msr_jntPosPort);

        //Adding OutputPorts
        this->addPort("JointPositionDes",output_jntPosPort);
        this->addPort("JointPositionDesToROS",output_jntPosPort_toROS);

        //Adding Properties
        this->addProperty("num_axes",num_axes).doc("Number of Axes");
        this->addProperty("max_jntPos",p_max).doc("The maximum joint position of each joint (rad)");
        this->addProperty("min_jntPos",p_min).doc("The minimum joint position of each joint (rad)");
        this->addProperty("max_vel",v_max).doc("Maximum Velocity in Trajectory");
        this->addProperty("max_acc",a_max).doc("Maximum Acceleration in Trajectory");

        lastCommndedPoseJntPos = std::vector<double>(7,0.0);
        jntVel = std::vector<double>(7,0.0);

        this->addOperation("moveTo",&CartesianGeneratorPos::moveTo,this,OwnThread)
        	  .doc("Set the position setpoint")
        	  .arg("setpoint", "position setpoint for end effector")
        	  .arg("time", "minimum time to execute trajectory" );

        //Adding Methods
        this->addOperation( "resetPosition",&CartesianGeneratorPos::resetPosition,this,OwnThread).doc("Reset generator's position" );

        jntState.header.frame_id = "arm_0_link";
        jntState.name.push_back("arm_1_joint");
        jntState.name.push_back("arm_2_joint");
        jntState.name.push_back("arm_3_joint");
        jntState.name.push_back("arm_4_joint");
        jntState.name.push_back("arm_5_joint");
        jntState.name.push_back("arm_6_joint");
        jntState.name.push_back("arm_7_joint");

    }

    CartesianGeneratorPos::~CartesianGeneratorPos()
    {
    }

    bool CartesianGeneratorPos::configureHook()
    {
    	//num_axes = num_axes_prop.rvalue();
    	//TODO:check if dimensions of v_max and a_max match num_axes

    	//v_max = v_max_property.rvalue();
    	//a_max = a_max_property.rvalue();

    	log(Info) << "nAxes = " << num_axes << endlog();

    	return true;

    }

    bool CartesianGeneratorPos::startHook()
    {
    	return true;
    }

    bool CartesianGeneratorPos::generateNewVelocityProfilesJntPosInput(RTT::base::PortInterface* portInterface){
    	time_passed = os::TimeService::Instance()->secondsSince(time_begin);
    	log(Info) << "a new Pose arrived from ROS" << endlog();
    	cout << "a new Pose arrived from ROS" << endl;

    	input_jntPosPort.read(cmdJntState);
    	for(int i=0; i < 7; i++){
    		if(cmdJntState.position[i]<p_min[i] || cmdJntState.position[i]>p_max[i]){
    			log(Info) << "Commanded joint position out of bounds" << endlog();
    			cout << "Commanded joint position out of bounds" << endl;
    			return false;
    		}

    	}


    	lastCommndedPoseJntPos = cmdJntState.position;

    	//Create joint specific velocity profiles
    	double maxDuration = 0.0;
    	std::vector<double> jntPos = std::vector<double>(7,0.0);

    	msr_jntPosPort.read(jntPos);


    	if ((int)motionProfile.size() == 0){//Only for the first run
    		for(int i = 0; i < (int)num_axes; i++)
			{
    			jntVel[i] = 0.0;
    		}
    	}else{
    		for(int i = 0; i < (int)motionProfile.size(); i++)
    		{
    			jntVel[i] = motionProfile[i].Vel(time_passed);
    			jntPos[i] = motionProfile[i].Pos(time_passed);
 	   		}
     	}

    	motionProfile.clear();

    	//TODO: Check dimensions
    	for(int i = 0; i < (int)lastCommndedPoseJntPos.size(); i++){
    		motionProfile.push_back(VelocityProfile_NonZeroInit(v_max[i], a_max[i]));
    		motionProfile[i].SetProfile(jntPos[i], lastCommndedPoseJntPos[i], jntVel[i]);
    		if(motionProfile[i].Duration() > maxDuration )
    			maxDuration = motionProfile[i].Duration();
    	}

    	//Do sync
    	for(int i = 0; i < (int)lastCommndedPoseJntPos.size(); i++){
    		motionProfile[i].SetProfileDuration(maxDuration);
    	}

    	//Set times
    	time_begin = os::TimeService::Instance()->getTicks();
    	cout << "A new set of motion profiles were successfully created." << endl;
    	return true;

    }

    bool CartesianGeneratorPos::generateNewVelocityProfilesCartPosInput(RTT::base::PortInterface* portInterface){
    	time_passed = os::TimeService::Instance()->secondsSince(time_begin);
    	log(Info) << "a new joint position arrived from ROS" << endlog();
    	cout << "a new joint position arrived from ROS" << endl;

    	input_cartPosPort.read(lastCommandedPose);
    	cout << "Pose.position.x  = " << lastCommandedPose.position.x << endl;
    	cout << "Pose.position.y  = " << lastCommandedPose.position.y << endl;
    	cout << "Pose.position.z  = " << lastCommandedPose.position.z << endl;

    	//Do IK and reset velocity profiles
    	if (!(KukaLWR_IK::ikSolver(lastCommandedPose, lastCommndedPoseJntPos))){
    		cout << "lastCommandedPose cannot be achieved, Destination point modified" << endl;
    	}

    	//Create joint specific velocity profiles
    	double maxDuration = 0.0;
    	std::vector<double> jntPos = std::vector<double>(7,0.0);

    	msr_jntPosPort.read(jntPos);


    	if ((int)motionProfile.size() == 0){//Only for the first run
    		for(int i = 0; i < (int)num_axes; i++)
			{
    			jntVel[i] = 0.0;
    		}
    	}else{
    		for(int i = 0; i < (int)motionProfile.size(); i++)
    		{
    			jntVel[i] = motionProfile[i].Vel(time_passed);
    			jntPos[i] = motionProfile[i].Pos(time_passed);
 	   		}
     	}

    	motionProfile.clear();

    	//TODO: Check dimensions
    	for(int i = 0; i < (int)lastCommndedPoseJntPos.size(); i++){
    		motionProfile.push_back(VelocityProfile_NonZeroInit(v_max[i], a_max[i]));
    		motionProfile[i].SetProfile (jntPos[i], lastCommndedPoseJntPos[i], jntVel[i]);
    		if(motionProfile[i].Duration() > maxDuration )
    			maxDuration = motionProfile[i].Duration();
    	}

    	//Do sync
    	for(int i = 0; i < (int)lastCommndedPoseJntPos.size(); i++){
    		motionProfile[i].SetProfileDuration(maxDuration);
    	}

    	//Set times
    	time_begin = os::TimeService::Instance()->getTicks();
    	cout << "A new set of motion profiles were successfully created." << endl;
    	return true;

    }

    void CartesianGeneratorPos::CartesianGeneratorPos::updateHook()
    {
    	time_passed = os::TimeService::Instance()->secondsSince(time_begin);
    	//Execute current velocity profile
    	if (motionProfile.size()==7){
    		jntState.position.clear();
    	    jntPosCmd.clear();
    	    for(int i = 0; i < (int)motionProfile.size(); i++){
    	    	jntPosCmd.push_back(motionProfile[i].Pos(time_passed));
    	    	jntState.position.push_back(motionProfile[i].Pos(time_passed));
    	    }
    	    output_jntPosPort.write(jntPosCmd);
    	    output_jntPosPort_toROS.write(jntState);
    	}
    }


    void CartesianGeneratorPos::stopHook()
    {
    }

    void CartesianGeneratorPos::cleanupHook()
    {
    }

    bool CartesianGeneratorPos::moveTo(std::vector<double> position, double time)
    {
	  return true;
    }

    void CartesianGeneratorPos::resetPosition()
    {
    	//This function is really not necessary since we are currently running in command mode 1: Joint Position Control Mode
    }

}//namespace



