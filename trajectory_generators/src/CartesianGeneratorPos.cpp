#include "CartesianGeneratorPos.hpp"
#include <ocl/Component.hpp>

ORO_CREATE_COMPONENT( trajectory_generators::CartesianGeneratorPos );

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
    	lastCommndedPoseJntPos = std::vector<double>(7,0.0);
    	v_max = std::vector<double>(7,2.0);
    	a_max = std::vector<double>(7,1.0);
    	jntVel = std::vector<double>(7,0.0);

        //Adding InputPorts
        this->addPort("CartesianPoseDes",cmd_cartPosPort);
        this->addPort("JointPositionMsr",msr_jntPosPort);

        //Adding OutputPorts
        this->addPort("JointPositionDes",cmd_jntPosPort);
        this->addPort("JointPositionDesToROS",cmd_jntPosPort_toROS);
        //Adding Properties
        //this->addProperty("max_vel",v_max).doc("Maximum Velocity in Trajectory");
        //this->addProperty("max_acc",a_max).doc("Maximum Acceleration in Trajectory");
        //this->addProperty("num_axes",num_axes).doc("Number of Axes");

        this->addOperation("moveTo",&CartesianGeneratorPos::moveTo,this,OwnThread)
        	  .doc("Set the position setpoint")
        	  .arg("setpoint", "position setpoint for end effector")
        	  .arg("time", "minimum time to execute trajectory" );

        //Adding Methods
        this->addOperation( "resetPosition",&CartesianGeneratorPos::resetPosition,this,OwnThread).doc("Reset generator's position" );

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

    void CartesianGeneratorPos::updateHook()
    {
    	time_passed = os::TimeService::Instance()->secondsSince(time_begin);
    	if(cmd_cartPosPort.read(lastCommandedPose) == RTT::NewData){
    		log(Info) << "a new Pose arrived from ROS" << endlog();
    		//Do IK and reset velocity profiles
    		this->ikSolver(lastCommandedPose, lastCommndedPoseJntPos);
    		//Create joint specific velocity profiles
    		double maxDuration = 0.0;
    		std::vector<double> jntPos;
    		msr_jntPosPort.read(jntPos);
    		for(int i = 0; i < (int)motionProfile.size(); i++)
    			jntVel[i] = motionProfile[i].getVel(time_passed);
    		motionProfile.clear();
    		//TODO: Check dimensions
    		for(int i = 0; i < (int)lastCommndedPoseJntPos.size(); i++){
    			motionProfile.push_back(VelocityProfile_NonZeroInit(a_max[i], v_max[i], lastCommndedPoseJntPos[i], jntPos[i], jntVel[i]));
    			//TODO: Add non zero velocities.
    			if(motionProfile[i].getDuration() > maxDuration )
    				maxDuration = motionProfile[i].getDuration();
    		}
    		//Do sync
    		for(int i = 0; i < (int)lastCommndedPoseJntPos.size(); i++){
    			motionProfile[i].setDuration(maxDuration);
    		}

    		//Set times
    		time_begin = os::TimeService::Instance()->getTicks();
    		time_passed = 0.0;

    	}else{
    		//Execute current velocity profile
    		if (motionProfile.size()==7){
    			std::vector<double> jntPosCmd;
    			sensor_msgs::JointState jntState = new sensor_msgs::JointState();
    			for(int i = 0; i < (int)motionProfile.size(); i++){
    				jntPosCmd.push_back(motionProfile[i].getPos(time_passed));
    				if(toROS){jntState.position.push_back(motionProfile[i].getPos(time_passed));}
    			}
    			cmd_jntPosPort.write(jntPosCmd);
    			if(toROS){cmd_jntPosPort_toROS.write(jntState);}
    		}
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

    bool CartesianGeneratorPos::ikSolver(geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr){
    	//TODO: Check size of jntPosDsr. Should be 7
    	poseDsr.position.x = 0.0;
    	poseDsr.position.y = 0.0;
    	poseDsr.position.z = 0.0;

    	poseDsr.orientation.x = 0.0;
    	poseDsr.orientation.y = 0.0;
    	poseDsr.orientation.z = 0.0;
    	poseDsr.orientation.w = 0.0;

    	for(int )



    	return true;

    }
}//namespace



