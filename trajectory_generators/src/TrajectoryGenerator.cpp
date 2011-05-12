
#include "TrajectoryGenerator.hpp"
#include <ocl/Component.hpp>

ORO_CREATE_COMPONENT( trajectory_generators::TrajectoryGenerator);

namespace trajectory_generators
{
    using namespace RTT;
    using namespace KDL;
    using namespace std;

    TrajectoryGenerator::TrajectoryGenerator(std::string name)
        : TaskContext(name,PreOperational)
    {
        //Creating TaskContext

        //Adding InputPorts
        this->addEventPort("CartesianPoseInput",input_cartPosPort, boost::bind(&TrajectoryGenerator::generateNewVelocityProfilesCartPosInput, this, _1));
        this->addEventPort("JointPositionInput",input_jntPosPort, boost::bind(&TrajectoryGenerator::generateNewVelocityProfilesJntPosInput, this, _1));
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

        jntState.header.frame_id = "arm_0_link";
        jntState.name.push_back("arm_1_joint");
        jntState.name.push_back("arm_2_joint");
        jntState.name.push_back("arm_3_joint");
        jntState.name.push_back("arm_4_joint");
        jntState.name.push_back("arm_5_joint");
        jntState.name.push_back("arm_6_joint");
        jntState.name.push_back("arm_7_joint");

    }

    TrajectoryGenerator::~TrajectoryGenerator()
    {
    }

    bool TrajectoryGenerator::configureHook()
    {
    	//num_axes = num_axes_prop.rvalue();
    	//TODO:check if dimensions of v_max and a_max match num_axes

    	//v_max = v_max_property.rvalue();
    	//a_max = a_max_property.rvalue();

    	log(Info) << "nAxes = " << num_axes << endlog();

    	return true;

    }

    bool TrajectoryGenerator::startHook()
    {
    	return true;
    }

    bool TrajectoryGenerator::generateNewVelocityProfilesJntPosInput(RTT::base::PortInterface* portInterface){
    	time_passed = os::TimeService::Instance()->secondsSince(time_begin);
    	log(Info) << "a new Pose arrived from ROS" << endlog();
    	cout << "a new Pose arrived from ROS" << endl;

    	input_jntPosPort.read(cmdJntState);
    	lastCommndedPoseJntPos = cmdJntState.position;

    	for(int i=0; i < 7; i++){
    		cout << "Joint " << i << " : " << lastCommndedPoseJntPos[i] << " /// ";
    		if(lastCommndedPoseJntPos[i]<p_min[i] || lastCommndedPoseJntPos[i]>p_max[i]){
    			log(Info) << "Commanded joint position out of bounds" << endlog();
    			cout << "Commanded joint position out of bounds" << endl;
    			return false;
    		}

    	}
    	cout << endl;

    	//Do FK and check if z value is within the limits
    	//TODO: Think of a more clever way (not hardcoded) to implement this restriction
    	if (!(KukaLWR_Kinematics::fkSolver(lastCommndedPoseJntPos, lastCommandedPose))){
    		cout << "lastCommandedPose cannot be achieved, Destination point modified" << endl;
    	}

    	//Show results
		cout << "Coordinate x : " << lastCommandedPose.position.x << endl;
		cout << "Coordinate y : " << lastCommandedPose.position.y << endl;
		cout << "Coordinate z : " << lastCommandedPose.position.z << endl;

    	if (lastCommandedPose.position.z < 0.2)
    	{
    		cout << "lastCommandedPose out of safety zone, Destination point modified" << endl;
    		lastCommndedPoseJntPos = std::vector<double>(7,0.0);
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

    bool TrajectoryGenerator::generateNewVelocityProfilesCartPosInput(RTT::base::PortInterface* portInterface){
    	time_passed = os::TimeService::Instance()->secondsSince(time_begin);
    	log(Info) << "a new joint position arrived from ROS" << endlog();
    	cout << "a new joint position arrived from ROS" << endl;

    	input_cartPosPort.read(lastCommandedPose);
    	cout << "Pose.position.x  = " << lastCommandedPose.position.x << endl;
    	cout << "Pose.position.y  = " << lastCommandedPose.position.y << endl;
    	cout << "Pose.position.z  = " << lastCommandedPose.position.z << endl;

    	//Do IK and reset velocity profiles
    	if (!(KukaLWR_Kinematics::ikSolver(lastCommandedPose, lastCommndedPoseJntPos))){
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

    void TrajectoryGenerator::updateHook()
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


    void TrajectoryGenerator::stopHook()
    {
    }

    void TrajectoryGenerator::cleanupHook()
    {

    }


}//namespace



