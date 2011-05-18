#include "kuka_IK-component.hpp"
#include <ocl/Component.hpp>

ORO_CREATE_COMPONENT(kuka_IK::kuka_IK)
namespace kuka_IK{

	using namespace RTT;
    using namespace KDL;
    using namespace std;

    kuka_IK::kuka_IK(string const& name): TaskContext(name,PreOperational)
    {
    	 this->addEventPort("CartesianPoseInput",input_cartPosPort, boost::bind(&kuka_IK::cartPosInputHandle, this, _1));
    	 this->addPort("JointPositionDes",output_jntPosPort);
    }

    bool kuka_IK::cartPosInputHandle(RTT::base::PortInterface* portInterface){

    	input_cartPosPort.read(commandedPose);
    	cout << "Pose.position.x  = " << commandedPose.position.x << endl;
    	cout << "Pose.position.y  = " << commandedPose.position.y << endl;
    	cout << "Pose.position.z  = " << commandedPose.position.z << endl;

    	//Do IK and reset velocity profiles
    	if (!(KukaLWR_Kinematics::ikSolver(commandedPose, commndedPoseJntPos))){
    		cout << "lastCommandedPose cannot be achieved, Destination point modified" << endl;
    	}
    	output_jntPosPort.write(commndedPoseJntPos);
    	return true;

    }

    bool kuka_IK::configureHook(){return true;}
    bool kuka_IK::startHook(){return true;}
    void kuka_IK::updateHook(){}
    void kuka_IK::stopHook(){}
    void kuka_IK::cleanupHook(){}

}
