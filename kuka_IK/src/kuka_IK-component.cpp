/***************************************************************************

    File:           kuka-IK-component.cpp
    Author(s):      Gajamohan Mohanarajah/Francisco Ramos
    Affiliation:    IDSC - ETH Zurich
    e-mail:         gajan@ethz.ch/framosde@ethz.ch
    Start date:	    11th April 2011
    Last update:	11th May 2011

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/
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
    	 this->addPort("JointPositionMsr",msr_jntPosPort);
    	 this->addPort("msrCartPosPort", cartPosPort);
    	 this->addPort("RobotStatePort", RobotStatePort);

    	 jntPos = std::vector<double>(7,0.0);

    	 // Creation of the Kinematic Robot Chain based on D-H parameters
    	 robotChain = Chain();
    	 robotChain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RotZ(M_PI), Vector(0.0, 0.0, 0.31))));
    	 robotChain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RotZ(M_PI), Vector(0.0, 0.0, 0.2))));
    	 robotChain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.2))));
    	 robotChain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RotZ(M_PI), Vector(0.0, 0.0, 0.2))));
    	 robotChain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.19))));
    	 robotChain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RotZ(M_PI))));
    	 robotChain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.078))));

    	 // Definition of the solver for joint velocity
    	 //KDL::ChainIkSolverVel_pinv ikvelsolver = KDL::ChainIkSolverVel_pinv(robotChain); // Other parameters are default eps=0.00001 & iter=150
    }

    bool kuka_IK::cartPosInputHandle(RTT::base::PortInterface* portInterface){
    	input_cartPosPort.read(commandedState);

    	KDL::ChainIkSolverVel_pinv ikvelsolver = KDL::ChainIkSolverVel_pinv(robotChain);

    	// Read out the robot joint position
    	msr_jntPosPort.read(jntPos);
    	KDL::JntArray q_init,qdot_out;
    	Eigen::Matrix<double, 7, 1> vec7d;
    	for(int i=0; i< 7; i++)
    		vec7d[i] = jntPos[i];

    	q_init.data = vec7d;

    	KDL::Twist v_in;
    	v_in.vel = KDL::Vector(commandedState.twist.twist.linear.x, commandedState.twist.twist.linear.y, commandedState.twist.twist.linear.z);
    	v_in.rot = KDL::Vector(commandedState.twist.twist.angular.x, commandedState.twist.twist.angular.y, commandedState.twist.twist.angular.z);

    	Eigen::Matrix<double, 7, 1> tmpVec;
    	for(int i=0; i< 7; i++)
    		tmpVec[i] = 0.0;
    	qdot_out.data = tmpVec;

        ikvelsolver.CartToJnt(q_init, v_in, qdot_out);

    	//Do IK and reset velocity profiles
    	commndedPoseJntPos = std::vector<double>(7,0.0);

    	//if (!(KukaLWR_Kinematics::ikSolver(jntPos, commandedState.pose.pose, commndedPoseJntPos))){
    	if (!(KukaLWR_Kinematics::ikSolverIterative7DOF(jntPos, commandedState.pose.pose, commndedPoseJntPos))){
    	//if (!(KukaLWR_Kinematics::ikSolverAnalytical7DOF(commandedState.pose.pose, commndedPoseJntPos))){
    		cout << "lastCommandedPose cannot be achieved" << endl;
    		for(int i = 0; i < 7; i++)  std::cout << commndedPoseJntPos[i] << " " ;
    		std::cout << std::endl;
    	}
        
        sensor_msgs::JointState tmpJntState;
        tmpJntState.position.clear();
        for(int i=0; i < 7; i++){
    		tmpJntState.position.push_back(commndedPoseJntPos[i]);
    		//tmpJntState.velocity.push_back(thetaDot(i));
    		tmpJntState.velocity.push_back(qdot_out.data(i));
    		//tmpJntState.velocity.push_back(0.0);
    	}

    	output_jntPosPort.write(tmpJntState);
    	return true;

    }

    bool kuka_IK::configureHook(){return true;}
    bool kuka_IK::startHook(){

    	return true;}
    void kuka_IK::updateHook(){
    }
    void kuka_IK::stopHook(){}
    void kuka_IK::cleanupHook(){}

}
