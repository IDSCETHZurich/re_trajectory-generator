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
    	if(commandedState.size()!=13)
    		std::cout << "vector dimensions does not agree" << std::endl;

    	KDL::ChainIkSolverVel_pinv ikvelsolver = KDL::ChainIkSolverVel_pinv(robotChain);

    	// Read out the robot joint position
    	msr_jntPosPort.read(jntPos);
    	KDL::JntArray q_init,qdot_out;
    	Eigen::Matrix<double, 7, 1> vec7d;
    	for(int i=0; i< 7; i++)
    		vec7d[i] = jntPos[i];

    	q_init.data = vec7d;

    	KDL::Twist v_in;
    	v_in.vel = KDL::Vector(commandedState[7],commandedState[8],commandedState[9]);
    	v_in.rot = KDL::Vector(commandedState[10],commandedState[11],commandedState[12]);
    	//std::cout << "LINE 87" << std::endl;

    	Eigen::Matrix<double, 7, 1> tmpVec;
    	for(int i=0; i< 7; i++)
    		tmpVec[i] = 0.0;
    	qdot_out.data = tmpVec;

        ikvelsolver.CartToJnt(q_init, v_in, qdot_out);

//        for ( int i = 0; i < 6; i++)
//        	xDot(i) = commandedState[i+7];

    	commandedPose.position.x = commandedState[0];
    	commandedPose.position.y = commandedState[1];
    	commandedPose.position.z = commandedState[2];
    	commandedPose.orientation.x = commandedState[3];
    	commandedPose.orientation.y = commandedState[4];
    	commandedPose.orientation.z = commandedState[5];
    	commandedPose.orientation.w = commandedState[6];
    	//rest is the twist


//    	RobotStatePort.read(tmpRobotData);
//		//TODO: Clean from code of Jacobian from FRI once we have tested the IkSolverVel
//        for ( int i = 0; i < 6; i++)
//          for ( int j = 0; j < 7; j++)
//  			Jacobian(i,j) = tmpRobotData.jacobian[i*7+j];
//        cout << "Jacobian = "  << endl << Jacobian << endl;
//        thetaDot = (	((Jacobian.transpose()*Jacobian).inverse())	*	Jacobian.transpose()	)	*	xDot;
//        cout << "thetaDot = "  << endl << thetaDot << endl;

#if DEBUG
    	cout << "Pose.position.y  = " << commandedPose.position.y << endl;
    	cout << "Pose.position.z  = " << commandedPose.position.z << endl;
#endif


#if DEBUG
    	std::cout << " kuka-IK-component.cpp: jntPos" << std::endl;
   		for(int i = 0; i < 7; i++)  std::cout << jntPos[i] << " " ;
#endif
    	//Do IK and reset velocity profiles
    	commndedPoseJntPos = std::vector<double>(7,0.0);

#if DEBUG
    	cout <<  endl << endl;
    	cout << "Beginning of Super Debugging" << endl;
    	geometry_msgs::Pose tmpPose;
    	cartPosPort.read(tmpPose);
    	Frame tmpFrame;
    	tf::PoseMsgToKDL(tmpPose, tmpFrame);

    	cout << "Frame from Robot" << endl << tmpFrame << endl;

     	//Debugging Iterative IK
		ChainFkSolverPos_recursive fksolver(chain);

		// Create joint array
		unsigned int nj = chain.getNrOfJoints();
		KDL::JntArray jointpositions = JntArray(nj);

		// Assign some values to the joint positions
		std::cout << "jntPos: ";
		for(unsigned int i=0;i<nj;i++){
			jointpositions(i)=jntPos[i];
			std::cout << jntPos[i] << " ";
		}
		std::cout << std::endl;

		// Create the frame that will contain the results
		KDL::Frame cartpos;

		// Calculate forward position kinematics
		bool kinematics_status;
		kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
		if(kinematics_status>=0){
			std::cout << cartpos <<std::endl;
			printf("%s \n","Success, thanks KDL!");
		}else{
			printf("%s \n","Error: could not calculate forward kinematics");
			std::cout << cartpos <<std::endl;
		}

		cout << "End of Super Debugging" << endl << endl;

		//End of Super Debugging
#endif


    	//if (!(KukaLWR_Kinematics::ikSolver(jntPos, commandedPose, commndedPoseJntPos))){
    	if (!(KukaLWR_Kinematics::ikSolverIterative7DOF(jntPos, commandedPose, commndedPoseJntPos))){
    		cout << "lastCommandedPose cannot be achieved" << endl;
    		for(int i = 0; i < 7; i++)  std::cout << commndedPoseJntPos[i] << " " ;
    		std::cout << std::endl;
    	}
    	//cout << "passing line 164" << endl;
        
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
