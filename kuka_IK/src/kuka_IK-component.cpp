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
    }

    bool kuka_IK::cartPosInputHandle(RTT::base::PortInterface* portInterface){
    	input_cartPosPort.read(commandedState);

    	if(commandedState.size()!=13)
    		std::cout << "vector dimensions does not agree" << std::endl;

    	commandedPose.position.x = commandedState[0];
    	commandedPose.position.y = commandedState[1];
    	commandedPose.position.z = commandedState[2];
    	commandedPose.orientation.x = commandedState[3];
    	commandedPose.orientation.y = commandedState[4];
    	commandedPose.orientation.z = commandedState[5];
    	commandedPose.orientation.w = commandedState[6];

    	//rest is the twist
    	RobotStatePort.read(tmpRobotData);

        for ( int i = 0; i < 6; i++)
          for ( int j = 0; j < 7; j++)
  			Jacobian(i,j) = tmpRobotData.jacobian[i*7+j];
        cout << "Jacobian = "  << endl << Jacobian << endl;

        for ( int i = 0; i < 6; i++)
        	xDot(i) = commandedState[i+7];

        thetaDot = (	((Jacobian.transpose()*Jacobian).inverse())	*	Jacobian.transpose()	)	*	xDot;
        cout << "thetaDot = "  << endl << thetaDot << endl;

#if DEBUG
    	cout << "Pose.position.y  = " << commandedPose.position.y << endl;
    	cout << "Pose.position.z  = " << commandedPose.position.z << endl;
#endif
    	// Read out the robot joint position
    	msr_jntPosPort.read(jntPos);

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
		KDL::Chain chain = Chain();
		chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RotZ(M_PI), Vector(0.0, 0.0, 0.31))));
		chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RotZ(M_PI), Vector(0.0, 0.0, 0.2))));
		chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.2))));
		chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RotZ(M_PI), Vector(0.0, 0.0, 0.2))));
		chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.19))));
		chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RotZ(M_PI))));
		chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.078))));

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
        
        sensor_msgs::JointState tmpJntState;
        tmpJntState.position.clear();
        for(int i=0; i < 7; i++){
    		tmpJntState.position.push_back(commndedPoseJntPos[i]);
    		tmpJntState.velocity.push_back(thetaDot[i]);
    		//tmpJntState.velocity.push_back(0.0);
    	}

    	output_jntPosPort.write(tmpJntState);
    	return true;

    }

    bool kuka_IK::configureHook(){return true;}
    bool kuka_IK::startHook(){
#if 0
    	std::cout << "Dimensionality Reduction - data acquisition STARTED" << std::endl;
    	xI=0;yI=0;y_inc=1;
    	logFile.open ("log.txt");
    	logFile << "Dimensionality Reduction LOGS\n";
#endif
    	return true;}
    void kuka_IK::updateHook(){
#if 0
    	//Dimensionality Reduction - data acquisition
    	int gridSize = 20;
    	if(xI < gridSize && yI < gridSize){
    		std::cout << std::endl << "============= xI=" << xI << ", yI = " << yI <<" ==============" << std::endl;
			//Generate Pose on Horizontal Grid
			geometry_msgs::Pose gridPointPose;
			gridPointPose.orientation.x = 0.0;
			gridPointPose.orientation.y = 0.0;
			gridPointPose.orientation.z = 0.0;
			gridPointPose.orientation.w = 1.0;

			gridPointPose.position.z = 0.77;
			gridPointPose.position.x = -0.55 + xI*0.01;
			gridPointPose.position.y = -0.15 + yI*0.01;

			msr_jntPosPort.read(jntPos);
			//std::cout << " KUKA-IK-component.cpp: jntPos" << std::endl;
			//for(int i = 0; i < 7; i++)  std::cout << jntPos[i] << " " ; std::cout << std::endl;
			std::cout << " KUKA-IK-component.cpp: commndedCartPos" << std::endl;
			std::cout << gridPointPose.position.x << " " << gridPointPose.position.y << " " << gridPointPose.position.z << std::endl;
			logFile << gridPointPose.position.x << "," << gridPointPose.position.y << "," << gridPointPose.position.z  << ", ";

			commndedPoseJntPos = std::vector<double>(7,0.0);

			if (!(KukaLWR_Kinematics::ikSolverIterative7DOF(jntPos, gridPointPose, commndedPoseJntPos))){
				cout << "lastCommandedPose cannot be achieved" << endl;
				for(int i = 0; i < 7; i++) {
					std::cout << commndedPoseJntPos[i] << " " ;
				}
				std::cout << std::endl;
				logFile << "IK Solver failed \n";
			}

			for(int i = 0; i < 7; i++)	logFile << commndedPoseJntPos[i] << "," ;
			logFile << "\n";

			//Sending out to trajectory_generator
			sensor_msgs::JointState tmpJntState;
			tmpJntState.position.clear();
			for(int i=0; i < 7; i++){
				tmpJntState.position.push_back(commndedPoseJntPos[i]);
			}
			//output_jntPosPort.write(tmpJntState);

			if ((yI == gridSize-1 && y_inc==1) || (yI == 0 && y_inc == -1) ) { y_inc=y_inc*-1;  xI++; }
			else yI = yI + y_inc;

    	}else{
    		std::cout << "Grid Scanning Done. Calling stopHook()" << std::endl;
    		logFile.close();
    		this->stop();
    	}
#endif
    }
    void kuka_IK::stopHook(){}
    void kuka_IK::cleanupHook(){}

}
