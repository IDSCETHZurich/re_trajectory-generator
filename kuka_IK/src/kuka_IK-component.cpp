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
        
        sensor_msgs::JointState tmpJntState;
        tmpJntState.position.clear();
        for(int i=0; i < 7; i++){
    		tmpJntState.position.push_back(commndedPoseJntPos[i]);
    	}

    	output_jntPosPort.write(tmpJntState);
    	return true;

    }

    bool kuka_IK::configureHook(){return true;}
    bool kuka_IK::startHook(){return true;}
    void kuka_IK::updateHook(){}
    void kuka_IK::stopHook(){}
    void kuka_IK::cleanupHook(){}

}
