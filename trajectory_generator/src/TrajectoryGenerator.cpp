/***************************************************************************

    File:           TrajectoryGenerator.cpp
    Author(s):      Gajamohan Mohanarajah/Francisco Ramos
    Affiliation:    IDSC - ETH Zurich
    e-mail:         gajan@ethz.ch/framosde@ethz.ch
    Start date:	    7th April 2011
    Last update:    11th May 2011

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

#include "TrajectoryGenerator.hpp"
#include <ocl/Component.hpp>

ORO_CREATE_COMPONENT_TYPE();
ORO_LIST_COMPONENT_TYPE(trajectory_generator::TrajectoryGenerator);

namespace trajectory_generator
{
    using namespace RTT;
    using namespace KDL;
    using namespace std;

    TrajectoryGenerator::TrajectoryGenerator(std::string name)
        : TaskContext(name,PreOperational)
    {
        //Creating TaskContext

        //Adding InputPorts

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

        this->addOperation("updateTG", &TrajectoryGenerator::updateTG, this, OwnThread);

        lastCommandedPoseJntPos = std::vector<double>(7,0.0);
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

    	//log(Info) << "nAxes = " << num_axes << endlog();

    	return true;

    }

    bool TrajectoryGenerator::startHook()
    {
    	std::cout << "TrajectoryGenerator::Trajectory generator running" << std::endl;
    	return true;
    }



    bool TrajectoryGenerator::generateNewVelocityProfilesJntPosInput(RTT::base::PortInterface* portInterface)
    {
    	time_passed = os::TimeService::Instance()->secondsSince(time_begin);
    	//log(Info) << "a new jnt pose arrived" << endlog();
#if DEBUG
    	cout << "a new jnt pose arrived" << endl;
#endif
    	input_jntPosPort.read(cmdJntState);
    	lastCommandedPoseJntPos = cmdJntState.position;

    	for(int i=0; i < 7; i++){
#if DEBUG
    		cout << "Joint " << i << " : " << lastCommandedPoseJntPos[i] << " /// ";
#endif
    		if(lastCommandedPoseJntPos[i]<p_min[i] || lastCommandedPoseJntPos[i]>p_max[i]){
    			//log(Info) << "Commanded joint position out of bounds" << endlog();
    			cout << "Commanded joint position out of bounds" << lastCommandedPoseJntPos[i] << endl;
    			return false;
    		}

    	}
#if DEBUG
    	cout << endl;
#endif

    	//Create joint specific velocity profiles
    	double maxDuration = 0.0;
    	std::vector<double> jntPos = std::vector<double>(7,0.0);
    	std::vector<double> finVel = std::vector<double>(7,0.0);

    	msr_jntPosPort.read(jntPos);


    	if ((int)motionProfile.size() == 0){//Only for the first run
    		for(int i = 0; i < (int)num_axes; i++)
			{
    			jntVel[i] = 0.0;
    		}
    	}else{

    		bool valid = false;

    		for(int i = 0; i < (int)motionProfile.size(); i++)
    		{
    			jntVel[i] = motionProfile[i].Vel(time_passed);
    			jntPos[i] = motionProfile[i].Pos(time_passed);
    			// Experimental: adding final velocities to the trajectory
    			// We calculate if the final state is reachable within the kinematic limits
    			// If not, we ask for a new final velocity until we get a valid value
    			valid = false;
    			while (!valid)
    			{
        			finVel[i] = v_max[i]*(-0.5 + 1*((double)rand()/(double)RAND_MAX));
        			if ((lastCommandedPoseJntPos[i] + 0.5*finVel[i]*abs(finVel[i]) < p_max[i]) &&
        				(lastCommandedPoseJntPos[i] + 0.5*finVel[i]*abs(finVel[i]) > p_min[i]))
        				valid = true;
    			}
 	   		}
     	}

    	motionProfile.clear();

    	//TODO: Check dimensions
    	for(int i = 0; i < (int)lastCommandedPoseJntPos.size(); i++){
    		motionProfile.push_back(VelocityProfile_NonZeroInit(v_max[i], a_max[i]));
//    		motionProfile[i].SetProfile(jntPos[i], lastCommandedPoseJntPos[i], jntVel[i]);
			motionProfile[i].SetProfile(jntPos[i], lastCommandedPoseJntPos[i], jntVel[i], finVel[i]);
    		if(motionProfile[i].Duration() > maxDuration )
    			maxDuration = motionProfile[i].Duration();
#if DEBUG
    	cout << "**********After Joint " << i << " MaxDuration " << maxDuration << endl;
#endif
    	}

    	//Do sync
/////////////////
    	for(int i = 0; i < (int)lastCommandedPoseJntPos.size(); i++){
    		motionProfile[i].SetProfileDuration(maxDuration);
    	}
/////////////////

    	//Set times
    	time_begin = os::TimeService::Instance()->getTicks();
#if DEBUG
    	cout << "A new set of motion profiles were successfully created." << endl;
#endif
    	return true;

    }


    bool TrajectoryGenerator::updateTG(void){
    	if (motionProfile.size()==7){
			time_passed = os::TimeService::Instance()->secondsSince(time_begin);
			log(Info) << time_passed << endlog();
			jntPosCmd.clear();
			jntState.position.clear();
			for(int i = 0; i < (int)motionProfile.size(); i++){
				jntPosCmd.push_back(motionProfile[i].Pos(time_passed));
				jntState.position.push_back(motionProfile[i].Pos(time_passed));
			}
			output_jntPosPort_toROS.write(jntState);
#if DEBUG
			log(Info) << jntPosCmd[0] << " " << jntPosCmd[1] << " " << jntPosCmd[2] << " "
										<< jntPosCmd[3] << " " << jntPosCmd[4] << " " << jntPosCmd[5] << " "
										<< jntPosCmd[6] << endlog();
#endif
			output_jntPosPort.write(jntPosCmd);
			return true;
    	}else{
    		return false;
    	}
    }


    void TrajectoryGenerator::updateHook()
    {
    	updateTG();
    }


    void TrajectoryGenerator::stopHook()
    {
    }

    void TrajectoryGenerator::cleanupHook()
    {

    }


}//namespace



