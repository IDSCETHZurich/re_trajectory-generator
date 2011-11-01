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

        this->addProperty("doSync",doSync).doc("Do synchronization");
        this->addProperty("addFinalVel",addFinalVel).doc("Incorporate final velocities");

        this->addOperation("updateTG", &TrajectoryGenerator::updateTG, this, OwnThread);

        lastCommandedPoseJntPos = std::vector<double>(7,0.0);
        lastCommandedPoseJntVel = std::vector<double>(7,0.0);
        jntPos = std::vector<double>(7,0.0);
        jntVel = std::vector<double>(7,0.0);

        jntState.header.frame_id = "arm_0_link";
        jntState.name.push_back("arm_1_joint");
        jntState.name.push_back("arm_2_joint");
        jntState.name.push_back("arm_3_joint");
        jntState.name.push_back("arm_4_joint");
        jntState.name.push_back("arm_5_joint");
        jntState.name.push_back("arm_6_joint");
        jntState.name.push_back("arm_7_joint");

        timeLogger.open("timeLog.txt");

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
		if(doSync==true && addFinalVel==true){
			doSync = false;
			cout << "Sync with non zero final velocities is not yet implemented" << endl;
		}
    	std::cout << "TrajectoryGenerator::Trajectory generator running" << std::endl;

    	return true;
    }



    bool TrajectoryGenerator::generateNewVelocityProfilesJntPosInput(RTT::base::PortInterface* portInterface)
    {
    	//Create joint specific velocity profiles
    	double maxDuration = 0.0;
    	double p_aux = 0.0;

    	time_passed = os::TimeService::Instance()->secondsSince(time_begin);
#if DEBUG
    	cout << "a new jnt pose arrived" << endl;
#endif
    	input_jntPosPort.read(cmdJntState);
    	lastCommandedPoseJntPos = cmdJntState.position;
    	lastCommandedPoseJntVel = cmdJntState.velocity;


    	for(int i=0; i < 7; i++){
    		if(lastCommandedPoseJntPos[i]<p_min[i] || lastCommandedPoseJntPos[i]>p_max[i]){
    			//log(Info) << "Commanded joint position out of bounds" << endlog();
    			cout << "Commanded joint position out of bounds " << lastCommandedPoseJntPos[i] << endl;
    			return false;
    		}

    		// We see if the final state is reachable within the kinematic limits
    		// First we calculate the margin of position that we have
    		p_aux = min(abs(lastCommandedPoseJntPos[i]-p_min[i]),abs(p_max[i]-lastCommandedPoseJntPos[i]));
    		// Then, if the motion due to deceleration/acceleration needed to stop_the_robot/reach_the_final_vel
    		// is higher than the distance that we have to the position limits, final state cannot be achieved
    		if (p_aux < 0.5*lastCommandedPoseJntVel[i]*lastCommandedPoseJntVel[i]/a_max[i]){
    			cout << "Commanded final velocity out of bounds " << lastCommandedPoseJntVel[i] << endl;
    			return false;
    		}
    	}


    	if ((int)motionProfile.size() == 0){//Only for the first run
    		for(int i = 0; i < (int)num_axes; i++)
			{
    			jntVel[i] = 0.0;
    			msr_jntPosPort.read(jntPos);
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
    	for(int i = 0; i < (int)lastCommandedPoseJntPos.size(); i++){
    		motionProfile.push_back(VelocityProfile_NonZeroInit(v_max[i], a_max[i]));
    		if(!addFinalVel)
    			motionProfile[i].SetProfile(jntPos[i], lastCommandedPoseJntPos[i], jntVel[i]);
    		else
    			motionProfile[i].SetProfile(jntPos[i], lastCommandedPoseJntPos[i], jntVel[i], lastCommandedPoseJntVel[i]);
    		if(motionProfile[i].Duration() > maxDuration )
    			maxDuration = motionProfile[i].Duration();
    	}

    	timeLogger << maxDuration << endl;

    	//Do sync
    	if(doSync){
			for(int i = 0; i < (int)lastCommandedPoseJntPos.size(); i++){
				motionProfile[i].SetProfileDuration(maxDuration);
			}
    	}

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
			jntState.header.stamp = ros::Time::now();
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
    	timeLogger.close();
    }

    void TrajectoryGenerator::cleanupHook()
    {

    }

}//namespace



