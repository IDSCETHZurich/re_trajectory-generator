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



        //addition from call. traj. contrl.
        this->ports()->addEventPort("newTrajectoryFromROS", iprt_trajectory,
                                    boost::bind(&TrajectoryGenerator::evNewTrajectory, this, _1));

        this->ports()->addPort("characterDrawnToROS", oprt_character_done);

        this->trajectory_iterator = this->trajectory.points.begin();

    }

    TrajectoryGenerator::~TrajectoryGenerator()
    {
    }

	int TrajectoryGenerator::jntVelScaling(sensor_msgs::JointState (&robotState))
	{
		std::vector<double> vM (7,0.0);
		std::vector<double> scale (7,1.0);
		double p_left, p_right;
		unsigned int i;

		for(i=0 ; i<robotState.velocity.size() ; i++)
		{
			// Limits for applying constant velocity limit or curve
			p_left = p_min[i] + 0.5*v_max[i]*v_max[i]/a_max[i];
			p_right = p_max[i] - 0.5*v_max[i]*v_max[i]/a_max[i];
			// Left phase-plane constraint
			if (robotState.position[i] < p_left)
				vM[i] = sign(robotState.velocity[i])*(sqrt(2*(p_left - p_min[i])*a_max[i]));

			// Constant Velocity Limits
			else if (robotState.position[i] < p_right)
				vM[i] = sign(robotState.velocity[i])*v_max[i];

			// Right phase-plane constraint
			else
				vM[i] = sign(robotState.velocity[i])*(sqrt(2*(p_max[i] - p_right)*a_max[i]));

			// If the requested velocity is higher than maximum, we calculate the scaling factor
			if (abs(robotState.velocity[i]) > abs(vM[i]))
			{
				scale[i] = robotState.velocity[i]/vM[i];
				cout << "The requested velocity is higher than it should. Rescaling from "
						<< robotState.velocity[i] << " to " << vM[i] << " by a factor " << scale[i] << endl;
			}
		}

		// Find maximum scale
		double scaleM = 1.0;
		for(i=0 ; i<scale.size() ; i++)
			if (scale[i]>scaleM)
				scaleM = scale[i];

//		// See when the velocities have to be scaled
//		cout << "vel:  " ;
//		for(int i=0; i<7; i++)
//			cout << " " << robotState.velocity[i];
//		cout << endl;
//
//		cout << "pos:  " ;
//		for(int i=0; i<7; i++)
//			cout << " " << robotState.position[i];
//		cout << endl;
//
//		cout << "vM:  " ;
//		for(int i=0; i<7; i++)
//			cout << " " << vM[i];
//		cout << endl;
//
//		cout << "scale (vector):  " ;
//		for(int i=0; i<7; i++)
//			cout << " " << scale[i];
//		cout << endl;
//
//		cout << "scale:" << scaleM << endl;


		// Recalculate velocities in the Robot Space if the scale factor is greater than one
		if (scaleM > 1.0)
		{
			for(i=0 ; i<robotState.velocity.size() ; i++)
				robotState.velocity[i] /= scaleM;



			return 1;
		}
		else
			return 0;

		// We never should reach this point
		return -1;
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
    	state = time_opt;

    	//Create joint specific velocity profiles
    	maxDuration = 0.0;
    	double p_aux = 0.0;

    	time_passed = os::TimeService::Instance()->secondsSince(time_begin);

#if DEBUG
    	cout << "a new jnt pose arrived" << endl;
#endif
    	input_jntPosPort.read(cmdJntState);

    	jntVelScaling(cmdJntState);

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

    	if (state == time_opt)
    	{
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
    	else if (state == iterating)
    	{
    		if(getNextPointOnCalligraphyTrajectory())
    			return true;
    		else
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

    void TrajectoryGenerator::evNewTrajectory(RTT::base::PortInterface* portInterface)
    {
    	state = iterating;

      if (iprt_trajectory.read(this->trajectory) == RTT::NewData)
      {
        //std::cout << "New trajectory received!" << std::endl;

        this->trajectory_iterator = this->trajectory.points.begin();

        //std::cout << "Start drawing" << std::endl;
      }
      else
        std::cout << "ERROR: no new trajectory" << std::endl;
    }


      bool TrajectoryGenerator::getNextPointOnCalligraphyTrajectory()
      {

        if (this->trajectory.points.size() == 0)
          return false;

        // the last point in the trajectory has been sent...
        if (this->trajectory_iterator == this->trajectory.points.end())
        {

          // send finished signal
          std_msgs::Bool finished;
          finished.data = true;
          oprt_character_done.write(finished);

          //std::cout << "Drawing finished" << std::endl;

          // reset trajectory
          this->trajectory.points.clear();
          this->trajectory_iterator = this->trajectory.points.begin();

          return false;
        }

        sensor_msgs::JointState jointstate;

        jointstate.position = this->trajectory_iterator->positions;
        jointstate.velocity = std::vector<double>(7, 0);
        jointstate.header.stamp = ros::Time::now();

        output_jntPosPort.write(this->trajectory_iterator->positions);

        this->trajectory_iterator++;

        return true;
      }


      /*
      // temporarily disable the joint synchronisation of the trajectory generator while drawing
      RTT::Property<bool> doSync = this->getPeer("trajectoryGenerator")->properties()->getProperty("doSync");
      doSync.set(false);
      */

}//namespace



