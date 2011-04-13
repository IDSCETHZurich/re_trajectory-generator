/*
 * VelocityProfile_NonZeroInit.cpp
 *
 *  Created on: Apr 7, 2011
 *      Author: demo
 */
#include "VelocityProfile_NonZeroInit.hpp"


namespace trajectory_generators {


using namespace RTT;
using namespace KDL;
using namespace std;

VelocityProfile_NonZeroInit::VelocityProfile_NonZeroInit(double maxAcc, double maxVel, double finalPos, double initPos, double initVel){
	//TODO: Perform checks

	// set Time scale to default
	log(Info) << "Building Velocity Profile with maxAc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") " << endlog();
	log(Info) << "------------------------------------------" << endlog();

	timeScale = 1;
	subProfileBuilder(maxAcc, maxVel, finalPos, initPos, initVel);
}

void VelocityProfile_NonZeroInit::subProfileBuilder(double maxAcc, double maxVel, double finalPos, double initPos, double initVel){


	// We need the distance (absolute) to be covered
	double Dp = abs(finalPos -  initPos) + 0.0001;
	// trajSign gives the direction of the trajectory
	// +1 for positive ((finalPos>initPos)
	double trajSign = 1.0;
	// -1 for negative ((finalPos<initPos)
	if (finalPos -  initPos < 0)
		trajSign = -1.0;

	if (Dp < 0.5* initVel*initVel / maxAcc){
	//slow down to zero and perform a new trajectory going back
		log(Info) << "Slow down to zero and perform a new trajectory going back " << endlog();
		std::vector<double> sp1;
		double T1 = abs(initVel)/maxAcc;
		double D1 = 0.5*initVel*T1;

		// Coeficients of the movement
		sp1.push_back(0.0); //t_0
		sp1.push_back(initPos);
		sp1.push_back(initVel);
		if (initVel < 0)
			sp1.push_back(maxAcc);
		else
			sp1.push_back(-maxAcc);
		//log(Info) << sp1 << endlog();
		subVelocityProfiles.push_back(sp1);
		//Duration will be set is this call
		subProfileBuilder(maxAcc, maxVel, finalPos, initPos+D1, 0.0);
		//TODO: Check the duration in this case. Might be wrong....
	//end of slow down to Zero and go back

	}else if ( Dp < (0.5/maxAcc)*(2*maxVel*maxVel - initVel*initVel)){
		log(Info) << "Triangular velocity profile " << endlog();
		//triangular velocity profile
		std::vector<double> sp1,sp2;
		//Adding the acceleration subProfile
		sp1.push_back(0.0); //t_0
		sp1.push_back(initPos);
		sp1.push_back(initVel);
		sp1.push_back(trajSign*maxAcc);
		subVelocityProfiles.push_back(sp1);

		//Adding the deceleration subProfile
		double maxVelProfile = sqrt(Dp*maxAcc + 0.5*initVel*initVel);
		double T1 = abs(trajSign*maxVelProfile-initVel)/maxAcc;

		sp2.push_back( T1 );
		sp2.push_back(initPos + 0.5*T1*(trajSign*maxVelProfile + initVel) );
		sp2.push_back( trajSign*maxVelProfile );
		sp2.push_back( -trajSign*maxAcc );
		subVelocityProfiles.push_back(sp2);

		duration = T1 + maxVelProfile/maxAcc;
		//log(Info) << sp1 << endlog();
		//log(Info) << sp2 << endlog();

	//end of triangular velocity profile
	}else{
		log(Info) << "Asymmetric trapezoidal velocity profile " << endlog();
		//Asymmetric trapezoidal velocity profile
		std::vector<double> sp1,sp2,sp3;
		//Adding the acceleration subProfile
		sp1.push_back( 0.0 ); //t_0
		sp1.push_back( initPos );
		sp1.push_back( initVel );
		sp1.push_back( trajSign*maxAcc );
		subVelocityProfiles.push_back(sp1);

		//Adding the constant velocity subProfile
		double T1 = abs(trajSign*maxVel-initVel)/maxAcc;
		double P1 = initPos + 0.5*T1*(trajSign*maxVel + initVel) ; // position at the start of the subProfile
		sp2.push_back( T1 );
		sp2.push_back( P1 );
		sp2.push_back( trajSign*maxVel );
		sp2.push_back( 0.0 );
		subVelocityProfiles.push_back(sp2);

		//Adding the Deceleration subProfile
		double T2 = trajSign * (1/maxVel) * (finalPos - P1 + (0.5*maxVel*maxVel)/(-trajSign*maxAcc));
		double P2 = P1 + (T2-T1)*trajSign*maxVel;
		sp3.push_back( T2 );
		sp3.push_back( P2 );
		sp3.push_back( trajSign*maxVel );
		sp3.push_back( -trajSign*maxAcc );
		subVelocityProfiles.push_back(sp3);

		duration = T2 + maxVel/maxAcc;

		//log(Info) << sp1 << endlog();
		//log(Info) << sp2 << endlog();
		//log(Info) << sp3 << endlog();
	//End of Asymmetric trapezoidal velocity profile
	}
}

VelocityProfile_NonZeroInit::~VelocityProfile_NonZeroInit() {
	// TODO Auto-generated destructor
}

bool VelocityProfile_NonZeroInit::setDuration(double newDuration){
	//Can be called more than one time
	timeScale = duration/newDuration;
	return true;
}

double VelocityProfile_NonZeroInit::getDuration(){
	return this->duration;
}

double VelocityProfile_NonZeroInit::getPos(double time){
	for( int i = 0 ; i < (int)subVelocityProfiles.size()-1 ; i++ ){
		if(time > subVelocityProfiles[i][0] && time <=  subVelocityProfiles[i+1][0] ){
			return subVelocityProfiles[i][1] \
					+ timeScale * subVelocityProfiles[i][2] * (time -  subVelocityProfiles[i][0]) \
					+ 0.5 * timeScale * timeScale * subVelocityProfiles[i][3]*(time -  subVelocityProfiles[i][0])*(time -  subVelocityProfiles[i][0]);
		}

	}
	// if we are here, time is higher than duration so we set the output to the last requested position
	int lastElement = subVelocityProfiles.size()-1;
	if (time > duration) time = duration;

	return subVelocityProfiles[lastElement][1] \
						+ subVelocityProfiles[lastElement][2] * (time -  subVelocityProfiles[lastElement][0]) \
						+ 0.5 * subVelocityProfiles[lastElement][3]*(time -  subVelocityProfiles[lastElement][0])*(time -  subVelocityProfiles[lastElement][0]);
}

double VelocityProfile_NonZeroInit::getVel(double time){
	for( int i = 0 ; i < (int)subVelocityProfiles.size()-1 ; i++ ){
		if(time > subVelocityProfiles[i][0] && time <=  subVelocityProfiles[i+1][0] ){
			return timeScale * subVelocityProfiles[i][2]  \
					+ timeScale * timeScale * subVelocityProfiles[i][3]*(time -  subVelocityProfiles[i][0]);
		}

	}
	// if we are here, time is higher than duration so we set the output to the final velocity
	int lastElement = subVelocityProfiles.size()-1;
	return  subVelocityProfiles[lastElement][2]  \
				+ subVelocityProfiles[lastElement][3]*(duration -  subVelocityProfiles[lastElement][0]);
}

}//end of namespace
