/*
 * VelocityProfile_NonZeroInit.cpp
 *
 *  Created on: Apr 7, 2011
 *      Author: demo
 */

#include <rtt/RTT.hpp>

#include "VelocityProfile_NonZeroInit.hpp"


namespace trajectory_generators {

VelocityProfile_NonZeroInit::VelocityProfile_NonZeroInit(double maxAcc, double maxVel, double finalPos, double initPos, double initVel){
	//TODO: Perform checks
	subProfileBuilder(maxAcc, maxVel, finalPos, initPos, initVel);
}

void VelocityProfile_NonZeroInit::subProfileBuilder(double maxAcc, double maxVel, double finalPos, double initPos, double initVel){
	// TODO Auto-generated constructor stub
	double Dp = abs(finalPos -  initPos) + 0.001;
	if (finalPos -  initPos < 0) initVel = -initVel;

	if (Dp < 0.5* initVel*initVel / maxAcc){
	//slow down to zero and go back
		std::vector<double> sp1;
		double T1 = abs(initVel)/maxAcc;
		double D1 = 0.5*initVel*T1;
		sp1.push_back(0.0); //t_0
		sp1.push_back(initPos);
		sp1.push_back(initVel);
		if ( initVel < 0 )
			sp1.push_back(maxAcc);
		else
			sp1.push_back(-maxAcc);
		subVelocityProfiles.push_back(sp1);
		//Duration will be set is this call
		subProfileBuilder(maxAcc, maxVel, finalPos, initPos-D1, 0.0);
	//end of slow down to Zero and go back

	}else if ( Dp < (0.5/maxAcc)*(2*maxVel*maxVel - initVel*initVel)){
	//triangular velocity profile
		std::vector<double> sp1,sp2;
		//Adding the acceleration subProfile
		sp1.push_back(0.0); //t_0
		sp1.push_back(initPos);
		sp1.push_back(initVel);
		sp1.push_back(maxAcc);
		subVelocityProfiles.push_back(sp1);

		//Adding the deceleration subProfile
		double maxVelProfile = sqrt(Dp*maxAcc - 0.5*initVel*initVel) - initVel;
		double T1 = (maxVelProfile-initVel)/maxAcc;
		sp2.push_back( T1 );
		sp2.push_back( 0.5*T1*(initVel+maxVelProfile) );
		sp2.push_back( maxVelProfile );
		sp2.push_back( -maxAcc );
		subVelocityProfiles.push_back(sp2);

		duration = T1 + maxVelProfile/maxAcc;
	//end of triangular velocity profile
	}else{
	//Asymmetric trapezoidal velocity profile
		std::vector<double> sp1,sp2,sp3;
		//Adding the acceleration subProfile
		sp1.push_back(0.0); //t_0
		sp1.push_back(initPos);
		sp1.push_back(initVel);
		sp1.push_back(maxAcc);
		subVelocityProfiles.push_back(sp1);

		//Adding the constant velocity subProfile
		double T1 = (maxVel-initVel)/maxAcc;
		double P1 = 0.5*T1*(initVel+maxVel); // position at the start of the subProfile
		sp2.push_back( T1 );
		sp2.push_back( P1 );
		sp2.push_back( maxVel );
		sp2.push_back( 0.0 );
		subVelocityProfiles.push_back(sp2);

		//Adding the Deceleration subProfile
		double T2 = (1/maxVel) * (Dp + 0.5*initVel*(initVel-2*maxVel)/maxAcc);
		double P2 = P1 + (T2-T1)*maxVel;
		sp3.push_back( T2 );
		sp3.push_back( P2 );
		sp3.push_back( maxVel );
		sp3.push_back( -maxAcc );
		subVelocityProfiles.push_back(sp3);
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
		if(time > subVelocityProfiles[i][0] && time <=  subVelocityProfiles[i][0] ){
			return subVelocityProfiles[i][1] \
					+ timeScale * subVelocityProfiles[i][2] * (time -  subVelocityProfiles[i][0]) \
					+ 0.5 * timeScale * timeScale * subVelocityProfiles[i][3]*(time -  subVelocityProfiles[i][0])*(time -  subVelocityProfiles[i][0]);
		}

	}
	return 0.0;
}


}//end of namespace
