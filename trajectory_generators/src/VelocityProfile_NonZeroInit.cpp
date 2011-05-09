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
	cout << "Building Velocity Profile with maxAc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") " << endl;
	cout << "------------------------------------------" << endl;

	timeScale = 1;
	duration = subProfileBuilder(maxAcc, maxVel, finalPos, initPos, initVel, 0.0); // Initial time set to zero for new trajectories
}

double VelocityProfile_NonZeroInit::subProfileBuilder(double maxAcc, double maxVel, double finalPos, double initPos, double initVel, double initTime){

	// We need the distance (absolute) to be covered
	double Dp = abs(finalPos -  initPos), tmpDuration;
	// trajSign gives the direction of the trajectory
	// +1 for positive ((finalPos>initPos)
	double trajSign = 1.0;
	// -1 for negative ((finalPos<initPos)
	if (finalPos -  initPos < 0)
		trajSign = -1.0;

	if(abs(finalPos-initPos) < 0.0001 && abs(initVel) < 0.0001 ){
		std::vector<double> sp1;
		sp1.push_back(initTime);
		sp1.push_back(initPos);
		sp1.push_back(0.0);
		sp1.push_back(0.0);

		subVelocityProfiles.push_back(sp1);

		tmpDuration = 0.0001;
		return tmpDuration;
	}

	if (Dp < 0.5* initVel*initVel / maxAcc){
	//slow down to zero and perform a new trajectory going back
		log(Info) << "Slow down to zero and perform a new trajectory going back " << endlog();
		std::vector<double> sp1;
		double T1 = abs(initVel)/maxAcc;
		double D1 = 0.5*initVel*T1;

		// Coefficients of the movement
		sp1.push_back(initTime); //t_0
		sp1.push_back(initPos);
		sp1.push_back(initVel);
		if (initVel < 0)
			sp1.push_back(maxAcc);
		else
			sp1.push_back(-maxAcc);
		subVelocityProfiles.push_back(sp1);
		//Duration will be set is this call
		tmpDuration = T1 + subProfileBuilder(maxAcc, maxVel, finalPos, initPos+D1, 0.0, T1 + initTime);
		//end of slow down to Zero and go back

	}else if ( Dp < (0.5/maxAcc)*(2*maxVel*maxVel - initVel*initVel)){
		log(Info) << "Triangular velocity profile " << endlog();
		//triangular velocity profile
		std::vector<double> sp1,sp2;
		//Adding the acceleration subProfile
		sp1.push_back(initTime); //t_0
		sp1.push_back(initPos);
		sp1.push_back(initVel);
		sp1.push_back(trajSign*maxAcc);
		subVelocityProfiles.push_back(sp1);

		//Adding the deceleration subProfile
		double maxVelProfile = sqrt(Dp*maxAcc + 0.5*initVel*initVel);
		double T1 =  abs(trajSign*maxVelProfile-initVel)/maxAcc;

		sp2.push_back( T1 + initTime );
		sp2.push_back(initPos + 0.5*T1*(trajSign*maxVelProfile + initVel) );
		sp2.push_back( trajSign*maxVelProfile );
		sp2.push_back( -trajSign*maxAcc );
		subVelocityProfiles.push_back(sp2);

		tmpDuration = T1 + maxVelProfile/maxAcc;

	//end of triangular velocity profile
	}else{
		log(Info) << "Asymmetric trapezoidal velocity profile " << endlog();
		//Asymmetric trapezoidal velocity profile
		std::vector<double> sp1,sp2,sp3;
		//Adding the acceleration subProfile
		sp1.push_back( initTime ); //t_0
		sp1.push_back( initPos );
		sp1.push_back( initVel );
		sp1.push_back( trajSign*maxAcc );
		subVelocityProfiles.push_back(sp1);

		//Adding the constant velocity subProfile
		double T1 = (trajSign*maxVel-initVel)/(trajSign*maxAcc);
		double P1 = initPos + 0.5*T1*(trajSign*maxVel + initVel) ; // position at the start of the subProfile
		sp2.push_back( T1 + initTime );
		sp2.push_back( P1 );
		sp2.push_back( trajSign*maxVel );
		sp2.push_back( 0.0 );
		subVelocityProfiles.push_back(sp2);

		//Adding the Deceleration subProfile
		double T2 =  1/(trajSign*maxVel) * (finalPos - P1 + T1*trajSign*maxVel - (0.5*maxVel*maxVel)/(trajSign*maxAcc));
		double P2 = P1 + (T2-T1)*trajSign*maxVel;
		sp3.push_back( T2 + initTime );
		sp3.push_back( P2 );
		sp3.push_back( trajSign*maxVel );
		sp3.push_back( -trajSign*maxAcc );
		subVelocityProfiles.push_back(sp3);

		tmpDuration = T2 + maxVel/maxAcc;

	//End of Asymmetric trapezoidal velocity profile
	}
	return tmpDuration;
}

VelocityProfile_NonZeroInit::~VelocityProfile_NonZeroInit() {
	// TODO Auto-generated destructor
}

bool VelocityProfile_NonZeroInit::setDuration(double newDuration){
	//Can be called more than one time
	timeScale = 1; //duration/newDuration;
	return true;
}

double VelocityProfile_NonZeroInit::getDuration(){
	return this->duration;
}

double VelocityProfile_NonZeroInit::getPos(double time){
	for( int i = 0 ; i < (int)subVelocityProfiles.size()-1 ; i++ ){
		if(time > subVelocityProfiles[i][0]/timeScale && time <=  subVelocityProfiles[i+1][0]/timeScale ){
			return subVelocityProfiles[i][1] \
					+ timeScale * subVelocityProfiles[i][2] * (time -  subVelocityProfiles[i][0]/timeScale) \
					+ 0.5 * timeScale * timeScale * subVelocityProfiles[i][3]*(time -  subVelocityProfiles[i][0]/timeScale)*(time -  subVelocityProfiles[i][0]/timeScale);
		}

	}
	// if we are here, time is in the last piece or higher than duration so we set the output to the last requested position
	int lastElement = subVelocityProfiles.size()-1;
	if (time > duration/timeScale) time = duration/timeScale;

	return subVelocityProfiles[lastElement][1] \
						+ timeScale *subVelocityProfiles[lastElement][2] * (time -  subVelocityProfiles[lastElement][0]/timeScale) \
						+ 0.5 * timeScale * timeScale * subVelocityProfiles[lastElement][3]*(time -  subVelocityProfiles[lastElement][0]/timeScale)*(time -  subVelocityProfiles[lastElement][0]/timeScale);
}

double VelocityProfile_NonZeroInit::getVel(double time){
	for( int i = 0 ; i < (int)subVelocityProfiles.size()-1 ; i++ ){
		if(time > subVelocityProfiles[i][0]/timeScale && time <=  subVelocityProfiles[i+1][0]/timeScale ){
			return timeScale * subVelocityProfiles[i][2]  \
					+ timeScale * timeScale * subVelocityProfiles[i][3]*(time -  subVelocityProfiles[i][0]/timeScale);
		}

	}
	// if we are here, time is in the last piece or higher than duration so we set the output to the final velocity
	int lastElement = subVelocityProfiles.size()-1;
	if (time > duration/timeScale) time = duration/timeScale;
	return  timeScale *subVelocityProfiles[lastElement][2]  \
				+ timeScale * timeScale * subVelocityProfiles[lastElement][3]*(time -  subVelocityProfiles[lastElement][0]/timeScale);
}

}//end of namespace
