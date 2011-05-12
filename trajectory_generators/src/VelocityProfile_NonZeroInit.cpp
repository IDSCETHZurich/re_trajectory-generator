/*
 * VelocityProfile_NonZeroInit.cpp
 *
 *  Created on: Apr 7, 2011
 *     Authors: Francisco Ramos
 *              Gajamohan Mohanarajah
 */

#include "VelocityProfile_NonZeroInit.hpp"

namespace trajectory_generators {

// constructs motion profile class with <maxvel> and <maxacc> as parameters of the trajectory.
VelocityProfile_NonZeroInit::VelocityProfile_NonZeroInit(double _maxvel, double _maxacc):
        duration(0), timeScale(0), maxVel(_maxvel), maxAcc(_maxacc),
        initPos(0), finalPos(0), initVel(0), initTime(0)

{}


// This definition of SetProfile function is needed for compatibility with the VelocityProfile interface
void VelocityProfile_NonZeroInit::SetProfile(double pos1,double pos2)
{
	initPos = pos1;
	finalPos = pos2;
	initVel = 0.0;
	initTime = 0.0;
    //TODO: Clear no longer needed verbose items when debugged
	cout << "Building Velocity Profile with maxAcc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") " << endl;
	cout << "------------------------------------------" << endl;

	duration = SubProfileBuilder(finalPos, initPos, initVel, initTime);
}


// This definition of SetProfile is added for completeness
bool VelocityProfile_NonZeroInit::SetProfile(double pos1,double pos2, double _inivel)
{
	//Precondition
	if (_inivel > maxVel)
	{
		cout << "VelocityProfile_NonZeroInit::SetProfile: Initial velocity higher than maximum" << endl;
		cout << "Profile will not be built" << endl;
		return false;
	}
	initPos = pos1;
	finalPos = pos2;
	initVel = _inivel;
	initTime = 0.0;
    //TODO: Clear no longer needed verbose items when debugged
	cout << "Building Velocity Profile with maxAcc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") " << endl;
	cout << "------------------------------------------" << endl;

	duration = SubProfileBuilder(finalPos, initPos, initVel, initTime);

	return true;
}


// This is the recommended use for SetProfile function
bool VelocityProfile_NonZeroInit::SetProfile(double pos1,double pos2, double _inivel, double _initime)
{
	//Precondition
	if (_inivel > maxVel)
	{
		cout << "VelocityProfile_NonZeroInit::SetProfile: Initial velocity higher than maximum" << endl;
		cout << "Profile will not be built" << endl;
		return false;
	}else if (_initime < 0)
	{
		cout << "VelocityProfile_NonZeroInit::SetProfile: Initial time cannot be negative" << endl;
		cout << "Profile will not be built" << endl;
		return false;
	}

    initPos = pos1;
    finalPos = pos2;
    initVel = _inivel;
    initTime = _initime;
	// set Time scale to default
	timeScale = 1;
    //TODO: Clear no longer needed verbose items when debugged
	cout << "Building Velocity Profile with maxAcc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") " << endl;
	cout << "------------------------------------------" << endl;

	duration = SubProfileBuilder(finalPos, initPos, initVel, initTime); // Initial time set to zero for new trajectories

	return true;
}


double VelocityProfile_NonZeroInit::SubProfileBuilder(double fPos, double iPos, double iVel, double iTime)
{

	// We need the distance (absolute) to be covered
	double Dp = abs(fPos -  iPos), tmpDuration;
	// trajSign gives the direction of the trajectory
	// +1 for positive ((fPos>iPos)
	double trajSign = 1.0;
	// -1 for negative ((fPos<iPos)
	if (fPos -  iPos < 0)
		trajSign = -1.0;

	if(abs(fPos-iPos) < 0.0001 && abs(iVel) < 0.0001 ){
		std::vector<double> sp1;
		sp1.push_back(iTime);
		sp1.push_back(iPos);
		sp1.push_back(0.0);
		sp1.push_back(0.0);

		subVelProfiles.push_back(sp1);

		tmpDuration = 0.0001;
		return tmpDuration;
	}

	if (Dp < 0.5* iVel*iVel / maxAcc){
	//slow down to zero and perform a new trajectory going back
		log(Info) << "Slow down to zero and perform a new trajectory going back " << endlog();
		std::vector<double> sp1;
		double T1 = abs(iVel)/maxAcc;
		double D1 = 0.5*iVel*T1;

		// Coefficients of the movement
		sp1.push_back(iTime); //t_0
		sp1.push_back(iPos);
		sp1.push_back(iVel);
		if (iVel < 0)
			sp1.push_back(maxAcc);
		else
			sp1.push_back(-maxAcc);
		subVelProfiles.push_back(sp1);
		//Duration will be set is this call
		tmpDuration = T1 + SubProfileBuilder(fPos, iPos+D1, 0.0, T1 + iTime);
		//end of slow down to Zero and go back

	}else if ( Dp < (0.5/maxAcc)*(2*maxVel*maxVel - iVel*iVel)){
		log(Info) << "Triangular velocity profile " << endlog();
		//triangular velocity profile
		std::vector<double> sp1,sp2;
		//Adding the acceleration subProfile
		sp1.push_back(iTime); //t_0
		sp1.push_back(iPos);
		sp1.push_back(iVel);
		sp1.push_back(trajSign*maxAcc);
		subVelProfiles.push_back(sp1);

		//Adding the deceleration subProfile
		double maxVelProfile = sqrt(Dp*maxAcc + 0.5*iVel*iVel);
		double T1 =  abs(trajSign*maxVelProfile-iVel)/maxAcc;

		sp2.push_back( T1 + iTime );
		sp2.push_back(iPos + 0.5*T1*(trajSign*maxVelProfile + iVel) );
		sp2.push_back( trajSign*maxVelProfile );
		sp2.push_back( -trajSign*maxAcc );
		subVelProfiles.push_back(sp2);

		tmpDuration = T1 + maxVelProfile/maxAcc;

	//end of triangular velocity profile
	}else{
		log(Info) << "Asymmetric trapezoidal velocity profile " << endlog();
		//Asymmetric trapezoidal velocity profile
		std::vector<double> sp1,sp2,sp3;
		//Adding the acceleration subProfile
		sp1.push_back( iTime ); //t_0
		sp1.push_back( iPos );
		sp1.push_back( iVel );
		sp1.push_back( trajSign*maxAcc );
		subVelProfiles.push_back(sp1);

		//Adding the constant velocity subProfile
		double T1 = (trajSign*maxVel-iVel)/(trajSign*maxAcc);
		double P1 = iPos + 0.5*T1*(trajSign*maxVel + iVel) ; // position at the start of the subProfile
		sp2.push_back( T1 + iTime );
		sp2.push_back( P1 );
		sp2.push_back( trajSign*maxVel );
		sp2.push_back( 0.0 );
		subVelProfiles.push_back(sp2);

		//Adding the Deceleration subProfile
		double T2 =  1/(trajSign*maxVel) * (fPos - P1 + T1*trajSign*maxVel - (0.5*maxVel*maxVel)/(trajSign*maxAcc));
		double P2 = P1 + (T2-T1)*trajSign*maxVel;
		sp3.push_back( T2 + iTime );
		sp3.push_back( P2 );
		sp3.push_back( trajSign*maxVel );
		sp3.push_back( -trajSign*maxAcc );
		subVelProfiles.push_back(sp3);

		tmpDuration = T2 + maxVel/maxAcc;

	//End of Asymmetric trapezoidal velocity profile
	}
	return tmpDuration;
}


void VelocityProfile_NonZeroInit::SetProfileDuration(double newDuration)
{
    // TODO: Synchronization has not been implemented yet because of v_i
	timeScale = 1; //duration/newDuration;
    if (timeScale > 1)
        return; // Cannot go below time optimal maneuver
    for (int i=0 ; i < (int)subVelProfiles.size() ; i++ ){
        subVelProfiles[i][0] /= timeScale;
        subVelProfiles[i][2] *= timeScale;
        subVelProfiles[i][3] *= timeScale*timeScale;
    }

}


// This function is defined for compatibility with VelocityProfile class
void VelocityProfile_NonZeroInit::SetProfileDuration(double pos1, double pos2, double newDuration)
{
    // TODO: Synchronization has not been implemented yet because of v_i
	timeScale = 1; //duration/newDuration;
    if (timeScale > 1)
        return; // Cannot go below time optimal maneuver
    for (int i=0 ; i < (int)subVelProfiles.size() ; i++ ){
        subVelProfiles[i][0] /= timeScale;
        subVelProfiles[i][2] *= timeScale;
        subVelProfiles[i][3] *= timeScale*timeScale;
    }

}


void VelocityProfile_NonZeroInit::SetMax(double _maxvel,double _maxacc)
{
    maxVel = _maxvel;
    maxAcc = _maxacc;
}


double VelocityProfile_NonZeroInit::Duration() const
{
	return duration/timeScale;
}


double VelocityProfile_NonZeroInit::Pos(double time) const
{
	for( int i = 0 ; i < (int)subVelProfiles.size()-1 ; i++ ){
		if( time > subVelProfiles[i][0] && time <=  subVelProfiles[i+1][0] ){
			return subVelProfiles[i][1]	+ subVelProfiles[i][2] * (time -  subVelProfiles[i][0]) \
					+ 0.5 * subVelProfiles[i][3]*(time -  subVelProfiles[i][0])*(time -  subVelProfiles[i][0]);
		}

	}
	// if we are here, time is in the last piece or higher than duration so we set the output to the last requested position
	int last = subVelProfiles.size()-1;
	if (time > duration/timeScale)
        time = duration/timeScale;

	return subVelProfiles[last][1] + subVelProfiles[last][2] * (time -  subVelProfiles[last][0]) \
            + 0.5 * subVelProfiles[last][3]*(time -  subVelProfiles[last][0])*(time -  subVelProfiles[last][0]);
}


double VelocityProfile_NonZeroInit::Vel(double time) const
{
	for( int i = 0 ; i < (int)subVelProfiles.size()-1 ; i++ ){
		if(time > subVelProfiles[i][0] && time <=  subVelProfiles[i+1][0] ){
			return subVelProfiles[i][2] + subVelProfiles[i][3]*(time -  subVelProfiles[i][0]);
		}

	}
	// if we are here, time is in the last piece or higher than duration so we set the output to the final velocity
/*	int last = subVelProfiles.size()-1;
	if (time > duration/timeScale)
        time = duration/timeScale;

	return  subVelProfiles[last][2] + subVelProfiles[last][3]*(time -  subVelProfiles[last][0]);
*/
    return 0.0;
}


double VelocityProfile_NonZeroInit::Acc(double time) const
{
	for( int i = 0 ; i < (int)subVelProfiles.size() ; i++ ){
		if(time > subVelProfiles[i][0] && time <=  subVelProfiles[i+1][0] ){
			return subVelProfiles[i][3];
		}

	}
	// if we are here, time is in the last piece or higher than duration so we set the output to the final acceleration (0.0)
    return 0.0;
}


VelocityProfile* VelocityProfile_NonZeroInit::Clone() const
{
    VelocityProfile_NonZeroInit* res =  new VelocityProfile_NonZeroInit(maxVel,maxAcc);
    res->SetProfile( this->initPos, this->finalPos, this->initVel );
    res->SetProfileDuration( this->duration/this->timeScale);
	return res;
}



VelocityProfile_NonZeroInit::~VelocityProfile_NonZeroInit()
{}


void VelocityProfile_NonZeroInit::Write(std::ostream& os) const
{
	os << "NONZEROINIT[" << maxVel << "," << maxAcc << "]";
}


}//end of namespace
