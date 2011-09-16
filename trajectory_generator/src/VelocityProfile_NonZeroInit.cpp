/***************************************************************************

    File:           KukaLWR_Kinematics.cpp
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

#include "VelocityProfile_NonZeroInit.hpp"

namespace trajectory_generator {

// constructs motion profile class with <maxvel> and <maxacc> as parameters of the trajectory.
VelocityProfile_NonZeroInit::VelocityProfile_NonZeroInit(double _maxvel, double _maxacc):
        duration(0.0), synchroTime(0.0), //timeScale(1),
        trajSign(1.0), syncSign(1.0),
        maxVel(abs(_maxvel)), maxAcc(abs(_maxacc)),
        initPos(0.0), finalPos(0.0), initVel(0.0), finalVel(0.0), initTime(0.0)

{}


// This definition of SetProfile function is needed for compatibility with the VelocityProfile interface
void VelocityProfile_NonZeroInit::SetProfile(double pos1,double pos2)
{
	initPos = pos1;
	finalPos = pos2;
	initVel = 0.0;
	finalVel = 0.0;
	initTime = 0.0;
	// set Time scale to default
//	timeScale = 1;
#if DEBUG
	cout << "2 Parameters Constructor/n Building Velocity Profile with maxAcc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") " << endl;
	cout << "------------------------------------------" << endl;
#endif
	duration = SubProfileBuilder(finalPos, initPos, initVel, finalVel, initTime);

	// Set SynchroTime to Duration as an initial value
	synchroTime = duration;
}


// This definition of SetProfile is added for completeness
bool VelocityProfile_NonZeroInit::SetProfile(double pos1,double pos2, double _inivel)
{
	//Precondition
	if (abs(_inivel) > maxVel)
	{
		cout << "VelocityProfile_NonZeroInit::SetProfile: Initial velocity higher than maximum" << endl;
		cout << "Profile will not be built" << endl;
		return false;
	}
	initPos = pos1;
	finalPos = pos2;
	initVel = _inivel;
	finalVel = 0.0;
	initTime = 0.0;
	// set Time scale to default
//	timeScale = 1;
#if DEBUG
	cout << "3 Parameters Constructor/nBuilding Velocity Profile with maxAcc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") " << endl;
	cout << "------------------------------------------" << endl;
#endif
	duration = SubProfileBuilder(finalPos, initPos, initVel, finalVel, initTime);

	// Set SynchroTime to Duration as an initial value
	synchroTime = duration;

	return true;
}


// This definition of SetProfile is added for completeness
bool VelocityProfile_NonZeroInit::SetProfile(double pos1,double pos2, double _inivel, double _finalvel)
{
	//Precondition
	if (abs(_inivel) > maxVel || abs(_finalvel) > maxVel )
	{
		cout << "VelocityProfile_NonZeroInit::SetProfile: Initial velocity higher than maximum" << endl;
		cout << "Profile will not be built" << endl;
		return false;
	}
	initPos = pos1;
	finalPos = pos2;
	initVel = _inivel;
	finalVel = _finalvel;
	initTime = 0.0;
	// set Time scale to default
//	timeScale = 1;
#if DEBUG
	cout << "4 Parameters Constructor/nBuilding Velocity Profile with maxAcc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") finalVel(" << finalVel << ")" << endl;
	cout << "------------------------------------------" << endl;
#endif
	duration = SubProfileBuilder(finalPos, initPos, initVel, finalVel, initTime);

	// Set SynchroTime to Duration as an initial value
	synchroTime = duration;

	return true;
}


// This is the recommended use for SetProfile function
bool VelocityProfile_NonZeroInit::SetProfile(double pos1,double pos2, double _inivel, double _finalvel, double _initime)
{
	//Precondition
	if (abs(_inivel) > maxVel || abs(_finalvel) > maxVel )
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
    finalVel = _finalvel;
    initTime = _initime;
	// set Time scale to default
//	timeScale = 1;
#if DEBUG
	cout << "4 Parameters Constructor/nBuilding Velocity Profile with maxAcc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") finalVel(" << finalVel << ") initTime(" << initTime << ")" << endl;
	cout << "------------------------------------------" << endl;
#endif
	duration = SubProfileBuilder(finalPos, initPos, initVel, finalVel, initTime); // Initial time set to zero for new trajectories

	// Set SynchroTime to Duration as an initial value
	synchroTime = duration;

	return true;
}


double VelocityProfile_NonZeroInit::SubProfileBuilder(double fPos, double iPos, double iVel, double fVel, double iTime)
{
	// We need these auxiliar variables for classifying the trajectory
	double Dp, Dv, meanVel, tfcrit, Dpcrit;

	// Auxiliar trajectory duration
	double tmpDuration;

	Dp = fPos-iPos; // Distance to cover
	Dv = fVel-iVel; // Velocity leap
	meanVel = (fVel+iVel)/2; // Mean velocity for critical trajectory
	tfcrit = abs((fVel-iVel)/maxAcc); // Duration of critical trajectory
	Dpcrit = meanVel*tfcrit; // Critical distance to be covered

	///////////////
	// TODO: Check on the velocity if we are going to hit the limits with the final state!!!
	// We cannot check here as the hardware limits are not available. Should this be changed??
	// Passing the check to the Trajectory Generator
	///////////////

	// trajSign gives the direction of the trajectory
	// +1 for trajectories over the Dpcrit
	trajSign = 0.0;
	if ( Dp > Dpcrit + epsilon )
		trajSign = 1.0;
	// -1 for trajectories under the Dpcrit
	else if ( Dp < Dpcrit - epsilon )
		trajSign = -1.0;
	// If the distance to cover is equal to critical, the direction depends on velocity leap
	else if ( Dv > epsilon )
		trajSign = 1.0;
	else if ( Dv < -epsilon )
		trajSign = -1.0;


#if DEBUG
		cout << "======================================" << endl;
    	cout << "trajSign " << trajSign << endl;
    	cout << "iPos " << iPos << endl;
    	cout << "fPos " << fPos << endl;
    	cout << "iVel " << iVel << endl;
    	cout << "fVel " << fVel << endl;
    	cout << "Dp " << Dp << endl;
       	cout << "Dpcrit " << Dpcrit << endl;
       	cout << "meanVel " << meanVel << endl;
#endif



	// If the initial state is equal to final state: we are at the target!!!
	if (trajSign == 0.0)
	{
		// We create a dummy maneuver
		std::vector<double> sp1;
		sp1.push_back(iTime);
		sp1.push_back(iPos);
		sp1.push_back(iVel);
		sp1.push_back(0.0);

		subVelProfiles.push_back(sp1);

		tmpDuration = epsilon;
	}
	else
	{
		// We calculate maximum velocity reached during maneuver
		double peakVel = sqrt( trajSign*Dp*maxAcc + (iVel*iVel+fVel*fVel)/2);


#if DEBUG
       	cout << "peakVel " << peakVel << endl;
#endif


		if ( peakVel < maxVel ){
		//slow down to zero and perform a new trajectory going back
			//log(Info) << "---- Triangular Velocity Profile ----" << endlog();
			// Two pieces trajectory
			std::vector<double> sp1,sp2;
			// Characteristic times
			double T20 = (trajSign*peakVel-iVel)/(trajSign*maxAcc);

			// Coefficients of the movement
			sp1.push_back( iTime ); //t_0
			sp1.push_back( iPos );
			sp1.push_back( iVel );
			sp1.push_back( trajSign*maxAcc );

			subVelProfiles.push_back(sp1);

			sp2.push_back( iTime + T20 );
			sp2.push_back( subVelProfiles[0][1] +
							(T20-iTime)*(subVelProfiles[0][2] + 0.5*(T20-iTime)*subVelProfiles[0][3]) );
			sp2.push_back( trajSign*peakVel );
			sp2.push_back( -trajSign*maxAcc );
			subVelProfiles.push_back(sp2);
#if DEBUG
	       	cout << "T20 " << T20 << endl;
#endif

			tmpDuration = T20 + (fVel-trajSign*peakVel)/(-trajSign*maxAcc);
		//end of triangular velocity profile

		}else{
			// log(Info) << "---- Trapezoidal Velocity Profile ----" << endlog();
			// Three pieces trajectory
			std::vector<double> sp1,sp2,sp3;
			// Characteristic times
			double T20 = (trajSign*maxVel-iVel)/(trajSign*maxAcc);
			double T30 = 1/(maxVel) * (trajSign*Dp + (fVel*fVel+iVel*iVel-trajSign*2*maxVel*iVel)/(2*maxAcc));

			//Adding the acceleration subProfile
			sp1.push_back( iTime ); //t_0
			sp1.push_back( iPos );
			sp1.push_back( iVel );
			sp1.push_back( trajSign*maxAcc );
			subVelProfiles.push_back(sp1);

			//Adding the constant velocity subProfile
			sp2.push_back( iTime + T20 );
			sp2.push_back( subVelProfiles[0][1] +
							(T20-iTime)*(subVelProfiles[0][2] + 0.5*(T20-iTime)*subVelProfiles[0][3]) );
			sp2.push_back( trajSign*maxVel );
			sp2.push_back( 0.0 );
			subVelProfiles.push_back(sp2);

			//Adding the Deceleration subProfile
			sp3.push_back( iTime + T30 );
			sp3.push_back( subVelProfiles[1][1] +
							(T30-iTime-T20)*(subVelProfiles[1][2] + 0.5*(T30-iTime-T20)*subVelProfiles[1][3]) );
			sp3.push_back( trajSign*maxVel );
			sp3.push_back( -trajSign*maxAcc );
			subVelProfiles.push_back(sp3);

			tmpDuration = T30 + (fVel-trajSign*maxVel)/(-trajSign*maxAcc);

#if DEBUG
	       	cout << "T20 " << T20 << endl;
	       	cout << "T30 " << T30 << endl;
	       	cout << "tmpDuration " << tmpDuration << endl;
#endif
		//End of Asymmetric trapezoidal velocity profile
		}
	} // if-else that detects if the final state is equal to the original

	//TODO: Add a final piece of trajectory that decelerates smoothly to zero
	if (abs(fVel) > epsilon )
	{
		// Decelerate system to zero
		std::vector<double> spDec;

		spDec.push_back( tmpDuration );
		spDec.push_back( fPos );
		spDec.push_back( fVel );
		spDec.push_back( -fVel/abs(fVel)*maxAcc );

		subVelProfiles.push_back(spDec);
	}

	// Finally, we add a steady piece giving the final state for times higher than duration+TDec
	std::vector<double> spSS;
	double TDec = abs(fVel)/maxAcc;

	spSS.push_back( tmpDuration+TDec );
	spSS.push_back( fPos+0.5*fVel*TDec  );
	spSS.push_back( 0.0 );
	spSS.push_back( 0.0 );

	subVelProfiles.push_back(spSS);

#if DEBUG
   	cout << "maxAcc " << maxAcc << endl;
   	cout << "maxVel " << maxVel << endl;
   	cout << "tmpDuration " << tmpDuration << endl;
   	cout << "======================================" << endl;
#endif
	return tmpDuration;
}


void VelocityProfile_NonZeroInit::SetProfileDuration(double newDuration)
{
    if (newDuration <= duration) {
    	synchroTime = duration;
#if 0
    	cout << "Requested time for synchronization is not valid" << endl;
#endif
    }
    else if (finalVel != 0.0)
    {
    	synchroTime = duration;
#if 0
    	cout << "Cannot synchronize for vf other than zero" << endl;
#endif
    }
    //Synchronizing trajectories
    else {
		double constVel = maxVel;

		// It synchronization time is very large, we need a double ramp profile
		if (initVel == 0.0 || (trajSign != abs(initVel)/initVel))
			syncSign = 1.0;
		else if (newDuration > (finalPos - initPos)/initVel + 0.5*initVel/(trajSign*maxAcc))
			syncSign = -1.0;

		if(abs(finalPos-initPos) < epsilon && abs(initVel) < epsilon )
			cout << "Zero trajectory. It will not be synchronized" << endlog();


		///// The trajectory generator has been changed to allow for vf other than zero
		///// At the same time, the posible trajectories have been reduced to two cases
		///// triangular and trapezoidal, without this deceleration one (particular case of triangular)

///////////////////
		else
		{
			synchroTime = newDuration;

			// Asymmetric trapezoidal or double ramp velocity profile (depending on syncTime)
			//log(Info) << "Synchronizing trajectory " << endlog();
			// Calculate the constant velocity along the maneuver
			if (syncSign == 1.0)
			{
				double aux = maxAcc*synchroTime + trajSign*initVel;
				constVel = 0.5*(aux - sqrt(aux*aux - 4*trajSign*maxAcc*(finalPos-initPos) - 2*initVel*initVel));
	//				cout << "Trapezoidal synchronized trajectory" << endlog();
			}
			else{
				constVel = (trajSign*(finalPos-initPos) - 0.5*initVel*initVel/maxAcc)/(synchroTime - initVel/(trajSign*maxAcc));
	//				cout << "Double ramp synchronized trajectory" << endlog();
			}


#if DEBUG
    	cout << "trajSign " << trajSign << endl;
    	cout << "syncSign " << syncSign << endl;
    	cout << "synchroTime " << synchroTime << endl;
    	cout << "constVel " << constVel << endl;
    	cout << "maxAcc " << maxAcc << endl;

#endif


    	// Change the acceleration in case the sync time is very large
			subVelProfiles[0][3] = syncSign*trajSign*maxAcc;

			// Calculate the modified constant velocity subprofile (2nd part)
			double T1 = (trajSign*constVel-initVel)/(syncSign*trajSign*maxAcc);
			double P1 = initPos + 0.5*T1*(trajSign*constVel + initVel) ; // position at the start of the subProfile
			subVelProfiles[1][0] = ( T1 + initTime );
			subVelProfiles[1][1] = ( P1 );
			subVelProfiles[1][2] = ( trajSign*constVel );
			subVelProfiles[1][3] = ( 0.0 );

			// Calculate the modified deceleration subprofile (3rd part)
			double T2 =  synchroTime - constVel/maxAcc;
			double P2 = P1 + (T2-T1)*trajSign*constVel;

			subVelProfiles[2][0] = ( T2 + initTime );
			subVelProfiles[2][1] = ( P2 );
			subVelProfiles[2][2] = ( trajSign*constVel );
			subVelProfiles[2][3] = ( -trajSign*maxAcc );

			// Finally, we add a steady piece giving the final state for times higher than duration+TDec
			if ( subVelProfiles.size() == 3 )
			{
				std::vector<double> spSS;

				spSS.push_back( synchroTime );
				spSS.push_back( finalPos );
				spSS.push_back( 0.0 );
				spSS.push_back( 0.0 );

				subVelProfiles.push_back(spSS);
			}
			else
			{
				subVelProfiles[3][0] = ( synchroTime );
				subVelProfiles[3][1] = ( finalPos );
				subVelProfiles[3][2] = ( 0.0 );
				subVelProfiles[3][3] = ( 0.0 );

			}


		//End of Synchronization
		}
////////////////////////
    }


#if DEBUG
	cout << "Synchronized trajectory" << endl;
	cout << "=======================" << endl;
	cout << "fPos " << finalPos << " iPos " << initPos << " iVel " << initVel << endl;
	if(abs(finalPos-initPos) > epsilon || abs(initVel) > epsilon )
	{
		int i;
		for( i = 0 ; i<(int)subVelProfiles.size()-1 ; i++ )
		{
			cout << "Trajectory piece " << i << endl;
			cout << "------------------" << endl;
			cout << "initTime: " << subVelProfiles[i][0] << "; initPos: " << subVelProfiles[i][1] << "; initVel: " \
					<< subVelProfiles[i][2] << "; initAcc: " << subVelProfiles[i][3] << endl;
			cout << "finalTime: " << subVelProfiles[i+1][0] << "; finalPos: " << Pos(subVelProfiles[i+1][0]-0.001) << "; finalVel: " \
					<< Vel(subVelProfiles[i+1][0]-0.001) << "; finalAcc: " << Acc(subVelProfiles[i+1][0]-0.001) << endl;
		}
		cout << "Trajectory piece " << i << endl;
		cout << "------------------" << endl;
		cout << "initTime: " << subVelProfiles[i][0] << "; initPos: " << subVelProfiles[i][1] << "; initVel: " \
				<< subVelProfiles[i][2] << "; initAcc: " << subVelProfiles[i][3] << endl;
		cout << "Final values" << endl;
		cout << "------------" << endl;
		cout << "finalTime: " << synchroTime << "; finalPos: " << Pos(synchroTime) << "; finalVel: " \
				<< Vel(synchroTime) << "; finalAcc: " << Acc(synchroTime) << endl;
	}
	else
	{
		cout << "Trajectory piece 0" << endl;
		cout << "------------------" << endl;
		cout << "Begin of printing" << endl;
		cout << "initTime: " << subVelProfiles[0][0] << "; initPos: " << subVelProfiles[0][1] << "; initVel: " \
				<< subVelProfiles[0][2] << "; initAcc: " << subVelProfiles[0][3] << endl;
		cout << "End of printing" << endl;
	}
#endif
   return;
}


// This function is defined for compatibility with VelocityProfile class
void VelocityProfile_NonZeroInit::SetProfileDuration(double pos1, double pos2, double newDuration)
{
    if (newDuration < duration) {
    	synchroTime = duration;
#if DEBUG
    	cout << "Requested time for synchronization is not valid" << endl;
#endif
    }
    else
    	this->SetProfileDuration(newDuration);

	return;
}


void VelocityProfile_NonZeroInit::SetMax(double _maxvel,double _maxacc)
{
    maxVel = _maxvel;
    maxAcc = _maxacc;
}


double VelocityProfile_NonZeroInit::Duration() const
{
	return synchroTime;//duration;///timeScale;
}


double VelocityProfile_NonZeroInit::Pos(double time) const
{
	for( int i = 0 ; i < (int)subVelProfiles.size()-1 ; i++ ){
		if( time >= subVelProfiles[i][0] && time <  subVelProfiles[i+1][0] ){
			return subVelProfiles[i][1]	+ subVelProfiles[i][2] * (time -  subVelProfiles[i][0]) \
					+ 0.5 * subVelProfiles[i][3]*(time -  subVelProfiles[i][0])*(time -  subVelProfiles[i][0]);
		}

	}
	// if we are here, time is in the last piece or higher than duration so we set the output to the last requested position
	int last = subVelProfiles.size()-1;

	return subVelProfiles[last][1];
}


double VelocityProfile_NonZeroInit::Vel(double time) const
{
	for( int i = 0 ; i < (int)subVelProfiles.size()-1 ; i++ ){
		if(time >= subVelProfiles[i][0] && time <  subVelProfiles[i+1][0] ){
			return subVelProfiles[i][2] + subVelProfiles[i][3]*(time -  subVelProfiles[i][0]);
		}

	}
	// if we are here, time is in the last piece or higher than duration so we set the output to the final velocity
	int last = subVelProfiles.size()-1;

	return  subVelProfiles[last][2];
}


double VelocityProfile_NonZeroInit::Acc(double time) const
{
	for( int i = 0 ; i < (int)subVelProfiles.size()-1 ; i++ ){
		if(time >= subVelProfiles[i][0] && time <  subVelProfiles[i+1][0] ){
			return subVelProfiles[i][3];
		}

	}
	// if we are here, time is in the last piece or higher than duration so we set the output to the final acceleration (0.0)
	int last = subVelProfiles.size()-1;

	return  subVelProfiles[last][3];
}


VelocityProfile* VelocityProfile_NonZeroInit::Clone() const
{
    VelocityProfile_NonZeroInit* res =  new VelocityProfile_NonZeroInit(maxVel,maxAcc);
    res->SetProfile( this->initPos, this->finalPos, this->initVel );
    res->SetProfileDuration( this->synchroTime );///this->timeScale);
	return res;
}



VelocityProfile_NonZeroInit::~VelocityProfile_NonZeroInit()
{}


void VelocityProfile_NonZeroInit::Write(std::ostream& os) const
{
	os << "NONZEROINIT[" << maxVel << "," << maxAcc << "]";
}


}//end of namespace
