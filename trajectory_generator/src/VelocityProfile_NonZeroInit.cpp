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
        maxVel(_maxvel), maxAcc(_maxacc),
        initPos(0.0), finalPos(0.0), initVel(0.0), initTime(0.0)

{}


// This definition of SetProfile function is needed for compatibility with the VelocityProfile interface
void VelocityProfile_NonZeroInit::SetProfile(double pos1,double pos2)
{
	initPos = pos1;
	finalPos = pos2;
	initVel = 0.0;
	initTime = 0.0;
	// set Time scale to default
//	timeScale = 1;
#if DEBUG
	cout << "2 Parameters Constructor/n Building Velocity Profile with maxAcc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") " << endl;
	cout << "------------------------------------------" << endl;
#endif
	duration = SubProfileBuilder(finalPos, initPos, initVel, initTime);

	// Set SynchroTime to Duration as an initial value
	synchroTime = duration;
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
	// set Time scale to default
//	timeScale = 1;
#if DEBUG
	cout << "3 Parameters Constructor/nBuilding Velocity Profile with maxAcc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") " << endl;
	cout << "------------------------------------------" << endl;
#endif
	duration = SubProfileBuilder(finalPos, initPos, initVel, initTime);

	// Set SynchroTime to Duration as an initial value
	synchroTime = duration;

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
//	timeScale = 1;
#if DEBUG
	cout << "4 Parameters Constructor/nBuilding Velocity Profile with maxAcc(" << maxAcc << ") maxVel(" << maxVel << ") finalPos(" << finalPos << ") initPos(" << \
			initPos << ") initVel(" << initVel << ") " << endl;
	cout << "------------------------------------------" << endl;
#endif
	duration = SubProfileBuilder(finalPos, initPos, initVel, initTime); // Initial time set to zero for new trajectories

	// Set SynchroTime to Duration as an initial value
	synchroTime = duration;

	return true;
}


double VelocityProfile_NonZeroInit::SubProfileBuilder(double fPos, double iPos, double iVel, double iTime)
{

	// We need the distance (absolute) to be covered
	double Dp = abs(fPos -  iPos), tmpDuration;
	// trajSign gives the direction of the trajectory
	// +1 for positive ((fPos>iPos)
	trajSign = 1.0;
	// -1 for negative ((fPos<iPos)
	if (fPos -  iPos < 0)
		trajSign = -1.0;

	if(abs(fPos-iPos) < epsilon && abs(iVel) < epsilon ){
		std::vector<double> sp1;
		sp1.push_back(iTime);
		sp1.push_back(iPos);
		sp1.push_back(0.0);
		sp1.push_back(0.0);

		subVelProfiles.push_back(sp1);

		tmpDuration = epsilon;
		return tmpDuration;
	}

	if (Dp < 0.5* iVel*iVel / maxAcc){
	//slow down to zero and perform a new trajectory going back
		//log(Info) << "Slow down to zero and perform a new trajectory going back " << endlog();
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
		//log(Info) << "Triangular velocity profile " << endlog();
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
		//log(Info) << "Asymmetric trapezoidal velocity profile " << endlog();
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
//	timeScale = 1; //duration/newDuration;
//    if (timeScale > 1)
//        return; // Cannot go below time optimal maneuver
//    for (int i=0 ; i < (int)subVelProfiles.size() ; i++ ){
//        subVelProfiles[i][0] /= timeScale;
//        subVelProfiles[i][2] *= timeScale;
//        subVelProfiles[i][3] *= timeScale*timeScale;
//    }
    if (newDuration <= duration) {
    	synchroTime = duration;
#if 0
    	cout << "Requested time for synchronization is not valid" << endl;
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

		else if (abs(finalPos-initPos) <= 0.5* initVel*initVel / maxAcc)
		{
//			cout << "Deceleration trajectory" << endl;

 	 	 	synchroTime = newDuration;

			// Synchronizing the backwards movement
			if (subVelProfiles.size() == 3)
			{
				// The first (deceleration) and the second (acceleration) pieces remain the same.
				// The third piece will start earlier and will keep a constant velocity (constVel)
				double T1, T2, P1, P2;
				double aux = maxAcc*synchroTime;// + trajSign*initVel; initial velocity is zero
				double Dp = finalPos-subVelProfiles[1][1];
				double trajSignNew = abs(Dp)/Dp;
				constVel = 0.5*(aux - sqrt(aux*aux - 4*trajSignNew*maxAcc*(Dp)));

				// Constants of the start of the third piece of trajectory
				T1 = subVelProfiles[1][0] + constVel/maxAcc;
				P1 = subVelProfiles[1][1] + 0.5*(T1-subVelProfiles[1][0])*(trajSignNew*constVel) ;

				// Third piece
				subVelProfiles[2][0] = ( T1 );
				subVelProfiles[2][1] = ( P1 );
				subVelProfiles[2][2] = ( trajSignNew*constVel );
				subVelProfiles[2][3] = ( 0.0 );

				// Constants of the start of the fourth (last) piece of trajectory
				T2 = synchroTime - constVel/maxAcc;
				P2 = subVelProfiles[2][1] + (T2-subVelProfiles[2][0])*trajSignNew*constVel;

				// Fourth piece
				std::vector<double> sp4;
				sp4.push_back( T2 );
				sp4.push_back( P2 );
				sp4.push_back( trajSignNew*constVel );
				sp4.push_back( -trajSignNew*maxAcc );
				subVelProfiles.push_back(sp4);

			}
		}
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
#if 0
			cout << "!!!!! ConstVelProfile !!!!! " << constVel << endl;
			cout << "!!!!! Synchro Sign !!!!! " << syncSign << endl;
			cout << "!!!!! Trajectory Sign !!!!! " << trajSign << endl;
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

			//If we had a triangular profile before (only two parts), we add a third subVelProfile for the decelerating part
			if ( subVelProfiles.size() == 2 )
			{
				std::vector<double> sp3;
				//Adding the Deceleration subProfile
				sp3.push_back( T2 + initTime );
				sp3.push_back( P2 );
				sp3.push_back( trajSign*constVel );
				sp3.push_back( -trajSign*maxAcc );
				subVelProfiles.push_back(sp3);
			}
			// If we had a trapezoidal profile, we already had this third part of the profile
			else {
				subVelProfiles[2][0] = ( T2 + initTime );
				subVelProfiles[2][1] = ( P2 );
				subVelProfiles[2][2] = ( trajSign*constVel );
				subVelProfiles[2][3] = ( -trajSign*maxAcc );
			}

		//End of Synchronization
		}
//		cout << "Synchro time has been changed to " << synchroTime << " from " << duration << endl;
	}

#if 0
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
    // TODO: Synchronization has not been implemented yet because of v_i
//	timeScale = 1; //duration/newDuration;
//    if (timeScale > 1)
//        return; // Cannot go below time optimal maneuver
//    for (int i=0 ; i < (int)subVelProfiles.size() ; i++ ){
//        subVelProfiles[i][0] /= timeScale;
//        subVelProfiles[i][2] *= timeScale;
//        subVelProfiles[i][3] *= timeScale*timeScale;
//    }
    if (newDuration < duration) {
    	synchroTime = duration;
#if DEBUG
    	cout << "Requested time for synchronization is not valid" << endl;
#endif
    }
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
	if (time > synchroTime)///timeScale)
        time = synchroTime;///timeScale;

	return subVelProfiles[last][1] + subVelProfiles[last][2] * (time -  subVelProfiles[last][0]) \
            + 0.5 * subVelProfiles[last][3]*(time -  subVelProfiles[last][0])*(time -  subVelProfiles[last][0]);
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
	if (time > synchroTime)///timeScale)
        time = synchroTime;///timeScale;

	return  subVelProfiles[last][2] + subVelProfiles[last][3]*(time -  subVelProfiles[last][0]);
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
	if (time < synchroTime)///timeScale)
		return  subVelProfiles[last][3];

	return 0.0;
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
