/***************************************************************************

    File:           KukaLWR_Kinematics.hpp
    Author(s):      Gajamohan Mohanarajah/Francisco Ramos
    Affiliation:    IDSC - ETH Zurich
    e-mail:         gajan@ethz.ch/framosde@ethz.ch
    Start date:	    tth April 2011
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

/**
 *  \file
 *  \par Velocity Profile with non zero initial velocities
 *   This file extends the VelocityProfile abstract class, generating velocity
 *   profiles which are time optimal (subjected to maximum accelerations and
 *   velocities) and may have initial velocities different from zero
 *
 *  \authors
 *      Francisco Ramos, Ph.D., Dipl. Ing., ETH Zurich/UCLM
 *      Gajamohan Mohanarajah, M.Sc., ETH Zurich
 *
 ****************************************************************************/

#ifndef VELOCITYPROFILE_NONZEROINIT_H
#define VELOCITYPROFILE_NONZEROINIT_H

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <ocl/OCL.hpp>
#include <kdl/velocityprofile.hpp>

namespace trajectory_generators
{

using namespace RTT;
using namespace KDL;

	/**
	 * \brief A VelocityProfile with non-zero initial velocity implementation.
	 */
    class VelocityProfile_NonZeroInit : public VelocityProfile
    {

    private:
        // It is a vector of SubProfiles, each of them with the coefficients
        // of a polynomial corresponding to that piece of the profile
        std::vector< std::vector<double> > subVelProfiles;
        double duration;
        double timeScale;

        // Specification of the motion profile
        double maxVel;
        double maxAcc;
        double initPos;
        double finalPos;
        double initVel;
        double initTime;

        // Method for creating subprofiles
        double SubProfileBuilder(double finalPos, double initPos, double initVel, double initTime);


    public:
        VelocityProfile_NonZeroInit(double _maxvel=0, double _maxacc=0);
	    // constructs motion profile class with <maxvel> and <maxacc> as parameters of the
	    // trajectory.

        // We add several interfaces. A non-passed argument is considered 0.0
	    bool SetProfile(double pos1,double pos2, double _inivel, double _initime);
	    bool SetProfile(double pos1,double pos2, double _inivel);
	    // And also a new interface for synchronization (TBD)
	    void SetProfileDuration(double newDuration);

	    // These are added for compatibility with KDL::VelocityProfile
	    virtual void SetProfile(double pos1, double pos2);
	    virtual void SetProfileDuration(double pos1, double pos2, double newduration);

	    virtual void SetMax(double _maxvel,double _maxacc);
	    virtual double Duration() const;
	    virtual double Pos(double time) const;
	    virtual double Vel(double time) const;
	    virtual double Acc(double time) const;
	    virtual void Write(std::ostream& os) const;
	    virtual VelocityProfile* Clone() const;
	    // returns copy of current VelocityProfile object. (virtual constructor)
	    ~VelocityProfile_NonZeroInit();

    };

} //end of namespace

#endif /* VELOCITYPROFILE_NONZEROINIT_H */
