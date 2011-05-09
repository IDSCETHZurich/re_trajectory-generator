/*
 * VelocityProfile_NonZeroInit.hpp
 *
 *  Created on: Apr 7, 2011
 *      Author: Gajan
 */

#ifndef VELOCITYPROFILE_NONZEROINIT_H
#define VELOCITYPROFILE_NONZEROINIT_H

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <ocl/OCL.hpp>
#include <kdl/velocityprofile.hpp>

namespace trajectory_generators
{

using namespace RTT;
using namespace std;
using namespace KDL;

	/*
	 * A VelocityProfile with non-zero initial velocity implementation.
	 */

	//TODO: When overriding a function, we keep a function as virtual, being the
	//new class also abstract ==> You cannot create instances of it
	//Either the base class needs to be updated or we need to fix to that base class
	//what imposes creating a new function for including initial velocity as a separated feature
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
	    void SetProfile(double pos1,double pos2, double _inivel, double _initime);
	    void SetProfile(double pos1,double pos2, double _inivel);
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
