/***************************************************************************

    File:           KukaLWR_Kinematics.hpp
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

/**
 *  \file
 *  \par Velocity Profile with non zero initial velocities
 *   This file extends the VelocityProfile abstract class, generating velocity
 *   profiles which are time optimal (subjected to maximum accelerations and
 *   velocities) and may have initial velocities different from zero.
 *   It calculates the coefficients of a second-order polynomial for each
 *   part of the trajectory (piece-wise, non-symmetric trapezoid velocity
 *   profile)
 *   Then it provides functions to obtain the necessary
 *   position/velocity/acceleration of the trajectory at a certain instant.
 *
 *   \par Creating a Velocity Profile
 *   To create the Velocity Profile, two steps must be accomplished:
 *    * The class constructor should be instantiated
 *    * The SetProfile function should be called with any the provided
 *      interfaces
 *
 *   \par Subprofiles
 *   Each profile consists of one or more subprofiles. A subprofile is each of
 *   the different parts the trajectory is divided in, but these are
 *   transparent to the user
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
        /** It is a vector of SubProfiles, each of them with the coefficients
         *  of a polynomial corresponding to that piece of the profile
         */
        std::vector< std::vector<double> > subVelProfiles;

        //! Time-optimal calculated duration of maneuver
        double duration;
        //! Time scale when a longer than time-optimal duration is required (synchronization)
        double timeScale;

        //! Maximum velocity achievable by the robot
        double maxVel;
        //! Maximum acceleration achievable by the robot
        double maxAcc;
        //! Initial position of the joint
        double initPos;
        //! Desired final position of the joint
        double finalPos;
        //! Velocity of the joint when maneuver starts
        double initVel;
        //! Beginning time of the manuever
        double initTime;

        //! Private method for creating the subprofiles of the trajectory
        // Method for creating subprofiles
        double SubProfileBuilder(double finalPos, double initPos, double initVel, double initTime);


    public:
        //! Constructs motion profile class with \p _maxvel and \p _maxacc as parameters of the trajectory
        VelocityProfile_NonZeroInit(double _maxvel=0, double _maxacc=0);

        // We add several interfaces. A non-passed argument is considered 0.0
        /** \brief Complete profile definition.
         *
         *  \param pos1 Initial position of the trajectory (should be actual robot position)
         *  \param pos2 Final desired position of the trajectory
         *  \param _inivel Must be smaller than \p _maxvel assigned in the constructor
         *  \param _initime Must not be negative (and tipically zero)
         */
	    bool SetProfile(double pos1,double pos2, double _inivel, double _initime);
        /** \brief Profile definition. Initial time of trajectory is assumed to be zero.
         *
         *  \param pos1 Initial position of the trajectory (should be actual robot position)
         *  \param pos2 Final desired position of the trajectory
         *  \param _inivel Must be smaller than \p _maxvel assigned in the constructor
         */
	    bool SetProfile(double pos1,double pos2, double _inivel);
	    /** \brief Interface for changing the profile duration.
	     *
	     *  \param newDuration The new duration assigned to the trajectory must be greater
	     *  than the optimal time
	     *  \todo Not implemented yet
	     */
	    void SetProfileDuration(double newDuration);

	    // These are added for compatibility with KDL::VelocityProfile
        /** \brief Profile definition. Initial time and velocity of trajectory are assumed to be zero
         *
         *  \note Added for compatibility with VelocityProfile Class
         */
	    virtual void SetProfile(double pos1, double pos2);
        /** \brief Interface for changing the profile duration.
         *
         *  \param pos1 Initial position of the trajectory (should be actual robot position)
         *  \param pos2 Final desired position of the trajectory
 	     *  \param newduration The new duration assigned to the trajectory must be greater
	     *  than the optimal time
         *  \note Added for compatibility with VelocityProfile Class.
         *   \p pos1 and \p pos2 are not used. A previous call to SetProfile is required
         */
	    virtual void SetProfileDuration(double pos1, double pos2, double newduration);

	    //! Changes the maximum values for velocity and acceleration
	    virtual void SetMax(double _maxvel,double _maxacc);
	    /** \brief Returns the calculated duration of the maneuver.
	     *
	     *  If a different from optimal duration has been set, it gives back the new set value.
	     */
	    virtual double Duration() const;
	    /** \brief Returns the calculated desired position of the maneuver at a certaint \p time.
	     *
	     *  If \p time is higher than \p duration then final desired position is returned.
	     */
	    virtual double Pos(double time) const;
	    /** \brief Returns the calculated desired velocity of the maneuver at a certaint \p time.
	     *
	     *  If \p time is higher than \p duration then 0.0 is returned.
	     */
	    virtual double Vel(double time) const;
	    /** \brief Returns the calculated desired acceleration of the maneuver at a certaint \p time.
	     *
	     *  If \p time is higher than \p duration then 0.0 is returned.
	     */
	    virtual double Acc(double time) const;
	    //! Writes the kind of trajectory in the stream \p os
	    virtual void Write(std::ostream& os) const;
	    //! Returns copy of current VelocityProfile object. (virtual constructor)
	    virtual VelocityProfile* Clone() const;
	    ~VelocityProfile_NonZeroInit();

    };

} //end of namespace

#endif /* VELOCITYPROFILE_NONZEROINIT_H */
