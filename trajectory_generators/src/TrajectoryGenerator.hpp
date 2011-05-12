/***************************************************************************

    File:           TrajectoryGenerator.hpp
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

/**
 *  \file
 *  \par Optimal time Trajectory Generator with velocity and acceleration constraints
 *   This file defines an OROCOS component which generates velocity profiles
 *   using the VelocityProfile_NonZeroInit class. This component listen to a port
 *   in which a ROS
 *   \par Creating a Velocity Profile
 *   To create the Velocity Profile, two steps must be accomplished:
 *    * The class constructor should be instantiated
 *    * The SetProfile function should be called with any the provided
 *      interfaces
 *
 *   \par Subprofiles
 *   Each profile consists of one or mor subprofile. A subprofile is each of
 *   the different parts the trajectory is decomposed in. These should be
 *   transparent to the user
 *
 *  \authors
 *      Francisco Ramos, Ph.D., Dipl. Ing., ETH Zurich/UCLM
 *      Gajamohan Mohanarajah, M.Sc., ETH Zurich
 *
 ****************************************************************************/

#ifndef __CARTESIAN_GENERATOR_POS_H__
#define __CARTESIAN_GENERATOR_POS_H__

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>

#include <rtt/marsh/Marshalling.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <rtt/os/TimeService.hpp>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

#include <sensor_msgs/JointState.h>

#include <std_msgs/Float64.h>



#include "VelocityProfile_NonZeroInit.hpp"
#include "KukaLWR_Kinematics.hpp"

namespace trajectory_generators
{
    /**
     * The description goes here
     *
     */
    class TrajectoryGenerator : public RTT::TaskContext
    {
    public:
        /**
         * Constructor of the class.
         *
         * @param name name of the TaskContext
         */
        TrajectoryGenerator(std::string name);
        virtual ~TrajectoryGenerator();

        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();

    private:
      bool moveTo(std::vector<double> position, double time=0);
      bool moveFinished() const;
      void resetPosition();

      bool generateNewVelocityProfilesCartPosInput(RTT::base::PortInterface* portInterface);
      bool generateNewVelocityProfilesJntPosInput(RTT::base::PortInterface* portInterface);

      unsigned int num_axes;
      std::vector<double> p_m, p_d, v_d;
      std::vector<double> v_max, a_max, p_max, p_min;
      //TODO: Question: Should this be a vector<VelocityProfile> (without the NonZero Init)???
      std::vector<VelocityProfile_NonZeroInit> motionProfile;

      RTT::os::TimeService::ticks	time_begin;
      RTT::os::TimeService::Seconds	time_passed;

      geometry_msgs::Pose lastCommandedPose;
      std::vector<double> lastCommndedPoseJntPos;
      std::vector<double> jntPosCmd;
      std::vector<double> jntVel;

      sensor_msgs::JointState jntState, cmdJntState; // to/fromROS

      double max_duration;
      bool is_moving;
      bool toROS;

      ///Robot specific data

    protected:
      /// Dataport containing commanded Cartesian pose
      RTT::InputPort< geometry_msgs::Pose > input_cartPosPort;
      /// Dataport containing commanded joint position
      RTT::InputPort< sensor_msgs::JointState > input_jntPosPort;
      /// Dataport containing the measured joint angles from the Robot
      RTT::InputPort< std::vector<double> > msr_jntPosPort;
      /// Dataport containing the desired joint angles
      RTT::OutputPort< std::vector<double> >  output_jntPosPort;
      /// Dataport containing the desired joint angles (In s)
      RTT::OutputPort< sensor_msgs::JointState >  output_jntPosPort_toROS;

  }; // class
}//namespace

#endif // __CARTESIAN_GENERATOR_POS_H__
