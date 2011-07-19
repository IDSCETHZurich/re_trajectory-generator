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
 *  \par Time optimal Trajectory Generator with velocity and acceleration constraints
 *   This component implements an event port
 *   which listens on commanded joint angles of the Robot.
 *   Based on the commands and the current state (position,velocity) of the Robot
 *   a time optimal velocity profile is created using the VelocityProfile_NonZeroInit class and
 *   sent out on an outputPort.
 *
 *
 *  \authors
 *      Gajamohan Mohanarajah, M.Eng., ETH Zurich
 *      Francisco Ramos, Ph.D., Dipl. Ing., ETH Zurich/UCLM
 *
 ****************************************************************************/

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

namespace trajectory_generator
{
    /**
     * \brief Trajectory Generator OROCOS component extends TaskContext from RTT
     *
     */
    class TrajectoryGenerator : public RTT::TaskContext
    {
    public:
        /**
         * \brief Constructor of the TrajectoryGenerator class.
         *
         * @param name name of the TaskContext
         */
        TrajectoryGenerator(std::string name);
        /**
         * \brief Destructor of the TrajectoryGenerator class.
         */
        virtual ~TrajectoryGenerator();

        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();


      /** \brief function handle for the input_jntPosPort
      	*
      	*  \param portInterface
      	*/
      bool generateNewVelocityProfilesJntPosInput(RTT::base::PortInterface* portInterface);

      unsigned int num_axes;

      ///@{
      /**
       * @brief Maximum velocity, acceleration and the position range of each rotational joint of the Robot
       */
           std::vector<double> v_max, a_max, p_max, p_min;
      ///@}

      std::vector<VelocityProfile_NonZeroInit> motionProfile;

      RTT::os::TimeService::ticks	time_begin;
      RTT::os::TimeService::Seconds	time_passed;

      ///@{
      /**
       * @brief Commands from the event(input) port
       */
      std::vector<double> jntPosCmd;
      sensor_msgs::JointState cmdJntState;
      ///@}

      ///@{
      /**
       * @brief intermediate command variables
       */
      std::vector<double> lastCommndedPoseJntPos;
      geometry_msgs::Pose lastCommandedPose;
      ///@}

      /// Initial joint velocities of each rotational joint of the Robot
      std::vector<double> jntVel;

      /// joint state going out on output_jntPosPort_toROS port
      sensor_msgs::JointState jntState;


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
