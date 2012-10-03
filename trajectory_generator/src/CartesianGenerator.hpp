// $Id: nAxisGeneratorCartesianPos.hpp,v 1.1.1.1 2003/12/02 20:32:06 kgadeyne Exp $
// Copyright (C) 2003 Klaas Gadeyne <klaas.gadeyne@mech.kuleuven.ac.be>
//                    Wim Meeussen  <wim.meeussen@mech.kuleuven.ac.be>
// Copyright (C) 2010 Ruben Smits <ruben.smits@mech.kuleuven.be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <rtt/os/TimeService.hpp>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <ocl/OCL.hpp>

#include <Eigen/Dense>

#include "VelocityProfile_NonZeroInit.hpp"

//#include "VelocityProfile_NonZeroInit.hpp"
using namespace Eigen;
using namespace std;
namespace trajectory_generator
{
    /**
     * This class implements a TaskContext that creates a path in
     * Cartesian space between the current cartesian position and a
     * new desired cartesian position. It uses trapezoidal
     * velocity-profiles for every dof using a maximum velocity and a
     * maximum acceleration. It generates frame and twist setpoints
     * which can be used by OCL::CartesianControllerPos,
     * OCL::CartesianControllerPosVel or OCL::CartesianControllerVel.
     *
     */
    class CartesianGenerator : public RTT::TaskContext
    {
    public:
        /**
         * Constructor of the class.
         *
         * @param name name of the TaskContext
         */
    	CartesianGenerator(std::string name);
        virtual ~CartesianGenerator();

        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();
        bool updateCG();

    private:
      //bool moveTo(geometry_msgs::Pose pose, double time=0);
      bool generateNewVelocityProfiles(RTT::base::PortInterface* portInterface);
      void resetPosition();

      KDL::Frame                        m_traject_end, m_traject_begin;
      KDL::Frame                        m_position_desi_local;
      KDL::Twist                        m_velocity_desi_local, m_velocity_begin_end, m_velocity_delta;
      std::vector<double>				m_maximum_velocity, m_maximum_acceleration;

      std::vector<VelocityProfile_NonZeroInit> 		motionProfile;
      RTT::os::TimeService::ticks					m_time_begin;
      RTT::os::TimeService::Seconds					m_time_passed;
      double										m_max_duration;

      Vector3d										currentRotationalAxis;
      double										deltaTheta;

      double t_sync;
      double theta_vel;
      double xi,yi,zi,xf,yf,zf;
      bool cg_initialized;
      double TrajVectorMagnitude;
      struct VectorDirection
      {
    	  double x;
    	  double y;
    	  double z;
      } TrajVectorDirection;

      bool move_robot;
      bool position_ready;
      bool orientation_ready;
      bool pose_ready;
      geometry_msgs::Pose desired_pose;



    protected:
      /// Dataport containing the current measured end-effector
      /// frame, shared with OCL::CartesianSensor
      RTT::InputPort< geometry_msgs::Pose >   		m_position_meas_port;

      RTT::InputPort< geometry_msgs::Pose > cmdCartPose;

      RTT::OutputPort<geometry_msgs::PoseStamped > 	m_position_desi_port2ROS;

      /// Dataport containing the current desired end-effector
      /// frame, shared with OCL::CartesianControllerPos,
      /// OCL::CartesianControllerPosVel
      RTT::OutputPort< geometry_msgs::Pose >  		m_position_desi_port;

      //RTT::OutputPort<bool> m_move_finished_port;

  }; // class
}//namespace
