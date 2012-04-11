/***************************************************************************

    File:           KukaLWR_Kinematics.hpp
    Author(s):      Gajamohan Mohanarajah/Francisco Ramos/Zdragkas Georgios
    Affiliation:    IDSC - ETH Zurich
    e-mail:         gajan@ethz.ch/framosde@ethz.ch
    Start date:	    6th May 2011
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
 *  \par 7 DOF Kuka LightWeight Robot Kinematics
 *   This file defines a class for calculating kinematic transformations
 *   on the KUKA LWR Robot
 *
 *   \par Constructor/Destructor
 *   This class has neither of them, as it only contains static functions and,
 *   hence, will not be instantiated
 *
 *  \par Number of degrees of freedom
 *  The Forward Kinematics transformation considers the whole 7 DOF of the
 *  robot, while the Inverse Kinematics only calculates 6 Joints, leaving
 *  the extra DOF fixed to zero. This allows us to obtain closed-form equations
 *  for the inverse kinematics.
 *
 *  \par Inverse kinematics solution
 *  Closed form equations based in the solution for an anthropomorphic arm
 *  with an spherical wrist. Extracted from Siciliano et al (2010)
 *  Only one of the four possible solutions for the kinematic
 *  chain is calculated. Specifically, the right-shoulder elbow-up one.
 *
 *  \authors
 *      Francisco Ramos, Ph.D., Dipl. Ing., ETH Zurich/UCLM
 *      Gajamohan Mohanarajah, M.Sc., ETH Zurich
 *
 ****************************************************************************/
#ifndef KUKALWR_KINEMATICS_HPP_
#define KUKALWR_KINEMATICS_HPP_
#define DEBUG 0

#include <rtt/RTT.hpp>
#include <tf_conversions/tf_kdl.h>

#include <geometry_msgs/Pose.h>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>

//#include <rtt/os/TimeService.hpp>

// Constant definition
#define PI 3.14159


namespace kuka_IK {

/**
 * \brief A concrete and limited implementation of the KUKA_LWR Kinematics both
        forward and inverse transformations
 */
class KukaLWR_Kinematics {
private:
    // @{
	static const double D1 = 0.310;
    static const double D3 = 0.400;
    static const double D5 = 0.390;
	/// Denavit Hartenberg relevant parameters of the robot
    static const double D7 = 0.078;
    static const double ALPHA = PI/2.0; // For joints from 0 to 5 and 0.0 for joint 6
    // @}
    //! Rotation limits of each joint (in radians)
    static const double JNT_LIMITS [];

public:
    // Analytical IK solver for 7dof, with arm angle as a parameter, implemented by Zdragkas Georgios
    static bool ikSolverAnalytical7DOF( geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr);
    //! Inverse kinematics of the KUKA LWR Robot (6 dof). Returns 1 if the requested position is reachable
    static bool ikSolver(std::vector<double> & jntPosMsr, geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr);
	//! Inverse kinematics of the KUKA LWR Robot (7 dof). Returns 1 if the requested position is reachable
	static bool ikSolverIterative7DOF(std::vector<double> & jntPosMsr, geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr);
    //! Forward kinematics of the KUKA LWR Robot (7 dof). Returns 1 if the requested position is reachable
	static bool fkSolver(const std::vector<double> & jntPosDsr, geometry_msgs::Pose & poseDsr);

	//RTT::os::TimeService::ticks time_begin_BENCHMARK;
	//RTT::os::TimeService::Seconds time_passed_BENCHMARK;
};

}

#endif /* KUKALWR_KYNEMATICS_HPP_ */
