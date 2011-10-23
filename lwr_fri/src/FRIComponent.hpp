// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>, 
// Copyright  (C)  2009  Wilm Decre <wilm dot decre at mech dot kuleuven dot be>

// Author: Ruben Smits, Wilm Decre
// Maintainer: Ruben Smits, Wilm Decre

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef _FRI_COMPONENT_HPP_
#define _FRI_COMPONENT_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <sys/socket.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>


#include <friComm.h>

namespace lwr_fri {

using namespace RTT;

class FRIComponent: public RTT::TaskContext {
public:
	FRIComponent(const std::string& name);
	~FRIComponent();

	virtual bool configureHook();
	virtual bool startHook();

	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();

private:

	tFriMsrData m_msr_data;
	tFriCmdData m_cmd_data;

        std::vector<double> m_jntPos;
        std::vector<double> m_jntVel;
        std::vector<double> m_jntTorques;

        geometry_msgs::Pose m_cartPos;
        geometry_msgs::Twist m_cartTwist;
        geometry_msgs::Wrench m_cartWrench;

	tFriKrlData m_fromKRL;
	tFriKrlData m_toKRL;
	//Eigen::Matrix<double,7,7> m_massTmp; Not correct so useless

	/**
	 * statistics
	 */
	OutputPort<tFriRobotState> m_RobotStatePort;
	OutputPort<tFriRobotData> m_RobotDataPort;
	OutputPort<tFriIntfState> m_FriStatePort;

	/**
	 * Current robot data
	 */
	OutputPort<std::vector<double> > m_msrJntPosPort;
	OutputPort<std::vector<double> > m_cmdJntPosPort;
	OutputPort<std::vector<double> > m_cmdJntPosFriOffsetPort;
	OutputPort<geometry_msgs::Pose>  m_msrCartPosPort;
	OutputPort<geometry_msgs::Pose>  m_cmdCartPosPort;
	OutputPort<geometry_msgs::Pose>  m_cmdCartPosFriOffsetPort;
	OutputPort<std::vector<double> >   m_msrJntTrqPort;
	OutputPort<std::vector<double> >   m_estExtJntTrqPort;
	OutputPort<geometry_msgs::Wrench> m_estExtTcpWrenchPort;
	//RTT::OutputPort<KDL::Jacobian> jacobianPort;
	//RTT::OutputPort<Eigen::MatrixXd > massMatrixPort;
	//RTT::OutputPort<std::vector<double> > gravityPort;

	InputPort<std::vector<double> > m_jntPosPort;
    InputPort<std::vector<double> > m_jntVelPort;
	InputPort<geometry_msgs::Pose> m_cartPosPort;
	InputPort<geometry_msgs::Twist> m_cartTwistPort;
	InputPort<std::vector<double> > m_addJntTrqPort;
	InputPort<geometry_msgs::Wrench> m_addTcpWrenchPort;
	//InputPort<JointImpedances> m_jntImpedancePort;
	//InputPort<CartesianImpedance> m_cartImpedancePort;

	int m_local_port,m_socket,m_remote_port, m_control_mode;

	const char* m_remote_address;
	struct sockaddr m_remote_addr;
	uint16_t counter;

	//IDSC Debugging
	RTT::os::TimeService::ticks	time_begin;
	RTT::os::TimeService::Seconds	time_passed;

	RTT::OperationCaller<bool(void)> updateTG;
	RTT::OperationCaller<bool(void)> updateCG;
};

}//Namespace LWR

#endif//_FRI_COMPONENT_HPP_


    


    
