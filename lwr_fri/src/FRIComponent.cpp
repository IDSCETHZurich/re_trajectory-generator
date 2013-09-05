// Copyright  (C)  2009  Ruben Smits <ruben dot_cart smits at mech dot kuleuven dot be>,
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

#include <ocl/Component.hpp>

#include <rtt/Logger.hpp>
#include <kdl/frames.hpp>

#include <netinet/in.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <string>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "FRIComponent.hpp"
#include <tf_conversions/tf_kdl.h>

namespace lwr_fri {

using namespace RTT;

FRIComponent::FRIComponent(const std::string& name) :
	TaskContext(name, PreOperational){

	this->addAttribute("fromKRL", m_fromKRL);
	this->addAttribute("toKRL", m_toKRL);

	this->addPort("RobotData", m_RobotDataPort).doc(
			"Port containing the data of the robot");
	this->addPort("RobotState", m_RobotStatePort).doc(
			"Port containing the status of the robot");
	this->addPort("FRIState", m_FriStatePort).doc(
			"Port containing the status of the FRI communication");
	this->addPort("msrJntPos", m_msrJntPosPort);
	this->addPort("cmdJntPos", m_cmdJntPosPort);
	this->addPort("cmdJntPosFriOffset", m_cmdJntPosFriOffsetPort);
	this->addPort("msrCartPos", m_msrCartPosPort);
	this->addPort("cmdCartPos", m_cmdCartPosPort);
	this->addPort("cmdCartPosFriOffset", m_cmdCartPosFriOffsetPort);
	this->addPort("msrJntTrq", m_msrJntTrqPort);
	this->addPort("estExtJntTrq", m_estExtJntTrqPort);
	this->addPort("estExtTcpWrench", m_estExtTcpWrenchPort);
	this->addPort("desJntPos", m_jntPosPort);
	this->addPort("desJntVel", m_jntVelPort);
	this->addPort("desCartPos", m_cartPosPort);
	this->addPort("desCartTwist", m_cartTwistPort);
	this->addPort("desAddJntTrq", m_addJntTrqPort);
	this->addPort("desAddTcpWrench", m_addTcpWrenchPort);
	//this->addPort("desJntImpedance", m_jntImpedancePort);
	//this->addPort("desCartImpedance", m_cartImpedancePort);

	this->addProperty("local_port", m_local_port);
	this->addProperty("control_mode", m_control_mode).doc("1=JntPos, 2=JntVel, 3=JntTrq, 4=CartPos, 5=CartForce, 6=CartTwist");

	m_jntPos.resize(LBR_MNJ);
	m_jntVel.resize(LBR_MNJ);
	m_jntTorques.resize(LBR_MNJ);

}

FRIComponent::~FRIComponent() {
}

bool FRIComponent::configureHook() {
	//IDSC Debugging
	time_begin = os::TimeService::Instance()->getTicks();

	//Check the sizes of all data:
	if (!FRI_CHECK_SIZES_OK) {
		log(Error) << "Padding on this platform is not OK :(" << endlog();
		return false;
	}
	//Check the byte order and float representation:
	{
		FRI_PREPARE_CHECK_BYTE_ORDER;
		if (!FRI_CHECK_BYTE_ORDER_OK) {
			log(Error)
					<< "Byte order and float representations are not OK on this platform :("
					<< endlog();
			return false;
		}
	}

	m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, 0, 0);

	struct sockaddr_in local_addr;
	bzero((char *) &local_addr, sizeof(local_addr));
	local_addr.sin_family = AF_INET;
	local_addr.sin_addr.s_addr = INADDR_ANY;
	local_addr.sin_port = htons(m_local_port);

	if (bind(m_socket, (sockaddr*) &local_addr, sizeof(sockaddr_in)) < 0) {
		log(Error) << "Binding of port failed with errno " << errno << endlog();
		return false;
	}

	//Add trajectoryGenerator as peer
	if(this->hasPeer("trajectoryGenerator")){
		updateGenerator = this->getPeer("trajectoryGenerator")->getOperation("updateTG");
	}

	if(this->hasPeer("cartesianGenerator")){
		updateGenerator = this->getPeer("cartesianGenerator")->getOperation("updateCG");
	}

	return true;

}

bool FRIComponent::startHook() {
	counter = 0;
	return true;
}

void FRIComponent::updateHook() {
	//Read:
	socklen_t addr_len = sizeof(m_remote_addr);

	//time_passed = os::TimeService::Instance()->secondsSince(time_begin);
	int n = recvfrom(m_socket, (void*) &m_msr_data, sizeof(m_msr_data), 0,
			&m_remote_addr, &addr_len);
	//time_passed = os::TimeService::Instance()->secondsSince(time_begin);

	if (sizeof(tFriMsrData) != n)
		log(Error) << "bad packet lenght: " << n << ", expected: "
				<< sizeof(tFriMsrData) << endlog();
	else {
		m_RobotStatePort.write(m_msr_data.robot);
		m_RobotDataPort.write(m_msr_data.data);
		m_FriStatePort.write(m_msr_data.intf);

		/*
		 if(msr_data.robot.power==0){
		 log(Warning)<<"Robot power down!! Stopping!!"<<endlog();
		 this->stop();
		 }
		 */
		m_fromKRL = m_msr_data.krl;
		for (unsigned int i = 0; i < LBR_MNJ; i++)
			m_jntPos[i] = m_msr_data.data.msrJntPos[i];
		m_msrJntPosPort.write(m_jntPos);

		for (unsigned int i = 0; i < LBR_MNJ; i++)
			m_jntPos[i] = m_msr_data.data.cmdJntPos[i];
		m_cmdJntPosPort.write(m_jntPos);

		for (unsigned int i = 0; i < LBR_MNJ; i++)
			m_jntPos[i] = m_msr_data.data.cmdJntPosFriOffset[i];
		m_cmdJntPosFriOffsetPort.write(m_jntPos);

		geometry_msgs::Quaternion quat;
		KDL::Frame cartPos;
		cartPos.M=KDL::Rotation(m_msr_data.data.msrCartPos[0],
				m_msr_data.data.msrCartPos[1], m_msr_data.data.msrCartPos[2],
				m_msr_data.data.msrCartPos[4], m_msr_data.data.msrCartPos[5],
				m_msr_data.data.msrCartPos[6], m_msr_data.data.msrCartPos[8],
				m_msr_data.data.msrCartPos[9], m_msr_data.data.msrCartPos[10]);
		cartPos.p.x(m_msr_data.data.msrCartPos[3]);
		cartPos.p.y(m_msr_data.data.msrCartPos[7]);
		cartPos.p.z(m_msr_data.data.msrCartPos[11]);
		tf::PoseKDLToMsg(cartPos,m_cartPos);
		m_msrCartPosPort.write(m_cartPos);

		cartPos.M = KDL::Rotation(m_msr_data.data.cmdCartPos[0],
				m_msr_data.data.cmdCartPos[1], m_msr_data.data.cmdCartPos[2],
				m_msr_data.data.cmdCartPos[4], m_msr_data.data.cmdCartPos[5],
				m_msr_data.data.cmdCartPos[6], m_msr_data.data.cmdCartPos[8],
				m_msr_data.data.cmdCartPos[9], m_msr_data.data.cmdCartPos[10]);
		cartPos.p.x(m_msr_data.data.cmdCartPos[3]);
		cartPos.p.y(m_msr_data.data.cmdCartPos[7]);
		cartPos.p.z(m_msr_data.data.cmdCartPos[11]);
		tf::PoseKDLToMsg(cartPos,m_cartPos);
		m_cmdCartPosPort.write(m_cartPos);

		cartPos.M = KDL::Rotation(m_msr_data.data.cmdCartPosFriOffset[0],
				m_msr_data.data.cmdCartPosFriOffset[1],
				m_msr_data.data.cmdCartPosFriOffset[2],
				m_msr_data.data.cmdCartPosFriOffset[4],
				m_msr_data.data.cmdCartPosFriOffset[5],
				m_msr_data.data.cmdCartPosFriOffset[6],
				m_msr_data.data.cmdCartPosFriOffset[8],
				m_msr_data.data.cmdCartPosFriOffset[9],
				m_msr_data.data.cmdCartPosFriOffset[10]);
		cartPos.p.x(m_msr_data.data.cmdCartPosFriOffset[3]);
		cartPos.p.y(m_msr_data.data.cmdCartPosFriOffset[7]);
		cartPos.p.z(m_msr_data.data.cmdCartPosFriOffset[11]);
		tf::PoseKDLToMsg(cartPos,m_cartPos);
		m_cmdCartPosFriOffsetPort.write(m_cartPos);

		for (unsigned int i = 0; i < LBR_MNJ; i++)
			m_jntTorques[i] = m_msr_data.data.msrJntTrq[i];

		m_msrJntTrqPort.write(m_jntTorques);

		for (unsigned int i = 0; i < LBR_MNJ; i++)
			m_jntTorques[i] = m_msr_data.data.estExtJntTrq[i];
		m_estExtJntTrqPort.write(m_jntTorques);

		m_cartWrench.force.x = m_msr_data.data.estExtTcpFT[0];
		m_cartWrench.force.y = m_msr_data.data.estExtTcpFT[1];
		m_cartWrench.force.z = m_msr_data.data.estExtTcpFT[2];
		m_cartWrench.torque.x = m_msr_data.data.estExtTcpFT[5];
		m_cartWrench.torque.y = m_msr_data.data.estExtTcpFT[4];
		m_cartWrench.torque.z = m_msr_data.data.estExtTcpFT[3];
		m_estExtTcpWrenchPort.write(m_cartWrench);

		//Fill in datagram to send:
		m_cmd_data.head.datagramId = FRI_DATAGRAM_ID_CMD;
		m_cmd_data.head.packetSize = sizeof(tFriCmdData);
		m_cmd_data.head.sendSeqCount = ++counter;
		m_cmd_data.head.reflSeqCount = m_msr_data.head.sendSeqCount;

		///TODO: How are we choosing this? -> only change in monitor mode
		if (m_msr_data.intf.state == FRI_STATE_MON) {
			if (m_control_mode == 1 || m_control_mode == 2) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_JNTPOS;
				for (unsigned int i = 0; i < LBR_MNJ; i++)
					m_cmd_data.cmd.jntPos[i] = m_msr_data.data.cmdJntPos[i];
			} else if (m_control_mode == 3) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_JNTTRQ;
				for (unsigned int i = 0; i < LBR_MNJ; i++)
					m_cmd_data.cmd.addJntTrq[i] = 0.0;
			} else if (m_control_mode == 4 || m_control_mode == 6) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_CARTPOS;
				for (unsigned int i = 0; i < FRI_CART_FRM_DIM; i++)
					m_cmd_data.cmd.cartPos[i] = m_msr_data.data.cmdCartPos[i];
			} else if (m_control_mode == 5) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_TCPFT;
				for (unsigned int i = 0; i < FRI_CART_VEC; i++)
					m_cmd_data.cmd.addTcpFT[i] = 0.0;
			}
		}
		//Only send if state is in FRI_STATE_CMD
		if (m_msr_data.intf.state == FRI_STATE_CMD) {
			if (m_control_mode == 1) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_JNTPOS;
				if(updateGenerator()){
					if (NewData == m_jntPosPort.read(m_jntPos)){
						last_cmd_jnt_pos.clear();
						last_cmd_jnt_pos.reserve(LBR_MNJ);
						for (unsigned int i = 0; i < LBR_MNJ; i++){
							m_cmd_data.cmd.jntPos[i] = m_jntPos[i];
							last_cmd_jnt_pos.push_back(m_jntPos[i]);
						}
//					std::cout << "FRI: "<< m_jntPos[0] << " " << m_jntPos[1] << " " << m_jntPos[2] << " "
//							<< m_jntPos[3] << " " << m_jntPos[4] << " " << m_jntPos[5] << " "
//							<< m_jntPos[6] << std::endl;
					}else{
						//std::cout << "NO new data" << std::endl;
					}
				}else{
					//std::cout << "sending measurements" << std::endl;
					for (unsigned int i = 0; i < LBR_MNJ; i++)
						m_cmd_data.cmd.jntPos[i] = m_msr_data.data.msrJntPos[i];
				}
				/*
				}else{
					std::cout << "updateGenerator() returned false" << std::endl;
 					if (last_cmd_jnt_pos.size() == 0) {
 						std::cout << "sending measurements" << std::endl;
 						for (unsigned int i = 0; i < LBR_MNJ; i++)
 							m_cmd_data.cmd.jntPos[i] = m_msr_data.data.msrJntPos[i];

 					} else {
 						std::cout << "sending last cmd data" << std::endl;
 						for (unsigned int i = 0; i < LBR_MNJ; i++)
 						m_cmd_data.cmd.jntPos[i] = last_cmd_jnt_pos[i];
 					}
				}
				std::cout << "FRI cmd: " << m_cmd_data.cmd.jntPos[0]*57.2957795 << " " << m_cmd_data.cmd.jntPos[1]*57.2957795 << " "
				 << m_cmd_data.cmd.jntPos[2]*57.2957795 << " " << m_cmd_data.cmd.jntPos[3]*57.2957795 << " "
				 << m_cmd_data.cmd.jntPos[4]*57.2957795 << " " << m_cmd_data.cmd.jntPos[5]*57.2957795 << " "
				 << m_cmd_data.cmd.jntPos[6]*57.2957795 << std::endl;
			*/

			} else	if (m_control_mode == 2) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_JNTPOS;
				if (NewData == m_jntVelPort.read(m_jntVel))
					for (unsigned int i = 0; i < LBR_MNJ; i++)
						m_cmd_data.cmd.jntPos[i] += m_jntVel[i]*m_msr_data.intf.desiredCmdSampleTime;
			} else if (m_control_mode == 3) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_JNTTRQ;
				if (NewData == m_addJntTrqPort.read(m_jntTorques))
					for (unsigned int i = 0; i < LBR_MNJ; i++)
						m_cmd_data.cmd.addJntTrq[i]
								= m_jntTorques[i];
			} else if (m_control_mode == 4) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_CARTPOS;
				if(updateGenerator()){
					if (NewData == m_cartPosPort.read(m_cartPos)) {
						KDL::Rotation rot = KDL::Rotation::Quaternion(
								m_cartPos.orientation.x, m_cartPos.orientation.y,
								m_cartPos.orientation.z, m_cartPos.orientation.w);
						m_cmd_data.cmd.cartPos[0] = rot.data[0];
						m_cmd_data.cmd.cartPos[1] = rot.data[1];
						m_cmd_data.cmd.cartPos[2] = rot.data[2];
						m_cmd_data.cmd.cartPos[4] = rot.data[3];
						m_cmd_data.cmd.cartPos[5] = rot.data[4];
						m_cmd_data.cmd.cartPos[6] = rot.data[5];
						m_cmd_data.cmd.cartPos[8] = rot.data[6];
						m_cmd_data.cmd.cartPos[9] = rot.data[7];
						m_cmd_data.cmd.cartPos[10] = rot.data[8];

						m_cmd_data.cmd.cartPos[3] = m_cartPos.position.x;
						m_cmd_data.cmd.cartPos[7] = m_cartPos.position.y;
						m_cmd_data.cmd.cartPos[11] = m_cartPos.position.z;

						//std::cout << "x = " << m_cartPos.position.x << std::endl;
					}//end of if NewData
				}//end of if updateCG
				else{// this means the Cartesian generator is still not initialized, i.e., received it's first command.
					for (unsigned int i = 0; i < FRI_CART_FRM_DIM; i++)
						m_cmd_data.cmd.cartPos[i] = m_msr_data.data.msrCartPos[i];
				}

			} else if (m_control_mode == 5) {
				m_cmd_data.cmd.cmdFlags = FRI_CMD_TCPFT;
				if (NewData == m_addTcpWrenchPort.read(m_cartWrench)) {
					m_cmd_data.cmd.addTcpFT[0] = m_cartWrench.force.x;
					m_cmd_data.cmd.addTcpFT[1] = m_cartWrench.force.y;
					m_cmd_data.cmd.addTcpFT[2] = m_cartWrench.force.z;
					m_cmd_data.cmd.addTcpFT[3] = m_cartWrench.torque.z;
					m_cmd_data.cmd.addTcpFT[4] = m_cartWrench.torque.y;
					m_cmd_data.cmd.addTcpFT[5] = m_cartWrench.torque.x;
				}
			} else if (m_control_mode == 6) {
			  m_cmd_data.cmd.cmdFlags = FRI_CMD_CARTPOS;
				if (NewData == m_cartTwistPort.read(m_cartTwist)) {
				  KDL::Twist t;
				  tf::TwistMsgToKDL (m_cartTwist, t);
				  KDL::Frame T_old;
				  T_old.M = KDL::Rotation(m_cmd_data.cmd.cartPos[0],
							  m_cmd_data.cmd.cartPos[1],
							  m_cmd_data.cmd.cartPos[2],
							  m_cmd_data.cmd.cartPos[4],
							  m_cmd_data.cmd.cartPos[5],
							  m_cmd_data.cmd.cartPos[6],
							  m_cmd_data.cmd.cartPos[8],
							  m_cmd_data.cmd.cartPos[9],
							  m_cmd_data.cmd.cartPos[10]);
				  T_old.p.x(m_cmd_data.cmd.cartPos[3]);
				  T_old.p.y(m_cmd_data.cmd.cartPos[7]);
				  T_old.p.z(m_cmd_data.cmd.cartPos[11]);
				  
				  KDL::Frame T_new = addDelta (T_old, t, m_msr_data.intf.desiredCmdSampleTime);

				  m_cmd_data.cmd.cartPos[0] = T_new.M.data[0];
				  m_cmd_data.cmd.cartPos[1] = T_new.M.data[1];
				  m_cmd_data.cmd.cartPos[2] = T_new.M.data[2];
				  m_cmd_data.cmd.cartPos[4] = T_new.M.data[3];
				  m_cmd_data.cmd.cartPos[5] = T_new.M.data[4];
				  m_cmd_data.cmd.cartPos[6] = T_new.M.data[5];
				  m_cmd_data.cmd.cartPos[8] = T_new.M.data[6];
				  m_cmd_data.cmd.cartPos[9] = T_new.M.data[7];
				  m_cmd_data.cmd.cartPos[10] = T_new.M.data[8];
				  m_cmd_data.cmd.cartPos[3] = T_new.p.x();
				  m_cmd_data.cmd.cartPos[7] = T_new.p.y();
				  m_cmd_data.cmd.cartPos[11] = T_new.p.z();
				}
			}
		}
		
		m_cmd_data.krl = m_toKRL;

		//time_passed = os::TimeService::Instance()->secondsSince(time_begin);
		//log(Info) << time_passed << " Data3 " << m_jntPos[0] << " " << m_jntPos[1] << " " << m_jntPos[2] << " " << m_jntPos[3] << " " << m_jntPos[4] << " " << m_jntPos[5] << " " << m_jntPos[6] << endlog();
		if (0 > sendto(m_socket, (void*) &m_cmd_data, sizeof(m_cmd_data), 0,
				(sockaddr*) &m_remote_addr, sizeof(m_remote_addr)))
			log(Error) << "Sending datagram failed." << endlog();
	} //end of if-else bad packt length
		this->trigger();
	}

	void FRIComponent::stopHook() {
	}

	void FRIComponent::cleanupHook() {
	}
}//namespace LWR

ORO_CREATE_COMPONENT(lwr_fri::FRIComponent)
