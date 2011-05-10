/*
 * KukaLWR_Kynematics.hpp
 *
 *  Created on: May 6, 2011
 *      Author: Gajan (gajan@ethz.ch)
 */
#ifndef KUKALWR_KINEMATICS_HPP_
#define KUKALWR_KINEMATICS_HPP_

#include <rtt/RTT.hpp>

#include <geometry_msgs/Pose.h>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>

#define PI 3.14159

namespace trajectory_generators {

class KukaLWR_Kinematics {
public:
	//TODO: Question: Do we need the constructor and destructor when our class is static??
	KukaLWR_Kinematics();
	virtual ~KukaLWR_Kinematics();
	//TODO:Question: Should we make the input of the functions const so that it be clear it cannot be changed?
	static bool ikSolver(geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr);
	static bool fkSolver(std::vector<double> & jntPosDsr, geometry_msgs::Pose & poseDsr);
};

}

#endif /* KUKALWR_KYNEMATICS_HPP_ */
