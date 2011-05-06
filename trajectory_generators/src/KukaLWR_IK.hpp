/*
 * KukaLWR_IK.hpp
 *
 *  Created on: May 6, 2011
 *      Author: Gajan (gajan@ethz.ch)
 */
#ifndef KUKALWR_IK_HPP_
#define KUKALWR_IK_HPP_

#include <rtt/RTT.hpp>

#include <geometry_msgs/Pose.h>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>

#define PI 3.14159

namespace trajectory_generators {

class KukaLWR_IK {
public:
	KukaLWR_IK();
	virtual ~KukaLWR_IK();
	static bool ikSolver(geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr);
};

}

#endif /* KUKALWR_IK_HPP_ */
