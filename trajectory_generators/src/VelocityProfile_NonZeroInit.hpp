/*
 * VelocityProfile_NonZeroInit.hpp
 *
 *  Created on: Apr 7, 2011
 *      Author: Gajan
 */
#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <ocl/OCL.hpp>

#ifndef VELOCITYPROFILE_NONZEROINIT_H_
#define VELOCITYPROFILE_NONZEROINIT_H_

namespace trajectory_generators {

class VelocityProfile_NonZeroInit{
public:
	VelocityProfile_NonZeroInit(double maxAcc, double maxVel, double finalPos, double initPos, double initVel);
	virtual ~VelocityProfile_NonZeroInit();
	double getDuration(void);
	bool setDuration(double newDuration);
	double getPos(double time);
	double getVel(double time);
private:
	double duration, timeScale;
	// Profile mode in order
	std::vector< std::vector<double> > subVelocityProfiles;
	double subProfileBuilder(double maxAcc, double maxVel, double finalPos, double initPos, double initVel, double initTime);
};

} //end of namespace

#endif /* VELOCITYPROFILE_NONZEROINIT_H_ */
