/*
 * KukaLWR_IK.cpp
 *
 *  Created on: May 6, 2011
 *      Author: demo
 */

#include "KukaLWR_IK.hpp"

namespace trajectory_generators {

bool KukaLWR_IK::ikSolver(geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr){
	//TODO: Check size of jntPosDsr. Should be 7
    bool output = true;
    double mod_pW, mod_pWxy, c2, s2, c3, s3;

    //TODO: Move these into a constants file
    double a2 = 0.4, a3 = 0.39, d6 = 0.078;
    std::vector<double> limit = std::vector<double>(7,170.0*PI/180.0);
    limit[1] = 120.0*PI/180.0;
    limit[3] = 120.0*PI/180.0;
    limit[5] = 120.0*PI/180.0;

    KDL::Rotation R = KDL::Rotation::Quaternion(poseDsr.orientation.x,poseDsr.orientation.y,poseDsr.orientation.z,poseDsr.orientation.w);
    KDL::Vector p(poseDsr.position.x, poseDsr.position.y, poseDsr.position.z);

    // wrist position
    KDL::Vector pW = p - d6 * R.UnitZ();

    // Calculate wrist position
    jntPosDsr[0] = std::atan2(pW[1], pW[0]);
    mod_pW = pow(pW.Norm(),2);

    c3 = (mod_pW - a2*a2 - a3*a3)/(2*a2*a3);
    // If c3>1, there is no solution for IKT
    if (c3>1){
		output = false;
		cout << "Warning!!! Attempt to access to a point out of the workspace. Zero array will be returned." << endl;
		jntPosDsr = std::vector<double>(7,0.0);
		return output;
    }

    s3 = -sqrt(1-c3*c3);
    jntPosDsr[3] = atan2(s3,c3)+PI/2;

    jntPosDsr[2] = 0.0;

    mod_pWxy = sqrt(pW[0]*pW[0] + pW[1]*pW[1]);
    s2 = ((a2+a3*c3)*pW[2] - a3*s3*mod_pWxy)/mod_pW;
    c2 = ((a2+a3*c3)*mod_pWxy + a3*s3*pW[2])/mod_pW;
    jntPosDsr[1] = atan2(s2,c2);

    // Calculate orientation (angles of the wrist joints)
    KDL::Rotation T01(cos(jntPosDsr[0]), 0.0, sin(jntPosDsr[0]), sin(jntPosDsr[0]), 0.0, -cos(jntPosDsr[0]), 0.0, 1.0, 0.0);
    KDL::Rotation T12(cos(jntPosDsr[1]), -sin(jntPosDsr[1]), 0.0, sin(jntPosDsr[1]), cos(jntPosDsr[1]), 0.0, 0.0 , 0.0, 1.0);
    KDL::Rotation T23(cos(jntPosDsr[3]), 0.0, sin(jntPosDsr[3]), sin(jntPosDsr[3]), 0.0, -cos(jntPosDsr[3]), 0.0, 1.0, 0.0);

    KDL::Rotation pose03 = T01*T12*T23;
    KDL::Rotation pose36 = pose03.Inverse() * R;

    jntPosDsr[4] = atan2(pose36(1,2),pose36(0,2));
    jntPosDsr[5] = atan2(sqrt(pose36(0,2)*pose36(0,2) + pose36(1,2)*pose36(1,2)), pose36(2,2));
    jntPosDsr[6] = atan2(pose36(2,1),-pose36(2,0));

    //Adjust to robot from IK coordinates (keeping joint coord. within the interval [-pi,pi])
    jntPosDsr[1] < -PI/2? jntPosDsr[1]+=3*PI/2 : jntPosDsr[1]-=PI/2;
    jntPosDsr[3] < -PI/2? jntPosDsr[3]+=3*PI/2 : jntPosDsr[3]-=PI/2;
    jntPosDsr[6] <     0? jntPosDsr[6]+=  PI   : jntPosDsr[6]-=PI;

    jntPosDsr[3] = -jntPosDsr[3]; //Correcting for the RobotRotation

    for(int i=0; i < (int)jntPosDsr.size(); i++){
    	if (jntPosDsr[i] < -limit[i]){
    		output = false;
    		jntPosDsr[i] = -limit[i];
    		cout << "Warning!!! IK gives values out of bounds for joint " << i << endl;
    	}
    	else if (jntPosDsr[i] > limit[i]) {
    		output = false;
    		jntPosDsr[i] = limit[i];
    		cout << "Warning!!! IK gives values out of bounds for joint " << i << endl;
    	}
    	cout << i << ":" << jntPosDsr[i] << endl;
    }

    return output;
}


}
