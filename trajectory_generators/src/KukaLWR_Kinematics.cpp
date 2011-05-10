/*
 * KukaLWR_Kinematics.cpp
 *
 *  Created on: May 6, 2011
 *      Author: demo
 */

#include "KukaLWR_Kinematics.hpp"

namespace trajectory_generators {

bool KukaLWR_Kinematics::ikSolver(geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr)
{
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

bool KukaLWR_Kinematics::fkSolver(std::vector<double> & jntPosDsr, geometry_msgs::Pose & poseDsr)
{
	//TODO: Include code  that calculates the forward kinematics transform for the KUKALWR
	// Local copy of the desired joint positions vector
    std::vector<double> jointPositions(jntPosDsr);

    //TODO: Move these into a constants file
    double d3 = 0.4, d5 = 0.39, d7 = 0.078;
    std::vector<double> limit = std::vector<double>(7,170.0*PI/180.0);
    limit[1] = 120.0*PI/180.0;
    limit[3] = 120.0*PI/180.0;
    limit[5] = 120.0*PI/180.0;

	
	//Check requested angles are within the limits
    for(int i=0; i < (int)jointPositions.size(); i++){
    	if (jointPositions[i] > abs(limit[i])){
    		jointPositions[i] = -limit[i];
    		cout << "Warning!!! IK gives values out of bounds for joint " << i << endl;
    		return false;
    	}
    }

    //Adjust to robot to FK coordinates from coordinates in the joint space
    jointPositions[1] = -jointPositions[1];
    jointPositions[3] = -jointPositions[3];
    jointPositions[5] = -jointPositions[5];
    jointPositions[1] > 0? jointPositions[1]-=PI : jointPositions[1]+=PI;
    jointPositions[3] > 0? jointPositions[3]-=PI : jointPositions[3]+=PI;
    jointPositions[5] > 0? jointPositions[5]-=PI : jointPositions[5]+=PI;
    jointPositions[6] > 0? jointPositions[6]-=PI : jointPositions[6]+=PI;

    // Calculate orientation transformation between frames
	KDL::Rotation T01(cos(jointPositions[0]), 0.0, sin(jointPositions[0]), sin(jointPositions[0]), 0.0, -cos(jointPositions[0]), 0.0, 1.0, 0.0);
	KDL::Rotation T12(cos(jointPositions[1]), 0.0, sin(jointPositions[1]), sin(jointPositions[1]), 0.0, -cos(jointPositions[1]), 0.0, 1.0, 0.0);
	KDL::Rotation T23(cos(jointPositions[2]), 0.0, sin(jointPositions[2]), sin(jointPositions[2]), 0.0, -cos(jointPositions[2]), 0.0, 1.0, 0.0);
	KDL::Rotation T34(cos(jointPositions[3]), 0.0, sin(jointPositions[3]), sin(jointPositions[3]), 0.0, -cos(jointPositions[3]), 0.0, 1.0, 0.0);
	KDL::Rotation T45(cos(jointPositions[4]), 0.0, sin(jointPositions[4]), sin(jointPositions[4]), 0.0, -cos(jointPositions[4]), 0.0, 1.0, 0.0);
	KDL::Rotation T56(cos(jointPositions[5]), 0.0, sin(jointPositions[5]), sin(jointPositions[5]), 0.0, -cos(jointPositions[5]), 0.0, 1.0, 0.0);
	KDL::Rotation T67(cos(jointPositions[6]), -sin(jointPositions[6]), 0.0, sin(jointPositions[6]), cos(jointPositions[6]), 0.0, 0.0, 0.0, 1.0);
	// Calculate traslation transformation between frames
	KDL::Vector p01(0.0, 0.0, 0.0);
	KDL::Vector p12(0.0, 0.0, 0.0);
	KDL::Vector p23(0.0, 0.0, d3);
	KDL::Vector p34(0.0, 0.0, 0.0);
	KDL::Vector p45(0.0, 0.0, d5);
	KDL::Vector p56(0.0, 0.0, 0.0);
	KDL::Vector p67(0.0, 0.0, d7);
	// Calculate total frames
	KDL::Frame F01(T01, p01);
	KDL::Frame F12(T12, p12);
	KDL::Frame F23(T23, p23);
	KDL::Frame F34(T34, p34);
	KDL::Frame F45(T45, p45);
	KDL::Frame F56(T56, p56);
	KDL::Frame F67(T67, p67);

	//TODO: Check correctness of the FK
	KDL::Frame F = F01*F12;
	//Show results
	cout << "Frame 2" << endl;
	cout << "Coordinate x : " << F.p[0] << endl;
	cout << "Coordinate y : " << F.p[1] << endl;
	cout << "Coordinate z : " << F.p[2] << endl;
	F = F*F23;
	//Show results
	cout << "Frame 3" << endl;
	cout << "Coordinate x : " << F.p[0] << endl;
	cout << "Coordinate y : " << F.p[1] << endl;
	cout << "Coordinate z : " << F.p[2] << endl;

	F = F*F34;
	//Show results
	cout << "Frame 4" << endl;
	cout << "Coordinate x : " << F.p[0] << endl;
	cout << "Coordinate y : " << F.p[1] << endl;
	cout << "Coordinate z : " << F.p[2] << endl;
	F = F*F45;
	//Show results
	cout << "Frame 5" << endl;
	cout << "Coordinate x : " << F.p[0] << endl;
	cout << "Coordinate y : " << F.p[1] << endl;
	cout << "Coordinate z : " << F.p[2] << endl;

	F = F*F56;
	//Show results
	cout << "Frame 6" << endl;
	cout << "Coordinate x : " << F.p[0] << endl;
	cout << "Coordinate y : " << F.p[1] << endl;
	cout << "Coordinate z : " << F.p[2] << endl;

	F = F*F67;
	//Show results
	cout << "Frame 7 (end effector)" << endl;
	cout << "Coordinate x : " << F.p[0] << endl;
	cout << "Coordinate y : " << F.p[1] << endl;
	cout << "Coordinate z : " << F.p[2] << endl;

	//Final orientation
	cout << "Rotation matrix (end effector)" << endl;
	cout << F(0,0) << "\t" << F(0,1) << "\t" << F(0,2) << "\t" << endl;
	cout << F(1,0) << "\t" << F(1,1) << "\t" << F(1,2) << "\t" << endl;
	cout << F(2,0) << "\t" << F(2,1) << "\t" << F(2,2) << "\t" << endl;


	// Forward Kinematics transformation
	KDL::Frame F07 = F01*F12*F23*F34*F45*F56*F67;

	// Generating pose
	geometry_msgs::Point p;
	p.x = F07(0,3);
	p.y = F07(1,3);
	p.z = F07(2,3);
	geometry_msgs::Quaternion q;
	F07.M.GetQuaternion (q.x, q.y, q.z, q.w);

	poseDsr.orientation = q;
	poseDsr.position = p;



	
	

	return true;
}

}
