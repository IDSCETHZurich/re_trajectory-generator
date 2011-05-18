/***************************************************************************

    File:           KukaLWR_Kinematics.cpp
    Author(s):      Gajamohan Mohanarajah/Francisco Ramos
    Affiliation:    IDSC - ETH Zurich
    E-mail:         gajan@ethz.ch/framosde@ethz.ch
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


#include "KukaLWR_Kinematics.hpp"

namespace kuka_IK {

// Vector of constants
const double KukaLWR_Kinematics::JNT_LIMITS [] = {170.0*PI/180.0,
  120.0*PI/180.0,
  170.0*PI/180.0,
  120.0*PI/180.0,
  170.0*PI/180.0,
  120.0*PI/180.0,
  170.0*PI/180.0};

bool KukaLWR_Kinematics::ikSolver(const geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr)
{
	// Returning value
	bool output = true;
	// Initialize variables
	jntPosDsr = std::vector<double>(7,0.0);

    // Auxiliar variables
    double mod_pW, mod_pWxy, c2, s2, c3, s3;

    // Getting transformation matrix
    KDL::Rotation R = KDL::Rotation::Quaternion(poseDsr.orientation.x,poseDsr.orientation.y,poseDsr.orientation.z,poseDsr.orientation.w);
    KDL::Vector p(poseDsr.position.x, poseDsr.position.y, poseDsr.position.z);

    // Inverse kinematics calculations. Closed form solution.
    // Based in the solution for an anthropomorfic arm with an spherical wrist.
    // Extracted from Siciliano et al (2010)
    // wrist position
    KDL::Vector pW = p - D7 * R.UnitZ();

    // Calculate wrist position
    jntPosDsr[0] = std::atan2(pW[1], pW[0]);
    mod_pW = pow(pW.Norm(),2);

    c3 = (mod_pW - D3*D3 - D5*D5)/(2*D3*D5);
    // If c3>1, there is no solution for IKT
    if (c3>1){
		output = false;
		cout << "ikSolver: Attempt to access to a point out of the workspace. Zero array will be returned." << endl;
		jntPosDsr = std::vector<double>(7,0.0);
		return output;
    }

    s3 = -sqrt(1-c3*c3);
    jntPosDsr[3] = atan2(s3,c3)+PI/2;

    // We do not use the extra dof for getting the inverse kinematics
    jntPosDsr[2] = 0.0;

    mod_pWxy = sqrt(pW[0]*pW[0] + pW[1]*pW[1]);
    s2 = ((D3+D5*c3)*pW[2] - D5*s3*mod_pWxy)/mod_pW;
    c2 = ((D3+D5*c3)*mod_pWxy + D5*s3*pW[2])/mod_pW;
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
    	if (abs(jntPosDsr[i]) > JNT_LIMITS[i]){
            output = false;
    		jntPosDsr[i] > 0? jntPosDsr[i] = JNT_LIMITS[i] : jntPosDsr[i] = -JNT_LIMITS[i];
    		cout << "Warning!!! IK gives values out of bounds for joint " << i << endl;
    	}
    	cout << i << ":" << jntPosDsr[i] << endl;
    }

    return output;
}



bool KukaLWR_Kinematics::fkSolver(const std::vector<double> & jntPosDsr, geometry_msgs::Pose & poseDsr)
{
	// Local copy of the desired joint positions vector
    std::vector<double> jntPos(jntPosDsr);
    bool output = true;
	
	//Check requested angles are within the limits
    for(int i=0; i < (int)jntPos.size(); i++){
    	if (abs(jntPos[i]) > JNT_LIMITS[i]){
    		jntPos[i] > 0? jntPos[i] = JNT_LIMITS[i] : jntPos[i] = -JNT_LIMITS[i];
    		cout << "Warning!!! IK gives values out of bounds for joint " << i << endl;
    		output = false;
    	}
    }

    //Adjust to robot to FK coordinates from coordinates in the joint space
    jntPos[1] > 0? jntPos[1]-=PI : jntPos[1]+=PI;
    jntPos[3] > 0? jntPos[3]-=PI : jntPos[3]+=PI;
    jntPos[5] > 0? jntPos[5]-=PI : jntPos[5]+=PI;
    jntPos[6] > 0? jntPos[6]-=PI : jntPos[6]+=PI;

	// Calculate joint's frames based on Denavit-Hartenberg convention
    KDL::Frame F01 = KDL::Frame::DH(0.0,    ALPHA,  0.0,    jntPos[0]);
    KDL::Frame F12 = KDL::Frame::DH(0.0,    ALPHA,  0.0,    jntPos[1]);
    KDL::Frame F23 = KDL::Frame::DH(0.0,    ALPHA,  D3,     jntPos[2]);
    KDL::Frame F34 = KDL::Frame::DH(0.0,    ALPHA,  0.0,    jntPos[3]);
    KDL::Frame F45 = KDL::Frame::DH(0.0,    ALPHA,  D5,     jntPos[4]);
    KDL::Frame F56 = KDL::Frame::DH(0.0,    ALPHA,  0.0,    jntPos[5]);
    KDL::Frame F67 = KDL::Frame::DH(0.0,    0.0,    D7,     jntPos[6]);

    // Only for debugging purposes
/*	KDL::Frame F = F01*F12;
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
*/

	// Forward Kinematics transformation
	KDL::Frame F07 = F01*F12*F23*F34*F45*F56*F67;

	// Generating pose
	geometry_msgs::Point p;
	geometry_msgs::Quaternion q;

	p.x = F07(0,3);
	p.y = F07(1,3);
	p.z = F07(2,3);
	F07.M.GetQuaternion (q.x, q.y, q.z, q.w);

	poseDsr.orientation = q;
	poseDsr.position = p;

	return output;
}

}
