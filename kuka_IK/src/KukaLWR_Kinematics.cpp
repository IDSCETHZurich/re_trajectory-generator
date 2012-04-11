#include "KukaLWR_Kinematics.hpp"

using namespace std;
using namespace KDL;


#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define DISTANCE_FROM_SINGULARITY 0.5*PI/180 //in rads!

#define DEBUG_IK 1

// Function declarations
std::vector< std::vector<double> > Tangent_type( double an,double bn,double cn,double ad,double bd,double cd,double theta_low,double theta_up,int &setsnumber);
std::vector< std::vector<double> > Cosine_type( double a, double b, double c, double theta_low, double theta_up,int &setsnumber);
std::vector<double> set_intersection (std::vector<double> set1, std::vector<double> set2, bool &intersection_is_empty );
KDL::Rotation Matrix_Transpose(KDL::Rotation A);


namespace kuka_IK {


// Joint angle limits
const double KukaLWR_Kinematics::JNT_LIMITS [] = {170.0*PI/180.0,
		120.0*PI/180.0,
		170.0*PI/180.0,
		120.0*PI/180.0,
		170.0*PI/180.0,
		120.0*PI/180.0,
		170.0*PI/180.0};

//#######################################################################################################
//################################ IK and FK functions ##################################################
//#######################################################################################################

bool KukaLWR_Kinematics::ikSolver(std::vector<double> & jntPosMsr, geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr)
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
#if DEBUG
		cout << i << ":" << jntPosDsr[i] << endl;
#endif
	}

	return output;
}

bool KukaLWR_Kinematics::ikSolverIterative7DOF(std::vector<double> & jntPosMsr, geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr) {
	// Define chain
	KDL::Chain chain = Chain();
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Rotation::RotZ(M_PI), Vector(0.0, 0.0, 0.31))));
	chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RotZ(M_PI), Vector(0.0, 0.0, 0.2))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.2))));
	chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RotZ(M_PI), Vector(0.0, 0.0, 0.2))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.19))));
	chain.addSegment(Segment(Joint(Joint::RotY), Frame(Rotation::RotZ(M_PI))));
	chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.0, 0.0, 0.078))));

	//Creation of joint arrays:
	int nj = chain.getNrOfJoints();
	JntArray q(nj);
	JntArray q_init(nj);

	JntArray q_min(nj);
	JntArray q_max(nj);
	for(int i = 0; i < nj; i++) {
		q_min(i) = -JNT_LIMITS[i];
		q_max(i) = JNT_LIMITS[i];
	}

	// Create the solvers
	ChainFkSolverPos_recursive fksolver1(chain); //Forward position solver
	ChainIkSolverVel_pinv iksolver1v(chain); //Inverse velocity solver
	//ChainIkSolverVel_wdls iksolver1v(chain);
	ChainIkSolverPos_NR_JL iksolver1(chain, q_min, q_max, fksolver1, iksolver1v, 1000, 1e-4); // Newton-Raphson with Joint Limits
	//ChainIkSolverPos_NR iksolver1(chain, fksolver1, iksolver1v, 1000, 1e-3); // Newton-Raphson without Joint Limits

	// Load initial values
	for(int i = 0; i < nj; i++) {
		q_init(i) = jntPosMsr[i];
#if DEBUG
		cout << q_init(i) << " ";
#endif
	}

	// Convert request to KDL
	Frame F_dest;
	tf::PoseMsgToKDL(poseDsr, F_dest);

	// Call solver
	int ret = iksolver1.CartToJnt(q_init, F_dest, q);
#if DEBUG
	std::cout << " q_init:  " ;
	for(int i=0; i<7; i++) {
		std::cout <<  q_init(i) << " "  ;
	}
	std::cout  << std::endl;

	std::cout << " q: " ;
	for(int i=0; i<7; i++) {
		std::cout <<  q(i) << " "  ;
	}
	std::cout  << std::endl;

	std::cout << "RET:"<< ret << std::endl;
#endif

	// Interprete results
	if (ret < 0) {
		for(int i = 0; i < nj; i++)
			jntPosDsr[i] = q_init(i);
		return false;
	} else {
		for(int i = 0; i < nj; i++) {
			jntPosDsr[i] = q(i);
		}
		return true;
	}
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
	// a , alpha, d, theta
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


// #################################################################################################################
// #################################################################################################################
// #################################################################################################################
// ############################### ANALYTICAL IK SOLVER 7DOF #######################################################
// #################################################################################################################
// #################################################################################################################
// #################################################################################################################
// Implementation from paper: "Analytical Inverse Kinematic Computation for 7-DOF Redundant Manipulators...
// With Joint Limits and Its Application to Redundancy Resolution",
// IEEE TRANSACTIONS ON ROBOTICS, VOL. 24, NO. 5, OCTOBER 2008

bool KukaLWR_Kinematics::ikSolverAnalytical7DOF( geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr)
{

	// Joint angles 1...7 and arm angle psi
	double theta1, theta2, theta3, theta4, theta5, theta6, theta7 , psi;

	#if DEBUG_IK
	cout << "IK begins!" << endl;
	cout << " x= "<<poseDsr.position.x<<" y= "<<poseDsr.position.y<<" z= "<< poseDsr.position.z << endl;
	#endif

	// variables for feasible set of arm angles
	std::vector<double> psiset;
	std::vector< std::vector<double> > psisetstemp;
	std::vector<std::vector<std::vector<double> > > array3D;
	array3D.resize(7);
	std::vector< std::vector<double> > array2D;
	std::vector< std::vector<double> > array2D_new;
	int sets_number;
	int set_number[7];
	bool intersection_is_empty;

	double a , b, alpha, beta,gamma, Delta, s2a,c2a, s1a,c1a;
	double an,bn,cn,ad,bd,cd, ax,bx,cx;
	unsigned int i,j,k;
	KDL::Vector d;
	KDL::Vector R_d;
	KDL::Vector xsw0;


	// robot dimension constants:
	// KUKA_LWR constants
	// Distances b:base, s:shoulder, e:elbow, w:wrist
	double dbs = D1;
	double dse = D3;
	double dew = D5;
	double dwt = D7;

	// Vectors
	KDL::Vector lbs0(0,   0,   dbs);
	KDL::Vector lse3(0,  -dse,   0);
	KDL::Vector lew4(0,   0,   dew);
	KDL::Vector lwt7(0,   0,   dwt);

	// Desired position and orientation matrices of tool-tip as seen from O:
	KDL::Rotation R70des = KDL::Rotation::Quaternion(poseDsr.orientation.x,poseDsr.orientation.y,poseDsr.orientation.z,poseDsr.orientation.w);
	KDL::Vector x70des(poseDsr.position.x, poseDsr.position.y, poseDsr.position.z);
	KDL::Frame T70des( R70des, x70des);

	// Step 1
	// Gives back Elbow joint, theta4 :
	// theta4 depends only on the tool tip position
	xsw0 = x70des  - lbs0 - R70des*lwt7;
	// unit vector of x_sw0
	double xsw0_norm = std::sqrt( xsw0(0)*xsw0(0) + xsw0(1)*xsw0(1) + xsw0(2)*xsw0(2));
	theta4 = std::acos( (xsw0_norm*xsw0_norm - dse*dse - dew*dew ) / (2*dse*dew) );

	// Get refernce plane
	KDL::Rotation R43;
	R43 = KDL::Rotation::RotZ(theta4)*KDL::Rotation::RotX(PI/2);
	KDL::Rotation R32ref;
	R32ref = KDL::Rotation::RotZ(0.0)*KDL::Rotation::RotX(-PI/2);

	double x = xsw0(0);
	double y = xsw0(1);
	double z = xsw0(2);

	d = lse3 + R43*lew4;
	 R_d = R32ref*d;
	 a = R_d(0);
	 b = R_d(2);
	 alpha = a*a+b*b;
	 beta = -2*z*a;
	 gamma = z*z-b*b;
	 Delta = beta*beta - 4*alpha*gamma;
	// solutions sin2, cos2:
	 s2a = (-beta + std::sqrt(Delta))/(2*alpha);
	 c2a = (z - a*s2a)/b;
	 if (x70des(0)==0 && x70des(1)==0)
	 {
		 s1a = 0;
		 c1a = 1;
	 }
	 else
	 {
		 s1a = y/(a*c2a - b*s2a);
		 c1a = x/(a*c2a - b*s2a);
	 }


      // Get As,Bs,Cs,Aw,Bw,Cw matrices
	KDL::Vector vectorX(c1a*c2a, s1a*c2a, s2a );
	KDL::Vector vectorY(c1a*s2a, s1a*s2a, -c2a);
	KDL::Vector vectorZ(-s1a, c1a, 0.0);
	KDL::Rotation R30o_a(vectorX,vectorY,vectorZ);
	KDL::Vector usw0 = xsw0/ xsw0.Norm();
	KDL::Vector vector1(0.0, usw0(2), -usw0(1) );
	KDL::Vector vector2(-usw0(2), 0.0, usw0(0) );
	KDL::Vector vector3(usw0(1), -usw0(0), 0.0 );
	KDL::Rotation usw0skew(vector1,vector2,vector3);
	KDL::Rotation Minus(-1,0,0,0,-1,0,0,0,-1);
	KDL::Rotation As = usw0skew*R30o_a;
	KDL::Rotation Bs = Minus*(usw0skew*usw0skew*R30o_a);
	KDL::Vector col1( usw0(0)*usw0(0), usw0(0)*usw0(1), usw0(0)*usw0(2));
	KDL::Vector col2( usw0(0)*usw0(1), usw0(1)*usw0(1), usw0(1)*usw0(2));
	KDL::Vector col3( usw0(0)*usw0(2), usw0(1)*usw0(2), usw0(2)*usw0(2));
	KDL::Rotation usw0usw0T( col1,col2,col3);
	KDL::Rotation Cs = usw0usw0T*R30o_a;
	KDL::Rotation AsT, BsT,CsT;
	AsT = Matrix_Transpose(As);
	BsT = Matrix_Transpose(Bs);
	CsT = Matrix_Transpose(Cs);
	KDL::Rotation Aw = R43.Inverse()*AsT*R70des;
	KDL::Rotation Bw = R43.Inverse()*BsT*R70des;
	KDL::Rotation Cw = R43.Inverse()*CsT*R70des;

	// ##########################################################
	// ############# Find Feasible Arm angle (psi) Sets: ########
	// ##########################################################

	// Search for feasible psi_sets in all joints:
	// array3D[i] keeps the feasible psi sets for angle_i
	// set_number[i] keeps the number of feasible sets for angle_i
	for (i=0;i<7;i++)
	{
		#if DEBUG_IK
		cout << "joint: "<< i<< endl;
		#endif

		switch (i)
		{
		case 0:  //joint 1
		{
			an = -As(1,1);
			bn = -Bs(1,1);
			cn = -Cs(1,1);
			ad = -As(0,1);
			bd = -Bs(0,1);
			cd = -Cs(0,1);
			array3D[i] = Tangent_type( an, bn, cn, ad, bd, cd, -JNT_LIMITS[i], JNT_LIMITS[i], sets_number);
			set_number[i] = sets_number;
			break;
		}
		case 1: //joint 2
		{
			ax = -As(2,1);
			bx = -Bs(2,1);
			cx = -Cs(2,1);
			array3D[i] = Cosine_type( ax, bx, cx, -JNT_LIMITS[i], JNT_LIMITS[i], sets_number);
			set_number[i] = sets_number;
			break;
		}
		case 2: // joint 3
		{
			an = As(2,2);
			bn = Bs(2,2);
			cn = Cs(2,2);
			ad = -As(2,0);
			bd = -Bs(2,0);
			cd = -Cs(2,0);
			array3D[i] = Tangent_type( an, bn, cn, ad, bd, cd, -JNT_LIMITS[i], JNT_LIMITS[i], sets_number);
			set_number[i] = sets_number;
			break;
		}
		case 3: // joint 4 , elbow joint
		{
			psiset.push_back(-PI);
			psiset.push_back( PI);
			psisetstemp.push_back(psiset);

			//psisetstemp.push_back(psiset);
			array3D[i] = psisetstemp;
			set_number[i] = 1;
			break;
		}
		case 4:
		{
			an = Aw(1,2);
			bn = Bw(1,2);
			cn = Cw(1,2);
			ad = Aw(0,2);
			bd = Bw(0,2);
			cd = Cw(0,2);
			array3D[i] = Tangent_type( an, bn, cn, ad, bd, cd, -JNT_LIMITS[i], JNT_LIMITS[i], sets_number);
			set_number[i] = sets_number;
			break;
		}
		case 5:
		{
			ax = Aw(2,2);
			bx = Bw(2,2);
			cx = Cw(2,2);
			array3D[i] = Cosine_type( ax, bx, cx, -JNT_LIMITS[i], JNT_LIMITS[i], sets_number);
			set_number[i] = sets_number;
			break;
		}
		case 6:
		{
			an = Aw(2,1);
			bn = Bw(2,1);
			cn = Cw(2,1);
			ad = -Aw(2,0);
			bd = -Bw(2,0);
			cd = -Cw(2,0);
			array3D[i] = Tangent_type( an, bn, cn, ad, bd, cd, -JNT_LIMITS[i], JNT_LIMITS[i], sets_number);
			set_number[i] = sets_number;
			break;
		}
		}
	}

	// #######################################################
	// ####### FIND INTERSECTIONS OF FEASIBLE PSI SETS: ######
	// #######################################################
#if DEBUG_IK
	std::cout << endl <<"Finding feasible psi_sets" << endl;
#endif

	// initialize array2D with Psi_1 sets:
	for (i=0;i< (unsigned int)set_number[0]; i++)
		array2D.push_back(array3D[0][i]);


	// Feasible psi sets are stored in array2D
	// array2D_new is used for temporary values
	i=0;
	do{
		for (j=0;j< array2D.size();j++)
		{
			for (k=0; k< (unsigned int)set_number[i+1]; k++)
			{
				psiset = set_intersection (array2D[j], array3D[i+1][k], intersection_is_empty );
				if (intersection_is_empty == false)
				{
					array2D_new.push_back(psiset);
				}
			}
		}
		array2D.clear();
		array2D = array2D_new;
		array2D_new.clear();

		i++;
	}while(i<6);


#if DEBUG_IK
	// Print feasible sets:
	cout << "Feasible sets: "<< endl;
for (i=0;i<array2D.size();i++)
{
	cout << array2D[i][0]*180/PI<<"  " <<array2D[i][1]*180/PI << endl;
}
#endif
//###################### -End of psi sets- ######################################
//###############################################################################

//###############################################################################
//################ Choose PSI angle:##############################################

// If psi optimal exists in feasible sets, psi = psi_optimal
// If it doesn't psi = (start or end of nearby feasible set)


KDL::Vector c1(1, 0, 0 );
KDL::Vector c2(0, 0, 1 );
KDL::Vector c3(0,-1, 0 );
KDL::Rotation R30dT(c1,c2,c3);
KDL::Rotation R70dT = KDL::Rotation::Identity();

KDL::Rotation Ras = As*R30dT;
KDL::Rotation Rbs = Bs*R30dT;
KDL::Rotation Rcs = Cs*R30dT;

double as = Ras(0,0) + Ras(1,1)+ Ras(2,2);
double bs = Rbs(0,0) + Rbs(1,1)+ Rbs(2,2);
double cs = Rcs(0,0) + Rcs(1,1)+ Rcs(2,2);
// Since R70dT is Identity matrix:
double aw = Aw(0,0) + Aw(1,1) + Aw(2,2);
double bw = Bw(0,0) + Bw(1,1) + Bw(2,2);
double cw = Cw(0,0) + Cw(1,1) + Cw(2,2);

// Weights for fs and fw
double rs = 0.5;
double rw = 1.0 - rs;

double asw = rs*as + rw*aw;
double bsw = rs*bs + rw*bw;
double psi_opt1 = 2*atan2(-bsw - sqrt(asw*asw+bsw*bsw), asw );
double psi_opt2 = 2*atan2(-bsw + sqrt(asw*asw+bsw*bsw), asw );
double psi_opt;

double fopt1 = (rs*(as*sin(psi_opt1)+bs*cos(psi_opt1)+cs) + rw*(aw*sin(psi_opt1)+bw*cos(psi_opt1)+cw) )/ (rs+rw) ;
double fopt2 = (rs*(as*sin(psi_opt2)+bs*cos(psi_opt2)+cs) + rw*(aw*sin(psi_opt2)+bw*cos(psi_opt2)+cw) )/ (rs+rw) ;

if (fopt1 > fopt2)
	psi_opt = psi_opt1;
else
	psi_opt = psi_opt2;

//Check if psi_opt is in a feasible set
bool psi_opt_feasible = false;

for (i=0; i<array2D.size(); i++)
{
	if ( psi_opt >= array2D[i][0]  &&  psi_opt <= array2D[i][1]  )
	{
		psi_opt_feasible = true;
		//cout << "psi_opt feasible" << endl;
		break;
	}
}

if (psi_opt_feasible == false)
{
	for (i=0; i<array2D.size(); i++)
	{
		if (psi_opt < array2D[i][0])
		{
			psi_opt = array2D[i][0];
			break;
		}
		else
		{
			psi_opt = array2D[i][1];
			break;
		}
	}

}

	psi = psi_opt;

	#if DEBUG_IK
	cout << "psi = "<< psi*180/PI << endl<<endl;
	#endif

	array2D.clear();
	array3D.clear();
	psisetstemp.clear();
	psiset.clear();



//################ -End of Psi angle ############################################
//###############################################################################

	// Now that we calculated psi, we can continue calculating theta1,2,3,5,6,7
	
	// Step 2
	// Shoulder joints :

	 theta1 = std::atan2( (As(1,1)*std::sin(psi)+Bs(1,1)*std::cos(psi)+Cs(1,1) ), (As(0,1)*std::sin(psi)+Bs(0,1)*std::cos(psi)+Cs(0,1) ) );
	 theta2 = std::acos(-As(2,1)*std::sin(psi)-Bs(2,1)*std::cos(psi)-Cs(2,1));
	 theta3 = std::atan2( -(As(2,2)*std::sin(psi)+Bs(2,2)*std::cos(psi)+Cs(2,2) ),  (As(2,0)*std::sin(psi)+Bs(2,0)*std::cos(psi)+ Cs(2,0) ) );

	// Step 3
	// Wrist joints:
	 theta5 = std::atan2(-(Aw(1,2)*std::sin(psi)+Bw(1,2)*std::cos(psi)+Cw(1,2) ), -( Aw(0,2)*std::sin(psi)+Bw(0,2)*std::cos(psi)+Cw(0,2) ));
	 theta6 = std::acos(Aw(2,2)*std::sin(psi)+ Bw(2,2)*std::cos(psi) + Cw(2,2));
	 theta7 = std::atan2(-( Aw(2,1)*std::sin(psi)+Bw(2,1)*std::cos(psi) + Cw(2,1) ),  (Aw(2,0)*std::sin(psi)+Bw(2,0)*std::cos(psi)+Cw(2,0) ));



#if DEBUG_IK
	cout << "theta1 = "<<theta1*180/PI << endl;
	cout << "theta2 = "<<theta2*180/PI << endl;
	cout << "theta3 = "<<theta3*180/PI << endl;
	cout << "theta4 = "<<theta4*180/PI << endl;
	cout << "theta5 = "<<theta5*180/PI << endl;
	cout << "theta6 = "<<theta6*180/PI << endl;
	cout << "theta7 = "<<theta7*180/PI << endl;
#endif

	jntPosDsr[0] = theta1;
	jntPosDsr[1] = theta2;
	jntPosDsr[2] = theta3;
	jntPosDsr[3] = theta4;
	jntPosDsr[4] = theta5;
	jntPosDsr[5] = theta6;
	jntPosDsr[6] = theta7;

	//stateLogger << theta1 << " " << theta2 << " " << theta3 << " " << theta4 << " " << theta5 << " " << theta6 << " " << theta7 << endl;

#if DEBUG_IK
	cout << "IK ends!" << endl;
#endif

	return true;
} // --End of function "ikSolverAnalytical7DOF()"

} // --End of namespace "kuka_IK"

//##########################################################################################################
//##########################################################################################################
//########################## Secondary Functions: ##########################################################
//##########################################################################################################
//##########################################################################################################

std::vector< std::vector<double> > Tangent_type( double an,double bn,double cn,double ad,double bd,double cd,double theta_low,double theta_up,int &setsnumber)
{
#define DEBUG_IK_TGNT 0

#if DEBUG_IK_TGNT
	cout << "Tangent_type()" <<endl;
#endif

	std::vector<double> psiset1, psiset2, psiset3;
	std::vector< std::vector<double> > psisets;

	double at,bt,ct, cond;
	double psi_0, psi0_minus,psi0_plus, theta_i_minus, theta_i_plus;
	double theta_max, theta_min;
	double psi_a, psi_b, psi_c;
	double y,z,k, alpha, beta, gamma, Delta;
	double sin_psi_a,sin_psi_b, cos_psi_a, cos_psi_b;
	double theta_c;
	double psi_up_a, psi_up_b, psi_low_a, psi_low_b, psi_up_c, psi_low_c, theta_up_c, theta_low_c;
	double psi1,psi2,psi3,psi4;

	at = bd*cn - bn*cd;
	bt = an*cd - ad*cn;
	ct = an*bd - ad*bn;
	cond = at*at + bt*bt - ct*ct;

	if (cond > 0)
	{
		#if DEBUG_IK_TGNT
		cout << "tangent: cond > 0" << endl;
		#endif
		//theta_i is minimized OR(!!!) maximized at the two arm angles:
		psi0_minus = 2*atan2( ( at-sqrt(cond) ) , (bt-ct) );
		psi0_plus  = 2*atan2( ( at+sqrt(cond) ) , (bt-ct) );

		theta_i_minus =  atan2(an*sin(psi0_minus) + bn*cos(psi0_minus) + cn, ad*sin(psi0_minus) + bd*cos(psi0_minus) + cd);
		theta_i_plus  =  atan2(an*sin(psi0_plus ) + bn*cos(psi0_plus ) + cn, ad*sin(psi0_plus)  + bd*cos(psi0_plus)  + cd);

		if (theta_i_minus > theta_i_plus)
		{
			theta_max = theta_i_minus;
			theta_min = theta_i_plus;
			//psi0_min = psi0_plus;
			//psi0_max = psi0_minus;
		}
		else
		{
			theta_max = theta_i_plus;
			theta_min = theta_i_minus;
			//psi0_min = psi0_minus;
			//psi0_max = psi0_plus;
		}

		// Determine profile
		if (theta_min > theta_up || theta_max < theta_low)
		{
			#if DEBUG_IK_TGNT
			cout << "tangent: profile 1" << endl;
			#endif
			//No feasible regions of the arm angle (psi) exist

			psiset1.push_back(2*PI);
			psiset1.push_back(3*PI);
			psisets.push_back(psiset1);
			setsnumber = 0; //for infeasible sets!!!
		}
		else if (theta_min < theta_low && theta_low <= theta_max && theta_max <= theta_up )
		{
			#if DEBUG_IK_TGNT
			cout << "tangent: profile 2" << endl;
			#endif
			y = tan(theta_low)*ad - an;
			z = tan(theta_low)*cd - cn;
			k = bn - tan(theta_low)*bd;
			alpha = y*y + k*k;
			beta  = 2*z*y;
			gamma = z*z - k*k;
			Delta = beta*beta - 4*alpha*gamma;
			sin_psi_a = ( - beta + sqrt(Delta) ) / (2*alpha);
			sin_psi_b = ( - beta - sqrt(Delta) ) / (2*alpha);
			cos_psi_a = ( sin_psi_a*(an-tan(theta_low)*ad) + cn-tan(theta_low)*cd)/ ( tan(theta_low)*bd - bn) ;
			cos_psi_b = ( sin_psi_b*(an-tan(theta_low)*ad) + cn-tan(theta_low)*cd)/ ( tan(theta_low)*bd - bn) ;

			psi_a = atan2( sin_psi_a, cos_psi_a);
			psi_b = atan2( sin_psi_b, cos_psi_b);
			psi_c = (psi_a + psi_b)/2;
			theta_c = atan2( an*sin(psi_c)+bn*cos(psi_c)+cn, ad*sin(psi_c)+bd*cos(psi_c) +cd);

			if (theta_c <= theta_low)
			{
				psiset1.push_back(-PI);
				psiset1.push_back( MIN(psi_a,psi_b));
				psisets.push_back(psiset1);
				psiset2.push_back( MAX(psi_a,psi_b));
				psiset2.push_back(PI);
				psisets.push_back(psiset2);
				setsnumber = 2;
			}
			else
			{
				psiset1.push_back(MIN(psi_a, psi_b));
				psiset1.push_back(MAX(psi_a, psi_b));
				psisets.push_back(psiset1);
				setsnumber = 1;
			}
		}
		else if (theta_low <= theta_min && theta_min <= theta_up && theta_max > theta_up )
		{
			#if DEBUG_IK_TGNT
			cout << "tangent: profile 3" << endl;
			#endif

			y = tan(theta_up)*ad - an;
			z = tan(theta_up)*cd - cn;
			k = bn - tan(theta_up)*bd;
			alpha = y*y + k*k;
			beta  = 2*z*y;
			gamma = z*z - k*k;
			Delta = beta*beta - 4*alpha*gamma;
			sin_psi_a = ( - beta + sqrt(Delta) ) / (2*alpha);
			sin_psi_b = ( - beta - sqrt(Delta) ) / (2*alpha);
			cos_psi_a = ( sin_psi_a*(an-tan(theta_up)*ad) + cn-tan(theta_up)*cd)/ ( tan(theta_up)*bd - bn) ;
			cos_psi_b = ( sin_psi_b*(an-tan(theta_up)*ad) + cn-tan(theta_up)*cd)/ ( tan(theta_up)*bd - bn) ;

			psi_a = atan2( sin_psi_a, cos_psi_a);
			psi_b = atan2( sin_psi_b, cos_psi_b);
			psi_c = (psi_a + psi_b)/2;
			theta_c = atan2( an*sin(psi_c)+bn*cos(psi_c)+cn, ad*sin(psi_c)+bd*cos(psi_c) +cd);

			if (theta_c >= theta_up)
			{
				psiset1.push_back(-PI);
				psiset1.push_back(MIN(psi_a,psi_b));
				psisets.push_back(psiset1);
				psiset2.push_back(MAX(psi_a,psi_b));
				psiset2.push_back(PI);
				psisets.push_back(psiset2);
				setsnumber = 2;
			}
			else
			{
				psiset1.push_back(MIN(psi_a,psi_b));
				psiset1.push_back(MAX(psi_a,psi_b));
				psisets.push_back(psiset1);
				setsnumber = 1;
			}
		}
		else if (theta_min < theta_low && theta_max > theta_up)
		{
			#if DEBUG_IK_TGNT
			cout << "tangent: profile 4" << endl;
			#endif
			// for theta_up:
			y = tan(theta_up)*ad - an;
			z = tan(theta_up)*cd - cn;
			k = bn - tan(theta_up)*bd;
			alpha = y*y + k*k;
			beta  = 2*z*y;
			gamma = z*z - k*k;
			Delta = beta*beta - 4*alpha*gamma;
			sin_psi_a = ( - beta + sqrt(Delta) ) / (2*alpha);
			sin_psi_b = ( - beta - sqrt(Delta) ) / (2*alpha);
			cos_psi_a = ( sin_psi_a*(an-tan(theta_up)*ad) + cn-tan(theta_up)*cd)/ ( tan(theta_up)*bd - bn) ;
			cos_psi_b = ( sin_psi_b*(an-tan(theta_up)*ad) + cn-tan(theta_up)*cd)/ ( tan(theta_up)*bd - bn) ;

			psi_up_a = atan2( sin_psi_a, cos_psi_a);
			psi_up_b = atan2( sin_psi_b, cos_psi_b);
			psi_up_c = (psi_up_a+psi_up_b)/2;
			theta_up_c = atan2( an*sin(psi_up_c)+bn*cos(psi_up_c)+cn, ad*sin(psi_up_c)+bd*cos(psi_up_c) +cd);

			// for theta_low:
			y = tan(theta_low)*ad - an;
			z = tan(theta_low)*cd - cn;
			k = bn - tan(theta_low)*bd;
			alpha = y*y + k*k;
			beta  = 2*z*y;
			gamma = z*z - k*k;
			Delta = beta*beta - 4*alpha*gamma;
			sin_psi_a = ( - beta + sqrt(Delta) ) / (2*alpha);
			sin_psi_b = ( - beta - sqrt(Delta) ) / (2*alpha);
			cos_psi_a = ( sin_psi_a*(an-tan(theta_low)*ad) + cn-tan(theta_low)*cd)/ ( tan(theta_low)*bd - bn) ;
			cos_psi_b = ( sin_psi_b*(an-tan(theta_low)*ad) + cn-tan(theta_low)*cd)/ ( tan(theta_low)*bd - bn) ;

			psi_low_a = atan2( sin_psi_a, cos_psi_a);
			psi_low_b = atan2( sin_psi_b, cos_psi_b);
			psi_low_c = (psi_low_a+psi_low_b)/2;
			theta_low_c = atan2( an*sin(psi_low_c)+bn*cos(psi_low_c)+cn, ad*sin(psi_low_c)+bd*cos(psi_low_c) +cd);

			if (theta_low_c < theta_low && theta_up_c > theta_up)
			{
				psi1 = MIN(psi_up_a,psi_up_b);
				psi2 = MAX(psi_up_a,psi_up_b);
				psi3 = MIN(psi_low_a,psi_low_b);
				psi4 = MAX(psi_low_a,psi_low_b);
				psiset1.push_back(-PI);
				psiset1.push_back(psi1);
				psisets.push_back(psiset1);
				psiset2.push_back(psi2);
				psiset2.push_back(psi3);
				psisets.push_back(psiset2);
				psiset3.push_back(psi4);
				psiset3.push_back(PI);
				psisets.push_back(psiset3);
				setsnumber = 3;
			}
			else if (theta_low_c > theta_low && theta_up_c > theta_up)
			{
				psi1 = MIN(psi_low_a,psi_low_b);
				psi2 = MIN(psi_up_a,psi_up_b);
				psi3 = MAX(psi_up_a,psi_up_b);
				psi4 = MAX(psi_low_a,psi_low_b);
				psiset1.push_back(psi1);
				psiset1.push_back(psi2);
				psisets.push_back(psiset1);
				psiset2.push_back(psi3);
				psiset2.push_back(psi4);
				psisets.push_back(psiset2);
				setsnumber = 2;
			}
			else if (theta_low_c < theta_low && theta_up_c < theta_up)
			{
				psi1 = MIN(psi_up_a,psi_up_b);
				psi2 = MIN(psi_low_a,psi_low_b);
				psi3 = MAX(psi_low_a,psi_low_b);
				psi4 = MAX(psi_up_a,psi_up_b);
				psiset1.push_back(psi1);
				psiset1.push_back(psi2);
				psisets.push_back(psiset1);
				psiset2.push_back(psi3);
				psiset2.push_back(psi4);
				psisets.push_back(psiset2);
				setsnumber = 2;
			}
		}
		else if (theta_low<= theta_min && theta_low <= theta_max && theta_max <= theta_up)
		{
			#if DEBUG_IK_TGNT
			cout << "tangent: profile 5" << endl;
			#endif
			// the entire domain is feasible:
			psiset1.push_back(-PI);
			psiset1.push_back(PI);
			psisets.push_back(psiset1);
			setsnumber = 1;
		}
	}
	else if (cond < 0)
	{
		#if DEBUG_IK_TGNT
		cout<< "tangent: cond<0"<<endl;
		#endif
		// MONOTONIC PROFILE!!!
		y = tan(theta_low)*ad - an;
		z = tan(theta_low)*cd - cn;
		k = bn - tan(theta_low)*bd;
		alpha = y*y + k*k;
		beta  = 2*z*y;
		gamma = z*z - k*k;
		Delta = beta*beta - 4*alpha*gamma;
		sin_psi_a = ( - beta + sqrt(Delta) ) / (2*alpha);
		sin_psi_b = ( - beta - sqrt(Delta) ) / (2*alpha);
		cos_psi_a = ( cn - tan(theta_low)*cd + sin_psi_a * (an-tan(theta_low)*ad) )/ (tan(theta_low)*bd - bn ) ;
		cos_psi_b = ( cn - tan(theta_low)*cd + sin_psi_b * (an-tan(theta_low)*ad) )/ (tan(theta_low)*bd - bn ) ;

		psi_low_a = atan2( sin_psi_a, cos_psi_a );
		psi_low_b = atan2( sin_psi_b, cos_psi_b );

		// only the above are needed
		// this gives the same results!!!???
		y = tan(theta_up)*ad - an;
		z = tan(theta_up)*cd - cn;
		k = bn - tan(theta_up)*bd;
		alpha = y*y + k*k;
		beta  = 2*z*y;
		gamma = z*z - k*k;
		Delta = beta*beta - 4*alpha*gamma;
		sin_psi_a = ( - beta + sqrt(Delta) ) / (2*alpha);
		sin_psi_b = ( - beta - sqrt(Delta) ) / (2*alpha);
		cos_psi_a = ( cn - tan(theta_up)*cd + sin_psi_a * (an-tan(theta_up)*ad) )/ (tan(theta_up)*bd - bn ) ;
		cos_psi_b = ( cn - tan(theta_up)*cd + sin_psi_b * (an-tan(theta_up)*ad) )/ (tan(theta_up)*bd - bn ) ;

		psi_up_a = atan2( sin_psi_a, cos_psi_a );
		psi_up_b = atan2( sin_psi_b, cos_psi_b );

		psiset1.push_back(MIN(psi_low_a,psi_low_b));
		psiset1.push_back(MAX(psi_up_a, psi_up_b));
		psisets.push_back(psiset1);
		setsnumber = 1;
	}
	else // cond == 0
	{
		#if DEBUG_IK_TGNT
		cout << "tangent: cond=0" << endl;
		#endif
		// DISCONTINUOUS PROFILES!!!
		// singularity for arm angle psi:
		psi_0 = 2*atan2(at, (bt-ct) );
		//singularity = [psi_0 psi_0];
		// NOT SURE FOR THIS : ????
		psiset1.push_back(-PI);
		psiset1.push_back(psi_0 - DISTANCE_FROM_SINGULARITY); // keep a safe distance from singularity
		psisets.push_back(psiset1);
		psiset2.push_back(psi_0 + DISTANCE_FROM_SINGULARITY); // keep a safe distance from singularity
		psiset2.push_back(PI);
		psisets.push_back(psiset2);
		setsnumber = 2;
	}

	return psisets;
}
//#######################################################################################################
//########################### Cosine_type() #############################################################
//#######################################################################################################
std::vector< std::vector<double> > Cosine_type( double a, double b, double c, double theta_low, double theta_up, int &setsnumber)
{
#define DEBUG_IK_COS 0

	#if DEBUG_IK_COS
	cout << "Cosine_type()" <<endl;
	#endif

	std::vector<double> psiset1, psiset2;
	std::vector< std::vector<double> > psisets;

	double theta_max, theta_min;
	double alpha, beta,gamma, Delta;
	double sin_psi_a,sin_psi_b,cos_psi_a, cos_psi_b;
	double psi_limit_min_a,psi_limit_min_b, psi_limit_max_a, psi_limit_max_b;
	double psi_limit_a, psi_limit_b;
	double theta_minus, theta_plus;
	double psi0, psi0_minus, psi0_plus;
	double psi1,psi2,psi3,psi4;

	double cond1 = a*a + b*b - (c-1)*(c-1);
	double cond2 = a*a + b*b - (c+1)*(c+1);

	if (cond1 != 0.0 && cond2 != 0.0)
	{
		#if DEBUG_IK_COS
		cout << "cosine: conds!=0" << endl;
		#endif
		psi0_minus = 2*atan2( (-b-sqrt(a*a + b*b) ), a );
		psi0_plus  = 2*atan2( (-b+sqrt(a*a + b*b) ), a );
		theta_minus = acos(a*sin(psi0_minus) + b*cos(psi0_minus) + c ) ;
		theta_plus  = acos(a*sin(psi0_plus)  + b*cos(psi0_plus)  + c ) ;

		if (theta_minus > theta_plus)
		{
			theta_max = theta_minus;
			theta_min = theta_plus;
			//psi0_min = psi0_plus;
			//psi0_max = psi0_minus;
		}
		else
		{
			theta_max = theta_plus;
			theta_min = theta_minus;
			//psi0_min = psi0_minus;
			//psi0_max = psi0_plus;
		}

		if (theta_min > theta_up || theta_max < theta_low)
		{
#if DEBUG_IK_COS
cout << "cosine: 1" << endl;
#endif
			//No feasible regions of the arm angle (psi) exist
			psiset1.push_back(2*PI);
			psiset1.push_back(3*PI);
			psisets.push_back(psiset1);
			setsnumber = 0; //for infeasible sets!!!
		}
		else if (theta_min < theta_low && theta_low <= theta_max && theta_max <= theta_up )
		{
			#if DEBUG_IK_COS
			cout << "cosine: 2" << endl;
			#endif
			alpha = a*a + b*b;
			beta  = -2*a*(cos(theta_low) -c);
			gamma = (cos(theta_low) - c)*(cos(theta_low) - c) - b*b;
			Delta = beta*beta - 4*alpha*gamma;
			sin_psi_a = ( - beta + sqrt(Delta) ) / (2*alpha);
			sin_psi_b = ( - beta - sqrt(Delta) ) / (2*alpha);
			cos_psi_a = (cos(theta_low)-a*sin_psi_a - c) / b;
			cos_psi_b = (cos(theta_low)-a*sin_psi_b - c) / b;

			psi_limit_min_a = atan2( sin_psi_a, cos_psi_a );
			psi_limit_min_b = atan2( sin_psi_b, cos_psi_b );

			if (psi_limit_min_a < psi_limit_min_b)
			{
				//    psi_set = [ -pi psi_limit_min_a     psi_limit_min_b pi  ]
				psiset1.push_back(-PI);
				psiset1.push_back(psi_limit_min_a);
				psiset2.push_back(psi_limit_min_b);
				psiset2.push_back(PI);
				psisets.push_back(psiset1);
				psisets.push_back(psiset2);
				setsnumber = 2;
			}
			else
			{
				//psi_set = [ -pi psi_limit_min_b     psi_limit_min_a pi ];
				psiset1.push_back(-PI);
				psiset1.push_back(psi_limit_min_b);
				psiset2.push_back(psi_limit_min_a);
				psiset2.push_back(PI);
				psisets.push_back(psiset1);
				psisets.push_back(psiset2);
				setsnumber = 2;
			}
		}
		else if (theta_low <= theta_min && theta_min <= theta_up && theta_max > theta_up)
		{
			#if DEBUG_IK_COS
			cout << "cosine: 3" << endl;
			#endif
			alpha = a*a + b*b;
			beta  = -2*a*(cos(theta_up) -c);
			gamma = (cos(theta_up) - c)*(cos(theta_up) - c) - b*b;
			Delta = beta*beta - 4*alpha*gamma;
			sin_psi_a = ( - beta + sqrt(Delta) ) / (2*alpha);
			sin_psi_b = ( - beta - sqrt(Delta) ) / (2*alpha);
			cos_psi_a = (cos(theta_up)-a*sin_psi_a - c) / b;
			cos_psi_b = (cos(theta_up)-a*sin_psi_b - c) / b;
			psi_limit_a = atan2( sin_psi_a, cos_psi_a );
			psi_limit_b = atan2( sin_psi_b, cos_psi_b );

			if (psi_limit_a > psi_limit_b)
			{
				//psi_set = [psi_limit_b psi_limit_a
				psiset1.push_back(psi_limit_b);
				psiset1.push_back(psi_limit_a);
				psisets.push_back(psiset1);
				setsnumber = 1;
			}
			else
			{
				//    psi_set = [psi_limit_a psi_limit_b
				psiset1.push_back(psi_limit_a);
				psiset1.push_back(psi_limit_b);
				psisets.push_back(psiset1);
				setsnumber = 1;
			}
		}
		else if (theta_min < theta_low && theta_max > theta_up)
		{
#if DEBUG_IK_COS
cout << "cosine: 4" << endl;
#endif
			alpha = a*a + b*b;
			beta  = -2*a*(cos(theta_low) -c);
			gamma = (cos(theta_low) - c)*(cos(theta_low) - c) - b*b;
			Delta = beta*beta - 4*alpha*gamma;
			sin_psi_a = ( - beta + sqrt(Delta) ) / (2*alpha);
			sin_psi_b = ( - beta - sqrt(Delta) ) / (2*alpha);
			cos_psi_a = (cos(theta_low)-a*sin_psi_a - c) / b;
			cos_psi_b = (cos(theta_low)-a*sin_psi_b - c) / b;
			psi_limit_min_a = atan2( sin_psi_a, cos_psi_a );
			psi_limit_min_b = atan2( sin_psi_b, cos_psi_b );

			//alpha = a*a + b*b;
			beta  = -2*a*(cos(theta_up) -c);
			gamma = (cos(theta_up) - c)*(cos(theta_up) - c) - b*b;
			Delta = beta*beta - 4*alpha*gamma;
			sin_psi_a = ( - beta + sqrt(Delta) ) / (2*alpha);
			sin_psi_b = ( - beta - sqrt(Delta) ) / (2*alpha);
			cos_psi_a = (cos(theta_up)-a*sin_psi_a - c) / b;
			cos_psi_b = (cos(theta_up)-a*sin_psi_b - c) / b;
			psi_limit_max_a = atan2( sin_psi_a, cos_psi_a );
			psi_limit_max_b = atan2( sin_psi_b, cos_psi_b );

			psi1 = MIN(psi_limit_max_a,psi_limit_max_b);
			psi4 = MAX(psi_limit_max_a,psi_limit_max_b);
			psi2 = MIN(psi_limit_min_a,psi_limit_min_b);
			psi3 = MAX(psi_limit_min_a,psi_limit_min_b);

			//psi_set = [ psi_limit_max1 psi_limit_min1      psi_limit_min2 psi_limit_max2
			//singularity = [-inf -inf];
			psiset1.push_back(psi1);
			psiset1.push_back(psi2);
			psisets.push_back(psiset1);
			psiset2.push_back(psi3);
			psiset2.push_back(psi4);
			psisets.push_back(psiset2);
			setsnumber = 2;
		}
		else if (theta_low<= theta_min && theta_low <= theta_max && theta_max <= theta_up )
		{
#if DEBUG_IK_COS
cout << "cosine: 5" << endl;
#endif
			psiset1.push_back(-PI);
			psiset1.push_back(PI);
			psisets.push_back(psiset1);
			setsnumber = 1;
		}
	}
	// FOR THE FOLLOWING TWO CASES, FURTHER INVESTIGATION MUST BE DONE: !!!
	else if (cond1 == 0.0)
	{
#if DEBUG_IK_COS
cout << "cosine: cond=0" << endl;
#endif
		// singularity at theta_i = 0 for
		psi0 = 2*atan2( a, ( b -(c-1) ) );
		//singularity = [ psi0 psi0];
		//psi_set = [-pi pi     -pi pi     -pi pi];
		psiset1.push_back(-PI);
		psiset1.push_back(psi0 - DISTANCE_FROM_SINGULARITY );
		psisets.push_back(psiset1);
		psiset2.push_back(psi0 + DISTANCE_FROM_SINGULARITY);
		psiset2.push_back(PI );
		psisets.push_back(psiset2);
		setsnumber = 2;
	}
	else if (cond2 == 0)
	{
		//disp('cos: singularity: cond2');
		// singularity at theta_i = pi for
		psi0 = 2*atan2( a, ( b -(c+1) ) );
		//singularity = [psi0 psi0];
		//psi_set = [-pi pi    -pi pi    -pi pi];
		psiset1.push_back(-PI);
		psiset1.push_back(psi0 - DISTANCE_FROM_SINGULARITY );
		psisets.push_back(psiset1);
		psiset2.push_back(psi0 + DISTANCE_FROM_SINGULARITY);
		psiset2.push_back(PI );
		psisets.push_back(psiset2);
		setsnumber = 2;
	}

	return psisets;
} // -- End of Cosine_type()

//###############################################################################################
//########################## set_intersection() #################################################
//###############################################################################################
std::vector<double> set_intersection (std::vector<double> set1, std::vector<double> set2, bool &intersection_is_empty )
{
	std::vector<double> intersection;

	double set1_start, set1_end, set2_start, set2_end;

	set1_start = set1[0];
	set1_end   = set1[1];
	set2_start = set2[0];
	set2_end   = set2[1];

	if (  set1_end < set2_start   ||  set2_end < set1_start )
	{
		intersection_is_empty = true;
		intersection.push_back(0.0);
		intersection.push_back(0.0);
	}
	else
	{
		intersection_is_empty = false;
		intersection.push_back(MAX(set1_start, set2_start));
		intersection.push_back(MIN(set1_end, set2_end));
	}

return intersection;

} // --End of "set_intersection()

//###########################################################################
//########### Matrix_Transpose() ############################################
//###########################################################################

KDL::Rotation Matrix_Transpose(KDL::Rotation A)
{
	KDL::Vector B1( A(0,0), A(0,1), A(0,2) );
	KDL::Vector B2( A(1,0), A(1,1), A(1,2) );
	KDL::Vector B3( A(2,0), A(2,1), A(2,2) );
	KDL::Rotation B(B1,B2,B3);

	return B;
}

