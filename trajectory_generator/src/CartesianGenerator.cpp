// Author : Gajan <gajan@ethz.ch>
// Credits: Ruben Smits <ruben.smits@mech.kuleuven.ac.be> (for template)

#include "CartesianGenerator.hpp"
#include <ocl/Component.hpp>
ORO_LIST_COMPONENT_TYPE(trajectory_generator::CartesianGenerator);
//ORO_CREATE_COMPONENT(trajectory_generator::CartesianGenerator);


namespace trajectory_generator
{

    using namespace RTT;
    using namespace KDL;
    using namespace std;

    CartesianGenerator::CartesianGenerator(string name)
        : TaskContext(name,PreOperational)
    {
        //Creating TaskContext
        //Adding Ports
        this->addPort("CartesianPoseMsr",m_position_meas_port);
        this->addPort("CartesianPoseDes",m_position_desi_port);
        this->addPort("CartesianPoseDes2ROS",m_position_desi_port2ROS); //2ROS
		this->addEventPort("cmdCartPose",cmdCartPose, boost::bind(&CartesianGenerator::generateNewVelocityProfiles, this, _1));        

        //Adding Properties
        this->addProperty("max_vel",m_maximum_velocity).doc("Maximum Velocity in Trajectory");
        this->addProperty("max_acc",m_maximum_acceleration).doc("Maximum Acceleration in Trajectory");

        //Adding Methods
        this->addOperation("resetPosition",&CartesianGenerator::resetPosition,this,OwnThread).doc("Reset generator's position");
        this->addOperation("updateCG", &CartesianGenerator::updateCG, this, OwnThread);
    }

    CartesianGenerator::~CartesianGenerator()
    {
    }

    bool CartesianGenerator::configureHook()
    {
		return true;
    }

    bool CartesianGenerator::startHook()
    {
		//initialize
		geometry_msgs::Pose pose;

		pose.position.x=0.25;
		pose.position.y=0.0;
		pose.position.z=0.52;//0.83
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;
		m_position_desi_port.write(pose);

		t_sync = 0.0;
		sleep(1);

		return true;

    }

    bool CartesianGenerator::updateCG()
    {
    	m_time_passed = os::TimeService::Instance()->secondsSince(m_time_begin);

    	if (m_time_passed <= t_sync)
    	{
			geometry_msgs::Pose pose;
			double theta;

			pose.position.x= TrajVectorDirection.x * m_maximum_velocity[0] *(m_time_passed) + xi;
			pose.position.y= TrajVectorDirection.y * m_maximum_velocity[0] *(m_time_passed) + yi;
			pose.position.z= TrajVectorDirection.z * m_maximum_velocity[0] *(m_time_passed) + zi;
			cout << "--- x = " << pose.position.x << endl;

			theta = theta_vel * m_time_passed;
			//cout << "--- Theta: " << theta << endl;

			Vector3d q;
			q = currentRotationalAxis*sin(theta/2);

			KDL::Rotation errorRotation = KDL::Rotation::Quaternion(q(0), q(1), q(2), cos(theta/2));

			(errorRotation*m_traject_begin.M).GetQuaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);

			m_position_desi_port.write(pose);

			//TO ROS Visualization
			geometry_msgs::PoseStamped poseStamped;
			poseStamped.header.frame_id = "/frame_id_1";
			poseStamped.header.stamp = ros::Time::now();
			poseStamped.pose = pose;
			m_position_desi_port2ROS.write(poseStamped);

			return true;
    	}
    	else
    	{
    		return false;
    	}
    }


    void CartesianGenerator::updateHook()
    {
    	updateCG();
    }

    void CartesianGenerator::stopHook()
    {
    }

    void CartesianGenerator::cleanupHook()
    {
    }

    bool CartesianGenerator::generateNewVelocityProfiles(RTT::base::PortInterface* portInterface)
    {

    	//m_time_passed = os::TimeService::Instance()->secondsSince(m_time_begin);
    	
    	geometry_msgs::Pose pose;
    	cmdCartPose.read(pose);
    	desired_pose = pose;

#if 1
    	std::cout << "A new pose arrived" << std::endl;
    	std::cout << "position: " << pose.position.x <<  " " << pose.position.y << " " << pose.position.z  << std::endl;
#endif

    	m_traject_end.p.x(pose.position.x);
		m_traject_end.p.y(pose.position.y);
		m_traject_end.p.z(pose.position.z);
		m_traject_end.M=Rotation::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);

		// get current position
		geometry_msgs::Pose pose_meas;

		m_position_meas_port.read(pose_meas);
		m_traject_begin.p.x(pose_meas.position.x);
		m_traject_begin.p.y(pose_meas.position.y);
		m_traject_begin.p.z(pose_meas.position.z);
		m_traject_begin.M=Rotation::Quaternion(pose_meas.orientation.x,pose_meas.orientation.y,pose_meas.orientation.z,pose_meas.orientation.w);

		xi = pose_meas.position.x;
		yi = pose_meas.position.y;
		zi = pose_meas.position.z;
		xf = pose.position.x;
		yf = pose.position.y;
		zf = pose.position.z;

		std::cout << "- Current x pos = " << xi << std::endl;

		TrajVectorMagnitude = sqrt( (xf-xi)*(xf-xi)+(yf-yi)*(yf-yi)+(zf-zi)*(zf-zi) );
		TrajVectorDirection.x = (xf-xi)/TrajVectorMagnitude;
		TrajVectorDirection.y = (yf-yi)/TrajVectorMagnitude;
		TrajVectorDirection.z = (zf-zi)/TrajVectorMagnitude;

//		double tx,ty,tz;
//		tx = abs(xf-xi)/m_maximum_velocity[0];
//		ty = abs(yf-yi)/m_maximum_velocity[0];
//		tz = abs(zf-zi)/m_maximum_velocity[0];

		t_sync = TrajVectorMagnitude/ m_maximum_velocity[0];

		KDL::Rotation errorRotation = (m_traject_end.M)*(m_traject_begin.M.Inverse());

		double x,y,z,w;
		errorRotation.GetQuaternion(x,y,z,w);

		Eigen::AngleAxis<double> aa;
		aa = Eigen::Quaterniond(w,x,y,z);
		currentRotationalAxis = aa.axis();
		deltaTheta = aa.angle();
		theta_vel = deltaTheta/t_sync;

		cout << "[CG::GenerateProfiles]:" << endl;

		m_time_begin = os::TimeService::Instance()->getTicks();
		m_time_passed = 0;

		return true;
    }

    void CartesianGenerator::resetPosition()
    {
    	std::cout << "resetPosition() called" << std::endl;
    	geometry_msgs::Pose pose;
		m_position_meas_port.read(pose);
		m_position_desi_port.write(pose);

    }
}//namespace
