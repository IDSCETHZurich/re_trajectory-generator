#ifndef __CARTESIAN_GENERATOR_POS_H__
#define __CARTESIAN_GENERATOR_POS_H__

#include <rtt/RTT.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <rtt/Property.hpp>
#include <rtt/PropertyBag.hpp>

#include <kdl/velocityprofile_trap.hpp>
#include <rtt/os/TimeService.hpp>

#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>

//#include <brics_actuator/CartesianPose.h>
//#include <brics_actuator/CartesianTwist.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

#include <sensor_msgs/JointState.h>

#include <std_msgs/Float64.h>

#include <ocl/OCL.hpp>

#include "VelocityProfile_NonZeroInit.hpp"

namespace trajectory_generators
{
    /**
     * The description goes here
     *
     */
    class CartesianGeneratorPos : public RTT::TaskContext
    {
    public:
        /**
         * Constructor of the class.
         *
         * @param name name of the TaskContext
         */
        CartesianGeneratorPos(std::string name);
        virtual ~CartesianGeneratorPos();

        virtual bool configureHook();
        virtual bool startHook();
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();

    private:
      bool moveTo(std::vector<double> position, double time=0);
      bool moveFinished() const;
      void resetPosition();
      bool ikSolver(geometry_msgs::Pose & poseDsr, std::vector<double> & jntPosDsr);

      unsigned int num_axes;
      std::vector<double> p_m, p_d, v_d;
      std::vector<double> v_max, a_max;
      std::vector<VelocityProfile_NonZeroInit> motionProfile;

      RTT::os::TimeService::ticks	time_begin;
      RTT::os::TimeService::Seconds	time_passed;

      geometry_msgs::Pose lastCommandedPose;
      std::vector<double> lastCommndedPoseJntPos;
      std::vector<double> jntVel;

      double max_duration;
      bool is_moving;
      bool toROS;

      ///Robot specific data

    protected:
      /// Dataport containing commanded Cartesian pose
      RTT::InputPort< geometry_msgs::Pose > cmd_cartPosPort;
      /// Dataport containing the measured joint angles
      RTT::InputPort< std::vector<double> > msr_jntPosPort;
      /// Dataport containing the desired joint angles
      RTT::OutputPort< std::vector<double> >  cmd_jntPosPort;
      /// Dataport containing the desired joint angles (In s)
      RTT::OutputPort< sensor_msgs::JointState >  cmd_jntPosPort_toROS;

  }; // class
}//namespace

#endif // __CARTESIAN_GENERATOR_POS_H__
