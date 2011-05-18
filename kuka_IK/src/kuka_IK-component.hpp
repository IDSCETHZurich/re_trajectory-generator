#ifndef OROCOS_KUKA_IK_COMPONENT_HPP
#define OROCOS_KUKA_IK_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include "KukaLWR_Kinematics.hpp"

namespace kuka_IK{
class kuka_IK
    : public RTT::TaskContext
{
 public:
    kuka_IK(string const& name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

 private:

   /** \brief function handle for the input_cartPosPort
		*
		*  \param portInterface
		*/
   bool cartPosInputHandle(RTT::base::PortInterface* portInterface);
   /**
    * @brief intermediate command variables
    */
   std::vector<double> commndedPoseJntPos;
   geometry_msgs::Pose commandedPose;
   ///@}

 protected:
       /// Dataport containing commanded Cartesian pose
       RTT::InputPort< geometry_msgs::Pose > input_cartPosPort;
       /// Dataport containing the command in joint angles
       RTT::OutputPort< sensor_msgs::JointState > output_jntPosPort;
};
}//end of name space
#endif
