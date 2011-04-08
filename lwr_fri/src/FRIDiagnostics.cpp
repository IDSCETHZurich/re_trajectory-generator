#include <ocl/ComponentLoader.hpp>
#include "FRIDiagnostics.hpp"
#include <string>
#include <bitset>

ORO_CREATE_COMPONENT(LWR::FRIDiagnostics);

namespace LWR{
  
  FRIDiagnostics::FRIDiagnostics(const std::string& name):
    RTT::TaskContext(name),
    RobotStatePort("RobotState"),
    FriStatePort("FriState")
  {
    
    ports()->addPort(&RobotStatePort);
    ports()->addPort(&FriStatePort);

    if(!ros::isInitialized()){
      char* argv[1];
      int argc=0;
      ros::init(argc,&argv[0],name);
    }
    nh=new ros::NodeHandle();
    updater=new diagnostic_updater::Updater();
    updater->setHardwareID("Kuka LWR Fast Research Interface");
    updater->add("FRI Communication Diagnostics",this,&FRIDiagnostics::fri_comm_diagnostics);
    updater->add("FRI Robot Diagnostics",this,&FRIDiagnostics::fri_robot_diagnostics);
  }

  FRIDiagnostics::~FRIDiagnostics(){
    delete updater;
    delete nh;
  }

  void FRIDiagnostics::updateHook(){
    RobotStatePort.Get(robotstate);
    FriStatePort.Get(fristate);

    updater->update();
  }

  void FRIDiagnostics::fri_comm_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat){
    
    if(fristate.quality==FRI_QUALITY_PERFECT)
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"Communication quality PERFECT");
    else if(fristate.quality==FRI_QUALITY_OK)
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"Communication quality OK");
    else if(fristate.quality==FRI_QUALITY_BAD)
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"Communication quality BAD");
    else
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"Communication quality UNACCEPTABLE");
    
    stat.add("Kuka System Time",fristate.timestamp);
    stat.add("State",fristate.state);
    stat.add("Quality",fristate.quality);
    stat.add("Desired Send Sample Time",fristate.desiredMsrSampleTime);
    stat.add("Desired Command Sample Time",fristate.desiredCmdSampleTime);
    stat.add("Safety Limits",fristate.safetyLimits);
    stat.add("Answer Rate",fristate.stat.answerRate);
    stat.add("Latency",fristate.stat.latency);
    stat.add("Jitter",fristate.stat.jitter);
    stat.add("Average Missed Answer Packages",fristate.stat.missRate);
    stat.add("Total Missed Packages",fristate.stat.missCounter);
  }

  void FRIDiagnostics::fri_robot_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat){
    
    std::bitset<7> power(robotstate.power);
    std::bitset<7> error(robotstate.error);
    std::bitset<7> warning(robotstate.warning);
    
    std::bitset<7> clear;
    if(clear==error)
      if(clear==warning)
	stat.summary(diagnostic_msgs::DiagnosticStatus::OK,"All drives OK");
      else
	stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,"Drives warning");
    else
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,"Drives Error");
    
      
    stat.add("Power",power.to_string());
    stat.add("Control Strategy",robotstate.control);
    stat.add("Error",error.to_string());
    stat.add("Warning",warning.to_string());
    stat.add("Temperature Joint 1",robotstate.temperature[0]);
    stat.add("Temperature Joint 2",robotstate.temperature[1]);
    stat.add("Temperature Joint 3",robotstate.temperature[2]);
    stat.add("Temperature Joint 4",robotstate.temperature[3]);
    stat.add("Temperature Joint 5",robotstate.temperature[4]);
    stat.add("Temperature Joint 6",robotstate.temperature[5]);
    stat.add("Temperature Joint 7",robotstate.temperature[6]);
    
  }
}


