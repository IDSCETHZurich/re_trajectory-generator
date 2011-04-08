// Copyright  (C)  2010  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>, 

// Author: Ruben Smits
// Maintainer: Ruben Smits

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation; either
// version 3.0 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

// You should have received a copy of the GNU General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <friComm.h>
#include <ocl/ComponentLoader.hpp>

namespace LWR{

  class FRIDiagnostics:public RTT::TaskContext{
  public:
    FRIDiagnostics(const std::string& name="FRIDiagnostics");
    ~FRIDiagnostics();

    virtual bool configureHook(){return true;};
    virtual bool startHook(){return true;};
    virtual void updateHook();
    virtual void stopHook(){};
    virtual void cleanupHook(){};

  private:
    RTT::ReadDataPort<tFriRobotState> RobotStatePort;
    RTT::ReadDataPort<tFriIntfState> FriStatePort;

    tFriIntfState fristate;
    tFriRobotState robotstate;

    ros::NodeHandle* nh;

    diagnostic_updater::Updater* updater;
    void fri_robot_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void fri_comm_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  };
}
    
