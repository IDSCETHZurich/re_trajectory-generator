/* Generated from orogen/lib/orogen/templates/typekit/mqueue/TransportPlugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "transports/mqueue/Registration.hpp"
#include "transports/mqueue/TransportPlugin.hpp"
#include <rtt/transports/mqueue/MQLib.hpp>
#include <rtt/types/TypekitPlugin.hpp>
using namespace RTT;

bool orogen_typekits::kuka_typekitMQueueTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    
    if ("/tFriCmdData" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            tFriCmdData_MQueueTransport());
        return true;
    }
    
    else if ("/tFriHeader" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            tFriHeader_MQueueTransport());
        return true;
    }
    
    else if ("/tFriIntfState" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            tFriIntfState_MQueueTransport());
        return true;
    }
    
    else if ("/tFriIntfStatistics" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            tFriIntfStatistics_MQueueTransport());
        return true;
    }
    
    else if ("/tFriKrlData" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            tFriKrlData_MQueueTransport());
        return true;
    }
    
    else if ("/tFriMsrData" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            tFriMsrData_MQueueTransport());
        return true;
    }
    
    else if ("/tFriRobotCommand" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            tFriRobotCommand_MQueueTransport());
        return true;
    }
    
    else if ("/tFriRobotData" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            tFriRobotData_MQueueTransport());
        return true;
    }
    
    else if ("/tFriRobotState" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            tFriRobotState_MQueueTransport());
        return true;
    }
    
    return false;
}
std::string orogen_typekits::kuka_typekitMQueueTransportPlugin::getTransportName() const
{ return "MQueue"; }
std::string orogen_typekits::kuka_typekitMQueueTransportPlugin::getTypekitName() const
{ return "/orogen/kuka_typekit"; }
std::string orogen_typekits::kuka_typekitMQueueTransportPlugin::getName() const
{ return "/orogen/kuka_typekit/MQueue"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::kuka_typekitMQueueTransportPlugin);

