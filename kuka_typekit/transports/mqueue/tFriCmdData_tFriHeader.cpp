/* Generated from orogen/lib/orogen/templates/typekit/mqueue/Type.cpp */

#include "Types.hpp"
#include "transports/mqueue/Registration.hpp"
#include <rtt/transports/mqueue/MQSerializationProtocol.hpp>


namespace orogen_typekits {
    RTT::types::TypeMarshaller*  tFriCmdData_MQueueTransport()
    {
        return new RTT::mqueue::MQSerializationProtocol< tFriCmdData >();
    }
}


/* Generated from orogen/lib/orogen/templates/typekit/mqueue/Type.cpp */

#include "Types.hpp"
#include "transports/mqueue/Registration.hpp"
#include <rtt/transports/mqueue/MQSerializationProtocol.hpp>


namespace orogen_typekits {
    RTT::types::TypeMarshaller*  tFriHeader_MQueueTransport()
    {
        return new RTT::mqueue::MQSerializationProtocol< tFriHeader >();
    }
}

