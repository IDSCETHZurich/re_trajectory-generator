/* Generated from orogen/lib/orogen/templates/typekit/mqueue/TransportPlugin.hpp */

#ifndef __OROGEN_GENERATED_KUKA_TYPEKIT_MQUEUE_PLUGIN_HPP
#define __OROGEN_GENERATED_KUKA_TYPEKIT_MQUEUE_PLUGIN_HPP

#include <rtt/types/TransportPlugin.hpp>

namespace orogen_typekits {
    class kuka_typekitMQueueTransportPlugin
        : public RTT::types::TransportPlugin
    {
    public:
        virtual bool registerTransport(std::string type_name, RTT::types::TypeInfo* ti);
        virtual std::string getTransportName() const;
        virtual std::string getTypekitName() const;
        virtual std::string getName() const;
    };

    extern kuka_typekitMQueueTransportPlugin kuka_typekitMQueueTransport;
}

#endif

