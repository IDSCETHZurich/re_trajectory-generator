/* Generated from orogen/lib/orogen/templates/typekit/Plugin.hpp */

#ifndef __OROGEN_GENERATED_KUKA_TYPEKIT_TYPEKIT_HPP
#define __OROGEN_GENERATED_KUKA_TYPEKIT_TYPEKIT_HPP

#include <rtt/types/TypekitPlugin.hpp>

namespace Typelib {
    class Registry;
}

namespace orogen_typekits {
    class kuka_typekitTypekitPlugin
        : public RTT::types::TypekitPlugin
    {
        Typelib::Registry* m_registry;

    public:
        kuka_typekitTypekitPlugin();
        ~kuka_typekitTypekitPlugin();
        bool loadTypes();
        bool loadOperators();
        bool loadConstructors();
        std::string getName();
    };

    extern kuka_typekitTypekitPlugin kuka_typekitTypekit;
}

#endif


