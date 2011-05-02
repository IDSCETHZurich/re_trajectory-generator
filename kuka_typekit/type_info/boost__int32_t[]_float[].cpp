/* Generated from orogen/lib/orogen/templates/typekit/type_info/ArrayInfo.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/internal/carray.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>

namespace orogen_typekits {
    struct int32_tArrayTypeInfo :
	public RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::int32_t > >
    {
        int32_tArrayTypeInfo()
            : RTT::types::CArrayTypeInfo< RTT::internal::carray< boost::int32_t > >("/int32_t[]") {}
    };

    RTT::types::TypeInfo* int32_t_ArrayTypeInfo()
    { return new int32_tArrayTypeInfo(); }
}



/* Generated from orogen/lib/orogen/templates/typekit/type_info/ArrayInfo.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/internal/carray.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>

namespace orogen_typekits {
    struct floatArrayTypeInfo :
	public RTT::types::CArrayTypeInfo< RTT::internal::carray< float > >
    {
        floatArrayTypeInfo()
            : RTT::types::CArrayTypeInfo< RTT::internal::carray< float > >("/float[]") {}
    };

    RTT::types::TypeInfo* float_ArrayTypeInfo()
    { return new floatArrayTypeInfo(); }
}


