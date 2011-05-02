/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct tFriCmdDataTypeInfo :
    
	public RTT::types::StructTypeInfo< tFriCmdData >
    
    {
        tFriCmdDataTypeInfo()
	
            : RTT::types::StructTypeInfo< tFriCmdData >("/tFriCmdData") {}
	
    };

    RTT::types::TypeInfo* tFriCmdData_TypeInfo()
    { return new tFriCmdDataTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< tFriCmdData >;
template class RTT::InputPort< tFriCmdData >;
template class RTT::Property< tFriCmdData >;
template class RTT::Attribute< tFriCmdData >;

template class RTT::internal::DataSource< tFriCmdData >;
template class RTT::internal::ValueDataSource< tFriCmdData >;
template class RTT::internal::ConstantDataSource< tFriCmdData >;
template class RTT::internal::AssignableDataSource< tFriCmdData >;
template class RTT::internal::ReferenceDataSource< tFriCmdData >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct tFriHeaderTypeInfo :
    
	public RTT::types::StructTypeInfo< tFriHeader >
    
    {
        tFriHeaderTypeInfo()
	
            : RTT::types::StructTypeInfo< tFriHeader >("/tFriHeader") {}
	
    };

    RTT::types::TypeInfo* tFriHeader_TypeInfo()
    { return new tFriHeaderTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< tFriHeader >;
template class RTT::InputPort< tFriHeader >;
template class RTT::Property< tFriHeader >;
template class RTT::Attribute< tFriHeader >;

template class RTT::internal::DataSource< tFriHeader >;
template class RTT::internal::ValueDataSource< tFriHeader >;
template class RTT::internal::ConstantDataSource< tFriHeader >;
template class RTT::internal::AssignableDataSource< tFriHeader >;
template class RTT::internal::ReferenceDataSource< tFriHeader >;



