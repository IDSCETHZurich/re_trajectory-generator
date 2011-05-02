/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct tFriRobotCommandTypeInfo :
    
	public RTT::types::StructTypeInfo< tFriRobotCommand >
    
    {
        tFriRobotCommandTypeInfo()
	
            : RTT::types::StructTypeInfo< tFriRobotCommand >("/tFriRobotCommand") {}
	
    };

    RTT::types::TypeInfo* tFriRobotCommand_TypeInfo()
    { return new tFriRobotCommandTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< tFriRobotCommand >;
template class RTT::InputPort< tFriRobotCommand >;
template class RTT::Property< tFriRobotCommand >;
template class RTT::Attribute< tFriRobotCommand >;

template class RTT::internal::DataSource< tFriRobotCommand >;
template class RTT::internal::ValueDataSource< tFriRobotCommand >;
template class RTT::internal::ConstantDataSource< tFriRobotCommand >;
template class RTT::internal::AssignableDataSource< tFriRobotCommand >;
template class RTT::internal::ReferenceDataSource< tFriRobotCommand >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct tFriRobotDataTypeInfo :
    
	public RTT::types::StructTypeInfo< tFriRobotData >
    
    {
        tFriRobotDataTypeInfo()
	
            : RTT::types::StructTypeInfo< tFriRobotData >("/tFriRobotData") {}
	
    };

    RTT::types::TypeInfo* tFriRobotData_TypeInfo()
    { return new tFriRobotDataTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< tFriRobotData >;
template class RTT::InputPort< tFriRobotData >;
template class RTT::Property< tFriRobotData >;
template class RTT::Attribute< tFriRobotData >;

template class RTT::internal::DataSource< tFriRobotData >;
template class RTT::internal::ValueDataSource< tFriRobotData >;
template class RTT::internal::ConstantDataSource< tFriRobotData >;
template class RTT::internal::AssignableDataSource< tFriRobotData >;
template class RTT::internal::ReferenceDataSource< tFriRobotData >;



