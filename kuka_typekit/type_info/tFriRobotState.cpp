/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct tFriRobotStateTypeInfo :
    
	public RTT::types::StructTypeInfo< tFriRobotState >
    
    {
        tFriRobotStateTypeInfo()
	
            : RTT::types::StructTypeInfo< tFriRobotState >("/tFriRobotState") {}
	
    };

    RTT::types::TypeInfo* tFriRobotState_TypeInfo()
    { return new tFriRobotStateTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< tFriRobotState >;
template class RTT::InputPort< tFriRobotState >;
template class RTT::Property< tFriRobotState >;
template class RTT::Attribute< tFriRobotState >;

template class RTT::internal::DataSource< tFriRobotState >;
template class RTT::internal::ValueDataSource< tFriRobotState >;
template class RTT::internal::ConstantDataSource< tFriRobotState >;
template class RTT::internal::AssignableDataSource< tFriRobotState >;
template class RTT::internal::ReferenceDataSource< tFriRobotState >;



