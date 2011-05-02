/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct tFriIntfStateTypeInfo :
    
	public RTT::types::StructTypeInfo< tFriIntfState >
    
    {
        tFriIntfStateTypeInfo()
	
            : RTT::types::StructTypeInfo< tFriIntfState >("/tFriIntfState") {}
	
    };

    RTT::types::TypeInfo* tFriIntfState_TypeInfo()
    { return new tFriIntfStateTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< tFriIntfState >;
template class RTT::InputPort< tFriIntfState >;
template class RTT::Property< tFriIntfState >;
template class RTT::Attribute< tFriIntfState >;

template class RTT::internal::DataSource< tFriIntfState >;
template class RTT::internal::ValueDataSource< tFriIntfState >;
template class RTT::internal::ConstantDataSource< tFriIntfState >;
template class RTT::internal::AssignableDataSource< tFriIntfState >;
template class RTT::internal::ReferenceDataSource< tFriIntfState >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct tFriIntfStatisticsTypeInfo :
    
	public RTT::types::StructTypeInfo< tFriIntfStatistics >
    
    {
        tFriIntfStatisticsTypeInfo()
	
            : RTT::types::StructTypeInfo< tFriIntfStatistics >("/tFriIntfStatistics") {}
	
    };

    RTT::types::TypeInfo* tFriIntfStatistics_TypeInfo()
    { return new tFriIntfStatisticsTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< tFriIntfStatistics >;
template class RTT::InputPort< tFriIntfStatistics >;
template class RTT::Property< tFriIntfStatistics >;
template class RTT::Attribute< tFriIntfStatistics >;

template class RTT::internal::DataSource< tFriIntfStatistics >;
template class RTT::internal::ValueDataSource< tFriIntfStatistics >;
template class RTT::internal::ConstantDataSource< tFriIntfStatistics >;
template class RTT::internal::AssignableDataSource< tFriIntfStatistics >;
template class RTT::internal::ReferenceDataSource< tFriIntfStatistics >;



