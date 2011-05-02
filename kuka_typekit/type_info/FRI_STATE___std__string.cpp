/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>



namespace orogen_typekits {
    struct FRI_STATETypeInfo :
    
	public RTT::types::TemplateTypeInfo< FRI_STATE >
    
    {
        FRI_STATETypeInfo()
	
            : RTT::types::TemplateTypeInfo< FRI_STATE >("/FRI_STATE") {}
	
    };

    RTT::types::TypeInfo* FRI_STATE_TypeInfo()
    { return new FRI_STATETypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< FRI_STATE >;
template class RTT::InputPort< FRI_STATE >;
template class RTT::Property< FRI_STATE >;
template class RTT::Attribute< FRI_STATE >;

template class RTT::internal::DataSource< FRI_STATE >;
template class RTT::internal::ValueDataSource< FRI_STATE >;
template class RTT::internal::ConstantDataSource< FRI_STATE >;
template class RTT::internal::AssignableDataSource< FRI_STATE >;
template class RTT::internal::ReferenceDataSource< FRI_STATE >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>

#include <rtt/typekit/StdStringTypeInfo.hpp>



namespace orogen_typekits {
    struct std_stringTypeInfo :
    
        public RTT::types::StdStringTypeInfo
    
    {
        std_stringTypeInfo()
	
	    : RTT::types::StdStringTypeInfo("/std/string") {}
        
    };

    RTT::types::TypeInfo* std_string_TypeInfo()
    { return new std_stringTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< ::std::string >;
template class RTT::InputPort< ::std::string >;
template class RTT::Property< ::std::string >;
template class RTT::Attribute< ::std::string >;

template class RTT::internal::DataSource< ::std::string >;
template class RTT::internal::ValueDataSource< ::std::string >;
template class RTT::internal::ConstantDataSource< ::std::string >;
template class RTT::internal::AssignableDataSource< ::std::string >;
template class RTT::internal::ReferenceDataSource< ::std::string >;



