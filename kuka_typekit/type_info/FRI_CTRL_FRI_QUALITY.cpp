/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>



namespace orogen_typekits {
    struct FRI_CTRLTypeInfo :
    
	public RTT::types::TemplateTypeInfo< FRI_CTRL >
    
    {
        FRI_CTRLTypeInfo()
	
            : RTT::types::TemplateTypeInfo< FRI_CTRL >("/FRI_CTRL") {}
	
    };

    RTT::types::TypeInfo* FRI_CTRL_TypeInfo()
    { return new FRI_CTRLTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< FRI_CTRL >;
template class RTT::InputPort< FRI_CTRL >;
template class RTT::Property< FRI_CTRL >;
template class RTT::Attribute< FRI_CTRL >;

template class RTT::internal::DataSource< FRI_CTRL >;
template class RTT::internal::ValueDataSource< FRI_CTRL >;
template class RTT::internal::ConstantDataSource< FRI_CTRL >;
template class RTT::internal::AssignableDataSource< FRI_CTRL >;
template class RTT::internal::ReferenceDataSource< FRI_CTRL >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/TemplateTypeInfo.hpp>



namespace orogen_typekits {
    struct FRI_QUALITYTypeInfo :
    
	public RTT::types::TemplateTypeInfo< FRI_QUALITY >
    
    {
        FRI_QUALITYTypeInfo()
	
            : RTT::types::TemplateTypeInfo< FRI_QUALITY >("/FRI_QUALITY") {}
	
    };

    RTT::types::TypeInfo* FRI_QUALITY_TypeInfo()
    { return new FRI_QUALITYTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< FRI_QUALITY >;
template class RTT::InputPort< FRI_QUALITY >;
template class RTT::Property< FRI_QUALITY >;
template class RTT::Attribute< FRI_QUALITY >;

template class RTT::internal::DataSource< FRI_QUALITY >;
template class RTT::internal::ValueDataSource< FRI_QUALITY >;
template class RTT::internal::ConstantDataSource< FRI_QUALITY >;
template class RTT::internal::AssignableDataSource< FRI_QUALITY >;
template class RTT::internal::ReferenceDataSource< FRI_QUALITY >;



