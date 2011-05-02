/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct tFriKrlDataTypeInfo :
    
	public RTT::types::StructTypeInfo< tFriKrlData >
    
    {
        tFriKrlDataTypeInfo()
	
            : RTT::types::StructTypeInfo< tFriKrlData >("/tFriKrlData") {}
	
    };

    RTT::types::TypeInfo* tFriKrlData_TypeInfo()
    { return new tFriKrlDataTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< tFriKrlData >;
template class RTT::InputPort< tFriKrlData >;
template class RTT::Property< tFriKrlData >;
template class RTT::Attribute< tFriKrlData >;

template class RTT::internal::DataSource< tFriKrlData >;
template class RTT::internal::ValueDataSource< tFriKrlData >;
template class RTT::internal::ConstantDataSource< tFriKrlData >;
template class RTT::internal::AssignableDataSource< tFriKrlData >;
template class RTT::internal::ReferenceDataSource< tFriKrlData >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <kuka_typekit/Types.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct tFriMsrDataTypeInfo :
    
	public RTT::types::StructTypeInfo< tFriMsrData >
    
    {
        tFriMsrDataTypeInfo()
	
            : RTT::types::StructTypeInfo< tFriMsrData >("/tFriMsrData") {}
	
    };

    RTT::types::TypeInfo* tFriMsrData_TypeInfo()
    { return new tFriMsrDataTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< tFriMsrData >;
template class RTT::InputPort< tFriMsrData >;
template class RTT::Property< tFriMsrData >;
template class RTT::Attribute< tFriMsrData >;

template class RTT::internal::DataSource< tFriMsrData >;
template class RTT::internal::ValueDataSource< tFriMsrData >;
template class RTT::internal::ConstantDataSource< tFriMsrData >;
template class RTT::internal::AssignableDataSource< tFriMsrData >;
template class RTT::internal::ReferenceDataSource< tFriMsrData >;



