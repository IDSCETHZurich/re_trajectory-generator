/* Generated from orogen/lib/orogen/templates/typekit/Types.hpp */

#ifndef __OROGEN_GENERATED_KUKA_TYPEKIT_TYPES_HPP
#define __OROGEN_GENERATED_KUKA_TYPEKIT_TYPES_HPP





#include "kuka_typekit/types/kuka_typekit/include/friComm.h"




// This is a hack. We include it unconditionally as it may be required by some
// typekits *and* it is a standard header. Ideally, we would actually check if
// some of the types need std::vector.
#include <vector>
#include <boost/cstdint.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/vector.hpp>




#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< FRI_CTRL >;
    extern template class RTT::internal::AssignableDataSource< FRI_CTRL >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< FRI_CTRL >;
    extern template class RTT::internal::ConstantDataSource< FRI_CTRL >;
    extern template class RTT::internal::ReferenceDataSource< FRI_CTRL >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< FRI_CTRL >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< FRI_CTRL >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< FRI_CTRL >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< FRI_CTRL >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< FRI_QUALITY >;
    extern template class RTT::internal::AssignableDataSource< FRI_QUALITY >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< FRI_QUALITY >;
    extern template class RTT::internal::ConstantDataSource< FRI_QUALITY >;
    extern template class RTT::internal::ReferenceDataSource< FRI_QUALITY >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< FRI_QUALITY >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< FRI_QUALITY >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< FRI_QUALITY >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< FRI_QUALITY >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< FRI_STATE >;
    extern template class RTT::internal::AssignableDataSource< FRI_STATE >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< FRI_STATE >;
    extern template class RTT::internal::ConstantDataSource< FRI_STATE >;
    extern template class RTT::internal::ReferenceDataSource< FRI_STATE >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< FRI_STATE >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< FRI_STATE >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< FRI_STATE >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< FRI_STATE >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< ::std::string >;
    extern template class RTT::internal::AssignableDataSource< ::std::string >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< ::std::string >;
    extern template class RTT::internal::ConstantDataSource< ::std::string >;
    extern template class RTT::internal::ReferenceDataSource< ::std::string >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< ::std::string >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< ::std::string >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< ::std::string >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< ::std::string >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< tFriCmdData >;
    extern template class RTT::internal::AssignableDataSource< tFriCmdData >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< tFriCmdData >;
    extern template class RTT::internal::ConstantDataSource< tFriCmdData >;
    extern template class RTT::internal::ReferenceDataSource< tFriCmdData >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< tFriCmdData >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< tFriCmdData >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< tFriCmdData >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< tFriCmdData >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< tFriHeader >;
    extern template class RTT::internal::AssignableDataSource< tFriHeader >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< tFriHeader >;
    extern template class RTT::internal::ConstantDataSource< tFriHeader >;
    extern template class RTT::internal::ReferenceDataSource< tFriHeader >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< tFriHeader >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< tFriHeader >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< tFriHeader >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< tFriHeader >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< tFriIntfState >;
    extern template class RTT::internal::AssignableDataSource< tFriIntfState >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< tFriIntfState >;
    extern template class RTT::internal::ConstantDataSource< tFriIntfState >;
    extern template class RTT::internal::ReferenceDataSource< tFriIntfState >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< tFriIntfState >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< tFriIntfState >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< tFriIntfState >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< tFriIntfState >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< tFriIntfStatistics >;
    extern template class RTT::internal::AssignableDataSource< tFriIntfStatistics >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< tFriIntfStatistics >;
    extern template class RTT::internal::ConstantDataSource< tFriIntfStatistics >;
    extern template class RTT::internal::ReferenceDataSource< tFriIntfStatistics >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< tFriIntfStatistics >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< tFriIntfStatistics >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< tFriIntfStatistics >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< tFriIntfStatistics >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< tFriKrlData >;
    extern template class RTT::internal::AssignableDataSource< tFriKrlData >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< tFriKrlData >;
    extern template class RTT::internal::ConstantDataSource< tFriKrlData >;
    extern template class RTT::internal::ReferenceDataSource< tFriKrlData >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< tFriKrlData >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< tFriKrlData >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< tFriKrlData >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< tFriKrlData >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< tFriMsrData >;
    extern template class RTT::internal::AssignableDataSource< tFriMsrData >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< tFriMsrData >;
    extern template class RTT::internal::ConstantDataSource< tFriMsrData >;
    extern template class RTT::internal::ReferenceDataSource< tFriMsrData >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< tFriMsrData >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< tFriMsrData >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< tFriMsrData >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< tFriMsrData >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< tFriRobotCommand >;
    extern template class RTT::internal::AssignableDataSource< tFriRobotCommand >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< tFriRobotCommand >;
    extern template class RTT::internal::ConstantDataSource< tFriRobotCommand >;
    extern template class RTT::internal::ReferenceDataSource< tFriRobotCommand >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< tFriRobotCommand >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< tFriRobotCommand >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< tFriRobotCommand >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< tFriRobotCommand >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< tFriRobotData >;
    extern template class RTT::internal::AssignableDataSource< tFriRobotData >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< tFriRobotData >;
    extern template class RTT::internal::ConstantDataSource< tFriRobotData >;
    extern template class RTT::internal::ReferenceDataSource< tFriRobotData >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< tFriRobotData >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< tFriRobotData >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< tFriRobotData >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< tFriRobotData >;
#endif

#ifdef CORELIB_DATASOURCE_HPP
    extern template class RTT::internal::DataSource< tFriRobotState >;
    extern template class RTT::internal::AssignableDataSource< tFriRobotState >;
#endif
#ifdef ORO_CORELIB_DATASOURCES_HPP
    extern template class RTT::internal::ValueDataSource< tFriRobotState >;
    extern template class RTT::internal::ConstantDataSource< tFriRobotState >;
    extern template class RTT::internal::ReferenceDataSource< tFriRobotState >;
#endif
#ifdef ORO_INPUT_PORT_HPP
    extern template class RTT::OutputPort< tFriRobotState >;
#endif
#ifdef ORO_OUTPUT_PORT_HPP
    extern template class RTT::InputPort< tFriRobotState >;
#endif
#ifdef ORO_PROPERTY_HPP
    extern template class RTT::Property< tFriRobotState >;
#endif
#ifdef ORO_CORELIB_ATTRIBUTE_HPP
    extern template class RTT::Attribute< tFriRobotState >;
#endif




namespace boost
{
    namespace serialization
    {

        template<typename Archive>
        void serialize(Archive& a, tFriCmdData& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("head", b.head);
a & make_nvp("krl", b.krl);
a & make_nvp("cmd", b.cmd);
        }

        template<typename Archive>
        void serialize(Archive& a, tFriHeader& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("sendSeqCount", b.sendSeqCount);
a & make_nvp("reflSeqCount", b.reflSeqCount);
a & make_nvp("packetSize", b.packetSize);
a & make_nvp("datagramId", b.datagramId);
        }

        template<typename Archive>
        void serialize(Archive& a, tFriIntfState& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("timestamp", b.timestamp);
a & make_nvp("state", b.state);
a & make_nvp("quality", b.quality);
a & make_nvp("desiredMsrSampleTime", b.desiredMsrSampleTime);
a & make_nvp("desiredCmdSampleTime", b.desiredCmdSampleTime);
a & make_nvp("safetyLimits", b.safetyLimits);
a & make_nvp("stat", b.stat);
        }

        template<typename Archive>
        void serialize(Archive& a, tFriIntfStatistics& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("answerRate", b.answerRate);
a & make_nvp("latency", b.latency);
a & make_nvp("jitter", b.jitter);
a & make_nvp("missRate", b.missRate);
a & make_nvp("missCounter", b.missCounter);
        }

        template<typename Archive>
        void serialize(Archive& a, tFriKrlData& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("realData", boost::serialization::make_array(b.realData, 16));
a & make_nvp("intData", boost::serialization::make_array(b.intData, 16));
a & make_nvp("boolData", b.boolData);
a & make_nvp("fill", b.fill);
        }

        template<typename Archive>
        void serialize(Archive& a, tFriMsrData& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("head", b.head);
a & make_nvp("krl", b.krl);
a & make_nvp("intf", b.intf);
a & make_nvp("robot", b.robot);
a & make_nvp("data", b.data);
        }

        template<typename Archive>
        void serialize(Archive& a, tFriRobotCommand& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("cmdFlags", b.cmdFlags);
a & make_nvp("jntPos", boost::serialization::make_array(b.jntPos, 7));
a & make_nvp("cartPos", boost::serialization::make_array(b.cartPos, 12));
a & make_nvp("addJntTrq", boost::serialization::make_array(b.addJntTrq, 7));
a & make_nvp("addTcpFT", boost::serialization::make_array(b.addTcpFT, 6));
a & make_nvp("jntStiffness", boost::serialization::make_array(b.jntStiffness, 7));
a & make_nvp("jntDamping", boost::serialization::make_array(b.jntDamping, 7));
a & make_nvp("cartStiffness", boost::serialization::make_array(b.cartStiffness, 6));
a & make_nvp("cartDamping", boost::serialization::make_array(b.cartDamping, 6));
        }

        template<typename Archive>
        void serialize(Archive& a, tFriRobotData& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("msrJntPos", boost::serialization::make_array(b.msrJntPos, 7));
a & make_nvp("msrCartPos", boost::serialization::make_array(b.msrCartPos, 12));
a & make_nvp("cmdJntPos", boost::serialization::make_array(b.cmdJntPos, 7));
a & make_nvp("cmdJntPosFriOffset", boost::serialization::make_array(b.cmdJntPosFriOffset, 7));
a & make_nvp("cmdCartPos", boost::serialization::make_array(b.cmdCartPos, 12));
a & make_nvp("cmdCartPosFriOffset", boost::serialization::make_array(b.cmdCartPosFriOffset, 12));
a & make_nvp("msrJntTrq", boost::serialization::make_array(b.msrJntTrq, 7));
a & make_nvp("estExtJntTrq", boost::serialization::make_array(b.estExtJntTrq, 7));
a & make_nvp("estExtTcpFT", boost::serialization::make_array(b.estExtTcpFT, 6));
a & make_nvp("jacobian", boost::serialization::make_array(b.jacobian, 42));
a & make_nvp("massMatrix", boost::serialization::make_array(b.massMatrix, 49));
a & make_nvp("gravity", boost::serialization::make_array(b.gravity, 7));
        }

        template<typename Archive>
        void serialize(Archive& a, tFriRobotState& b, unsigned int version)
        {
            using boost::serialization::make_nvp;
            a & make_nvp("power", b.power);
a & make_nvp("control", b.control);
a & make_nvp("error", b.error);
a & make_nvp("warning", b.warning);
a & make_nvp("temperature", boost::serialization::make_array(b.temperature, 7));
        }

    }
}


#endif

