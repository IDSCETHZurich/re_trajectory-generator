/* Generated from orogen/lib/orogen/templates/typekit/Plugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "Plugin.hpp"

#include <iostream>
#include <boost/cstdint.hpp>
#include <boost/lexical_cast.hpp>

#include <rtt/types/TypeInfoRepository.hpp>
#include "type_info/Registration.hpp"

using namespace RTT;

orogen_typekits::kuka_typekitTypekitPlugin::kuka_typekitTypekitPlugin()
{}

orogen_typekits::kuka_typekitTypekitPlugin::~kuka_typekitTypekitPlugin()
{}


#define TYPEKIT_PACKAGE_NAME_aux0(target) #target
#define TYPEKIT_PACKAGE_NAME_aux(target) "kuka_typekit-typekit-" TYPEKIT_PACKAGE_NAME_aux0(target)
#define TYPEKIT_PACKAGE_NAME TYPEKIT_PACKAGE_NAME_aux(OROCOS_TARGET)
bool orogen_typekits::kuka_typekitTypekitPlugin::loadTypes()
{
    RTT::types::TypeInfoRepository::shared_ptr ti_repository = RTT::types::TypeInfoRepository::Instance();

    RTT::types::TypeInfo* ti = 0;
    
        
    ti = FRI_CTRL_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = FRI_QUALITY_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = FRI_STATE_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = float_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = float_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = float_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = float_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = float_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = float_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = int32_t_ArrayTypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = std_string_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = tFriCmdData_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = tFriHeader_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = tFriIntfState_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = tFriIntfStatistics_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = tFriKrlData_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = tFriMsrData_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = tFriRobotCommand_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = tFriRobotData_TypeInfo();
    ti_repository->addType( ti );
        
    
        
    ti = tFriRobotState_TypeInfo();
    ti_repository->addType( ti );
        
    

    return true;
}

bool orogen_typekits::kuka_typekitTypekitPlugin::loadOperators()
{ return true; }
bool orogen_typekits::kuka_typekitTypekitPlugin::loadConstructors()
{ return true; }
std::string orogen_typekits::kuka_typekitTypekitPlugin::getName()
{ return "/orogen/kuka_typekit"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::kuka_typekitTypekitPlugin);

