/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "MASremapper.hh"

#include <gazebo/physics/physics.hh>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpMASremapper)

namespace gazebo {

GazeboYarpMASremapper::GazeboYarpMASremapper() : ModelPlugin()
{
}

GazeboYarpMASremapper::~GazeboYarpMASremapper()
{
    if(m_MASremapperDevice.isValid()) {m_MASremapperDevice.close();}
    yarp::os::Network::fini();
}

void GazeboYarpMASremapper::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "GazeboYarpMASremapper::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    if (!_parent)
    {
        gzerr << "GazeboYarpMASremapper plugin requires a parent Model." << std::endl;
        return;
    }

    // Get the model scoped name
    m_robotName = _parent->GetScopedName(true);

    // The remapper is a YARP device that has already been added in the factory...

    // Getting .ini configuration file parameters from sdf
    ::yarp::os::Property remapper_properties;

    bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent,_sdf,remapper_properties);

    if (!configuration_loaded)
    {
        yError() << "GazeboYarpMASremapper : File .ini not found, load failed." ;
        return;
    }

    // Open the device driver
    if (!m_MASremapperDevice.open(remapper_properties))
    {
        yError() << "GazeboYarpMASremapper Plugin Load failed: error in opening the part driver";
    }
    
    /*
     * Attach the embObjInertials and/or embObjIMU drivers listed in the options, to the remapper device
     */
    // Parse from the options the list of drivers names to attach
    std::vector<std::string> driverNameList;
    GazeboYarpPlugins::getVectorOfStringFromListInConfig("networks", remapper_properties, driverNameList);
    
    // Retrieve the driver objects from the Handler and build the list of drivers to attach
    ::yarp::dev::PolyDriverList driverList;
    std::string driverScopedName;
    for (size_t nameIdx=0; nameIdx<driverNameList.size(); nameIdx++)
    {
        driverScopedName = m_robotName+"::"+driverNameList[nameIdx];
        yarp::dev::PolyDriver* driver2add = GazeboYarpPlugins::Handler::getHandler()->getDevice(driverScopedName);
        if (!driver2add)
        {
            yError() << "GazeboYarpMASremapper associated driver retrieval from Handler failed.";
        }
        driverList.push(driver2add,driverNameList[nameIdx].c_str());
    }

    // Attach the drivers
    m_MASremapperDevice.view(m_iWrap);
    if(!m_iWrap->attachAll(driverList) )
    {
        yError() << "GazeboYarpForceTorque : error in connecting wrapper and device ";
    }

    // Get the remapper device scoped name "<robotName>::<driverName>". The driver name is unique in the robot URDF model
    driverScopedName = m_MASremapperDevice.getValue("name").toString();
    
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    if (!GazeboYarpPlugins::Handler::getHandler()->setDevice(driverScopedName,&m_MASremapperDevice))
    {
        yError() << "GazeboYarpMASremapper associated driver registering to Handler failed.";
    }
}

}
