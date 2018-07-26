/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "embObjInertials.hh"
#include "embObjInertialsDriver.h"

#include <gazebo/physics/physics.hh>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpEmbObjInertials)

namespace gazebo {

GazeboYarpEmbObjInertials::GazeboYarpEmbObjInertials() : ModelPlugin()
{
}

GazeboYarpEmbObjInertials::~GazeboYarpEmbObjInertials()
{
    if(m_embObjInertialsDriver.isValid()) {m_embObjInertialsDriver.close();}
    yarp::os::Network::fini();
}

void GazeboYarpEmbObjInertials::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "GazeboYarpEmbObjInertials::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    if (!_parent)
    {
        gzerr << "GazeboYarpEmbObjInertials plugin requires a parent Model." << std::endl;
        return;
    }

    // Get the model scoped name
    m_robotName = _parent->GetName();

    // Add my gazebo device driver to the factory.
    ::yarp::dev::Drivers::factory().add(new ::yarp::dev::DriverCreatorOf< ::yarp::dev::GazeboYarpEmbObjInertialsDriver>
                                        ("gazebo_embObjInertials", "multipleanalogsensorsserver", "GazeboYarpEmbObjInertialsDriver"));

    // Getting .ini configuration file parameters from sdf
    ::yarp::os::Property driver_properties;

    bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent,_sdf,driver_properties);

    if (!configuration_loaded)
    {
        yError() << "GazeboYarpEmbObjInertials : File .ini not found, load failed." ;
        return;
    }

    // Open the device driver
    if (!m_embObjInertialsDriver.open(driver_properties))
    {
        yError() << "GazeboYarpEmbObjInertials Plugin Load failed: error in opening the part driver";
    }
    
    // Get the driver scoped name "<robotName>::<driverName>". The driver name is unique in the robot URDF model
    //m_embObjInertialsDriver.view(m_iInertials);
    //std::string driverScopedName = m_iInertials->getDriverScopedName();
    std::string deviceScopedName = m_robotName+"::"+m_embObjInertialsDriver.getValue("name").toString();
    
    //Insert the pointer in the singleton handler for retriving it in a remapper or wrapper device
    if (!GazeboYarpPlugins::Handler::getHandler()->setDevice(deviceScopedName,&m_embObjInertialsDriver))
    {
        yError() << "GazeboYarpEmbObjInertials associated driver registering to Handler failed.";
    }
}

}
