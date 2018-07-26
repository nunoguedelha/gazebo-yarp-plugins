/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "MASwrapper.hh"

#include <gazebo/physics/physics.hh>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpMASwrapper)

namespace gazebo {

GazeboYarpMASwrapper::GazeboYarpMASwrapper() : ModelPlugin()
{
}

GazeboYarpMASwrapper::~GazeboYarpMASwrapper()
{
    if(m_MASwrapperDevice.isValid()) {m_MASwrapperDevice.close();}
    yarp::os::Network::fini();
}

void GazeboYarpMASwrapper::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "GazeboYarpMASwrapper::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    if (!_parent)
    {
        gzerr << "GazeboYarpMASwrapper plugin requires a parent Model." << std::endl;
        return;
    }

    // Get the model scoped name
    m_robotName = _parent->GetName();

    // The remapper is a YARP device that has already been added in the factory...

    // Getting .ini configuration file parameters from sdf
    ::yarp::os::Property wrapper_properties;

    bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent,_sdf,wrapper_properties);

    if (!configuration_loaded)
    {
        yError() << "GazeboYarpMASwrapper : File .ini not found, load failed." ;
        return;
    }

    // Open the wrapper device
    if (!m_MASwrapperDevice.open(wrapper_properties))
    {
        yError() << "GazeboYarpMASwrapper Plugin Load failed: error in opening the part wrapper";
    }
    
    /*
     * Attach the embObjInertials,or embObjIMU, or a remapper listed in the options, to the server device
     */
    // Parse from the options the list of devices names to attach
    std::vector<std::string> deviceNameList;
    GazeboYarpPlugins::getVectorOfStringFromListInConfig("networks", wrapper_properties, deviceNameList);
    
    // Retrieve the device objects from the Handler and build the list of devices to attach
    ::yarp::dev::PolyDriverList deviceList;
    std::string deviceScopedName;
    
    // We could consider that the Multiple Analog Sensors Server can have only one device attached but we choose
    // not to do that assumption here for reducing the dependencies. If the number of devices to attach exceeds
    // the wrapper rules, the wrapper itself raises the error.
    for (size_t nameIdx=0; nameIdx<deviceNameList.size(); nameIdx++)
    {
        deviceScopedName = m_robotName+"::"+deviceNameList[nameIdx];
        yarp::dev::PolyDriver* driver2add = GazeboYarpPlugins::Handler::getHandler()->getDevice(deviceScopedName);
        if (!driver2add)
        {
            yError() << "GazeboYarpMASwrapper associated driver retrieval from Handler failed.";
        }
        deviceList.push(driver2add,deviceNameList[nameIdx].c_str());
    }

    // Attach the drivers. (If the number of devices to attach exceeds the wrapper rules, the wrapper itself
    // raises the error.)
    m_MASwrapperDevice.view(m_iWrap);
    if(!m_iWrap->attachAll(deviceList) )
    {
        yError() << "GazeboYarpMASwrapper : error in connecting wrapper and device ";
    }
}

}
