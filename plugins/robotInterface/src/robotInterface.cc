/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "robotInterface.hh"

#include <gazebo/physics/physics.hh>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <boost/process.hpp>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpRobotInterface)

namespace bp = boost::process;

namespace gazebo {

GazeboYarpRobotInterface::GazeboYarpRobotInterface() : ModelPlugin()
{
}

GazeboYarpRobotInterface::~GazeboYarpRobotInterface()
{
    yarp::os::Network::fini();
}

void GazeboYarpRobotInterface::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "GazeboYarpRobotInterface::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    if (!_parent)
    {
        gzerr << "GazeboYarpRobotInterface plugin requires a parent Model." << std::endl;
        return;
    }

    // Get the model scoped name
    m_robotName = _parent->GetScopedName(true);

    // Getting .ini configuration file parameters from sdf.
    // We could use the default config file only but we prefer not to rely
    // on the robot name environment variable for finding the config file.
    ::yarp::os::Property robotInterface_properties;

    bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent,_sdf,robotInterface_properties);

    if (!configuration_loaded)
    {
        yError() << "GazeboYarpRobotInterface : File .ini not found, load failed." ;
        return;
    }

    // Overwrite robot name with the scoped name
    robotInterface_properties.put("gazeboYarpPluginsRobotName",m_robotName.c_str());

    // Get "robot" (YARP_ROBOT_NAME) used for finding the robot configuration, and the
    // configuration parameters
    std::string programName = "yarprobotinterface";
    std::string yarpRobotNameTag = "--robot";
    std::string yarpRobotName = robotInterface_properties.find("robot").asString();
    std::string hardwareConfigFileTag = "--config";
    std::string hardwareConfigFile = robotInterface_properties.find("config").asString();
    char* argValues[6];
    argValues[0] = &programName[0];
    argValues[1] = &yarpRobotNameTag[0];
    argValues[2] = &yarpRobotName[0];
    argValues[3] = &hardwareConfigFileTag[0];
    argValues[4] = &hardwareConfigFile[0];
    argValues[5] = NULL;
    std::string yarpcommand = "yarprobotinterface --robot " + yarpRobotName + " --config " + hardwareConfigFile;

    // Run the yarprobotinterface executable
    bp::spawn(yarpcommand.c_str());
    
    yarp::os::Time::delay(10);
}

}
