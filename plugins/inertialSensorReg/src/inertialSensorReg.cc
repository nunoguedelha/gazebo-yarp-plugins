/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "inertialSensorReg.hh"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/sensors/ImuSensor.hh>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

GZ_REGISTER_SENSOR_PLUGIN(gazebo::GazeboYarpInertialSensorReg)

namespace gazebo {

GazeboYarpInertialSensorReg::GazeboYarpInertialSensorReg() : SensorPlugin()
{
}

GazeboYarpInertialSensorReg::~GazeboYarpInertialSensorReg()
{
}

void GazeboYarpInertialSensorReg::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    if (!_sensor) {
        gzerr << "GazeboYarpInertialSensorReg plugin requires an IMUSensor." << std::endl;
        return;
    }

    _sensor->SetActive(true);

#if GAZEBO_MAJOR_VERSION >= 7
    m_sensorName = _sensor->ScopedName();
#else
    m_sensorName = _sensor->GetScopedName();
#endif
    //Insert the pointer in the singleton handler for retriving it in the yarp driver
    GazeboYarpPlugins::Handler::getHandler()->setSensor(_sensor.get());
}

}
