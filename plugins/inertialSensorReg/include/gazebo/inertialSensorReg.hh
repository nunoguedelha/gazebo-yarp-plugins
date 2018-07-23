/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_INERTIALSENSORREG_HH
#define GAZEBOYARP_INERTIALSENSORREG_HH

#include <gazebo/common/Plugin.hh>

#include <string>


namespace gazebo
{
    namespace sensors {
        class ImuSensor;
    }

    /// \class GazeboYarpInertialSensorReg
    /// Gazebo Plugin for the emulation of the yarp inertial MTB device in Gazebo.
    ///
    /// This plugin adds the inertial Gazebo sensor to the Handler database for
    /// later retrieval by the driver 'EmbObjInertialsDriver'. This driver will
    /// expose the sensor measurements and metadata to a remapper or wrapper device.
    ///
    /// The only relevant information needed by this plugin is the sensor scoped name
    /// provided by Gazebo.
    ///
    class GazeboYarpInertialSensorReg : public SensorPlugin
    {
    public:
        GazeboYarpInertialSensorReg();
        virtual ~GazeboYarpInertialSensorReg();
        /**
         * Saves the gazebo sensor pointer for later retrieval by the driver
         */
        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    private:
        std::string m_sensorName;
    };
}

#endif // GAZEBOYARP_INERTIALSENSORREG_HH
