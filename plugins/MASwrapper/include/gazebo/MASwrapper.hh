/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_MASWRAPPER_HH
#define GAZEBOYARP_MASWRAPPER_HH

#include <gazebo/common/Plugin.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <string>


namespace gazebo
{
    namespace sensors {
        class ImuSensor;
    }

    /// \class GazeboYarpMASwrapper
    /// Gazebo Plugin emulating the "multipleanalogsensorsserver" device in Gazebo.
    ///
    /// This plugin instantiates the "multipleanalogsensorsserver" device for attaching
    /// a MAS remapper device or a sensors device 'embObjInertials'. Once the devices are
    /// attached, the server/wrapper device publishes the sensor data on a YARP port. The
    /// name of the device to attach as well as the destination port are defined in the
    /// file embedded in the URDF model, which parameters are passed to the Model plugin
    /// calling the methods of this class.
    ///
    /// The MAS wrapper/server can be configurated using the yarpConfigurationFile sdf
    /// tag, that contains a Gazebo URI pointing at a yarp .ini configuration file
    /// containing the configuration parameters of the server.
    ///
    /// The yarpConfigurationFile is the exact same configuration file used on the real
    /// robot for the same purposes.
    ///
    class GazeboYarpMASwrapper : public ModelPlugin
    {
    public:
        GazeboYarpMASwrapper();
        virtual ~GazeboYarpMASwrapper();
        /**
         * Saves the gazebo pointer, retrieves the configuration parameters from the
         * sdf file, creates and opens the remapper device with those parameters as input
         * configuration for the remapper (enabled connected sensors).
         */
        virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:
        yarp::dev::PolyDriver m_MASwrapperDevice;
        yarp::dev::IMultipleWrapper* m_iWrap;
        std::string m_robotName;
    };
}

#endif // GAZEBOYARP_MASWRAPPER_HH
