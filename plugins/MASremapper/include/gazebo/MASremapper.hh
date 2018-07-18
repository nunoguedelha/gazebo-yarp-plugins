/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_MASREMAPPER_HH
#define GAZEBOYARP_MASREMAPPER_HH

#include <gazebo/common/Plugin.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>

#include <string>


// Forward declarations
namespace yarp {
    namespace dev {
        class MultipleAnalogSensorsRemapper;
    }
}

namespace gazebo
{
    namespace sensors {
        class ImuSensor;
    }

    /// \class GazeboYarpMASremapper
    /// Gazebo Plugin emulating the "multipleanalogsensorsremapper" device in Gazebo.
    ///
    /// This plugin instantiates the "multipleanalogsensorsremapper" device for remapping
    /// the set of inertial sensors accessible through the EMS board(s) which are connected
    /// (attached) to the MAS remapper device. The remapping is defined by the configuration
    /// file embedded in the URDF model, which parameters are passed to the Model plugin
    /// calling the methods of this class.
    ///
    /// The MAS remapper can be configurated using the yarpConfigurationFile sdf
    /// tag, that contains a Gazebo URI pointing at a yarp .ini configuration file
    /// containing the configuration parameters of the remapper.
    ///
    /// The yarpConfigurationFile is the exact same configuration file used on the real
    /// robot for the same purposes.
    ///
    class GazeboYarpMASremapper : public ModelPlugin
    {
    public:
        GazeboYarpMASremapper();
        virtual ~GazeboYarpMASremapper();
        /**
         * Saves the gazebo pointer, retrieves the configuration parameters from the
         * sdf file, creates and opens the remapper device with those parameters as input
         * configuration for the remapper (enabled connected sensors).
         */
        virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:
        yarp::dev::PolyDriver m_MASremapperDevice;
        yarp::dev::IMultipleWrapper* m_iWrap;
        std::string m_robotName;
    };
}

#endif // GAZEBOYARP_MASREMAPPER_HH
