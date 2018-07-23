/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#ifndef GAZEBOYARP_EMBOBJINERTIALS_HH
#define GAZEBOYARP_EMBOBJINERTIALS_HH

#include <gazebo/common/Plugin.hh>

#include <yarp/dev/PolyDriver.h>

#include <string>


// Forward declarations
namespace yarp {
    namespace dev {
        class GazeboYarpEmbObjInertialsDriver;
    }
}

namespace gazebo
{
    namespace sensors {
        class ImuSensor;
    }

    /// \class GazeboYarpEmbObjInertials
    /// Gazebo Plugin emulating the "embObjInertials" firmware device in Gazebo.
    ///
    /// This plugin instantiates the "embObjInertials" device for the Gazebo simulator
    /// to expose the inertial sensors connected to a virtual EMS board. The virtual
    /// EMS board is defined by the configuration file embedded in the URDF model, and
    /// which parameters are passed to the Model plugin calling the methods of this class.
    /// Since an EMS board can host a set of sensors distributed accross a full part of
    /// the robot (e.g. EMS10 in the left leg), the respective configuration file in the
    /// URDF shall be handled by a model plugin versus a sensor plugin.
    ///
    /// The virtual EMS board can be configurated using the yarpConfigurationFile sdf
    /// tag, that contains a Gazebo URI pointing at a yarp .ini configuration file
    /// containing the configuration parameters of the board.
    ///
    /// The yarpConfigurationFile is the exact same configuration file used on the real
    /// robot for the same purposes.
    ///
    class GazeboYarpEmbObjInertials : public ModelPlugin
    {
    public:
        GazeboYarpEmbObjInertials();
        virtual ~GazeboYarpEmbObjInertials();
        /**
         * Retrieves the configuration parameters from the sdf file, creates and opens
         * the device driver with those parameters as input configuration for the driver
         * (enabled sensors connected to the EMS).
         */
        virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:
        yarp::dev::PolyDriver m_embObjInertialsDriver;
        yarp::dev::GazeboYarpEmbObjInertialsDriver* m_iInertials;
        std::string m_robotName;
    };
}

#endif // GAZEBOYARP_EMBOBJINERTIALS_HH
