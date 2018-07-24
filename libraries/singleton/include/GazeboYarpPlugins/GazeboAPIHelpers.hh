/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_API_HELPERS_HH
#define GAZEBOYARP_API_HELPERS_HH

#include <gazebo/sensors/ImuSensor.hh>

namespace GazeboYarpPlugins {

/**
 * Templates for handling interface differences between Gazebo versions
 */
template <bool isGazeboMajorVersionGTE_6,bool isGazeboMajorVersionGTE_7>
class AllGazeboSensorMeasurementGetters {};

#if (GAZEBO_MAJOR_VERSION >= 7)
// Specialization for Gazebo version >= 7
template <>
class AllGazeboSensorMeasurementGetters<true,true>
{
public:
    typedef ignition::math::Vector3d returnType;
    
    static returnType LinearAcceleration(gazebo::sensors::ImuSensor* sensor)
    {
        return sensor->LinearAcceleration();
    }
    
    static double LastUpdateTime(gazebo::sensors::ImuSensor* sensor)
    {
        return sensor->LastUpdateTime().Double();
    }
};
#elif (GAZEBO_MAJOR_VERSION >= 6)
// Specialization for Gazebo version = 6
template <>
class AllGazeboSensorMeasurementGetters<true,false>
{
public:
    typedef ignition::math::Vector3d returnType;
    
    static returnType LinearAcceleration(gazebo::sensors::ImuSensor* sensor)
    {
        return sensor->LinearAcceleration();
    }
    
    static double LastUpdateTime(gazebo::sensors::ImuSensor* sensor)
    {
        return sensor->GetLastUpdateTime().Double();
    }
};
#else
// Specialization for Gazebo version < 6
template <>
class AllGazeboSensorMeasurementGetters<false,false>
{
public:
    typedef gazebo::math::Vector3 returnType;
    
    static returnType LinearAcceleration(gazebo::sensors::ImuSensor* sensor)
    {
        return sensor->GetLinearAcceleration();
    }
    
    static double LastUpdateTime(gazebo::sensors::ImuSensor* sensor)
    {
        return sensor->GetLastUpdateTime().Double();
    }
};
#endif

typedef AllGazeboSensorMeasurementGetters<GAZEBO_MAJOR_VERSION>=6,GAZEBO_MAJOR_VERSION>=7> GazeboSensorMeasurementGetter;

}

#endif /* GAZEBOYARP_API_HELPERS_HH */
