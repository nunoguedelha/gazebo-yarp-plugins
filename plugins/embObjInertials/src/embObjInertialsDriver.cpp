/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "embObjInertialsDriver.h"
#include <GazeboYarpPlugins/ConfHelpers.hh>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/sensors/ImuSensor.hh>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <yarp/os/LockGuard.h>

#include <assert.h>

using namespace yarp::dev;


GazeboYarpEmbObjInertialsDriver::GazeboYarpEmbObjInertialsDriver() {}
GazeboYarpEmbObjInertialsDriver::~GazeboYarpEmbObjInertialsDriver() {}

/**
 *
 * Export an inertial sensor.
 * The 3 channels are non calibrated 3-axis (X, Y, Z) acceleration data.
 *
 */
//void GazeboYarpEmbObjInertialsDriver::onUpdate(const gazebo::common::UpdateInfo &updateInfo)
//{
//    // Get current Gazebo timestamp (ms). This is a global timestamp.
//    // The individual timestamps of each sensor might slightly differ
//    // as it is the case on the real robot.
//    // This timestamp can be accessed by the wrapper via the IPreciselyTimed
//    // interface. Instead, the Analog wrapper is using the its own stamp
//    // along with the Yarp clock. If Gazebo is launched with the clock plugin,
//    // the MTB driver and the wrapper will have the same timestamp.
//    m_lastGazeboTimestamp.update(updateInfo.simTime.Double());
//
//    // Get all sensors data
//    std::vector<gazebo::sensors::ImuSensor*>::iterator sensorIter;
//    int bufferOffset;
//    for (sensorIter = m_enabledSensors.begin(),
//         bufferOffset = sensorDataStartOffset+sensorTimestpNmeasOffset;
//         sensorIter < m_enabledSensors.end();
//         sensorIter++,
//         bufferOffset+=sensorDataLength)
//    {
//#if GAZEBO_MAJOR_VERSION >= 6
//        ignition::math::Vector3d linear_acceleration = (*sensorIter)->LinearAcceleration();
//#else
//        gazebo::math::Vector3 linear_acceleration = (*sensorIter)->GetLinearAcceleration();
//#endif
//
//#if GAZEBO_MAJOR_VERSION >= 7
//        double sensorLastTimestamp = (*sensorIter)->LastUpdateTime().Double();
//#else
//        double sensorLastTimestamp = (*sensorIter)->GetLastUpdateTime().Double();
//#endif
//
//        //Fill the 3 channels measurement data, applying the m/s^2 to raw
//        //fullscale gain
//        m_dataMutex.wait();
//        m_inertialmtbOutBuffer[bufferOffset] = sensorLastTimestamp;
//        for (unsigned idx = 0; idx < 3; idx++) {
//            m_inertialmtbOutBuffer[bufferOffset+1+idx] = 1e04/5.9855*linear_acceleration[idx];
//        }
//        m_dataMutex.post();
//    }
//}


/**
 * DeviceDriver
 */
bool GazeboYarpEmbObjInertialsDriver::open(yarp::os::Searchable& config)
{
    // Get from the options the list of enabled sensors
    std::vector<std::string> sensorNameList;
    GazeboYarpPlugins::getVectorOfStringFromListInConfig("SERVICE/SETTINGS/enabledSensors", config, sensorNameList);
    
    // Get from the options the list of their respective types
    std::vector<std::string> sensorTypeList;
    GazeboYarpPlugins::getVectorOfStringFromListInConfig("SERVICE/PROPERTIES/SENSORS/type", config, sensorTypeList);

    // Get the parent model (robot name) for rebuilding the sensors scoped names
    std::string robotName(config.find("gazeboYarpPluginsRobotName").asString().c_str());

    // Build the enabled sensors metadata per type (Gazebo pointers, name, frameName, fixedGain)
    setEnabledSensorsMetadata(robotName,sensorNameList,sensorTypeList);

    //Connect the driver to the gazebo simulation
    m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpEmbObjInertialsDriver::onUpdate, this, _1));

    return true;
}

bool GazeboYarpEmbObjInertialsDriver::close()
{
    this->m_updateConnection.reset();
    return true;
}

bool GazeboYarpEmbObjInertialsDriver::setEnabledSensorsMetadata(std::string robotName,
                                                                std::vector<std::string>& sensorNameList,
                                                                std::vector<std::string>& sensorTypeList)
{return false;}
//{
//    // get the list of registered active sensors (connected to a sensor plugin)
//    std::vector<std::string> activeSensors = GazeboYarpPlugins::Handler::getHandler()->getSensors();
//
//    // resize the sensors vector
//    m_enabledSensors.resize(enabledSensors.size());
//
//    // Go through the sensors listed in 'enabledSensors'
//    for (int sensorIter = 0; sensorIter < enabledSensors.size(); sensorIter++)
//    {
//        // get the next sensor scoped name from the input list
//        std::string sensorScopedName;
//        if (!GazeboYarpEmbObjInertialsDriver::
//            getNameCompletionFromList(activeSensors,
//                                      enabledSensors.get(sensorIter).asString(),
//                                      sensorScopedName))
//        {
//            yError() << "GazeboYarpEmbObjInertialsDriver Error: required sensor name "
//            << enabledSensors.get(sensorIter).asString() << " was not found";
//            return false;
//        }
//
//        //Get the gazebo pointer (we assume it has been previously added by a sensor plugin)
//        gazebo::sensors::ImuSensor* sensor =
//        dynamic_cast<gazebo::sensors::ImuSensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));
//
//        if (!sensor) {
//            yError() << "GazeboYarpEmbObjInertialsDriver Error: required sensor scoped name "
//            << sensorScopedName << " was not found";
//            return false;
//        }
//
//        // Add sensor to the building vector
//        m_enabledSensors[sensorIter] = sensor;
//    }
//
//    return true;
//}

//bool GazeboYarpEmbObjInertialsDriver::buildOutBufferFixedData(std::string robotPart,
//                                                              yarp::os::Bottle & enabledSensors)
//{
//    // Resize the output buffer
//    m_nbChannels = sensorDataStartOffset + LUTpart2maxSensors[robotPart]*sensorDataLength;
//    m_inertialmtbOutBuffer.resize(m_nbChannels,0);
//
//    /*
//     * Go through the buffer and fill the metadata
//     */
//    m_dataMutex.wait();
//
//    // number of enabled sensors and VERsion of the format
//    m_inertialmtbOutBuffer(0) = double(enabledSensors.size());
//    m_inertialmtbOutBuffer(1) = version;
//
//    // got through all enabled sensors
//    for (int sensorIdx = 0, bufferOffset = sensorDataStartOffset;
//         sensorIdx < enabledSensors.size();
//         sensorIdx++, bufferOffset+=sensorDataLength)
//    {
//        std::vector<std::string> explodedSensorName =
//        GazeboYarpPlugins::splitString(enabledSensors.get(sensorIdx).asString(),"_");
//        // Get the sensor label (1b1, 1b2, ...)
//        std::string sensorLabel = *(explodedSensorName.end()-1);
//        // Get the sensor type (accelerometer or gyroscope)
//        std::string sensorType = *(explodedSensorName.end()-2);
//        // Set the metadata in the output buffer
//        m_inertialmtbOutBuffer(bufferOffset+sensorIdxOffset) = double(LUTmtbId2PosEnum[sensorLabel]);
//        m_inertialmtbOutBuffer(bufferOffset+sensorTypeOffset) = double(LUTmtbType2enum[sensorType]);
//    }
//
//    // The remaining content is already by default set to zero
//
//    m_dataMutex.post();
//
//    return true;
//}



/**
 * IThreeAxisGyroscopes
 */
size_t GazeboYarpEmbObjInertialsDriver::getNrOfThreeAxisGyroscopes() const
{
    return genericGetNrOfSensors(m_enabledSensors.threeAxisGyroscopes);
}

yarp::dev::MAS_status GazeboYarpEmbObjInertialsDriver::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return genericGetStatus(m_sensorsMeasurements.threeAxisGyroscopes, "ThreeAxisGyroscopes", sens_index);
}

bool GazeboYarpEmbObjInertialsDriver::getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const
{
    return genericGetName(m_sensorsMetadata.threeAxisGyroscopes, "ThreeAxisGyroscopes", sens_index, name);
}

bool GazeboYarpEmbObjInertialsDriver::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const
{
    return genericGetFrameName(m_sensorsMetadata.threeAxisGyroscopes, "ThreeAxisGyroscopes", sens_index, frameName);
}

bool GazeboYarpEmbObjInertialsDriver::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return genericGetMeasure(m_sensorsMetadata.threeAxisGyroscopes, "ThreeAxisGyroscopes",
                             m_sensorsMeasurements.threeAxisGyroscopes, sens_index, out, timestamp);
}

/**
 * IThreeAxisLinearAccelerometers methods
 */
size_t GazeboYarpEmbObjInertialsDriver::getNrOfThreeAxisLinearAccelerometers() const
{
    return genericGetNrOfSensors(m_enabledSensors.threeAxisLinearAccelerometers);
}

yarp::dev::MAS_status GazeboYarpEmbObjInertialsDriver::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return genericGetStatus(m_sensorsMeasurements.threeAxisLinearAccelerometers, "ThreeAxisLinearAccelerometers", sens_index);
}

bool GazeboYarpEmbObjInertialsDriver::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const
{
    return genericGetName(m_sensorsMetadata.threeAxisLinearAccelerometers, "ThreeAxisLinearAccelerometers", sens_index, name);
}

bool GazeboYarpEmbObjInertialsDriver::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const
{
    return genericGetFrameName(m_sensorsMetadata.threeAxisLinearAccelerometers, "ThreeAxisLinearAccelerometers", sens_index, frameName);
}

bool GazeboYarpEmbObjInertialsDriver::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    return genericGetMeasure(m_sensorsMetadata.threeAxisLinearAccelerometers, "ThreeAxisLinearAccelerometers",
                             m_sensorsMeasurements.threeAxisLinearAccelerometers, sens_index, out, timestamp);
}


/**
 * Generic methods
 */
size_t GazeboYarpEmbObjInertialsDriver::genericGetNrOfSensors(const std::vector<gazebo::sensors::ImuSensor*>& enabledSensors) const
{
    return enabledSensors.size();
}

MAS_status GazeboYarpEmbObjInertialsDriver::genericGetStatus(const std::vector<sensorMeasurement_t>& measurementsVector, const std::string& tag, size_t sens_index) const
{
    if (~checkSensorIndex(measurementsVector, tag, sens_index)) {return MAS_UNKNOWN;}

    std::lock_guard<std::mutex> guard(measurementsVector[sens_index].dataMutex);
    return measurementsVector[sens_index].status;
}

bool GazeboYarpEmbObjInertialsDriver::genericGetName(const std::vector<sensorMetadata_t>& metadataVector, const std::string& tag,
                                                     size_t sens_index, std::string& name) const
{
    if (~checkSensorIndex(metadataVector, tag, sens_index)) {return false;}

    name = metadataVector[sens_index].name;
    return true;
}

bool GazeboYarpEmbObjInertialsDriver::genericGetFrameName(const std::vector<sensorMetadata_t>& metadataVector, const std::string& tag,
                                                          size_t sens_index, std::string& frameName) const
{
    if (~checkSensorIndex(metadataVector, tag, sens_index)) {return false;}

    frameName = metadataVector[sens_index].frameName;
    return true;
}

bool GazeboYarpEmbObjInertialsDriver::genericGetMeasure(const std::vector<sensorMetadata_t>& metadataVector, const std::string& tag,
                                                        const std::vector<sensorMeasurement_t>& measurementsVector,
                                                        size_t sens_index, sig::Vector& out, double& timestamp) const
{
    if (~checkSensorIndex(metadataVector, tag, sens_index)) {return false;}

    std::lock_guard<std::mutex> guard(measurementsVector[sens_index].dataMutex);
    
    assert(metadataVector.size() == measurementsVector.size());
    
    timestamp = measurementsVector[sens_index].timestamp.getTime();
    out = measurementsVector[sens_index].measurement;
    
    return true;
}

/**
 * Helper and getter methods
 */
std::string GazeboYarpEmbObjInertialsDriver::getDriverScopedName() const
{
    return m_driverScopedName;
}

bool GazeboYarpEmbObjInertialsDriver::getNameCompletionFromList(std::vector<std::string> &stringList,
                                                           std::string const &endingString,
                                                           std::string &fullString)
{
    // search the list
    bool found = false;
    for (std::vector<std::string>::iterator iter=stringList.begin();
         iter<stringList.end() && !found;
         iter++)
    {
        found=GazeboYarpPlugins::hasEnding(*iter,endingString);
        if (found)
        {
            fullString = *iter;
            stringList.erase(iter);
        }
    }
    return found;
}

template <class T>
bool GazeboYarpEmbObjInertialsDriver::checkSensorIndex(const std::vector<T>& aVector,
                                                       const std::string& tag,
                                                       size_t sens_index) const
{
    if (sens_index >= aVector.size())
    {
        yError("GazeboYarpEmbObjInertialsDriver: No sensor of type %s with index %lu (nr of sensors: %lu).",
               tag.c_str(), sens_index, aVector.size());
        return false;
    }
    return true;
}

