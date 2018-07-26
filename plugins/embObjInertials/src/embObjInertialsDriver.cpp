/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "embObjInertialsDriver.h"
#include <GazeboYarpPlugins/ConfHelpers.hh>
#include <GazeboYarpPlugins/GazeboAPIHelpers.hh>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/sensors/ImuSensor.hh>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <yarp/os/LockGuard.h>

#include <assert.h>
#include <map>

using namespace yarp::dev;
using namespace GazeboYarpPlugins;


GazeboYarpEmbObjInertialsDriver::GazeboYarpEmbObjInertialsDriver() {}
GazeboYarpEmbObjInertialsDriver::~GazeboYarpEmbObjInertialsDriver() {}

/**
 * Export an inertial sensor.
 * The 3 channels are non calibrated 3-axis (X, Y, Z) acceleration data.
 */
void GazeboYarpEmbObjInertialsDriver::onUpdate(const gazebo::common::UpdateInfo &updateInfo)
{
    // Get current Gazebo timestamp (ms). for debug purposes. This is a global
    // timestamp. The individual timestamps of each sensor might slightly differ
    // as it is the case on the real robot.
    // The wrapper device includes in the yarp frame envelope the yarp clock.
    // If Gazebo is launched with the clock plugin, the Gazebo sensor clocks
    // and the yarp port envelope clock will be the same (if we neglect the
    // delays between sensor timestamps in the same yarp frame.
    m_globalLastTimeStamp.update(updateInfo.simTime.Double());

    // Update all the gyroscopes data
    std::vector<gazebo::sensors::ImuSensor*>::iterator sensorIter;
    std::vector<sensorMeasurement_t>::iterator sensorMeasIter;
    std::vector<sensorMetadata_t>::iterator sensorMetaIter;
    for (sensorIter = m_enabledSensors.threeAxisGyroscopes.begin(),
         sensorMeasIter = m_sensorsMeasurements.threeAxisGyroscopes.begin(),
         sensorMetaIter = m_sensorsMetadata.threeAxisGyroscopes.begin();
         sensorIter < m_enabledSensors.threeAxisGyroscopes.end();
         sensorIter++,sensorMeasIter++,sensorMetaIter++)
    {
        // Get the sensor measurement and timestamp
        ignition::math::Vector3d angular_velocity = (*sensorIter)->AngularVelocity();
        double sensorLastTimestamp = (*sensorIter)->LastUpdateTime().Double();
        double fixedGain = sensorMetaIter->fixedGain;

        // Lock the mutex
        std::lock_guard<std::mutex> guard(sensorMeasIter->dataMutex);

        /* Set the timestamp and the measurement (3 channels measurement), applying the m/s^2 to raw fullscale gain:
         * To convert the gyroscope measure in dps (deg/s) the conversion factor is (fullscale in dps)/(fullscale in raw)
         * = (250)/(2^15) ~= 7.6274e-03.
         */
        sensorMeasIter->timestamp.update(sensorLastTimestamp);
        sensorMeasIter->status = MAS_OK;
        sensorMeasIter->measurement[0] = angular_velocity[0]*fixedGain;
        sensorMeasIter->measurement[1] = angular_velocity[1]*fixedGain;
        sensorMeasIter->measurement[2] = angular_velocity[2]*fixedGain;
    }

    // Update all the accelerometers data
    for (sensorIter = m_enabledSensors.threeAxisLinearAccelerometers.begin(),
         sensorMeasIter = m_sensorsMeasurements.threeAxisLinearAccelerometers.begin(),
         sensorMetaIter = m_sensorsMetadata.threeAxisLinearAccelerometers.begin();
         sensorIter < m_enabledSensors.threeAxisLinearAccelerometers.end();
         sensorIter++,sensorMeasIter++,sensorMetaIter++)
    {
        // Get the sensor measurement and timestamp
        ignition::math::Vector3d linear_acceleration = (*sensorIter)->LinearAcceleration();
        double sensorLastTimestamp = (*sensorIter)->LastUpdateTime().Double();
        double fixedGain = sensorMetaIter->fixedGain;

        // Lock the mutex
        std::lock_guard<std::mutex> guard(sensorMeasIter->dataMutex);
        
        /* Set the timestamp and the measurement (3 channels measurement), applying the m/s^2 to raw fullscale gain:
         * To convert the accelerometer measure in m/s^2 the conversion factor is (fullscale in m/s^2)/(fullscale in raw)
         * = (2*g)/(2^15) ~= 5.9855e-04.
         */
        sensorMeasIter->timestamp.update(sensorLastTimestamp);
        sensorMeasIter->status = MAS_OK;
        sensorMeasIter->measurement[0] = linear_acceleration[0]*fixedGain;
        sensorMeasIter->measurement[1] = linear_acceleration[1]*fixedGain;
        sensorMeasIter->measurement[2] = linear_acceleration[2]*fixedGain;
    }
}


/**
 * DeviceDriver
 */
bool GazeboYarpEmbObjInertialsDriver::open(yarp::os::Searchable& config)
{
    // Get from the options the list of enabled sensors
    std::vector<std::string> sensorNameList;
    yarp::os::Bottle& settingsGrp = config.findGroup("SERVICE").findGroup("SETTINGS");
    GazeboYarpPlugins::getVectorOfStringFromListInConfig("enabledSensors", settingsGrp, sensorNameList);
    
    // Get from the options the list of their respective types
    std::vector<std::string> sensorTypeList;
    yarp::os::Bottle& sensorsGrp = config.findGroup("SERVICE").findGroup("PROPERTIES").findGroup("SENSORS");
    GazeboYarpPlugins::getVectorOfStringFromListInConfig("type", sensorsGrp, sensorTypeList);

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

bool GazeboYarpEmbObjInertialsDriver::setEnabledSensorsMetadata(const std::string& robotName,
                                                                const std::vector<std::string>& sensorNameList,
                                                                const std::vector<std::string>& sensorTypeList)
{
    // get the list of registered active sensors (connected to a sensor plugin)
    std::vector<std::string> activeSensors = GazeboYarpPlugins::Handler::getHandler()->getSensors();

    // Split the input sensor list into vectors by modality (gyroscopes, accelerometers, ...)
    std::map<std::string,std::vector<std::string> > sensorNamesByType;
    std::pair<std::map<std::string,std::vector<std::string> >::iterator,bool> mapInsertRet;
    std::vector<std::string>::const_iterator sensorNamesIter;
    std::vector<std::string>::const_iterator sensorTypesIter;
    for  (sensorNamesIter = sensorNameList.begin(),
          sensorTypesIter = sensorTypeList.begin();
          sensorNamesIter < sensorNameList.end();
          sensorNamesIter++,sensorTypesIter++)
    {
        // Try to insert a new element <key,emptyNameList> in the map
        std::vector<std::string> aNameList;
        aNameList.reserve(MAX(sensorNameList.size(), sensorTypeList.size()));
        mapInsertRet = sensorNamesByType.insert(std::pair<std::string,std::vector<std::string> >
                                                      (*sensorTypesIter, aNameList));
        
        // If the key is new in the map, the method returns a pointer to the new pair, otherwise
        // there is no insertion and the method returns a pointer to the existing pair.
        // Now, add the sensor name in the new/existing list.
        mapInsertRet.first->second.push_back(*sensorNamesIter);
    }
    
    std::vector<std::string> gyroNameList = GazeboYarpPlugins::concatStringVectors<std::string,2>({
        sensorNamesByType["eoas_gyros_st_l3g4200d"],
        sensorNamesByType["eoas_imu_gyr"]
    });
    
    std::vector<double> gyroGainList = GazeboYarpPlugins::concatStringVectors<double,2>({
        std::vector<double>(sensorNamesByType.count("eoas_gyros_st_l3g4200d"),(2^15)/(250)),
        std::vector<double>(sensorNamesByType.count("eoas_imu_gyr"),1)
    });
    
    std::vector<std::string> linAccNameList = GazeboYarpPlugins::concatStringVectors<std::string,3>({
        sensorNamesByType["eoas_accel_st_lis3x"],
        sensorNamesByType["eoas_accel_mtb_int"],
        sensorNamesByType["eoas_imu_acc"]
    });
    
    std::vector<double> linAccGainList = GazeboYarpPlugins::concatStringVectors<double,3>({
        std::vector<double>(sensorNamesByType.count("eoas_accel_st_lis3x"),1),
        std::vector<double>(sensorNamesByType.count("eoas_accel_mtb_int"),(2^15)/(2*9.81)),
        std::vector<double>(sensorNamesByType.count("eoas_imu_acc"),1)
    });
    
    // Resize the capacity of sensor metadata structures accordingly.
    m_enabledSensors.threeAxisGyroscopes.reserve(gyroNameList.size());
    m_enabledSensors.threeAxisLinearAccelerometers.reserve(linAccNameList.size());
    m_sensorsMetadata.threeAxisGyroscopes.reserve(gyroNameList.size());
    m_sensorsMetadata.threeAxisLinearAccelerometers.reserve(linAccNameList.size());
    
    // Resize the sensor analog data structures and initialize.
    m_sensorsMeasurements.threeAxisGyroscopes.resize(gyroNameList.size());
    m_sensorsMeasurements.threeAxisLinearAccelerometers.resize(linAccNameList.size());
    
    // Go through the lists 'gyroNameList' and 'linAccNameList' and Build the sensors metadata: Gazebo pointers, name,
    // frameName, fixedGain.
    return (convert2metadataFormat(robotName,gyroNameList,gyroGainList,
                                   activeSensors,
                                   m_sensorsMetadata.threeAxisGyroscopes,
                                   m_enabledSensors.threeAxisGyroscopes) &&
            convert2metadataFormat(robotName,linAccNameList,linAccGainList,
                                   activeSensors,
                                   m_sensorsMetadata.threeAxisLinearAccelerometers,
                                   m_enabledSensors.threeAxisLinearAccelerometers));
}


/**
 * Go through the lists 'gyroNameList' and 'linAccNameList' and Build the sensors metadata: Gazebo pointers, name,
 * frameName, fixedGain.
 */
bool GazeboYarpEmbObjInertialsDriver::convert2metadataFormat(const std::string& robotName,
                                                             const std::vector<std::string>& sensorNameList,
                                                             const std::vector<double>& sensorGainList,
                                                             std::vector<std::string>& activeSensors,
                                                             std::vector<sensorMetadata_t>& metadataVector,
                                                             std::vector<gazebo::sensors::ImuSensor*>& enabledSensors)
{
    std::vector<std::string>::const_iterator sensorNamesIter;
    std::vector<double>::const_iterator sensorGainsIter;

    for (sensorNamesIter = sensorNameList.begin(),
         sensorGainsIter = sensorGainList.begin();
         sensorNamesIter < sensorNameList.end();
         sensorNamesIter++,sensorGainsIter++)
    {
        // Fill and push the metadata element
        sensorMetadata_t metadata = {*sensorNamesIter,*sensorNamesIter,*sensorGainsIter};
        metadataVector.push_back(metadata);
        
        // Get the sensor scoped name
        std::string sensorScopedName;
        if (!getNameCompletionFromList(activeSensors,robotName,*sensorNamesIter,sensorScopedName))
        {
            yError() << "GazeboYarpEmbObjInertialsDriver Error: required sensor name "
            << *sensorNamesIter << " was not found";
            return false;
        }
        
        //Get the gazebo pointer (we assume it has been previously added by a sensor plugin)
        gazebo::sensors::ImuSensor* sensorPtr =
        dynamic_cast<gazebo::sensors::ImuSensor*>(GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName));
        
        if (!sensorPtr) {
            yError() << "GazeboYarpEmbObjInertialsDriver Error: required sensor scoped name "
            << sensorScopedName << " was not found";
            return false;
        }
        
        // Add sensor to the building vector
        enabledSensors.push_back(sensorPtr);
    }
    
    return true;
}


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
                                                                const std::string& robotName,
                                                                std::string const &endingString,
                                                                std::string &fullString)
{
    // search the list
    bool found = false;
    for (std::vector<std::string>::iterator iter = stringList.begin();
         iter < stringList.end() && !found;
         iter++)
    {
        found = GazeboYarpPlugins::hasEnding(*iter,endingString);
        if (found)
        {
            std::vector<std::string> explodedScopedSensorName = GazeboYarpPlugins::splitString(*iter,":");
            
            // The vector should be at least of 3 elements because
            // scopedSensorName should be something similar to
            // [worldName::]modelName::linkOrJointName::sensorName
            if( explodedScopedSensorName.size() < 3 )
            {
                yError() << "GazeboYarpEmbObjInertialsDriver error: unexpected scopedSensorName " << *iter;
                return false;
            }
            // Extract the robot name from the sensor scoped name and compare with the robot name from the driver
            // configuration
            std::string robotNameInSensorScopedName = explodedScopedSensorName[explodedScopedSensorName.size()-3];
            if (robotName.compare(robotNameInSensorScopedName))
            {
                yError() << "GazeboYarpEmbObjInertialsDriver error: robot name not set in sensor configuration " << *iter;
                return false;
            }
            
            // Everything went well, return full string and remove it from the list since each sensor appears only
            // once in the configuration files
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

