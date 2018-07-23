/*
 * Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_EMBOBJINERTIALSDRIVER_H
#define GAZEBOYARP_EMBOBJINERTIALSDRIVER_H

#include <vector>
#include <map>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <mutex>

#include <boost/shared_ptr.hpp>

//Forward declarations
namespace yarp {
    namespace dev {
        class GazeboYarpEmbObjInertialsDriver;
    }
}

namespace gazebo {
    namespace common {
        class UpdateInfo;
    }
    namespace sensors {
        class ImuSensor;
    }
    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }
}

/// \class GazeboYarpEmbObjInertialsDriver
///
/// This class implements the driver that exposes the sensors data
/// and metadata through the Multiple Analog Sensors Interface, as
/// part of the "embObjInertials" device emulated by the the Gazebo
/// plugin of same name. Refer to the class 'GazeboYarpEmbObjInertials'
/// definition for further information.
class yarp::dev::GazeboYarpEmbObjInertialsDriver:
    public yarp::dev::IThreeAxisGyroscopes,
    public yarp::dev::IThreeAxisLinearAccelerometers,
    public yarp::dev::DeviceDriver
{
public:
    GazeboYarpEmbObjInertialsDriver();

    virtual ~GazeboYarpEmbObjInertialsDriver();

    void onUpdate(const gazebo::common::UpdateInfo&) {}

    /**
     * Yarp interfaces start here
     */

    /* DeviceDriver */
    virtual bool open(yarp::os::Searchable& config) override;
    virtual bool close() override;

    /* IThreeAxisGyroscopes */
    virtual size_t getNrOfThreeAxisGyroscopes() const override;
    virtual yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;
    virtual bool getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const override;
    virtual bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */
    virtual size_t getNrOfThreeAxisLinearAccelerometers() const override;
    virtual yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;
    virtual bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const override;
    virtual bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* Specific getters */
    virtual std::string getDriverScopedName() const;

private:
    /**
     * Build the vector of enabled sensors as per the ordered
     * list retrieved from the part ini configuration file
     */
    bool setEnabledSensorsMetadata(std::string robotName,
                                   std::vector<std::string>& sensorNameList,
                                   std::vector<std::string>& sensorTypeList);
    /**
     *
     * @brief search the list of strings for a string matching the given end string.
     *
     * @param [in]  stringList   pick list of strings
     * @param [in]  endingString end string
     * @param [out] fullString   completed string (an element from stringList) if there is a match
     * @return                   'true' if a match was found
     */
    inline bool getNameCompletionFromList(std::vector<std::string> &stringList,
                                          std::string const &endingString,
                                          std::string &fullString);

    /**
     * Sensor analog data and metadata structure types
     */
    typedef struct {
        mutable std::mutex    dataMutex; //mutex for accessing the data
        yarp::sig::Vector     measurement;
        yarp::os::Stamp       timestamp;
        yarp::dev::MAS_status status;
    } sensorMeasurement_t;
    
    typedef struct {
        std::string name;
        std::string frameName;
        double      fixedGain;
    } sensorMetadata_t;
    
    typedef struct {
        std::vector<gazebo::sensors::ImuSensor*> threeAxisGyroscopes;
        std::vector<gazebo::sensors::ImuSensor*> threeAxisLinearAccelerometers;
    } enabledSensors_t;
    
    typedef struct {
        std::vector<sensorMetadata_t> threeAxisGyroscopes;
        std::vector<sensorMetadata_t> threeAxisLinearAccelerometers;
    } sensorsMetadata_t;
    
    typedef struct {
        std::vector<sensorMeasurement_t> threeAxisGyroscopes;
        std::vector<sensorMeasurement_t> threeAxisLinearAccelerometers;
    } sensorsMeasurements_t;
    
    template <class T>
    inline bool checkSensorIndex(const std::vector<T>& aVector,
                                 const std::string& tag,
                                 size_t sens_index) const;

    /* Implementation of the Yarp interfaces, common to all sensor types */
    size_t genericGetNrOfSensors(const std::vector<gazebo::sensors::ImuSensor*>& enabledSensors) const;
    MAS_status genericGetStatus(const std::vector<sensorMeasurement_t>& measurementsVector, const std::string& tag, size_t sens_index) const;
    bool genericGetName(const std::vector<sensorMetadata_t>& metadataVector, const std::string& tag,
                        size_t sens_index, std::string& name) const;
    bool genericGetFrameName(const std::vector<sensorMetadata_t>& metadataVector, const std::string& tag,
                             size_t sens_index, std::string& frameName) const;
    bool genericGetMeasure(const std::vector<sensorMetadata_t>& metadataVector, const std::string& tag,
                           const std::vector<sensorMeasurement_t>& measurementsVector,
                           size_t sens_index, sig::Vector& out, double& timestamp) const;

    /* Yarp interface parameters */
    std::string           m_driverScopedName = "left_leg-eb10-inertials"; // unique name identifying the driver in the Handler
    sensorsMetadata_t     m_sensorsMetadata;
    sensorsMeasurements_t m_sensorsMeasurements; // buffer for all sensors measurements
    enabledSensors_t      m_enabledSensors;

    /* Connection and synchro with Gazebo */
    gazebo::event::ConnectionPtr m_updateConnection;
};

#endif // GAZEBOYARP_EMBOBJINERTIALSDRIVER_H
