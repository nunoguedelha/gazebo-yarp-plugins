/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// Autogenerated by Thrift Compiler (0.12.0-yarped)
//
// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <ClockServer.h>

#include <yarp/os/idl/WireTypes.h>

namespace GazeboYarpPlugins {

class ClockServer_pauseSimulation :
        public yarp::os::Portable
{
public:
    void init();
    bool write(yarp::os::ConnectionWriter& connection) const override;
    bool read(yarp::os::ConnectionReader& connection) override;
};

bool ClockServer_pauseSimulation::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) {
        return false;
    }
    if (!writer.writeTag("pauseSimulation", 1, 1)) {
        return false;
    }
    return true;
}

bool ClockServer_pauseSimulation::read(yarp::os::ConnectionReader& connection)
{
    YARP_UNUSED(connection);
    return true;
}

void ClockServer_pauseSimulation::init()
{
}

class ClockServer_continueSimulation :
        public yarp::os::Portable
{
public:
    void init();
    bool write(yarp::os::ConnectionWriter& connection) const override;
    bool read(yarp::os::ConnectionReader& connection) override;
};

bool ClockServer_continueSimulation::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) {
        return false;
    }
    if (!writer.writeTag("continueSimulation", 1, 1)) {
        return false;
    }
    return true;
}

bool ClockServer_continueSimulation::read(yarp::os::ConnectionReader& connection)
{
    YARP_UNUSED(connection);
    return true;
}

void ClockServer_continueSimulation::init()
{
}

class ClockServer_stepSimulation :
        public yarp::os::Portable
{
public:
    std::int32_t numberOfSteps;
    void init(const std::int32_t numberOfSteps);
    bool write(yarp::os::ConnectionWriter& connection) const override;
    bool read(yarp::os::ConnectionReader& connection) override;
};

bool ClockServer_stepSimulation::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) {
        return false;
    }
    if (!writer.writeTag("stepSimulation", 1, 1)) {
        return false;
    }
    if (!writer.writeI32(numberOfSteps)) {
        return false;
    }
    return true;
}

bool ClockServer_stepSimulation::read(yarp::os::ConnectionReader& connection)
{
    YARP_UNUSED(connection);
    return true;
}

void ClockServer_stepSimulation::init(const std::int32_t numberOfSteps)
{
    this->numberOfSteps = numberOfSteps;
}

class ClockServer_stepSimulationAndWait :
        public yarp::os::Portable
{
public:
    std::int32_t numberOfSteps;
    void init(const std::int32_t numberOfSteps);
    bool write(yarp::os::ConnectionWriter& connection) const override;
    bool read(yarp::os::ConnectionReader& connection) override;
};

bool ClockServer_stepSimulationAndWait::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) {
        return false;
    }
    if (!writer.writeTag("stepSimulationAndWait", 1, 1)) {
        return false;
    }
    if (!writer.writeI32(numberOfSteps)) {
        return false;
    }
    return true;
}

bool ClockServer_stepSimulationAndWait::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) {
        return false;
    }
    return true;
}

void ClockServer_stepSimulationAndWait::init(const std::int32_t numberOfSteps)
{
    this->numberOfSteps = numberOfSteps;
}

class ClockServer_resetSimulationTime :
        public yarp::os::Portable
{
public:
    void init();
    bool write(yarp::os::ConnectionWriter& connection) const override;
    bool read(yarp::os::ConnectionReader& connection) override;
};

bool ClockServer_resetSimulationTime::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) {
        return false;
    }
    if (!writer.writeTag("resetSimulationTime", 1, 1)) {
        return false;
    }
    return true;
}

bool ClockServer_resetSimulationTime::read(yarp::os::ConnectionReader& connection)
{
    YARP_UNUSED(connection);
    return true;
}

void ClockServer_resetSimulationTime::init()
{
}

class ClockServer_getSimulationTime :
        public yarp::os::Portable
{
public:
    double _return;
    void init();
    bool write(yarp::os::ConnectionWriter& connection) const override;
    bool read(yarp::os::ConnectionReader& connection) override;
};

bool ClockServer_getSimulationTime::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) {
        return false;
    }
    if (!writer.writeTag("getSimulationTime", 1, 1)) {
        return false;
    }
    return true;
}

bool ClockServer_getSimulationTime::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) {
        return false;
    }
    if (!reader.readFloat64(_return)) {
        reader.fail();
        return false;
    }
    return true;
}

void ClockServer_getSimulationTime::init()
{
    _return = (double)0;
}

class ClockServer_getStepSize :
        public yarp::os::Portable
{
public:
    double _return;
    void init();
    bool write(yarp::os::ConnectionWriter& connection) const override;
    bool read(yarp::os::ConnectionReader& connection) override;
};

bool ClockServer_getStepSize::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) {
        return false;
    }
    if (!writer.writeTag("getStepSize", 1, 1)) {
        return false;
    }
    return true;
}

bool ClockServer_getStepSize::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) {
        return false;
    }
    if (!reader.readFloat64(_return)) {
        reader.fail();
        return false;
    }
    return true;
}

void ClockServer_getStepSize::init()
{
    _return = (double)0;
}

class ClockServer_resetSimulation :
        public yarp::os::Portable
{
public:
    void init();
    bool write(yarp::os::ConnectionWriter& connection) const override;
    bool read(yarp::os::ConnectionReader& connection) override;
};

bool ClockServer_resetSimulation::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) {
        return false;
    }
    if (!writer.writeTag("resetSimulation", 1, 1)) {
        return false;
    }
    return true;
}

bool ClockServer_resetSimulation::read(yarp::os::ConnectionReader& connection)
{
    YARP_UNUSED(connection);
    return true;
}

void ClockServer_resetSimulation::init()
{
}

class ClockServer_resetSimulationState :
        public yarp::os::Portable
{
public:
    void init();
    bool write(yarp::os::ConnectionWriter& connection) const override;
    bool read(yarp::os::ConnectionReader& connection) override;
};

bool ClockServer_resetSimulationState::write(yarp::os::ConnectionWriter& connection) const
{
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) {
        return false;
    }
    if (!writer.writeTag("resetSimulationState", 1, 1)) {
        return false;
    }
    return true;
}

bool ClockServer_resetSimulationState::read(yarp::os::ConnectionReader& connection)
{
    YARP_UNUSED(connection);
    return true;
}

void ClockServer_resetSimulationState::init()
{
}

// Constructor
ClockServer::ClockServer()
{
    yarp().setOwner(*this);
}

void ClockServer::pauseSimulation()
{
    ClockServer_pauseSimulation helper;
    helper.init();
    if (!yarp().canWrite()) {
        yError("Missing server method '%s'?", "void ClockServer::pauseSimulation()");
    }
    yarp().write(helper);
}

void ClockServer::continueSimulation()
{
    ClockServer_continueSimulation helper;
    helper.init();
    if (!yarp().canWrite()) {
        yError("Missing server method '%s'?", "void ClockServer::continueSimulation()");
    }
    yarp().write(helper);
}

void ClockServer::stepSimulation(const std::int32_t numberOfSteps)
{
    ClockServer_stepSimulation helper;
    helper.init(numberOfSteps);
    if (!yarp().canWrite()) {
        yError("Missing server method '%s'?", "void ClockServer::stepSimulation(const std::int32_t numberOfSteps)");
    }
    yarp().write(helper);
}

void ClockServer::stepSimulationAndWait(const std::int32_t numberOfSteps)
{
    ClockServer_stepSimulationAndWait helper;
    helper.init(numberOfSteps);
    if (!yarp().canWrite()) {
        yError("Missing server method '%s'?", "void ClockServer::stepSimulationAndWait(const std::int32_t numberOfSteps)");
    }
    yarp().write(helper, helper);
}

void ClockServer::resetSimulationTime()
{
    ClockServer_resetSimulationTime helper;
    helper.init();
    if (!yarp().canWrite()) {
        yError("Missing server method '%s'?", "void ClockServer::resetSimulationTime()");
    }
    yarp().write(helper);
}

double ClockServer::getSimulationTime()
{
    double _return = (double)0;
    ClockServer_getSimulationTime helper;
    helper.init();
    if (!yarp().canWrite()) {
        yError("Missing server method '%s'?", "double ClockServer::getSimulationTime()");
    }
    bool ok = yarp().write(helper, helper);
    return ok ? helper._return : _return;
}

double ClockServer::getStepSize()
{
    double _return = (double)0;
    ClockServer_getStepSize helper;
    helper.init();
    if (!yarp().canWrite()) {
        yError("Missing server method '%s'?", "double ClockServer::getStepSize()");
    }
    bool ok = yarp().write(helper, helper);
    return ok ? helper._return : _return;
}

void ClockServer::resetSimulation()
{
    ClockServer_resetSimulation helper;
    helper.init();
    if (!yarp().canWrite()) {
        yError("Missing server method '%s'?", "void ClockServer::resetSimulation()");
    }
    yarp().write(helper);
}

void ClockServer::resetSimulationState()
{
    ClockServer_resetSimulationState helper;
    helper.init();
    if (!yarp().canWrite()) {
        yError("Missing server method '%s'?", "void ClockServer::resetSimulationState()");
    }
    yarp().write(helper);
}

// help method
std::vector<std::string> ClockServer::help(const std::string& functionName)
{
    bool showAll = (functionName == "--all");
    std::vector<std::string> helpString;
    if (showAll) {
        helpString.emplace_back("*** Available commands:");
        helpString.emplace_back("pauseSimulation");
        helpString.emplace_back("continueSimulation");
        helpString.emplace_back("stepSimulation");
        helpString.emplace_back("stepSimulationAndWait");
        helpString.emplace_back("resetSimulationTime");
        helpString.emplace_back("getSimulationTime");
        helpString.emplace_back("getStepSize");
        helpString.emplace_back("resetSimulation");
        helpString.emplace_back("resetSimulationState");
        helpString.emplace_back("help");
    } else {
        if (functionName == "pauseSimulation") {
            helpString.emplace_back("void pauseSimulation() ");
            helpString.emplace_back("Pause the simulation if it was running ");
        }
        if (functionName == "continueSimulation") {
            helpString.emplace_back("void continueSimulation() ");
            helpString.emplace_back("Resume the simulation if it was paused ");
        }
        if (functionName == "stepSimulation") {
            helpString.emplace_back("void stepSimulation(const std::int32_t numberOfSteps = 1) ");
            helpString.emplace_back("Steps the simulation for the provided number of steps. ");
            helpString.emplace_back("The input paramter is the number of steps, not the time (Usually 1 step = 1ms but this is not guaranteed) ");
            helpString.emplace_back("@note: this function (will be) not blocking, i.e. it will return immediately. Currently calling this function ");
            helpString.emplace_back("twice before the previous call actually ends its computation gives and undefined behavior. ");
            helpString.emplace_back("@param numberOfSteps number of steps to simulate ");
        }
        if (functionName == "stepSimulationAndWait") {
            helpString.emplace_back("void stepSimulationAndWait(const std::int32_t numberOfSteps = 1) ");
            helpString.emplace_back("Steps the simulation for the provided number of steps. ");
            helpString.emplace_back("The input paramter is the number of steps, not the time (Usually 1 step = 1ms but this is not guaranteed) ");
            helpString.emplace_back("@note: this function is blocking ");
            helpString.emplace_back("@param numberOfSteps number of steps to simulate ");
        }
        if (functionName == "resetSimulationTime") {
            helpString.emplace_back("void resetSimulationTime() ");
            helpString.emplace_back("Reset the simulation time back to zero ");
        }
        if (functionName == "getSimulationTime") {
            helpString.emplace_back("double getSimulationTime() ");
            helpString.emplace_back("Get the current simulation time ");
            helpString.emplace_back("@return the simulation time. ");
        }
        if (functionName == "getStepSize") {
            helpString.emplace_back("double getStepSize() ");
            helpString.emplace_back("Get the current step size in seconds. ");
            helpString.emplace_back("@return the step size in seconds ");
        }
        if (functionName == "resetSimulation") {
            helpString.emplace_back("void resetSimulation() ");
            helpString.emplace_back("Reset the simulation state and time ");
        }
        if (functionName == "resetSimulationState") {
            helpString.emplace_back("void resetSimulationState() ");
            helpString.emplace_back("Reset the simulation state back to initial pose ");
        }
        if (functionName == "help") {
            helpString.emplace_back("std::vector<std::string> help(const std::string& functionName = \"--all\")");
            helpString.emplace_back("Return list of available commands, or help message for a specific function");
            helpString.emplace_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
            helpString.emplace_back("@return list of strings (one string per line)");
        }
    }
    if (helpString.empty()) {
        helpString.emplace_back("Command not found");
    }
    return helpString;
}

// read from ConnectionReader
bool ClockServer::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::idl::WireReader reader(connection);
    reader.expectAccept();
    if (!reader.readListHeader()) {
        reader.fail();
        return false;
    }

    std::string tag = reader.readTag();
    bool direct = (tag == "__direct__");
    if (direct) {
        tag = reader.readTag();
    }
    while (!reader.isError()) {
        if (tag == "pauseSimulation") {
            if (!direct) {
                ClockServer_pauseSimulation helper;
                helper.init();
                yarp().callback(helper,*this, "__direct__");
            } else {
                pauseSimulation();
            }
            yarp::os::idl::WireWriter writer(reader);
            if (!writer.isNull()) {
                if (!writer.writeOnewayResponse()) {
                    return false;
                }
            }
            reader.accept();
            return true;
        }
        if (tag == "continueSimulation") {
            if (!direct) {
                ClockServer_continueSimulation helper;
                helper.init();
                yarp().callback(helper,*this, "__direct__");
            } else {
                continueSimulation();
            }
            yarp::os::idl::WireWriter writer(reader);
            if (!writer.isNull()) {
                if (!writer.writeOnewayResponse()) {
                    return false;
                }
            }
            reader.accept();
            return true;
        }
        if (tag == "stepSimulation") {
            std::int32_t numberOfSteps;
            if (!reader.readI32(numberOfSteps)) {
                numberOfSteps = 1;
            }
            if (!direct) {
                ClockServer_stepSimulation helper;
                helper.init(numberOfSteps);
                yarp().callback(helper,*this, "__direct__");
            } else {
                stepSimulation(numberOfSteps);
            }
            yarp::os::idl::WireWriter writer(reader);
            if (!writer.isNull()) {
                if (!writer.writeOnewayResponse()) {
                    return false;
                }
            }
            reader.accept();
            return true;
        }
        if (tag == "stepSimulationAndWait") {
            std::int32_t numberOfSteps;
            if (!reader.readI32(numberOfSteps)) {
                numberOfSteps = 1;
            }
            stepSimulationAndWait(numberOfSteps);
            yarp::os::idl::WireWriter writer(reader);
            if (!writer.isNull()) {
                if (!writer.writeListHeader(0)) {
                    return false;
                }
            }
            reader.accept();
            return true;
        }
        if (tag == "resetSimulationTime") {
            if (!direct) {
                ClockServer_resetSimulationTime helper;
                helper.init();
                yarp().callback(helper,*this, "__direct__");
            } else {
                resetSimulationTime();
            }
            yarp::os::idl::WireWriter writer(reader);
            if (!writer.isNull()) {
                if (!writer.writeOnewayResponse()) {
                    return false;
                }
            }
            reader.accept();
            return true;
        }
        if (tag == "getSimulationTime") {
            double _return;
            _return = getSimulationTime();
            yarp::os::idl::WireWriter writer(reader);
            if (!writer.isNull()) {
                if (!writer.writeListHeader(1)) {
                    return false;
                }
                if (!writer.writeFloat64(_return)) {
                    return false;
                }
            }
            reader.accept();
            return true;
        }
        if (tag == "getStepSize") {
            double _return;
            _return = getStepSize();
            yarp::os::idl::WireWriter writer(reader);
            if (!writer.isNull()) {
                if (!writer.writeListHeader(1)) {
                    return false;
                }
                if (!writer.writeFloat64(_return)) {
                    return false;
                }
            }
            reader.accept();
            return true;
        }
        if (tag == "resetSimulation") {
            if (!direct) {
                ClockServer_resetSimulation helper;
                helper.init();
                yarp().callback(helper,*this, "__direct__");
            } else {
                resetSimulation();
            }
            yarp::os::idl::WireWriter writer(reader);
            if (!writer.isNull()) {
                if (!writer.writeOnewayResponse()) {
                    return false;
                }
            }
            reader.accept();
            return true;
        }
        if (tag == "resetSimulationState") {
            if (!direct) {
                ClockServer_resetSimulationState helper;
                helper.init();
                yarp().callback(helper,*this, "__direct__");
            } else {
                resetSimulationState();
            }
            yarp::os::idl::WireWriter writer(reader);
            if (!writer.isNull()) {
                if (!writer.writeOnewayResponse()) {
                    return false;
                }
            }
            reader.accept();
            return true;
        }
        if (tag == "help") {
            std::string functionName;
            if (!reader.readString(functionName)) {
                functionName = "--all";
            }
            auto _return = help(functionName);
            yarp::os::idl::WireWriter writer(reader);
            if (!writer.isNull()) {
                if (!writer.writeListHeader(2)) {
                    return false;
                }
                if (!writer.writeTag("many", 1, 0)) {
                    return false;
                }
                if (!writer.writeListBegin(BOTTLE_TAG_INT32, static_cast<uint32_t>(_return.size()))) {
                    return false;
                }
                for (const auto& _ret : _return) {
                    if (!writer.writeString(_ret)) {
                        return false;
                    }
                }
                if (!writer.writeListEnd()) {
                    return false;
                }
            }
            reader.accept();
            return true;
        }
        if (reader.noMore()) {
            reader.fail();
            return false;
        }
        std::string next_tag = reader.readTag();
        if (next_tag == "") {
            break;
        }
        tag.append("_").append(next_tag);
    }
    return false;
}

} // namespace GazeboYarpPlugins
