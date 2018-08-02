# Copyright (C) 2007-2018 Istituto Italiano di Tecnologia ADVR & iCub Facility & RBCS Department
# Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi, Francesco Romano and Nuno Guedelha
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)

set(GAZEBOYARP_PLUGIN_MANIFESTS_INSTALL_DIR,"share/yarp/plugins")

PROJECT(Plugin_EmbObjInertials)

include(AddGazeboYarpPluginTarget)

yarp_prepare_plugin(embObjInertials CATEGORY device
                    TYPE yarp::dev::GazeboYarpEmbObjInertialsDriver
                    INCLUDE embObjInertialsDriver.h)

set(LIB_COMMON_NAME gazebo_yarp_lib_common)
if(CMAKE_VERSION VERSION_LESS 3.0.0)
    get_property(GAZEBO_YARP_COMMON_HEADERS GLOBAL PROPERTY GAZEBO_YARP_COMMON_HEADERS)
    unset(LIB_COMMON_NAME)
endif()

add_gazebo_yarp_plugin_target(LIBRARY_NAME embObjInertials
                              INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
                              INCLUDE_DIRS include/gazebo include/yarp/dev
                              SYSTEM_INCLUDE_DIRS ${GAZEBO_YARP_COMMON_HEADERS} ${Boost_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${SDFORMAT_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS}
                              LINKED_LIBRARIES ${LIB_COMMON_NAME} gazebo_yarp_singleton ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES}
                              HEADERS include/yarp/dev/embObjInertialsDriver.h
                              SOURCES src/embObjInertialsDriver.cpp
                                      src/yarp_plugin_embObjInertials.cpp
                                 )

yarp_install(FILES embObjInertials.ini  DESTINATION /Users/nunoguedelha/dev/robotology-superbuild/build/install/share/yarp/plugins)
