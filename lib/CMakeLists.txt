# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(libSnapCircuits)

find_package(OpenCV REQUIRED)

## Specify the source files
set(folder_source src/utils.cpp
                  src/snapCircuitsPart.cpp
                  src/snapCircuitsBoard.cpp)

## Specify the header files
set(folder_header include/snapCircuits/utils.h
                  include/snapCircuits/nanosvg.h
                  include/snapCircuits/snapCircuitsPart.h
                  include/snapCircuits/snapCircuitsBoard.h)

include_directories(${PROJECT_SOURCE_DIR}/include)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_definitions(${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES)

## Declare a C++ library
add_library(${PROJECT_NAME} ${folder_source} ${folder_header})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(snap_circuits ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
