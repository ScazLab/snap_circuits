# Copyright: (C) 2016 Social Robotics Lab - Yale University
# Authors: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(snapCircuits)

find_package(OpenCV REQUIRED)

## Specify the source files
set(folder_source_snap_circuits src/utils.cpp
                                src/snapCircuitsPart.cpp
                                src/snapCircuitsBoard.cpp)

## Specify the header files
set(folder_header_snap_circuits include/snapCircuits/utils.h
                                include/snapCircuits/snapCircuitsPart.h
                                include/snapCircuits/snapCircuitsBoard.h)

set(folder_header_nanosvg   include/nanosvg/nanosvg.h
                            include/nanosvg/nanosvgrast.h)

include_directories(${PROJECT_SOURCE_DIR}/include)

source_group("Source Files" FILES ${folder_source_snap_circuits})
source_group("Header Files" FILES ${folder_header_snap_circuits} ${folder_header_nanosvg})

add_definitions(${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES)

## Declare a C++ library
add_library(HEADER_ONLY_TARGET ${folder_header_nanosvg})
set_target_properties(HEADER_ONLY_TARGET PROPERTIES LINKER_LANGUAGE CXX)

add_library(${PROJECT_NAME} ${folder_source_snap_circuits} ${folder_header_snap_circuits})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(snap_circuits ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install( FILES ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
