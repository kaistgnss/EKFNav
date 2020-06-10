cmake_minimum_required (VERSION 2.6)

project (EKFNav)

#######################################
## Check for ROS ##
#######################################

option(BUILD_WITH_ROS "Build ROS node and use catkin." ON)

if (NOT catkin_FOUND)
   SET(BUILD_WITH_ROS OFF)
endif(NOT catkin_FOUND)

if (BUILD_WITH_ROS)
  ## Find catkin macros and libraries
	## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	## is used, also find other catkin packages
	## libraries found here are added to catkin_LIBRARIES and linked automatically
	find_package(catkin COMPONENTS serial roslib roscpp rosconsole tf gps_msgs nav_msgs sensor_msgs)
	
  ## LIBRARIES: libraries you create in this project that dependent projects also need
	## CATKIN_DEPENDS: catkin_packages dependent projects also need
	## DEPENDS: system dependencies of this project that dependent projects also need
	catkin_package(
	  INCLUDE_DIRS include
	  LIBRARIES novatel
	  CATKIN_DEPENDS serial roslib roscpp rosconsole tf gps_msgs nav_msgs sensor_msgs
	  DEPENDS Boost
	)
else()
	SET(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")
	SET(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_PREFIX}/bin")
	SET(CATKIN_PACKAGE_INCLUDE_DESTINATION "${CMAKE_INSTALL_PREFIX}/include")
	# TODO: need to find serial library here
	find_package(serial REQUIRED)
	set(catkin_INCLUDE_DIRS ${serial_INCLUDE_DIRS})
	set(catkin_LIBRARIES ${serial_LIBRARIES})
endif (BUILD_WITH_ROS)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


if (CMAKE_BUILD_TYPE MATCHES Debug)
  set(LIB_NAME novatel_d)
  MESSAGE("Building DEBUG library ${LIB_NAME}")
else ()
  set(LIB_NAME novatel)
  MESSAGE("Building RELEASE library ${LIB_NAME}")
endif ()

# Declare a cpp library

add_library(${LIB_NAME}
  src/novatel.cpp	src/signa_core.cpp	src/matops.c
  src/HgDataParser.cpp	src/Serial.cpp       
)


add_executable(EKFNav src/main.cpp)
target_link_libraries(EKFNav
               ${LIB_NAME}
               ${catkin_LIBRARIES} 
               ${Boost_LIBRARIES})
