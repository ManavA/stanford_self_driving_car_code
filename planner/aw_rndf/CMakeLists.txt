cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++0x")

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

  # ftgl + freetype
pkg_check_modules(FTGL REQUIRED ftgl)
find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})
link_directories(${FTGL_LIBRARY_DIRS})

rosbuild_add_library(aw_rndf src/aw_CheckpointList.cpp src/aw_Mission.cpp src/aw_SpeedLimitList.cpp src/aw_checkPoint.cpp src/aw_exit.cpp src/aw_lane.cpp src/aw_laneSegment.cpp src/aw_netElement.cpp src/aw_perimeter.cpp src/aw_perimeterPoint.cpp src/aw_roadNetwork.cpp src/aw_segment.cpp src/aw_spot.cpp src/aw_stop.cpp src/aw_SpeedLimit.cpp src/aw_StringTools.cpp src/aw_wayPoint.cpp src/aw_zone.cpp src/aw_laneQuadTree.cpp src/aw_roadNetworkSearch.cpp src/aw_intersection.cpp src/aw_trafficLight.cpp src/aw_crosswalk.cpp)
rosbuild_add_library(aw_rndf src/aw_roadNetworkGL.cpp src/aw_trafficLightGL.cpp)
