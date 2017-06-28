cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "-std=c++0x")

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Old DGC make system include and lib directories.
# @TODO: Remove once fully migrated to ROS
include_directories(../../../include)
link_directories(../../../lib)

#common commands for building c++ executables and libraries
rosbuild_add_library(localize_terrainmap src/terrainmap.cpp)
target_link_libraries(localize_terrainmap lltransform)

rosbuild_add_executable(localize_velodyne src/localizeVelodyne.cpp src/veloClient.cpp src/laserMap.cpp)
target_link_libraries(localize_velodyne localize_terrainmap)
target_link_libraries(localize_velodyne z)

rosbuild_add_executable(fake_localize src/fake_localize.cpp)
#rosbuild_add_executable(map_velodyne src/map_velodyne.cpp)
#target_link_libraries(map_velodyne localize_terrainmap)

#rosbuild_add_executable(generate_rndf src/generate_rndf.cpp)
#target_link_libraries(generate_rndf localize_terrainmap)
#rosbuild_add_executable(velo_calibrate src/velo_calibrate.cpp)
#target_link_libraries(velo_calibrate lltransform)
#rosbuild_add_executable(test_localize_accuracy src/test_localize_accuracy.cpp)
#target_link_libraries(test_localize_accuracy lltransform)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
