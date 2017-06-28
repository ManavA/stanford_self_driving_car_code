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

find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})
rosbuild_add_library(bil src/bil.cpp)
rosbuild_add_library(imagery src/imagery.cpp src/imagery_proj.cpp src/imageryServer.cpp src/textureCache.cpp)
target_link_libraries(imagery bil)
target_link_libraries(imagery ${FREETYPE_LIBARIES})
