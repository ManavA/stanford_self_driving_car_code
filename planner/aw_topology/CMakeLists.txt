cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#set(ROS_BUILD_TYPE Debug)

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
pkg_check_modules(FTGL REQUIRED ftgl)
find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})
link_directories(${FTGL_LIBRARY_DIRS})

rosbuild_add_library(${PROJECT_NAME} src/aw_BlockadeManager.cpp src/aw_graph_tools.cpp src/aw_RndfGraphBuilder.cpp)
rosbuild_add_library(${PROJECT_NAME} src/aw_RndfIntersection.cpp src/aw_Topology.cpp)
rosbuild_add_library(${PROJECT_NAME} src/aw_match_to_graph.cpp src/aw_RndfGraph.cpp src/aw_RndfVertex.cpp)
rosbuild_add_library(${PROJECT_NAME} src/aw_Vehicle.cpp src/aw_Graph.cpp src/aw_RndfEdge.cpp src/aw_RndfGraphSearch.cpp)
rosbuild_add_library(${PROJECT_NAME} src/aw_Route.cpp src/aw_VehicleManager.cpp)

target_link_libraries(${PROJECT_NAME} ${FREETYPE_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${FTGL_LIBRARIES})

