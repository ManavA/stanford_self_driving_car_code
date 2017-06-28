cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

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

set(SOURCES
	src/aw_ChsmPlanner.cpp
	src/aw_CrosswalkManager.cpp
	src/aw_IntersectionManager.cpp
	src/aw_LaneChangeManager.cpp
	src/aw_MergeFeasabilityCheck.cpp
	src/aw_PassObstacleManager.cpp
	src/aw_RouteSampler.cpp
#	src/aw_SituationInterpretation.cpp
	src/aw_StActive.cpp
	src/aw_StBase.cpp
	src/aw_StCrosswalk.cpp
	src/aw_StDrive.cpp
	src/aw_StDriveKTurn.cpp
	src/aw_StDrivePassObstacle.cpp
	src/aw_StError.cpp
	src/aw_StIntersection.cpp
	src/aw_StIntersectionTrafficLightQueue.cpp
	src/aw_StIntersectionTrafficLightStop.cpp
	src/aw_StIntersectionTrafficLightWait.cpp
	src/aw_StLaneChange.cpp
	src/aw_StPause.cpp
	src/aw_StReplan.cpp
	src/aw_StStop.cpp
	src/aw_StTrafficLight.cpp
	src/aw_StWaitForActivation.cpp
	src/aw_StZone.cpp
	src/aw_TrafficLightManager.cpp
	src/collisionCheck.cpp
	src/obstacleUpdates.cpp
	src/trajectoryEvaluator.cpp
)

rosbuild_add_library(aw_chsm_planning ${SOURCES} )


target_link_libraries(aw_chsm_planning ${FREETYPE_LIBRARIES} ${FTGL_LIBRARIES})


