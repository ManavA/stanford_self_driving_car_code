cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-DEIGEN_NO_DEBUG)
rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} 
			src/descriptor_3d.cpp
			src/generic/neighborhood_feature.cpp
			src/bounding_box_raw.cpp
			src/shared/spectral_analysis.cpp
			src/bounding_box_spectral.cpp
			src/shape_spectral.cpp
			src/generic/orientation_generic.cpp
			src/orientation_normal.cpp
			src/orientation_tangent.cpp
			src/position.cpp
			src/generic/spin_image_generic.cpp
			src/spin_image_normal.cpp
			src/spin_image_tangent.cpp
			src/spin_image_custom.cpp
			src/curvature.cpp
			src/channel.cpp)
#rosbuild_add_openmp_flags (${PROJECT_NAME})
#Does not compile with OpenMP on gcc-4.3			

rosbuild_add_executable(interest_point_example examples/interest_point_example.cpp)
target_link_libraries(interest_point_example ${PROJECT_NAME})

