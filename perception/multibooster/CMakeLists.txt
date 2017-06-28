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

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


rosbuild_add_boost_directories()


rosbuild_add_library(name_mapping name_mapping.cpp)
rosbuild_add_library(multibooster multibooster.cpp multibooster_dataset.cpp weak_classifier_tree.cpp)
target_link_libraries(multibooster name_mapping)	

rosbuild_add_executable(multitool multitool.cpp)
target_link_libraries(multitool multibooster name_mapping)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin/test)

rosbuild_add_gtest(test_name_mapping test/test_name_mapping.cpp)
rosbuild_add_gtest_build_flags(test_name_mapping)
target_link_libraries(test_name_mapping name_mapping)

rosbuild_add_gtest(test_multibooster_dataset test/test_multibooster_dataset.cpp)
rosbuild_add_gtest_build_flags(test_multibooster_dataset)
target_link_libraries(test_multibooster_dataset multibooster name_mapping)

rosbuild_add_gtest(test_multibooster test/test_multibooster.cpp)
rosbuild_add_gtest_build_flags(test_multibooster)
target_link_libraries(test_multibooster multibooster name_mapping)

rosbuild_add_executable(speed_test test/speed_test.cpp)
target_link_libraries(speed_test multibooster name_mapping)	

#rosbuild_add_link_flags(multibooster "-lprofiler")
