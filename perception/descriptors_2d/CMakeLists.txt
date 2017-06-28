cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_download_test_data(http://pr.willowgarage.com/data/descriptors_2d/frame0000.jpg test/frame0000.jpg 6788a6d2e153de3e8bf06a4d831a5957)
rosbuild_download_test_data(http://pr.willowgarage.com/data/descriptors_2d/correct-output/hog.results test/correct-output/hog.results e36fb69799513979eb3c411a7c78117d)
rosbuild_download_test_data(http://pr.willowgarage.com/data/descriptors_2d/correct-output/surf.results test/correct-output/surf.results 72eb220ee99f467da5bffe0c9b91874c)
rosbuild_download_test_data(http://pr.willowgarage.com/data/descriptors_2d/correct-output/sch.results test/correct-output/sch.results b25f4fdb7bdfa9af7b7cb3c37532b86f)

rosbuild_add_library(descriptors_2d src/descriptors_2d.cpp)
rosbuild_add_executable(example src/example.cpp)
target_link_libraries(example descriptors_2d)
rosbuild_add_executable(hist_test src/hist_test.cpp)
rosbuild_add_executable(surf_test src/surf_test.cpp)

# This unit test fails on 64-bit, #2141
include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
   CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
   CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
# This test is also failing on 32-bit, #3292
#rosbuild_add_gtest(test/test_descriptors_2d test/test_descriptors_2d.cpp)
#rosbuild_add_gtest_build_flags(test/test_descriptors_2d)
#target_link_libraries(test/test_descriptors_2d descriptors_2d)
endif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "i386" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "unknown")
