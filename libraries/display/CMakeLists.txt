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

  # ipp
include_directories($ENV{IPPROOT}/include)
link_directories($ENV{IPPROOT}/sharedlib)

  # qt4
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGl REQUIRED)
include(${QT_USE_FILE})
qt4_wrap_cpp(QT_MOC_FILES include/display.h include/glWidget.h include/displayGL.h)

rosbuild_add_library(${PROJECT_NAME} src/display.cpp src/displayGL_2d.cpp src/displayGL.cpp)
rosbuild_add_library(${PROJECT_NAME} src/displayGL_1d.cpp src/displayGL_3d.cpp src/glWidget.cpp)
rosbuild_add_library(${PROJECT_NAME} ${QT_MOC_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${FREETYPE_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${FTGL_LIBRARIES})


