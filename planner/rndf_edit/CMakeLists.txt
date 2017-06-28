cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

  # ftgl + freetype
pkg_check_modules(FTGL REQUIRED ftgl)
find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})
link_directories(${FTGL_LIBRARY_DIRS})

  # qt4
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGl REQUIRED)
include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})  # for uic generated files
qt4_wrap_ui(QT_UI_FILES src/rndf_edit_qtgui.ui)
qt4_wrap_cpp(QT_MOC_FILES include/RNDFEditGLView.h include/REFindElement.h include/rndf_edit_gui.h include/rndf_edit_app.h include/REConsole.h include/RECrosswalk.h include/RELane.h include/REPerimeter.h include/REPerimeterPoint.h include/RESegment.h include/RESpot.h include/RESpotPoint.h include/RETrafficLight.h include/REWayPoint.h include/REZone.h include/glwidget.h)
qt4_add_resources(QT_QRC_FILES src/tbimages.qrc)

  # help..how to include qwt?!?
include_directories(/usr/include/qwt-qt4)

rosbuild_add_executable(${PROJECT_NAME} src/conout.cpp src/RECrosswalk.cpp src/REPerimeterPoint.cpp src/RESpotPoint.cpp src/REZone.cpp)
rosbuild_add_executable(${PROJECT_NAME} src/rndf_edit_gui.cpp src/glwidget.cpp src/RELane.cpp src/RESegment.cpp src/RETrafficLight.cpp)
rosbuild_add_executable(${PROJECT_NAME} src/rndf_edit.cpp src/REConsole.cpp src/REPerimeter.cpp src/RESpot.cpp)
rosbuild_add_executable(${PROJECT_NAME} src/REWayPoint.cpp src/RNDFEditGLView.cpp src/REFindElement.cpp)

rosbuild_add_executable(${PROJECT_NAME} ${QT_UI_FILES} ${QT_MOC_FILES} ${QT_QRC_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} qwt-qt4)
target_link_libraries(${PROJECT_NAME} ${FREETYPE_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${FTGL_LIBRARIES})


