cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

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
rosbuild_add_boost_directories()

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
qt4_wrap_ui(QT_UI_FILES src/paw2_qtgui.ui)
qt4_wrap_cpp(QT_MOC_FILES include/paw2GlView.h include/paw2_gui.h include/rdConsole.h include/paw2App.h)
qt4_add_resources(QT_QRC_FILES src/tbimages.qrc)

  # help..how to include qwt?!?
include_directories(/usr/include/qwt-qt4)

rosbuild_add_executable(${PROJECT_NAME} src/3dm-Henry.cpp src/conout.cpp src/graphics.cpp src/paw2.cpp)
rosbuild_add_executable(${PROJECT_NAME} src/paw2Velodyne.cpp src/tbimages.cpp src/baseDemo.cpp src/drawSensorData.cpp)
rosbuild_add_executable(${PROJECT_NAME} src/paw2GlView.cpp src/bufferedStringBuf.cpp)
rosbuild_add_executable(${PROJECT_NAME} src/paw2_gui.cpp src/rdConsole.cpp src/circle_demo.cpp src/fakeObstacleTracker.cpp)
rosbuild_add_executable(${PROJECT_NAME} src/paw2Camera.cpp src/paw2InternalData.cpp src/static_map_demo.cpp)
rosbuild_add_executable(${PROJECT_NAME} ${QT_UI_FILES} ${QT_MOC_FILES})

rosbuild_link_boost(${PROJECT_NAME} thread system)
target_link_libraries(${PROJECT_NAME} GL boost_system)
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} qwt-qt4)
target_link_libraries(${PROJECT_NAME} ${FREETYPE_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${FTGL_LIBRARIES})

#rosbuild_add_executable(makeCircleRNDF src/makeCircleRNDF.cpp)
#rosbuild_add_executable(labeled_car src/labeled_car.cpp)
#rosbuild_add_executable(fake_car src/fake_car.cpp)



