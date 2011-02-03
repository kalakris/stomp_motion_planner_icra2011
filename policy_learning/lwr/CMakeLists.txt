cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

add_definitions(-DNDEBUG)

rosbuild_add_library(lwr
  src/lwr.cpp
  src/parameters.cpp)

rosbuild_add_executable(lwr_regression_test test/lwr_test.cpp)
rosbuild_declare_test(lwr_regression_test)
target_link_libraries(lwr_regression_test gtest)
target_link_libraries(lwr_regression_test lwr)
rosbuild_add_rostest(launch/lwr_test.test)
