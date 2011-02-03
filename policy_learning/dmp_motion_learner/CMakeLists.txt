PROJECT(mrinalrulesi)

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
rosbuild_gensrv()

rosbuild_add_executable(dmp_motion_learner_node
	src/dmp_motion_learner_node.cpp
	src/dmp_motion_learner.cpp
)
target_link_libraries(dmp_motion_learner_node dmp)

rosbuild_add_executable(dmp_learner_regression_test test/dmp_motion_learner_test.cpp)
#rosbuild_declare_test(dmp_learner_regression_test)
target_link_libraries(dmp_learner_regression_test gtest)
target_link_libraries(dmp_learner_regression_test dmp)
#rosbuild_add_rostest(launch/dmp_motion_learner_test.test)
