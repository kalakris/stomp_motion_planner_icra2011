cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

add_definitions(-msse -msse2)
add_definitions(-DEIGEN_NO_DEBUG)
add_definitions(-DNDEBUG)

rosbuild_add_library(chomp
	src/chomp_collision_point.cpp
	src/chomp_collision_space.cpp
	src/chomp_cost.cpp
	src/chomp_optimizer.cpp
	src/chomp_parameters.cpp
	src/chomp_planner_node.cpp
	src/chomp_robot_model.cpp
	src/chomp_trajectory.cpp
	src/treefksolverjointposaxis.cpp
	src/treefksolverjointposaxis_partial.cpp
)	

rosbuild_add_executable(chomp_planner_node
	src/chomp_planner_node.cpp
)
target_link_libraries(chomp_planner_node chomp)

#rosbuild_add_executable(chomp_cost_server
#	src/chomp_cost_server.cpp
#)
#target_link_libraries(chomp_cost_server chomp)
