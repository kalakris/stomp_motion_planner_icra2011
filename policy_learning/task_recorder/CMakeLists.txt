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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_library(${PROJECT_NAME}
	src/joint_states_recorder.cpp
	src/accelerometer_states_recorder.cpp
	src/tactile_states_recorder.cpp
	src/point_cloud_recorder.cpp
	src/imu_states_recorder.cpp
	src/accumulator.cpp
)

rosbuild_add_executable(task_recorder_node
	src/task_recorder_node.cpp
)
target_link_libraries(task_recorder_node ${PROJECT_NAME})

rosbuild_add_executable(point_cloud_converter_node
	src/point_cloud_converter_node.cpp
)

rosbuild_add_executable(imu_converter_node
	src/imu_converter_node.cpp
)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
