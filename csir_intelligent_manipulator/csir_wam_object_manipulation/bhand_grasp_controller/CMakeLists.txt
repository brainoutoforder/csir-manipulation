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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
rosbuild_add_executable(bhand_grasp_controller src/bhand_grasp_controller.cpp src/bhand_grasp_controller_main.cpp)

# Python test
rosbuild_add_pyunit(test/utest_bhand_grasp_controller.py)

# Google test
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
##rosbuild_add_gtest(utest_bhand_grasp_controller test/utest_bhand_grasp_controller.cpp src/bhand_grasp_controller.cpp)

# ROS tests
# C++ unit tests
#rosbuild_add_executable(utest_bhand_grasp_controller EXCLUDE_FROM_ALL test/utest_bhand_grasp_controller.cpp src/bhand_grasp_controller.cpp)
#rosbuild_add_gtest_build_flags(utest_bhand_grasp_controller)
#rosbuild_add_rostest(test/utest_bhand_grasp_controller.test)
# C++ node tests
rosbuild_add_executable(ntest_bhand_grasp_controller EXCLUDE_FROM_ALL test/ntest_bhand_grasp_controller.cpp src/bhand_grasp_controller.cpp)
rosbuild_add_gtest_build_flags(ntest_bhand_grasp_controller)
rosbuild_add_rostest(test/cpp_bhand_grasp_controller.test)
# Python node tests
rosbuild_add_rostest(test/python_bhand_grasp_controller.test)

