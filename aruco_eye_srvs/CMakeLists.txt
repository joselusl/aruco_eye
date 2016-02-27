cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME aruco_eye_srvs)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11)
add_definitions(-std=c++11)



# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)




find_package(catkin REQUIRED
                COMPONENTS message_generation std_msgs)



add_service_files(
    DIRECTORY
        srv
    FILES
        SetBool.srv



)



generate_messages(DEPENDENCIES std_msgs)




catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
	)



include_directories(${catkin_INCLUDE_DIRS})

