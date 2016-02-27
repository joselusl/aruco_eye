cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME aruco_eye_ros)
project(${PROJECT_NAME})

add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

 


set(ARUCO_ROS_SOURCE_DIR
	src/source) 
	
set(ARUCO_ROS_INCLUDE_DIR
	src/include)

set(ARUCO_ROS_SOURCE_FILES
	${ARUCO_ROS_SOURCE_DIR}/aruco_eye_ros_detector.cpp 
  ${ARUCO_ROS_SOURCE_DIR}/aruco_eye_ros_display.cpp 
)
 
set(ARUCO_ROS_HEADER_FILES
	${ARUCO_ROS_INCLUDE_DIR}/aruco_eye_ros_detector.h
	${ARUCO_ROS_INCLUDE_DIR}/aruco_eye_ros_display.h
)




find_package(catkin REQUIRED
                COMPONENTS roscpp message_filters image_transport cv_bridge geometry_msgs tf aruco_eye_msgs aruco_eye_srvs aruco_eye_core)

find_package(OpenCV REQUIRED)


catkin_package(
    DEPENDS OpenCV
    CATKIN_DEPENDS roscpp message_filters image_transport cv_bridge geometry_msgs tf aruco_eye_msgs aruco_eye_srvs aruco_eye_core
  )


include_directories(${ARUCO_ROS_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME}_lib ${ARUCO_ROS_SOURCE_FILES} ${ARUCO_ROS_HEADER_FILES})
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME} ${ARUCO_ROS_SOURCE_DIR}/aruco_eye_ros_detector_node.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME}_display ${ARUCO_ROS_SOURCE_DIR}/aruco_eye_ros_display_node.cpp)
add_dependencies(${PROJECT_NAME}_display ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_display ${PROJECT_NAME}_lib)
target_link_libraries(${PROJECT_NAME}_display ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_display ${catkin_LIBRARIES})
