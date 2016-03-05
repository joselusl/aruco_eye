cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME aruco_eye_core)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11)
add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

 


#ARUCO EYE
set(ARUCO_EYE_CORE_SOURCE_DIR
	src/source) 
	
set(ARUCO_EYE_CORE_INCLUDE_DIR
	src/include)

set(ARUCO_EYE_CORE_SOURCE_FILES
        ${ARUCO_EYE_CORE_SOURCE_DIR}/arucoEye.cpp
)
 
set(ARUCO_EYE_CORE_HEADER_FILES
        ${ARUCO_EYE_CORE_INCLUDE_DIR}/aruco_eye_core/arucoEye.h
)




find_package(catkin REQUIRED
		COMPONENTS pugixml aruco_lib)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)


catkin_package(
        INCLUDE_DIRS ${ARUCO_EYE_CORE_INCLUDE_DIR}
        LIBRARIES ${PROJECT_NAME}
        DEPENDS OpenCV Boost
        CATKIN_DEPENDS pugixml aruco_lib
  )


include_directories(${ARUCO_EYE_CORE_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME} ${ARUCO_EYE_CORE_SOURCE_FILES} ${ARUCO_EYE_CORE_HEADER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
