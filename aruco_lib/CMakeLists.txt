cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME aruco_lib)
project(${PROJECT_NAME})

### Use version 2011 of C++ (c++11)
add_definitions(-std=c++11)


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




#STANDARD ARUCO LIB
set(LIB_ARUCO_SOURCE_DIR
	src/source) 
	
set(LIB_ARUCO_INCLUDE_DIR
	src/include)

set(LIB_ARUCO_SOURCE_FILES
        ${LIB_ARUCO_SOURCE_DIR}/ar_omp
        ${LIB_ARUCO_SOURCE_DIR}/arucofidmarkers.cpp
        ${LIB_ARUCO_SOURCE_DIR}/board.cpp
        ${LIB_ARUCO_SOURCE_DIR}/boarddetector.cpp
        ${LIB_ARUCO_SOURCE_DIR}/cameraparameters.cpp
        ${LIB_ARUCO_SOURCE_DIR}/chromaticmask
        ${LIB_ARUCO_SOURCE_DIR}/cvdrawingutils.cpp
        ${LIB_ARUCO_SOURCE_DIR}/highlyreliablemarkers
        ${LIB_ARUCO_SOURCE_DIR}/marker.cpp
        ${LIB_ARUCO_SOURCE_DIR}/markerdetector.cpp
        ${LIB_ARUCO_SOURCE_DIR}/subpixelcorner
)
 
set(LIB_ARUCO_HEADER_FILES
        ${LIB_ARUCO_INCLUDE_DIR}/ar_omp
        ${LIB_ARUCO_INCLUDE_DIR}/aruco.h
        ${LIB_ARUCO_INCLUDE_DIR}/arucofidmarkers.h
        ${LIB_ARUCO_INCLUDE_DIR}/board.h
        ${LIB_ARUCO_INCLUDE_DIR}/boarddetector.h
        ${LIB_ARUCO_INCLUDE_DIR}/cameraparameters.h
        ${LIB_ARUCO_INCLUDE_DIR}/chromaticmask
        ${LIB_ARUCO_INCLUDE_DIR}/cvdrawingutils.h
        ${LIB_ARUCO_INCLUDE_DIR}/exports.h
        ${LIB_ARUCO_INCLUDE_DIR}/highlyreliablemarkers
        ${LIB_ARUCO_INCLUDE_DIR}/marker.h
        ${LIB_ARUCO_INCLUDE_DIR}/markerdetector.h
        ${LIB_ARUCO_INCLUDE_DIR}/subpixelcorner
)


find_package(catkin REQUIRED)

#opencv
find_package(OpenCV REQUIRED)


catkin_package(
	INCLUDE_DIRS ${LIB_ARUCO_INCLUDE_DIR}
        LIBRARIES ${PROJECT_NAME}
	DEPENDS OpenCV
  )


include_directories(${LIB_ARUCO_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME} ${LIB_ARUCO_SOURCE_FILES} ${LIB_ARUCO_HEADER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

