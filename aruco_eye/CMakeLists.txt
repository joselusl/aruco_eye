cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME aruco_eye)
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
set(ROSARUCOLIB_SOURCE_DIR
	src/source) 
	
set(ROSARUCOLIB_INCLUDE_DIR
	src/include)

set(ROSARUCOLIB_SOURCE_FILES
	${ROSARUCOLIB_SOURCE_DIR}/arucoEye.cpp
)
 
set(ROSARUCOLIB_HEADER_FILES
	${ROSARUCOLIB_INCLUDE_DIR}/arucoEye.h
)

 


#ARUCO EYE TEST
set(ROSARUCOLIBTEST_SOURCE_DIR
	src/source) 
	
set(ROSARUCOLIBTEST_INCLUDE_DIR
	src/include)

set(ROSARUCOLIBTEST_SOURCE_FILES
	${ROSARUCOLIBTEST_SOURCE_DIR}/arucoEye_test.cpp
)
 
set(ROSARUCOLIBTEST_HEADER_FILES

)



find_package(catkin REQUIRED
		COMPONENTS lib_pugixml aruco_lib)

find_package(OpenCV REQUIRED)



catkin_package(
        INCLUDE_DIRS ${ROSARUCOLIB_INCLUDE_DIR}
        LIBRARIES ${PROJECT_NAME}
        DEPENDS OpenCV
        CATKIN_DEPENDS lib_pugixml aruco_lib
  )


include_directories(${ROSARUCOLIB_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})



add_library(${PROJECT_NAME} ${ROSARUCOLIB_SOURCE_FILES} ${ROSARUCOLIB_HEADER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME}_test ${ROSARUCOLIBTEST_SOURCE_FILES} ${ROSARUCOLIBTEST_HEADER_FILES})
add_dependencies(${PROJECT_NAME}_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test aruco_eye)
target_link_libraries(${PROJECT_NAME}_test ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})
