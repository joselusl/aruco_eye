//////////////////////////////////////////////////////
//  aruco_eye_ros_detector_node.cpp
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: claudiocimarelli
//
//////////////////////////////////////////////////////



//I/O stream
//std::cout
#include <iostream>


//Aruco Eye
#include "aruco_eye_ros/aruco_eye_ros_detector.h"



int main(int argc,char **argv)
{
    ArucoEyeROS MyArucoEyeROS(argc, argv);
    std::cout<<"[ROSNODE] Starting "<<ros::this_node::getName()<<std::endl;

    MyArucoEyeROS.run();

    return 0;
}
