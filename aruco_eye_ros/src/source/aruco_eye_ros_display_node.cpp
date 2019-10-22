//////////////////////////////////////////////////////
//  aruco_eye_ros_display_node.cpp
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
#include "aruco_eye_ros/aruco_eye_ros_display.h"



int main(int argc,char **argv)
{
    ArucoEyeDisplayROS MyArucoEyeDisplayROS(argc, argv);
    std::cout<<"[ROSNODE] Starting "<<ros::this_node::getName()<<std::endl;

    MyArucoEyeDisplayROS.run();

    return 0;
}
