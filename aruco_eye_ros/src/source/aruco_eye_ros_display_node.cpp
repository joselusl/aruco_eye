//////////////////////////////////////////////////////
//  droneArucoEyeROSModuleNode.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Jan 15, 2014
//      Author: joselusl
//
//////////////////////////////////////////////////////



//I/O stream
//std::cout
#include <iostream>


//Aruco Eye
#include "aruco_eye_ros_display.h"



int main(int argc,char **argv)
{
    ArucoEyeDisplayROS MyArucoEyeDisplayROS(argc, argv);
    std::cout<<"[ROSNODE] Starting "<<ros::this_node::getName()<<std::endl;
  	
    MyArucoEyeDisplayROS.open();

    MyArucoEyeDisplayROS.run();

    return 0;
}
