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
#include "droneArucoEyeROSModule.h"



int main(int argc,char **argv)
{
    ArucoEyeROS MyArucoEyeROS(argc, argv);
    std::cout<<"[ROSNODE] Starting "<<ros::this_node::getName()<<std::endl;
  	
    MyArucoEyeROS.open();

    MyArucoEyeROS.run();

    return 0;
}
