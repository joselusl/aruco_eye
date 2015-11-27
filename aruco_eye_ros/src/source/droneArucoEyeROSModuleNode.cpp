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


//Opencv
#include <opencv2/opencv.hpp>

//ROS
#include "ros/ros.h"

//Aruco Eye
#include "droneArucoEyeROSModule.h"



using namespace std;






int main(int argc,char **argv)
{

    ArucoEyeROS MyArucoEyeROS(argc, argv);
    cout<<"[ROSNODE] Starting "<<ros::this_node::getName()<<endl;
  	

    MyArucoEyeROS.open();


  	
	try
	{
        ros::spin();
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }

    return 1;
}
