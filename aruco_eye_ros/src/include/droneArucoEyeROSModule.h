//////////////////////////////////////////////////////
//  droneArucoEyeROSModule.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Jan 15, 2014
//      Author: joselusl
//
//////////////////////////////////////////////////////


#ifndef _ARUCO_EYE_ROS_H
#define _ARUCO_EYE_ROS_H




//I/O stream
//std::cout
#include <iostream>

//String
//std::string, std::getline()
#include <string>

//Vector
//std::vector
#include <vector>



//Opencv
#include <opencv2/opencv.hpp>


//Aruco
#include "aruco.h"
//Aruco JL Lib
#include "arucoEye.h"


//ROS
#include "ros/ros.h"


//Aruco Messages
#include "aruco_eye_msgs/Marker.h"
#include "aruco_eye_msgs/MarkerList.h"


//ROS Images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


// Camera calibration
//TODO




#define VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE

#define DISPLAY_ARUCO_EYE



//configurations
const bool ARUCO_EYE_CONFIG_enableErosion=false;
const aruco::MarkerDetector::ThresholdMethods ARUCO_EYE_CONFIG_thresholdMethod=aruco::MarkerDetector::ADPT_THRES;
const double ARUCO_EYE_CONFIG_ThresParam1=7;
const double ARUCO_EYE_CONFIG_ThresParam2=7;
const aruco::MarkerDetector::CornerRefinementMethod ARUCO_EYE_CONFIG_methodCornerRefinement=aruco::MarkerDetector::LINES;
const int ARUCO_EYE_CONFIG_ThePyrDownLevel=0;
const float ARUCO_EYE_CONFIG_minSize=0.045;//0.03;
const float ARUCO_EYE_CONFIG_maxSize=0.5;//0.5;




/////////////////////////////////////////
// Class ArucoEyeROS
//
//   Description
//
/////////////////////////////////////////
class ArucoEyeROS
{
    // Aruco List File
protected:
    std::string arucoListFile;


private:
    int configureArucoEye(std::string arucoListFile, std::string cameraCalibrationFile);


    //ArucoRetina
protected:
    ArucoEye MyArucoEye;


    // Aruco Detector frame name
protected:
    std::string aruco_detector_frame_name;



    // Camera Calibration
protected:
    // Calibration using file
    std::string cameraCalibrationFile;
    // TODO: Calibration using topic


    //Images received
protected:
    std::string imageTopicName;
    cv_bridge::CvImagePtr cvImage;
    cv::Mat imageMat;
    //Subscriber
    ros::Subscriber imageSubs;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);


    // Output image
    // TODO


    //Aruco Visual Markers detected
protected:
    std::string arucoListTopicName;
    ros::Publisher arucoListPubl; ////Publishers
    aruco_eye_msgs::MarkerList arucoListMsg; //Messages
    bool publishArucoList();


    //Constructors and destructors
public:
    ArucoEyeROS(int argc,char **argv);
    ~ArucoEyeROS();

    //Init and close
public:
    void init();
    void close();

    //Open
 public:
    void open();

protected:
    void readParameters();



    //Drawing
#ifdef DISPLAY_ARUCO_EYE
protected:
    //Name
    std::string arucoEyeWindow;
public:
    int drawArucoCodes(std::string windowName, int waitingTime=1, bool drawDetectedCodes=true, bool draw3DReconstructedCodes=true);
#endif

};




#endif
