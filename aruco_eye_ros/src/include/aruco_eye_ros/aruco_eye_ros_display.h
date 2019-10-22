//////////////////////////////////////////////////////
//  aruco_eye_ros_display.h
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: claudiocimarelli
//
//////////////////////////////////////////////////////


#ifndef _ARUCO_EYE_ROS_DISPLAY_H
#define _ARUCO_EYE_ROS_DISPLAY_H




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


//aruco
#include <aruco.h>
//aruco_eye Lib
#include "aruco_eye_core/arucoEye.h"


//ROS
#include <ros/ros.h>


//Aruco Messages
#include <perception_msgs/PointInImage.h>
#include <perception_msgs/Marker.h>
#include <perception_msgs/MarkerList.h>

// Mesage filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

//ROS Images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


// Camera calibration
#include <sensor_msgs/CameraInfo.h>


//// Tf2
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



// Services
#include <robot_component_srvs/SetBool.h>


//#define VERBOSE_ARUCO_EYE_ROS

/////////////////////////////////////////
// Class ArucoEyeROS
//
//   Description
//
/////////////////////////////////////////
class ArucoEyeDisplayROS
{
    // Configure
private:
    int configureArucoEye(std::string cameraCalibrationFile);
    static aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                         bool useRectifiedParameters);

    //ArucoRetina
protected:
    ArucoEye MyArucoEye;
    // Camera Calibration
    // Calibration using file
    std::string cameraCalibrationFile;
    std::string camera_info_topic_name;
    ros::Subscriber cameraInfoSub;
    void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);

    // Images
    image_transport::ImageTransport* imageTransport;
    //Images received
    std::string imageTopicName;
    cv_bridge::CvImagePtr cvImage;
    cv::Mat imageMat;
    //Subscriber
    message_filters::Subscriber<sensor_msgs::Image>* imageSubs;

    //Aruco Visual Markers detected

    std::string arucoListTopicName;
    message_filters::Subscriber<perception_msgs::MarkerList>* arucoListSub;
    // Subscriber
    perception_msgs::MarkerList arucoListMsg; //Messages

    // Synchronization between topics

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, perception_msgs::MarkerList> TheSyncPolicy;
    message_filters::Synchronizer<TheSyncPolicy>* messagesSyncronizer;
    void imageAndArucoListCallback(const sensor_msgs::ImageConstPtr& image, const perception_msgs::MarkerListConstPtr& arucoList);


    // Output image
    std::string outputImageTopicName;
    cv::Mat outputImageMat;
    // Publisher
    image_transport::Publisher outputImagePub;

    //Constructors and destructors
public:
    ArucoEyeDisplayROS(int argc,char **argv);
    ~ArucoEyeDisplayROS();

    //Init and close
protected:
    void init();
    void close();

    int open();

    //Open
 public:
    int run();

protected:
    void readParameters();

    // Drawing Aruco visual markers
public:
    int drawArucoCodes(bool drawDetectedCodes=true, bool draw3DReconstructedCodes=true);
protected:
    // Flag
    bool flagDisplayOutputImage;
    // Name
    std::string arucoEyeWindow;
public:
    char displayArucoCodes(std::string windowName, int waitingTime=1);


    // Display image ROS service
protected:
    std::string enableDisplayImageSrvName;
    ros::ServiceServer enableDisplayImageSrv;
    bool enableDisplayImageCallback(robot_component_srvs::SetBool::Request  &req, robot_component_srvs::SetBool::Response &res);



};




#endif
