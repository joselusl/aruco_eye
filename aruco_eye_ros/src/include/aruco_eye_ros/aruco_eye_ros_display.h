//////////////////////////////////////////////////////
//  aruco_eye_ros_display.h
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: joselusl
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
#include "aruco_lib/aruco.h"
//aruco_eye Lib
#include "aruco_eye_core/arucoEye.h"


//ROS
#include "ros/ros.h"


//Aruco Messages
#include "aruco_eye_msgs/PointInImage.h"
#include "aruco_eye_msgs/Marker.h"
#include "aruco_eye_msgs/MarkerList.h"

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


//// Tf
#include <tf/transform_datatypes.h>



// Services
#include "aruco_eye_srvs/SetBool.h"


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

    //ArucoRetina
protected:
    ArucoEye MyArucoEye;


    // Camera Calibration
protected:
    // Calibration using file
    std::string cameraCalibrationFile;

    // Calibration using topic
protected:
    std::string camera_info_topic_name;
    ros::Subscriber cameraInfoSub;
    void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);
protected:
    aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                                    bool useRectifiedParameters);

    // Images
protected:
    image_transport::ImageTransport* imageTransport;

    //Images received
protected:
    std::string imageTopicName;
    cv_bridge::CvImagePtr cvImage;
    cv::Mat imageMat;
    //Subscriber
    message_filters::Subscriber<sensor_msgs::Image>* imageSubs;

    //Aruco Visual Markers detected
protected:
    std::string arucoListTopicName;
    message_filters::Subscriber<aruco_eye_msgs::MarkerList>* arucoListSub;
    // Subscriber
    aruco_eye_msgs::MarkerList arucoListMsg; //Messages

    // Synchronization between topics
protected:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, aruco_eye_msgs::MarkerList> TheSyncPolicy;
    message_filters::Synchronizer<TheSyncPolicy>* messagesSyncronizer;
    void imageAndArucoListCallback(const sensor_msgs::ImageConstPtr& image, const aruco_eye_msgs::MarkerListConstPtr& arucoList);


    // Output image
protected:
    std::string outputImageTopicName;
    cv::Mat outputImageMat;
    // Publisher
    image_transport::Publisher outputImagePub;



    //Constructors and destructors
public:
    ArucoEyeDisplayROS(int argc,char **argv);
    ~ArucoEyeDisplayROS();

    //Init and close
public:
    void init();
    void close();

    //Open
 public:
    int open();

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
    bool enableDisplayImageCallback(aruco_eye_srvs::SetBool::Request  &req, aruco_eye_srvs::SetBool::Response &res);



};




#endif
