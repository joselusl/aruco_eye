//////////////////////////////////////////////////////
//  aruco_eye_ros_detector.h
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: claudiocimarelli
//
//////////////////////////////////////////////////////


#ifndef _ARUCO_EYE_ROS_DETECTOR_H
#define _ARUCO_EYE_ROS_DETECTOR_H




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
#include <aruco_eye_core/arucoEye.h>


//ROS
#include <ros/ros.h>


//Aruco Messages
#include <perception_msgs/PointInImage.h>
#include <perception_msgs/Marker.h>
#include <perception_msgs/MarkerList.h>


//ROS Images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


// Camera calibration
#include <sensor_msgs/CameraInfo.h>


// Tf2
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>



//#define VERBOSE_ARUCO_EYE_ROS



// configurations
// TODO
//const aruco::MarkerDetector::ThresholdMethods ARUCO_EYE_CONFIG_thresholdMethod=aruco::MarkerDetector::ADPT_THRES;
//const double ARUCO_EYE_CONFIG_ThresParam1=7;
//const double ARUCO_EYE_CONFIG_ThresParam2=7;
//const aruco::MarkerDetector::CornerRefinementMethod ARUCO_EYE_CONFIG_methodCornerRefinement=aruco::MarkerDetector::LINES;
//const float ARUCO_EYE_CONFIG_minSize=0.045;//0.03;
//const float ARUCO_EYE_CONFIG_maxSize=0.5;//0.5;

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
    std::string arucoDictionary;
    float markerSize;


private:
    int configureArucoEye(std::string & dictionary, float sizeInMeters, std::string & cameraCalibrationFile);


    //ArucoRetina
protected:
    ArucoEye MyArucoEye;


    // Aruco Detector frame name
protected:
    std::string aruco_detector_frame_name;

    std::string aruco_marker_child_base_name;



    // Camera Calibration
protected:
    // Calibration using file
    std::string TheCameraCalibrationFile;

    // Calibration using topic
protected:
    std::string camera_info_topic_name;
    ros::Subscriber cameraInfoSub;
    void cameraInfoCallback(const sensor_msgs::CameraInfo &msg);
protected:
    static aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
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
    image_transport::Subscriber imageSubs;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);


    //Aruco Visual Markers detected
protected:
    std::string arucoListTopicName;
    ros::Publisher arucoListPubl; ////Publishers
    perception_msgs::MarkerList arucoListMsg; //Messages
    bool publishArucoList();


    //Constructors and destructors
public:
    ArucoEyeROS(int argc,char **argv);
    ~ArucoEyeROS();

    //Init and close
public:
    void init();
    void close();

    // Configure
protected:
    int configure();
    int openROS();


    //Open and Run
 public:
    int setUp();
    int run();

    // Read parameters
protected:
    void readParameters();



    // Tf
protected:
    tf2_ros::TransformBroadcaster* tfTransformBroadcaster;
protected:
    static geometry_msgs::Transform arucoMarker2Tf(const aruco::Marker &marker);

};




#endif
