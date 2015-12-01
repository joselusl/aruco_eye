//////////////////////////////////////////////////////
//  droneArucoEyeROSModule.cpp
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Jan 15, 2014
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "droneArucoEyeROSModule.h"



using namespace std;


////////// Drone Aruco Eye ///////////
ArucoEyeROS::ArucoEyeROS(int argc, char **argv)
{
    //Ros Init
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    imageTransport=new image_transport::ImageTransport(nh);
    tfTransformBroadcaster=new tf::TransformBroadcaster;

    init();
    return;
}


ArucoEyeROS::~ArucoEyeROS()
{
    // Delete
    delete imageTransport;
    delete tfTransformBroadcaster;
    // CLose
    close();
    return;
}



int ArucoEyeROS::configureArucoEye(std::string arucoListFile, std::string cameraCalibrationFile)
{
    int error=0;

    //configure aruco detector
    if(MyArucoEye.configureArucoDetector(ARUCO_EYE_CONFIG_enableErosion,
                                          ARUCO_EYE_CONFIG_thresholdMethod,
                                          ARUCO_EYE_CONFIG_ThresParam1,
                                          ARUCO_EYE_CONFIG_ThresParam2,
                                          ARUCO_EYE_CONFIG_methodCornerRefinement,
                                          ARUCO_EYE_CONFIG_ThePyrDownLevel,
                                          ARUCO_EYE_CONFIG_minSize,
                                          ARUCO_EYE_CONFIG_maxSize) )
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        cout<<"[AE-ROS] Error configuring Aruco Eye: configureArucoDetector"<<endl;
#endif
        error=1;
    }

    //set aruco list
    if(MyArucoEye.setArucoList(arucoListFile))
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        cout<<"[AE-ROS] Error configuring Aruco Eye: setArucoList"<<endl;
#endif
        error=2;
    }

    //set camera parameters
    if(MyArucoEye.setCameraParameters(cameraCalibrationFile))
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        cout<<"[AE-ROS] Error configuring Aruco Eye: setCameraParameters()"<<endl;
#endif
        error=-1;
    }

    //End
    return error;
}


void ArucoEyeROS::init()
{


    return;
}


void ArucoEyeROS::close()
{
    if(!MyArucoEye.close())
        return;

    return;
}

void ArucoEyeROS::readParameters()
{
    // Config files
    //
    ros::param::param<std::string>("~aruco_list_file", arucoListFile,"arUcoList.xml");
    std::cout<<"arucoListFile="<<arucoListFile<<std::endl;
    //
    ros::param::param<std::string>("~camera_calibration_file", cameraCalibrationFile, "camera.yaml");
    std::cout<<"cameraCalibrationFile="<<cameraCalibrationFile<<std::endl;

    // TODO parameters of the aruco detector -> Now hardcoded!
    //

    // Other parameters
    //
    ros::param::param<std::string>("~aruco_detector_frame_name", aruco_detector_frame_name,"aruco_detector_frame");
    std::cout<<"aruco_detector_frame_name="<<aruco_detector_frame_name<<std::endl;
    //
    ros::param::param<std::string>("~aruco_marker_child_base_name", aruco_marker_child_base_name,"aruco_marker_");
    std::cout<<"aruco_marker_child_base_name="<<aruco_marker_child_base_name<<std::endl;

    // Topic names
    //
    ros::param::param<std::string>("~image_topic_name", imageTopicName, "camera/image_raw");
    std::cout<<"image_topic_name="<<imageTopicName<<std::endl;
    //
    ros::param::param<std::string>("~camera_info_topic_name", camera_info_topic_name, "camera/camera_info");
    std::cout<<"camera_info_topic_name="<<camera_info_topic_name<<std::endl;
    //
    ros::param::param<std::string>("~aruco_list_topic_name", arucoListTopicName, "aruco_eye/aruco_observation");
    std::cout<<"aruco_list_topic_name="<<arucoListTopicName<<std::endl;
    //
    ros::param::param<std::string>("~output_image_topic_name", outputImageTopicName, "aruco_eye/aruco_observation_image/image_raw");
    std::cout<<"output_image_topic_name="<<outputImageTopicName<<std::endl;



    return;
}

int ArucoEyeROS::open()
{
    ros::NodeHandle nh;

    // Read parameters
    readParameters();


    //configure droneArucoEye
    int errorConfigureArucoEye=configureArucoEye(arucoListFile, cameraCalibrationFile);
    if(errorConfigureArucoEye > 0)
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        cout<<"[AE-ROS] Critical Error configuring Aruco Eye"<<endl;
#endif
        return 1;
    }
    else if(errorConfigureArucoEye > 0)
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        cout<<"[AE-ROS] Partial Configuration of Aruco Eye"<<endl;
#endif
    }

    // Subscriber Image
    imageSubs = imageTransport->subscribe(imageTopicName, 1, &ArucoEyeROS::imageCallback, this);
    // Subscriber Camerainfo
    if(!MyArucoEye.isTheCameraParametersSet())
        cameraInfoSub=nh.subscribe(camera_info_topic_name, 1, &ArucoEyeROS::cameraInfoCallback, this);

    // Publisher aruco 3D pose
    arucoListPubl = nh.advertise<aruco_eye_msgs::MarkerList>(arucoListTopicName, 1, true);
    // Publisher output image
    outputImagePub = imageTransport->advertise(outputImageTopicName, 1);


#ifdef DISPLAY_ARUCO_EYE
    //Name
    arucoEyeWindow="arucoEye";
    //Create gui
    cv::namedWindow(arucoEyeWindow, 1);
#endif

    //End
    return 0;
}


void ArucoEyeROS::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Current Time Stamp
    ros::Time curr_stamp(msg->header.stamp);

    //Transform image message to Opencv to be processed
    try
    {
        cvImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        ROS_ERROR("cv_bridge exception: %s", e.what());
#endif
        return;
    }
    imageMat=cvImage->image;


    //Set image to aruco eye
    if(MyArucoEye.setInputImage(imageMat))
        return;


    if(arucoListPubl.getNumSubscribers()>0)
    {
        // Prepare message to be published
        arucoListMsg.markers.clear();
        // Header
        arucoListMsg.header.stamp=curr_stamp;
        arucoListMsg.header.frame_id=this->aruco_detector_frame_name;
    }


    //Run aruco eye
    unsigned int numCodesDetected, numCodesReconstructed;
    if(MyArucoEye.run(numCodesDetected, numCodesReconstructed))
        return;
#ifdef VERBOSE_ARUCO_EYE_ROS
    cout<<"[AE-ROS] numCodesDetected="<<numCodesDetected<<"; ";
    cout<<"numCodesReconstructed="<<numCodesReconstructed<<endl;
#endif


    // Get the markers
    std::vector<ArucoMarker> TheMarkers;
    MyArucoEye.getMarkersList(TheMarkers);

    // Iterate over the markers to fill the messages
    for(unsigned int i=0; i<TheMarkers.size(); i++)
    {
        aruco_eye_msgs::Marker TheMarkerMsg;

        // Detected
        aruco::Marker TheArucoMarker=TheMarkers[i].getMarker();

        std::string child_name = aruco_marker_child_base_name+std::to_string(TheArucoMarker.id);
        std::string parent_name = aruco_detector_frame_name;

        if(arucoListPubl.getNumSubscribers()>0)
        {
            TheMarkerMsg.header.stamp=curr_stamp;
            TheMarkerMsg.header.frame_id=child_name;
            TheMarkerMsg.id=TheArucoMarker.id;
        }

        // Reconstructed
        if(TheMarkers[i].is3DReconstructed())
        {
            // TF
            tf::Transform transform = arucoMarker2Tf(TheArucoMarker);
            tfTransformBroadcaster->sendTransform(tf::StampedTransform(transform, curr_stamp,
                                                  parent_name, child_name));

            // Fill the message
            if(arucoListPubl.getNumSubscribers()>0)
            {
                geometry_msgs::Pose poseMsg;
                tf::poseTFToMsg(transform, poseMsg);
                TheMarkerMsg.pose.pose=poseMsg;
                // TODO Covariance
            }

        }


        if(arucoListPubl.getNumSubscribers()>0)
        {
            // Push
            arucoListMsg.markers.push_back(TheMarkerMsg);
        }
    }

    if(arucoListPubl.getNumSubscribers()>0)
    {
        // Publish Aruco List
        if(publishArucoList())
            return;
    }

    // Draw aruco codes
    drawArucoCodes(true,true);

    // Publish Output Image
    if(outputImagePub.getNumSubscribers()>0)
    {
        if(!MyArucoEye.getOutputImage(outputImageMat))
        {
            cv_bridge::CvImage out_msg;
            out_msg.header.stamp = curr_stamp;
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            out_msg.image = outputImageMat;
            outputImagePub.publish(out_msg.toImageMsg());
        }
    }


#ifdef DISPLAY_ARUCO_EYE
    // Display
    displayArucoCodes(arucoEyeWindow,1);
#endif


    return;
}


void ArucoEyeROS::cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
    if(!MyArucoEye.setCameraParameters(rosCameraInfo2ArucoCamParams(msg, true)))
        cameraInfoSub.shutdown();

    return;
}

int ArucoEyeROS::run()
{
    try
    {
        ros::spin();
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }

    return 0;
}



bool ArucoEyeROS::publishArucoList()
{

    //publish
    arucoListPubl.publish(arucoListMsg);

    //end
    return 1;
}

int ArucoEyeROS::drawArucoCodes(bool drawDetectedCodes, bool draw3DReconstructedCodes)
{
    return MyArucoEye.drawDetectedArucoCodes(drawDetectedCodes, draw3DReconstructedCodes);
}

#ifdef DISPLAY_ARUCO_EYE
char ArucoEyeROS::displayArucoCodes(std::string windowName, int waitingTime)
{
    return MyArucoEye.displayDetectedArucoCodes(windowName, waitingTime);
}
#endif


tf::Transform ArucoEyeROS::arucoMarker2Tf(const aruco::Marker &marker)
{
    cv::Mat rot(3, 3, CV_32FC1);
    cv::Rodrigues(marker.Rvec, rot);
    cv::Mat tran = marker.Tvec;

//    cv::Mat rotate_to_ros(3, 3, CV_32FC1);
//    // -1 0 0
//    // 0 0 1
//    // 0 1 0
//    rotate_to_ros.at<float>(0,0) = -1.0;
//    rotate_to_ros.at<float>(0,1) = 0.0;
//    rotate_to_ros.at<float>(0,2) = 0.0;
//    rotate_to_ros.at<float>(1,0) = 0.0;
//    rotate_to_ros.at<float>(1,1) = 0.0;
//    rotate_to_ros.at<float>(1,2) = 1.0;
//    rotate_to_ros.at<float>(2,0) = 0.0;
//    rotate_to_ros.at<float>(2,1) = 1.0;
//    rotate_to_ros.at<float>(2,2) = 0.0;
//    rot = rot*rotate_to_ros.t();

    tf::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                       rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                       rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

    tf::Vector3 tf_orig(tran.at<float>(0,0), tran.at<float>(1,0), tran.at<float>(2,0));

    return tf::Transform(tf_rot, tf_orig);
}


aruco::CameraParameters ArucoEyeROS::rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                                bool useRectifiedParameters)
{
    cv::Mat cameraMatrix(3, 3, CV_32FC1);
    cv::Mat distorsionCoeff(4, 1, CV_32FC1);
    cv::Size size(cam_info.height, cam_info.width);

    if ( useRectifiedParameters )
    {
      cameraMatrix.setTo(0);
      cameraMatrix.at<float>(0,0) = cam_info.P[0];   cameraMatrix.at<float>(0,1) = cam_info.P[1];   cameraMatrix.at<float>(0,2) = cam_info.P[2];
      cameraMatrix.at<float>(1,0) = cam_info.P[4];   cameraMatrix.at<float>(1,1) = cam_info.P[5];   cameraMatrix.at<float>(1,2) = cam_info.P[6];
      cameraMatrix.at<float>(2,0) = cam_info.P[8];   cameraMatrix.at<float>(2,1) = cam_info.P[9];   cameraMatrix.at<float>(2,2) = cam_info.P[10];

      for(int i=0; i<4; ++i)
        distorsionCoeff.at<float>(i, 0) = 0;
    }
    else
    {
      for(int i=0; i<9; ++i)
        cameraMatrix.at<float>(i%3, i-(i%3)*3) = cam_info.K[i];

      if(cam_info.D.size() == 4)
      {
        for(int i=0; i<4; ++i)
          distorsionCoeff.at<float>(i, 0) = cam_info.D[i];
      }
      else
      {
        ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
        for(int i=0; i<4; ++i)
          distorsionCoeff.at<float>(i, 0) = 0;
      }
    }

    return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}
