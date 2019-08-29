//////////////////////////////////////////////////////
//  aruco_eye_ros_detector.cpp
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: claudiocimarelli
//
//////////////////////////////////////////////////////


#include "aruco_eye_ros/aruco_eye_ros_detector.h"



using namespace std;


////////// Drone Aruco Eye ///////////
ArucoEyeROS::ArucoEyeROS(int argc, char **argv)
{
    //Ros Init
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    markerSize=0.1;

    imageTransport=new image_transport::ImageTransport(nh);
    tfTransformBroadcaster=new tf2_ros::TransformBroadcaster();

    // Init
    init();
    setUp();
}


ArucoEyeROS::~ArucoEyeROS()
{
    // Delete
    delete imageTransport;
    delete tfTransformBroadcaster;
    // Close
    close();
}



int ArucoEyeROS::configureArucoEye(std::string & dictionary, float sizeInMeters, std::string & cameraCalibrationFile)
{
    int error=0;

    //set aruco list
    if(MyArucoEye.setDictionary(dictionary))
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        cout<<"[AE-ROS] Error configuring Aruco Eye: setArucoList"<<endl;
#endif
        error=2;
    }

    MyArucoEye.setMarkerSize(sizeInMeters);

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
    MyArucoEye.init();
}


void ArucoEyeROS::close()
{
    MyArucoEye.close();
}

void ArucoEyeROS::readParameters()
{
    // Config files
    //
    ros::param::param<std::string>("~aruco_dictionary", arucoDictionary, "ARUCO_MIP_36h12");
    std::cout<<"aruco_dictionary"<<arucoDictionary<<std::endl;

    std:string tmp_markerSize;
    ros::param::param<std::string>("~marker_size", tmp_markerSize, "0.16");
    std::cout<<"marker_Size="<<tmp_markerSize<<std::endl;
    markerSize=stof(tmp_markerSize);
    //
    ros::param::param<std::string>("~camera_calibration_file", TheCameraCalibrationFile, "");
    if(!TheCameraCalibrationFile.empty())
        std::cout << "TheCameraCalibrationFile=" << TheCameraCalibrationFile << std::endl;

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

}

int ArucoEyeROS::configure()
{
    //configure droneArucoEye
    int errorConfigureArucoEye=configureArucoEye(arucoDictionary, markerSize, TheCameraCalibrationFile);
    if(errorConfigureArucoEye > 0)
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        cout<<"[AE-ROS] Critical Error configuring Aruco Eye"<<endl;
#endif
        return 1;
    }
    else if(errorConfigureArucoEye < 0)
    {
#if 1 || VERBOSE_ARUCO_EYE_ROS
        cout<<"[AE-ROS] Partial Configuration of Aruco Eye. Camera calibration needed"<<endl;
#endif
    }

    return 0;
}

int ArucoEyeROS::openROS()
{
    // NH
    ros::NodeHandle nh;

    // Subscriber Image
    imageSubs = imageTransport->subscribe(imageTopicName, 1, &ArucoEyeROS::imageCallback, this);
    // Subscriber Camerainfo
    if(!MyArucoEye.isTheCameraParametersSet())
    {
#if 1 || VERBOSE_ARUCO_EYE_ROS
        std::cout<<"[AE-ROS] Subscribing to camera info topic name to get camera parameters"<<std::endl;
#endif
        cameraInfoSub=nh.subscribe(camera_info_topic_name, 1, &ArucoEyeROS::cameraInfoCallback, this);
    }

    // Publisher aruco 3D pose
    arucoListPubl = nh.advertise<perception_msgs::MarkerList>(arucoListTopicName, 1, true);

    return 0;
}

int ArucoEyeROS::setUp()
{
    // Read parameters
    readParameters();
    // Configure
    configure();
    // Open ROS
    openROS();
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

        // Image topic name
        arucoListMsg.imageTopicName=imageSubs.getTopic();
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
    for(auto & TheMarker : TheMarkers)
    {
        perception_msgs::Marker TheMarkerMsg;

        // Detected
        aruco::Marker TheArucoMarker=TheMarker.getMarker();

        // Header + frame names
        std::string child_name = aruco_marker_child_base_name+std::to_string(TheArucoMarker.id);
        std::string parent_name = aruco_detector_frame_name;

        if(arucoListPubl.getNumSubscribers()>0)
        {
            // Time stamp
            TheMarkerMsg.header.stamp=curr_stamp;

            // Frame name
            TheMarkerMsg.header.frame_id=child_name;

            // Id
            TheMarkerMsg.id=std::to_string(TheArucoMarker.id);

            // Size of the visual marker
            TheMarkerMsg.size.resize(2);
            TheMarkerMsg.size[0]=TheArucoMarker.ssize;
            TheMarkerMsg.size[1]=TheArucoMarker.ssize;

            // Image points
            for(size_t j=0; j<TheArucoMarker.size(); j++)
            {
                perception_msgs::LabeledPointInImage labeled_point_i;
                labeled_point_i.value="";
                labeled_point_i.pointsInImage.x=TheArucoMarker[j].x;
                labeled_point_i.pointsInImage.y=TheArucoMarker[j].y;
                TheMarkerMsg.labeledPointsInImage.push_back(labeled_point_i);
            }

            // Flag 3D reconstruction initialization
            TheMarkerMsg.is3dReconstructed=false;

            // Confidence
            // TODO
        }

        // Reconstructed
        if(TheMarker.is3DReconstructed())
        {
            // TF
            geometry_msgs::TransformStamped transform_stamped;

            // Header
            transform_stamped.header.stamp=curr_stamp;
            transform_stamped.header.frame_id=parent_name;
            // Child frame id
            transform_stamped.child_frame_id=child_name;
            // Transform
            transform_stamped.transform = arucoMarker2Tf(TheArucoMarker);
            // Publish
            tfTransformBroadcaster->sendTransform(transform_stamped);

            // Fill the message
            if(arucoListPubl.getNumSubscribers()>0)
            {
                // Flag
                TheMarkerMsg.is3dReconstructed=true;

                // Pose
                // Position
                TheMarkerMsg.pose.pose.position.x=transform_stamped.transform.translation.x;
                TheMarkerMsg.pose.pose.position.y=transform_stamped.transform.translation.y;
                TheMarkerMsg.pose.pose.position.z=transform_stamped.transform.translation.z;
                // orientation
                TheMarkerMsg.pose.pose.orientation=transform_stamped.transform.rotation;


                // Covariance
                // TODO
                // TheMarkerMsg.pose.covariance;
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
}


void ArucoEyeROS::cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
//    aruco::CameraParameters camParam = rosCameraInfo2ArucoCamParams(msg, true);
//    if(!MyArucoEye.setCameraParameters(camParam))
//    {
//        cameraInfoSub.shutdown();
//        std::cout<<"[AE-ROS] Camera calibration parameters received!"<<std::endl;
//    }

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
    return 0;
}


geometry_msgs::Transform ArucoEyeROS::arucoMarker2Tf(const aruco::Marker &marker)
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

    tf2::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                       rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                       rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

    tf2::Vector3 tf_orig(tran.at<float>(0,0), tran.at<float>(1,0), tran.at<float>(2,0));



    tf2::Transform tf2_transform=tf2::Transform(tf_rot, tf_orig);

    geometry_msgs::Transform transform=tf2::toMsg(tf2_transform);


    return transform;
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
