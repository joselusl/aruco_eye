//////////////////////////////////////////////////////
//  aruco_eye_ros_display.cpp
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "aruco_eye_ros/aruco_eye_ros_display.h"



using namespace std;


////////// Drone Aruco Eye ///////////
ArucoEyeDisplayROS::ArucoEyeDisplayROS(int argc, char **argv)
{
    //Ros Init
    ros::init(argc, argv, ros::this_node::getName());
    // Node handle
    ros::NodeHandle nh;

    init();
    return;
}


ArucoEyeDisplayROS::~ArucoEyeDisplayROS()
{
    // Close windows
    cv::destroyAllWindows();

    // Delete
    delete imageTransport;
    delete messagesSyncronizer;
    delete imageSubs;
    delete arucoListSub;

    // CLose
    close();
    return;
}



int ArucoEyeDisplayROS::configureArucoEye(std::string cameraCalibrationFile)
{
    int error=0;

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


void ArucoEyeDisplayROS::init()
{

    return;
}


void ArucoEyeDisplayROS::close()
{
    if(!MyArucoEye.close())
        return;

    return;
}

void ArucoEyeDisplayROS::readParameters()
{
    // Config files
    //
    ros::param::param<std::string>("~camera_calibration_file", cameraCalibrationFile, "");
    if(!cameraCalibrationFile.empty())
        std::cout<<"cameraCalibrationFile="<<cameraCalibrationFile<<std::endl;

    // Other parameters
    // Display image
    ros::param::param<bool>("~flag_display_output_image", flagDisplayOutputImage, true);
    std::cout<<"flag_display_output_image="<<flagDisplayOutputImage<<std::endl;
    // Display image window name
    ros::param::param<std::string>("~display_output_image_window_name", arucoEyeWindow, "arUco_display_window");
    std::cout<<"display_output_image_window_name="<<arucoEyeWindow<<std::endl;


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

    // Service names
    //
    ros::param::param<std::string>("~display_output_image_service_name", enableDisplayImageSrvName, "aruco_eye/enable_display_output_image");
    std::cout<<"display_output_image_service_name="<<enableDisplayImageSrvName<<std::endl;


    // End
    return;
}

int ArucoEyeDisplayROS::open()
{
    // Node handle
    ros::NodeHandle nh;

    // Read parameters
    readParameters();


    // Configure
    //configure droneArucoEye
    int errorConfigureArucoEye=configureArucoEye(cameraCalibrationFile);
    if(errorConfigureArucoEye > 0)
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        cout<<"[AE-ROS] Partial Configuration of Aruco Eye"<<endl;
#endif
    }


    // Image transport
    imageTransport=new image_transport::ImageTransport(nh);

    // Subscriber Image
    imageSubs=new message_filters::Subscriber<sensor_msgs::Image>();
    imageSubs->subscribe(nh, imageTopicName, 10);

    // Subscriber Camerainfo
    if(!MyArucoEye.isTheCameraParametersSet())
        cameraInfoSub=nh.subscribe(camera_info_topic_name, 1, &ArucoEyeDisplayROS::cameraInfoCallback, this);

    // Subscriber aruco list
    arucoListSub=new message_filters::Subscriber<aruco_eye_msgs::MarkerList>();
    arucoListSub->subscribe(nh, arucoListTopicName, 10);

    // messagesSyncronizer
    messagesSyncronizer=new message_filters::Synchronizer<TheSyncPolicy>(TheSyncPolicy( 10 ));
    messagesSyncronizer->connectInput(*imageSubs, *arucoListSub);
    messagesSyncronizer->registerCallback( boost::bind( &ArucoEyeDisplayROS::imageAndArucoListCallback, this, _1, _2 ) );


    // Publisher output image
    outputImagePub = imageTransport->advertise(outputImageTopicName, 1);


    // Services
    enableDisplayImageSrv=nh.advertiseService(enableDisplayImageSrvName, &ArucoEyeDisplayROS::enableDisplayImageCallback, this);



    //Create gui
    if(this->flagDisplayOutputImage)
        cv::namedWindow(arucoEyeWindow, 1);


    //End
    return 0;
}



void ArucoEyeDisplayROS::cameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
    if(!MyArucoEye.setCameraParameters(rosCameraInfo2ArucoCamParams(msg, true)))
        cameraInfoSub.shutdown();

    return;
}



void ArucoEyeDisplayROS::imageAndArucoListCallback(const sensor_msgs::ImageConstPtr& image, const aruco_eye_msgs::MarkerListConstPtr& arucoList)
{
    // Check time stamps
    if(image->header.stamp!=arucoList->header.stamp)
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        ROS_ERROR("Error synchronizing images and list of visual marker topics");
#endif
        return;
    }

    // Check image topic
    if(arucoList->imageTopicName!=imageSubs->getTopic())
    {
#ifdef VERBOSE_ARUCO_EYE_ROS
        std::cout<<"1="<<arucoList->imageTopicName<<std::endl;
        std::cout<<"2="<<imageSubs->getTopic()<<std::endl;
        ROS_ERROR("Error with the topic names");
#endif
        return;
    }

    // Current Time Stamp
    ros::Time curr_stamp(image->header.stamp);

    //Transform image message to OpenCV to be processed
    try
    {
        cvImage = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
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


    // Read visual markers information
    MyArucoEye.clearMarkersList();
    for(unsigned int i=0; i<arucoList->markers.size(); i++)
    {
        // Aux variables
        aruco::Marker TheMarker;
        ArucoMarker TheArucoMarker;

        // Image points
        for(unsigned int j=0; j<arucoList->markers[i].pointsInImage.size(); j++)
        {
            cv::Point2f ThePointInImage(arucoList->markers[i].pointsInImage[j].x, arucoList->markers[i].pointsInImage[j].y);
            TheMarker.push_back(ThePointInImage);
        }

        // Id
        TheMarker.id=arucoList->markers[i].id;

        // Size
        TheMarker.ssize=arucoList->markers[i].size;

        // 3D reconstruction
        if(arucoList->markers[i].is3dReconstructed)
        {
            TheArucoMarker.set3DReconstructed(true);

            //geometry_msgs::Point position=arucoList->markers[i].pose.pose.position;
            //geometry_msgs::Quaternion orientation=arucoList->markers[i].pose.pose.orientation;

            geometry_msgs::Pose poseMsg=arucoList->markers[i].pose.pose;

            tf::Pose tfPose;
            tf::poseMsgToTF(poseMsg, tfPose);

            tf::Vector3 Tran=tfPose.getOrigin();
            tf::Matrix3x3 Rot=tfPose.getBasis();

            cv::Mat cvTran(3, 1, CV_32FC1);
            cv::Mat cvRot(3, 3, CV_32FC1);

            cvTran.at<float>(0)=Tran.getX();
            cvTran.at<float>(1)=Tran.getY();
            cvTran.at<float>(2)=Tran.getZ();

            for(int k=0; k<3; k++)
            {
                cvRot.at<float>(k,0)=Rot.getRow(k).getX();
                cvRot.at<float>(k,1)=Rot.getRow(k).getY();
                cvRot.at<float>(k,2)=Rot.getRow(k).getZ();
            }


            TheMarker.Tvec=cvTran;
            cv::Rodrigues(cvRot, TheMarker.Rvec);

        }

        // Push
        TheArucoMarker.setMarker(TheMarker);
        MyArucoEye.addMarkerToMarkersList(TheArucoMarker);

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


    // Display
    if(flagDisplayOutputImage)
        displayArucoCodes(arucoEyeWindow,1);


    return;
}


int ArucoEyeDisplayROS::run()
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


int ArucoEyeDisplayROS::drawArucoCodes(bool drawDetectedCodes, bool draw3DReconstructedCodes)
{
    return MyArucoEye.drawDetectedArucoCodes(drawDetectedCodes, draw3DReconstructedCodes);
}


char ArucoEyeDisplayROS::displayArucoCodes(std::string windowName, int waitingTime)
{
    return MyArucoEye.displayDetectedArucoCodes(windowName, waitingTime);
}


bool ArucoEyeDisplayROS::enableDisplayImageCallback(aruco_eye_srvs::SetBool::Request  &req, aruco_eye_srvs::SetBool::Response &res)
{
    this->flagDisplayOutputImage=req.data;
    res.success=true;

    if(this->flagDisplayOutputImage)
    {
        cv::namedWindow(arucoEyeWindow, 1);
        //cv::startWindowThread();
    }
    else
        cv::destroyWindow(this->arucoEyeWindow);

    return true;
}


aruco::CameraParameters ArucoEyeDisplayROS::rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
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
