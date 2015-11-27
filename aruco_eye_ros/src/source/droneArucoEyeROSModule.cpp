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

    init();
    return;
}


ArucoEyeROS::~ArucoEyeROS()
{
    close();
    return;
}



int ArucoEyeROS::configureArucoEye(std::string arucoListFile, std::string cameraCalibrationFile)
{

    //configure aruco detector
    if(!MyArucoEye.configureArucoDetector(ARUCO_EYE_CONFIG_enableErosion,
                                          ARUCO_EYE_CONFIG_thresholdMethod,
                                          ARUCO_EYE_CONFIG_ThresParam1,
                                          ARUCO_EYE_CONFIG_ThresParam2,
                                          ARUCO_EYE_CONFIG_methodCornerRefinement,
                                          ARUCO_EYE_CONFIG_ThePyrDownLevel,
                                          ARUCO_EYE_CONFIG_minSize,
                                          ARUCO_EYE_CONFIG_maxSize) )
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE
        cout<<"[DAE] Error configuring Aruco Retina"<<endl;
#endif
        return 0;
    }

    //set aruco list
    if(!MyArucoEye.setArucoList(arucoListFile))
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE
        cout<<"[DAE] Error configuring Aruco Retina"<<endl;
#endif
        return 0;
    }

    //set camera parameters
    if(!MyArucoEye.setCameraParameters(cameraCalibrationFile))
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE
        cout<<"[DAE] Error configuring Aruco Retina"<<endl;
#endif
        return 0;
    }

    //End
    return 1;
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
    // TODO
    // parameters of the aruco detector -> Now hardcoded!
    //
    ros::param::param<std::string>("~aruco_detector_frame_name", aruco_detector_frame_name,"aruco_detector_frame");
    std::cout<<"aruco_detector_frame_name="<<aruco_detector_frame_name<<std::endl;


    // Topic names
    //
    ros::param::param<std::string>("~image_topic_name", imageTopicName, "camera/image_raw");
    std::cout<<"image_topic_name="<<imageTopicName<<std::endl;
    //
    ros::param::param<std::string>("~aruco_list_topic_name", arucoListTopicName, "arucoObservation");
    std::cout<<"aruco_list_topic_name="<<arucoListTopicName<<std::endl;

    return;
}

void ArucoEyeROS::open()
{
    ros::NodeHandle nh;


    // Read parameters
    readParameters();



    //configure droneArucoEye
    if(!configureArucoEye(arucoListFile,cameraCalibrationFile))
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE
        cout<<"[DAE-ROS] Error configuring Aruco Eye"<<endl;
#endif
        return;
    }



    // Subscriber
    imageSubs = nh.subscribe(imageTopicName, 1, &ArucoEyeROS::imageCallback, this);

    //Publisher aruco 3D pose
    arucoListPubl = nh.advertise<aruco_eye_msgs::MarkerList>(arucoListTopicName, 1, true);



#ifdef DISPLAY_ARUCO_EYE
    //Name
    arucoEyeWindow="arucoEye";
    //Create gui
    cv::namedWindow(arucoEyeWindow,1);
#endif


    //End
    return;
}


void ArucoEyeROS::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    //Transform image message to Opencv to be processed
    try
    {
        cvImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
#ifdef VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE
        ROS_ERROR("cv_bridge exception: %s", e.what());
#endif
        return;
    }
    imageMat=cvImage->image;


    //Set image to aruco eye
    if(!MyArucoEye.setInputImage(imageMat))
        return;


    // Prepare message to be published
    arucoListMsg.markers.clear();
    // Header
    arucoListMsg.header.stamp=msg->header.stamp;
    arucoListMsg.header.frame_id=this->aruco_detector_frame_name;


    //Run aruco eye
    unsigned int numCodesDetected, numCodesReconstructed;
    if(!MyArucoEye.run(numCodesDetected, numCodesReconstructed))
        return;
#ifdef VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE
    cout<<"[DAE-ROS] numCodesDetected="<<numCodesDetected<<"; ";
    cout<<"numCodesReconstructed="<<numCodesReconstructed<<endl;
#endif


    // Iterate over the markers to fill the message
    std::vector<ArucoMarker> TheMarkers;
    MyArucoEye.getMarkersList(TheMarkers);
    for(unsigned int i=0; i<TheMarkers.size(); i++)
    {
        aruco_eye_msgs::Marker TheMarkerMsg;
        TheMarkerMsg.header.stamp=arucoListMsg.header.stamp;
        TheMarkerMsg.header.frame_id=arucoListMsg.header.frame_id;

        // Detected
        aruco::Marker TheArucoMarker=TheMarkers[i].getMarker();
        TheMarkerMsg.id=TheArucoMarker.id;

        // Reconstructed
        if(TheMarkers[i].is3DReconstructed())
        {
            // Fill the message
            // TODO
        }



        // Push
        arucoListMsg.markers.push_back(TheMarkerMsg);
    }


    //Publish
    if(!publishArucoList())
        return;


#ifdef DISPLAY_ARUCO_EYE
    //Draw
    drawArucoCodes(arucoEyeWindow,1,true,true);
#endif



    return;
}


/*
//Run
bool ArucoEyeROS::run()
{

    //Get codes reconstructed
    int idMarker;
    cv::Mat matHomog_aruco_GMR_wrt_drone_GMR;
    for(unsigned int i=0; i<numCodesReconstructed;i++)
    {
        if(!MyDroneArucoEye.getDroneMarkerI(i,idMarker,matHomog_aruco_GMR_wrt_drone_GMR))
        {
            continue;
        }

        double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0, pitch = 0.0, roll = 0.0;
        referenceFrames::getxyzYPRfromHomogMatrix_wYvPuR( matHomog_aruco_GMR_wrt_drone_GMR, &x, &y, &z, &yaw, &pitch, &roll);

#ifdef VERBOSE_DRONE_ARUCO_EYE_ROS_MODULE
        cout<<"[DAE-ROS] Marker id="<<idMarker<<"; HomogTransInWorld="<<matHomog_aruco_GMR_wrt_drone_GMR<<endl;

        std::cout<<"[DAE-ROS]  Homog_aruco_GMR_wrt_drone_GMR_wYvPuR =\n"<<
                   "    x = "   << x   << " y = "     << y     << " z = "   << z    << endl <<
                   "    yaw = " << yaw*(180.0/M_PI) << " pitch = " << pitch*(180.0/M_PI) << " roll = "<< roll*(180.0/M_PI) << endl;
#endif

        //message
        arucoListMsg.obs.push_back( aruco_msgs::Observation3D() );
        arucoListMsg.obs[i].id = idMarker;
        arucoListMsg.obs[i].x = x;
        arucoListMsg.obs[i].y = y;
        arucoListMsg.obs[i].z = z;
        arucoListMsg.obs[i].yaw   = yaw;
        arucoListMsg.obs[i].pitch = pitch;
        arucoListMsg.obs[i].roll  = roll;
    }



    //end
    return true;
}

*/

bool ArucoEyeROS::publishArucoList()
{

    //publish
    arucoListPubl.publish(arucoListMsg);

    //end
    return 1;
}



int ArucoEyeROS::drawArucoCodes(std::string windowName, int waitingTime, bool drawDetectedCodes, bool draw3DReconstructedCodes)
{
    return MyArucoEye.drawAndDisplayDetectedArucoCodes(windowName, waitingTime, drawDetectedCodes, draw3DReconstructedCodes);
}
