//////////////////////////////////////////////////////
//  arucoEye.cpp
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: claudiocimarelli
//
//////////////////////////////////////////////////////


#include "aruco_eye_core/arucoEye.h"



using namespace std;


///////////////////////////////////////////
/// \brief ArucoMarker
//////////////////////////////////////////////
ArucoMarker::ArucoMarker()
{
    flag3DReconstructed=false;
}

aruco::Marker ArucoMarker::getMarker() const
{
    return TheMarker;
}
int ArucoMarker::setMarker(aruco::Marker& TheMarker)
{
    this->TheMarker=TheMarker;
    return 0;
}

bool ArucoMarker::is3DReconstructed() const
{
    return flag3DReconstructed;
}

int ArucoMarker::set3DReconstructed(bool flag3DReconstructed)
{
    this->flag3DReconstructed=flag3DReconstructed;
    return 0;
}





//////////////// Aruco Eye /////////////////////////
ArucoEye::ArucoEye()
{
    init();
}

ArucoEye::~ArucoEye()
{
    close();
}

int ArucoEye::init()
{
    flagNewImage=false;
    flagCameraParametersSet=false;

//    if(configureArucoDetector())
//        return 1;

    return 0;
}

int ArucoEye::close()
{
    return 0;
}


int ArucoEye::configure(std::string & dictionary, std::string & cameraParametersFile)
{
    int error=0;

    //Camera parameters
    if(setCameraParameters(cameraParametersFile))
    {
        error=2;
    }

    setDictionary(dictionary);

    return error;
}

bool ArucoEye::isTheCameraParametersSet() const
{
    return this->flagCameraParametersSet;
}

int ArucoEye::setCameraParameters(std::string & filename)
{
    if(filename.empty())
        return 2;
    if ( !boost::filesystem::exists( filename ) )
    {
        std::cout << "Can't find the file: "<< filename << std::endl;
        return 1;
    }
    else
    {
        TheCameraParameters.readFromXMLFile(filename);
        if(TheCameraParameters.isValid())
        {
            flagCameraParametersSet=true;
            return 0;
        }
        return 1;
    }
    return 2;
}


int ArucoEye::setCameraParameters(aruco::CameraParameters & camParam)
{
    TheCameraParameters=camParam;
    if(TheCameraParameters.isValid())
    {
        flagCameraParametersSet=true;
        return 0;
    }
    return 1;
}

int ArucoEye::setDictionary(std::string &dictionary)
{
    MDetector.setDictionary(dictionary);
    return 0;
}


int ArucoEye::run(unsigned int &numCodesDetected, unsigned int &numCodesReconstructed)
{
    //Checks
    if(!flagNewImage)
        return 1;

    if(InputImage.size().height==0 || InputImage.size().width==0)
        return 2;

    //Detection of markers in the image passed
    std::vector<aruco::Marker> TheDetectedMarkers;
    TheDetectedMarkers = MDetector.detect(InputImage);//,TheCameraParameters);

    //Count
    numCodesDetected=0;
    numCodesReconstructed=0;

    TheMarkers.clear();

    for(auto & TheDetectedMarker : TheDetectedMarkers)
    {
        ArucoMarker TheArucoMarker;
        // 2D Detection
        numCodesDetected++;
        if(TheCameraParameters.isValid())
        {
        TheDetectedMarker.calculateExtrinsics(TheMarkerSize,TheCameraParameters);
        TheArucoMarker.set3DReconstructed(true);
        numCodesReconstructed++;
        }
#ifdef VERBOSE_ARUCO_EYE
        else
            cout<<"[AE] Invalid camera parameters. Unable to reconstruct 3d"<<endl;
#endif
        // Push in the List
        TheArucoMarker.setMarker(TheDetectedMarker);
        TheMarkers.push_back(TheArucoMarker);
    }
    flagNewImage=false;

    return 0;
}

int ArucoEye::drawDetectedArucoCodes(bool drawDetectedCodes, bool draw3DReconstructedCodes)
{
    //////// Drawings //////

    //Checks
    if(InputImage.size().height==0 || InputImage.size().width==0)
        return 1;


    //Copy
    InputImage.copyTo(OutputImage);


    //print marker info and draw the markers in image
    if(drawDetectedCodes)
    {
        for(auto & TheMarker : TheMarkers)
        {
            TheMarker.getMarker().draw(OutputImage,cv::Scalar(0,0,255),true);
        }
    }


    //draw a 3d in each marker if there is 3d info
    if(draw3DReconstructedCodes)
    {
        if(TheCameraParameters.isValid())
        {
            for(auto & TheMarker : TheMarkers)
            {
                if(TheMarker.is3DReconstructed())
                {
                    aruco::Marker TheArucoMarker=TheMarker.getMarker();
                    //aruco::CvDrawingUtils::draw3dCube(OutputImage,TheMarkers[i],TheCameraParameters);
                    aruco::CvDrawingUtils::draw3dAxis(OutputImage, TheArucoMarker, TheCameraParameters);
                }
            }
        }
    }

    return 0;

}

char ArucoEye::displayDetectedArucoCodes(std::string windowName, int waitingTime)
{
    //show input with augmented information and  the thresholded image
    cv::imshow(windowName,OutputImage);

    //end
    return cv::waitKey(waitingTime);//wait for key to be pressed
}


char ArucoEye::drawAndDisplayDetectedArucoCodes(std::string windowName, int waitingTime, bool drawDetectedCodes, bool draw3DReconstructedCodes)
{
    // Draw
    if(!drawDetectedArucoCodes(drawDetectedCodes, draw3DReconstructedCodes))
        return '0';

    // Display
    return displayDetectedArucoCodes(windowName, waitingTime);
}



int ArucoEye::setInputImage(cv::Mat InputImageIn)
{
    flagNewImage=true;
    InputImageIn.copyTo(InputImage);
    return 0;
}


int ArucoEye::getOutputImage(cv::Mat &OutputImageOut) const
{
    OutputImage.copyTo(OutputImageOut);
    return 0;
}


int ArucoEye::getMarkersList(std::vector<ArucoMarker> &TheMarkers) const
{
    TheMarkers=this->TheMarkers;
    return 0;
}

int ArucoEye::setMarkersList(const std::vector<ArucoMarker> TheMarkers)
{
    this->TheMarkers=TheMarkers;
    return 0;
}

int ArucoEye::addMarkerToMarkersList(ArucoMarker TheMarker)
{
    this->TheMarkers.push_back(TheMarker);
    return 0;
}

int ArucoEye::clearMarkersList()
{
    this->TheMarkers.clear();
    return 0;
}

int ArucoEye::setMarkerSize(float sizeInMeters) {
    TheMarkerSize=sizeInMeters;
    return 0;
}

