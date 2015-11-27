//////////////////////////////////////////////////////
//  arucoEye.h
//
//  Created on: Jul 3, 2013
//      Author: joselusl
//
//  Last modification on: Oct 28, 2013
//      Author: joselusl
//
//////////////////////////////////////////////////////


#ifndef _ARUCO_EYE_H
#define _ARUCO_EYE_H



//I/O stream
//std::cout
#include <iostream>

//String
//std::string, std::getline()
#include <string>

//String stream
//std::istringstream
#include <sstream>

//File Stream
//std::ofstream, std::ifstream
#include <fstream>

//Vector
//std::vector
#include <vector>

//Math
//sin(), cos()
#include <cmath>



//Opencv
#include <opencv2/opencv.hpp>

//Aruco
#include "aruco.h"

//PUGIXML
#include "pugixml.hpp"

//#include "cvg_utils_library.h"


//#define VERBOSE_ARUCO_EYE






/////////////////////////////////////////
// Class homogeneusTransformation
//
//   Description
//
/////////////////////////////////////////
class homogeneusTransformation
{
public:
    //Mat homog <-> Tvec & RVec
    static int createMatHomogFromVecs(cv::Mat& MatHomog, cv::Mat TransVec, cv::Mat RotVec);
    static int calculateVecsFromMatHomog(cv::Mat& TransVec, cv::Mat& RotVec, cv::Mat MatHomog);

    //Otros
    static int calculateTraVecYPRFromMatHomog(cv::Mat *YPRVec, cv::Mat *TraVec, cv::Mat MatHomog);
    static int calculateMatHomogFromYPRVecTraVec(cv::Mat* MatHomog, cv::Mat YPRVec, cv::Mat TraVec);
    static int calculateMatHomogFromRotMatTraVec(cv::Mat* MatHomog, cv::Mat RotMat, cv::Mat TraVec);

};



/////////////////////////////////////////
// Class ArucoCodeDefinition
//
//   Description
//
/////////////////////////////////////////
class ArucoCodeDefinition
{
protected:
    int id; //aruco id
    double size; //lado del aruco en metros
    bool flagSizeSet;
		
public:
    ArucoCodeDefinition();
    ~ArucoCodeDefinition();

public:
    int setArucoCode(int idIn, double sizeIn);
    int getId() const;
    bool isSizeSet() const;
    double getSize() const;
  
};



/////////////////////////////////////////
// Class ArucoListDefinition
//
//   Description
//
/////////////////////////////////////////
class ArucoListDefinition
{
protected:
    std::vector<ArucoCodeDefinition> ArucoListDefinitionCodes;


public:
    ArucoListDefinition();
    ~ArucoListDefinition();

    int loadListFromXmlFile(std::string filePath);

    bool isCodeByIdInList(int idCode) const;
    bool isCodeWithSizeInList(int idCode) const;

    int getCodeSizeById(double &sizeCode, int idCode);

    int getCodeSize(double &sizeCode, unsigned int codePosition);

};


class ArucoMarker
{
public:
    ArucoMarker();

protected:
    aruco::Marker TheMarker;
public:
    aruco::Marker getMarker() const;
    int setMarker(aruco::Marker& TheMarker);

protected:
    bool flag3DReconstructed;
public:
    bool is3DReconstructed() const;
    int set3DReconstructed(bool flag3DReconstructed);




};



/////////////////////////////////////////
// Class ArucoEye
//
//   Description
//
/////////////////////////////////////////
class ArucoEye
{
    //Images
protected:
    bool flagNewImage;
    cv::Mat InputImage;
    cv::Mat OutputImage;

    ///////// Configs
private:
    //Configs
    aruco::CameraParameters TheCameraParameters;


    //Markers List
    std::vector<ArucoMarker> TheMarkers;

    aruco::MarkerDetector MDetector;


protected:
    ArucoListDefinition ArucoList;


public:
    ArucoEye();
    ~ArucoEye();

    int init();
    int close();

public:
    //configure
    int configure(std::string arucoListFile, std::string cameraParametersFile);

    //setting aruco list
public:
    int setArucoList(std::string arucoListFile);

    //Setting the camera parameters
public:
    int setCameraParameters(std::string filename);

    //Configure ArucoDetector
public:
    int configureArucoDetector(bool enableErosion=false, aruco::MarkerDetector::ThresholdMethods thresholdMethod=aruco::MarkerDetector::ADPT_THRES, double ThresParam1=7, double ThresParam2=7, aruco::MarkerDetector::CornerRefinementMethod methodCornerRefinement=aruco::MarkerDetector::LINES, int ThePyrDownLevel=0, float minSize=0.03, float maxSize=0.5);

    //run
public:
    int run(unsigned int &numCodesDetected, unsigned int &numCodesReconstructed);






    //Drawing
public:
    int drawDetectedArucoCodes(bool drawDetectedCodes=true, bool draw3DReconstructedCodes=true);
    char drawAndDisplayDetectedArucoCodes(std::string windowName, int waitingTime=1, bool drawDetectedCodes=true, bool draw3DReconstructedCodes=true);


public:
    int setInputImage(cv::Mat InputImageIn);
    int getOutputImage(cv::Mat &OutputImageOut);


public:
    int getMarkersList(std::vector<ArucoMarker> &TheMarkers3D);

};




#endif
