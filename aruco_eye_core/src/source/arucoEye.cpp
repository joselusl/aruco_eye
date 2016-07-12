//////////////////////////////////////////////////////
//  arucoEye.cpp
//
//  Created on:
//      Author: joselusl
//
//  Last modification on:
//      Author: joselusl
//
//////////////////////////////////////////////////////


#include "aruco_eye_core/arucoEye.h"



using namespace std;




/////////////////// ArucoCodeDefinition ////////////////
ArucoCodeDefinition::ArucoCodeDefinition()
{
    id=-1;
    size=-1;
    flagSizeSet=false;
    //type=-1;

    return;
}

ArucoCodeDefinition::~ArucoCodeDefinition()
{

    return;
}


int ArucoCodeDefinition::setArucoCode(int idIn, float sizeIn)
{
    id=idIn;
    if(sizeIn<=0)
    {
        size=-1.0;
        flagSizeSet=false;
    }
    else
    {
        size=sizeIn;
        flagSizeSet=true;
    }
    return 0;
}

int ArucoCodeDefinition::getId() const
{
    return id;
}

bool ArucoCodeDefinition::isSizeSet() const
{
    return flagSizeSet;
}

float ArucoCodeDefinition::getSize() const
{
    return size;
}




/////////////// ArucoListDefinition ///////////////////

ArucoListDefinition::ArucoListDefinition()
{
    return;
}


ArucoListDefinition::~ArucoListDefinition()
{
    ArucoListDefinitionCodes.clear();

    return;
}

int ArucoListDefinition::loadListFromXmlFile(std::string filePath)
{
    int error=0;

    //XML document
    pugi::xml_document doc;
    std::ifstream nameFile(filePath.c_str());
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result)
    {
        cout<<"I cannot open xml file: "<<filePath<<endl;
        return 1;
    }


    ///Aruco lists
    pugi::xml_node aruco = doc.child("arucoList");


    std::string readingValue;
    int id;
    double size;

    ArucoCodeDefinition ArucoAux;

    for(pugi::xml_node arucoMarker = aruco.child("arucoMarker");arucoMarker; arucoMarker = arucoMarker.next_sibling("arucoMarker"))
    {
        readingValue=arucoMarker.child_value("id");
        istringstream convertid(readingValue);
        convertid>>id;

        readingValue=arucoMarker.child_value("size");
        if(readingValue=="")
            size=-1.0;
        else
        {
            istringstream convertsize(readingValue);
            convertsize>>size;
        }

        ArucoAux.setArucoCode(id,size);

        //Añadimos. TODO check that already doesnt exist
        if(!isCodeByIdInList(id))
        {
            ArucoListDefinitionCodes.push_back(ArucoAux);
        }
        else
        {
            error=2;
        }
    }

    return error;
}



bool ArucoListDefinition::isCodeByIdInList(int idCode) const
{
    for(unsigned int i=0;i<ArucoListDefinitionCodes.size();i++)
    {
        if(ArucoListDefinitionCodes[i].getId()==idCode)
            return true;
    }

    return false;
}

bool ArucoListDefinition::isCodeWithSizeInList(int idCode) const
{
    for(unsigned int i=0;i<ArucoListDefinitionCodes.size();i++)
    {
        if(ArucoListDefinitionCodes[i].getId()==idCode)
        {
            if(ArucoListDefinitionCodes[i].isSizeSet())
                return true;
            else
                return false;
        }

    }

    return false;

}


int ArucoListDefinition::getCodeSizeById(float &sizeCode, int idCode) const
{
    sizeCode=-1.0;

    for(unsigned int i=0;i<ArucoListDefinitionCodes.size();i++)
    {
        if(ArucoListDefinitionCodes[i].getId()==idCode)
        {
            if(ArucoListDefinitionCodes[i].isSizeSet())
            {
                sizeCode=ArucoListDefinitionCodes[i].getSize();
                return 0;
            }
            else
            {
                sizeCode=-1.0;
                return 1;
            }
        }
    }
    return 2;
}

int ArucoListDefinition::getCodeSize(float &sizeCode, unsigned int codePosition) const
{
    if(codePosition<ArucoListDefinitionCodes.size())
    {
        sizeCode=ArucoListDefinitionCodes[codePosition].getSize();
        return 0;
    }
    else
    {
        sizeCode=-1.0;
        return 1;
    }
    return 2;
}





///////////////////////////////////////////
/// \brief ArucoMarker
//////////////////////////////////////////////
ArucoMarker::ArucoMarker()
{
    flag3DReconstructed=false;
    return;
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
    return;
}

ArucoEye::~ArucoEye()
{
    close();
    return;
}

int ArucoEye::init()
{
    flagNewImage=false;
    flagCameraParametersSet=false;

    if(configureArucoDetector())
        return 1;

    return 0;
}

int ArucoEye::close()
{
    return 0;
}


int ArucoEye::configure(std::string arucoListFile, std::string cameraParametersFile)
{
    int error=0;

    //Camera parameters
    if(setCameraParameters(cameraParametersFile))
    {
        error=2;
    }

    //Aruco List
    if(setArucoList(arucoListFile))
        error=3;


    //Aruco Detector
    if(configureArucoDetector())
        error=3;



    return error;
}

bool ArucoEye::isTheCameraParametersSet() const
{
    return this->flagCameraParametersSet;
}

int ArucoEye::setCameraParameters(std::string filename)
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


int ArucoEye::setCameraParameters(aruco::CameraParameters camParam)
{
    TheCameraParameters=camParam;
    if(TheCameraParameters.isValid())
    {
        flagCameraParametersSet=true;
        return 0;
    }
    return 1;
}


int ArucoEye::setArucoList(std::string arucoListFile)
{
    if(arucoListFile.empty())
        return 2;
    if ( !boost::filesystem::exists( arucoListFile ) )
    {
        std::cout << "Can't find the file: "<< arucoListFile << std::endl;
        return 1;
    }
    else
        return ArucoList.loadListFromXmlFile(arucoListFile);
}


//Configure ArucoDetector
int ArucoEye::configureArucoDetector(bool enableErosion, aruco::MarkerDetector::ThresholdMethods thresholdMethod, double ThresParam1, double ThresParam2, aruco::MarkerDetector::CornerRefinementMethod methodCornerRefinement, int ThePyrDownLevel, float minSize, float maxSize)
{
    //Marker detector
    if(ThePyrDownLevel>0)
        MDetector.pyrDown(ThePyrDownLevel);


    //Threshold
    //Params
    //MDetector.getThresholdParams( ThresParam1,ThresParam2);
    MDetector.setThresholdParams(ThresParam1,ThresParam2);

    //Method
    //enum ThresholdMethods {FIXED_THRES,ADPT_THRES,CANNY};
    MDetector.setThresholdMethod(thresholdMethod);
    //ThresholdMethods = MDetector.getThresholdMethod();


    //Corner refinement method
    //enum CornerRefinementMethod {NONE,HARRIS,SUBPIX,LINES};
    MDetector.setCornerRefinementMethod(methodCornerRefinement);


    //Erosion for chesstable
    MDetector.enableErosion(enableErosion);


    //Specifies the min and max sizes of the markers as a fraction of the image size. By size we mean the maximum of cols and rows.
    //@param min size of the contour to consider a possible marker as valid (0,1]
    //@param max size of the contour to consider a possible marker as valid [0,1)
    MDetector.setMinMaxSize(minSize,maxSize);

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
    MDetector.detect(InputImage, TheDetectedMarkers);//,TheCameraParameters);

    //Count
    numCodesDetected=0;
    numCodesReconstructed=0;

    TheMarkers.clear();

    for(int i=0; i<TheDetectedMarkers.size(); i++)
    {
        //Code is in the list
        if(ArucoList.isCodeByIdInList(TheDetectedMarkers[i].id))
        {
            ArucoMarker TheArucoMarker;

            // 2D Detection
            numCodesDetected++;

            // 3D Reconstruction
            //Code is in the list
            if(ArucoList.isCodeWithSizeInList(TheDetectedMarkers[i].id))
            {
                if(TheCameraParameters.isValid())
                {
                    float theMarkerSize;
                    if(!ArucoList.getCodeSizeById(theMarkerSize, TheDetectedMarkers[i].id))
                    {
                        TheDetectedMarkers[i].calculateExtrinsics(theMarkerSize,TheCameraParameters);
                        TheArucoMarker.set3DReconstructed(true);
                        numCodesReconstructed++;
                    }
                }
#ifdef VERBOSE_ARUCO_EYE
                else
                    cout<<"[AE] Invalid camera parameters. Unable to reconstruct 3d"<<endl;
#endif
            }

            // Push in the List
            TheArucoMarker.setMarker(TheDetectedMarkers[i]);
            TheMarkers.push_back(TheArucoMarker);
        }
        else
            continue;
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
        for(unsigned int i=0;i<TheMarkers.size();i++)
        {
            TheMarkers[i].getMarker().draw(OutputImage,cv::Scalar(0,0,255),true);
        }
    }


    //draw a 3d in each marker if there is 3d info
    if(draw3DReconstructedCodes)
    {
        if(TheCameraParameters.isValid())
        {
            for(unsigned int i=0;i<TheMarkers.size();i++)
            {
                if(TheMarkers[i].is3DReconstructed())
                {
                    aruco::Marker TheArucoMarker=TheMarkers[i].getMarker();
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

