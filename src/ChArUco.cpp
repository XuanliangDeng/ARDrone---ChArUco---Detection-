#include "ChArUco.h"



using namespace std;

ChArUco::ChArUco(int squaresX_, int squaresY_, int dictionaryId_, float squareLength_, float markerLength_,
                 bool showRejected_, bool refindStrategy_, string cameraFile_, string paramsFile_)
{
    squaresX = squaresX_;
    squaresY = squaresY_;
    dictionaryId = dictionaryId_;
    squareLength = squareLength_;
    markerLength = markerLength_;
    showRejected = showRejected_;
    refindStrategy = refindStrategy_;
    cameraFile = cameraFile_;
    paramsFile = paramsFile_;
}

bool ChArUco::readCameraParameters( string filename, Mat &camMatrix, Mat &distCoeffs)
{
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}

bool ChArUco::readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params)
{
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

void ChArUco::process(const sensor_msgs::ImageConstPtr& cam_image)
{

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(cam_image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Ptr<aruco::Dictionary> dictionary =
            aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    float axisLength = 0.5f * ((float)min(squaresX, squaresY) * (squareLength));

    Ptr<aruco::CharucoBoard> charucoboard =
            aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();


    Mat image_rgb,imageCopy;
    cv_ptr->image.copyTo(image_rgb);

    double tick = (double)getTickCount();

    vector< int > markerIds, charucoIds;
// This is the most important part about ORB SLAM the vector vector markerCorners
    vector< vector< Point2f > > markerCorners, rejectedMarkers;
    vector< Point2f > charucoCorners;
    Vec3d rvec, tvec;

    aruco::detectMarkers(image_rgb, dictionary, markerCorners, markerIds, detectorParams, rejectedMarkers);
    if (refindStrategy)
        aruco::refineDetectedMarkers(image_rgb, board, markerCorners, markerIds, rejectedMarkers, camMatrix, distCoeffs);

    // interpolate charuco corners
    int interpolatedCorners = 0;
    if (markerIds.size() > 0){
        interpolatedCorners = aruco::interpolateCornersCharuco(markerCorners, markerIds, image_rgb, charucoboard, charucoCorners, charucoIds, camMatrix, distCoeffs);
    }

    // estimate charuco board pose
    bool validPose = false;
    if (camMatrix.total() != 0){
        validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard, camMatrix, distCoeffs, rvec, tvec);

        // Print out
        //std::cout << "rvec = " << rvec <<std::endl;
        //std::cout << "tvec = " << tvec <<std::endl;

    }
// The above part is about marker detection , you should be familiar with the data structure of every detail of above. Debug
    double currentTime = ((double) getTickCount() - tick) / getTickFrequency();
    totalTime += currentTime;
    totalIterations++;
    if (totalIterations % 30 == 0) {


        // This part is about print our information!!!!!!!


        cout << "Marker Detection Start..." << endl;
        for(auto n = markerCorners.begin(); n != markerCorners.end(); n++ )
        {
            cout<<*n<<endl;

        }
        cout<< "Marker Detection End.........."<< endl;


        cout << "Convert point2f to keypoint...." << endl;

        // Convert point2f to keypoints
        vector< KeyPoint > kpts;
        vector< Point2f >::iterator col;
        for (auto row = markerCorners.begin(); row != markerCorners.end(); row++) {
            for (col = row->begin(); col != row->end(); col++) {
                kpts.push_back(KeyPoint(*col, 1.f));
                //cout << "Keypoint = " << *col << endl;
            }
        }



        vector<Point2f> XY;
        Point2f xy;
        for (int i=0; i<kpts.size(); i++){
            xy = kpts[i].pt;
            XY.push_back(xy);
        }
        cout << XY  << endl;


        cout << "Keypoint size = " << kpts.size() << endl;

        /*Debug part

        cout << "These are marker Ids!"<< endl;
        for(auto xx = markerIds.begin(); xx != markerIds.end(); xx++ )
        {
            cout<<*xx<<endl;

        }
        cout << "Charuco Detection start..."<< endl;
        for(auto m = charucoCorners.begin(); m != charucoCorners.end(); m++ )
        {
            cout<<*m<<endl;
        }

        cout<< "Charuco corners End.........."<< endl;

        cout << "These are chcorner Ids!"<< endl;
        for(auto yy = charucoIds.begin(); yy != charucoIds.end(); yy++ )
        {
            cout<<*yy<<endl;

        }
        Print out information ends!!!!!!!!
        cout << "Detection Time = " << currentTime * 1000 << " ms "
        << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;*/
    }

    // draw results
    image_rgb.copyTo(imageCopy);
    if (markerIds.size() > 0) {
        aruco::drawDetectedMarkers(imageCopy, markerCorners);
    }

    if (showRejected && rejectedMarkers.size() > 0)
        aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(), Scalar(100, 0, 255));

    if (interpolatedCorners > 0) {
        Scalar color;
        color = Scalar(255, 0, 0);
        aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
    }

    if (validPose)
        aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

    imshow("chArUco_board_detection", imageCopy);
    cvWaitKey(1);

}





