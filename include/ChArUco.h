//
// Created by parallels on 7/16/18.
//

#ifndef DETECTION_CPP_CHARUCO_H
#define DETECTION_CPP_CHARUCO_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <iostream>
#include <ctime>
#include <string>

using namespace cv;

class ChArUco {
public:

    int squaresX, squaresY, dictionaryId, totalIterations = 0;
    float squareLength, markerLength, totalTime = 0;
    bool showRejected, refindStrategy;
    std::string paramsFile, cameraFile;
    Mat camMatrix, distCoeffs;
    Ptr <aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

    //ChArUco(int squaresX_, int squareY_, int dictionaryId_, float squareLength_, float markerLength_,
    //      bool showRejected_, bool refindStrategy_, string cameraFile_, string paramsFile_)
    ChArUco(int squaresX_ = 5, int squaresY_ = 7, int dictionaryId_= 14, float squareLength_ = 0.078, float markerLength_ = 0.058,
            bool showRejected_ = false, bool refindStrategy_ = false,
            std::string cameraFile_ = "/home/parallels/opencv_contrib/modules/aruco/samples/calibrate_camera.yml",
            std::string paramsFile_ = "/home/parallels/opencv_contrib/modules/aruco/samples/detector_params.yml");

    ~ChArUco() {};

    bool readCameraParameters(std::string filename, Mat &camMatrix, Mat &distCoeffs);

    bool readDetectorParameters(std::string filename, Ptr <aruco::DetectorParameters> &params);

    void process(const sensor_msgs::ImageConstPtr &cam_image);
};

#endif //DETECTION_CPP_CHARUCO_H

