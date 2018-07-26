# Camera pose estimation with ROS and ChArUco markers
This is summer project 2018, supervised by Professor Patricio A. Vela &amp; Yipu Zhao.  
The main tools are OpenCV, ChArUco, ROS. 

## Device description
The drone I use for this project is Parrot AR Drone 2.0. The ROS version is Kinetic with OpenCV 3.3.1(including OpenCV_contrib). ChArUco stands for Chessboard + ArUco = ChArUco. The functions related to ChArUco marker detection are already included in OpenCV(after 3.0 version). 

## Software dependencies
* Ubuntu - tested with Ubuntu 16.04
* ROS - tested with Kinetic version [ROS Kinetic Installation](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* OpenCV - tested with OpenCV 3.3.1 with corresponding opencv_contrib
For opencv_contrib and OpenCV3 installation, these instructions on this website could be very helpful(could skip those steps about virtual environment)   [OpenCV Installation](https://www.learnopencv.com/install-opencv3-on-ubuntu/)

## Camera calibration
Before any tests, the first key step is to calibrate the camera we use. The function for this is included in directory and the file name is calibrate_camera_charuco.cpp.
```C++
$ cd ~/opencv_contrib/modules/aruco/samples/
```
To calibrate camera on ARDrone, we have two options: recorded video/live camera.  
Here we recorded a short video for calibration. The chessboard and video for calibration are included in /Data directory. The detector parameters camera calibration result are stored in /Params directory.  
* First compile the cpp file, go to directory where your calibrate_camera_charuco.cpp is and use the following command
```C++
g++ -std=c++11 calibrate_camera_charuco.cpp `pkg-config --libs --cflags opencv` -o calibrate_camera_charuco
```
* Second calibrate your own camera, use the following command, if everything goes on well, you should get a yaml file containg camera parameters
```C++
./calibrate_camera_charuco -d=DICTIONARY_NUM --dp=PATH_TO_DETECTOR_YAMLFILE -h=MARKER_NUM_Y --ml=MARKER_LENGTH -sl=SQUARE_LENGTH -w=MARKER_NUM_X calibrate_camera.yml -v=PATH_TO_VIDEO 
```
For me, my code look like this
```C++
./calibrate_camera_charuco -d=14 --dp='/home/parallels/opencv_contrib/modules/aruco/samples/detector_params.yml' -h=7 --ml=0.025 -sl=0.034 -w=5 calibrate_camera.yml -v='/home/parallels/ardrone_videos/output.avi' 
```



