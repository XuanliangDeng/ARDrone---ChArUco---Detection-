#include "ChArUco.h"


int main(int argc, char **argv){

    ChArUco *obj = new ChArUco();
    obj->readCameraParameters("/home/parallels/opencv_contrib/modules/aruco/samples/calibrate_camera.yml", obj->camMatrix, obj->distCoeffs);
    obj->readDetectorParameters("/home/parallels/opencv_contrib/modules/aruco/samples/detector_params.yml", obj->detectorParams);

    ros::init(argc,argv,"Display_Images");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/front/image_raw",1000,&ChArUco::process, obj);

    ros::spin();
    return 0;
}
