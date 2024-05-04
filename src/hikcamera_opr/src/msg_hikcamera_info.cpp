#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>

#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"        

#include "MvCameraControl.h"
#include "CameraParams.h"

#include "hikcamera.h"

sensor_msgs::CameraInfo getCameraInfo(ros::NodeHandle rosHandle)
{
    cv::String cameraIntrinsicsPath;
    struct _cameraInfoMat
    {
        int width;
        int height;
        cv::Mat cameraMatrix;
        cv::Mat disCoffes;

    }cameraInfoMat;
    
    
    sensor_msgs::CameraInfo cameraInfo;

    rosHandle.param("camera_instrinsics_path_yaml", cameraIntrinsicsPath, cv::String("~/caliberation_param.yaml"));
    cv::FileStorage fs(cameraIntrinsicsPath, cv::FileStorage::READ);
    fs["imageWidth"] >> cameraInfoMat.width;
    fs["imageHeight"] >> cameraInfoMat.D;
    fs["cameraMatrix"] >> cameraInfo.K;
    fs["disCoffes"] >> cameraInfo.D;
    fs.release();

    cameraInfo.height 
    rosHandle.param("ros_image_publish_rate", loopRate, 100);
    vector<double> D{0.000094, -0.011701, 0.000383, -0.000507, 0.000000};
    boost::array<double, 9> K = {
        404.005825, 0.000000, 335.580380,
        0.000000, 404.368809, 250.727020,
        0.000000, 0.000000, 1.000000  
    };
    
     boost::array<double, 12> P = {
        402.124725, 0.000000, 335.482488, 0.000000,
        0.000000, 403.765045, 250.954855, 0.000000,
        0.000000, 0.000000, 1.000000, 0.000000
    };
    boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    cam.height = 480;
    cam.width = 640;
    cam.distortion_model = "plumb_bob";
    cam.D = D;
    cam.K = K;
    cam.P = P;
    cam.R = r;
    cam.binning_x = 0;
    cam.binning_y = 0;
    cam.header.frame_id = "camera";  //frame_id为camera，也就是相机名字
    cam.header.stamp = ros::Time::now();
    cam.header.stamp.nsec = 0;
    return cam;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "msg_camera_info");
    ros::NodeHandle rosHandle;
    
    int nRet = MV_OK;

    sensor_msgs::CameraInfo cameraInfo;
    ros::Publisher infoPub = rosHandle.advertise<sensor_msgs::CameraInfo>("/msg_camera/info", 1000);

    // ros::Publisher imgPub = rosHandle.advertise<sensor_msgs::Image>("/msg_camera/img", 100);
    // ros::Publisher msgPub = rosHandle.advertise<std_msgs::String>("/msg_camera/std_msgs", 100);

    int loopRate = 1;
    ros::Rate loop_rate(loopRate);
    
    

    while(ros::ok()){


        imgPub.publish(imgMsg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // hikCamera.stop_grabbing();


    return 0;
}
