#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"        

#include "image_transport/image_transport.h"

#include "MvCameraControl.h"
#include "CameraParams.h"

#include "hikcamera.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "msg_camera_grabbing");
    ros::NodeHandle rosHandle;
    HikCamera hikCamera(rosHandle, 0);
    
    int nRet = MV_OK;
    sensor_msgs::Image imgOneFrame;
    CAMERA_INIT_INFO cameraInitInfo = hikCamera.camera_init();

    image_transport::ImageTransport imgIt(rosHandle);
    image_transport::Publisher imgPub = imgIt.advertise("/msg_camera/img_stream", 1);

    // ros::Publisher imgPub = rosHandle.advertise<sensor_msgs::Image>("/msg_camera/img", 100);
    ros::Publisher msgPub = rosHandle.advertise<std_msgs::String>("/msg_camera/std_msgs", 100);

    nRet = hikCamera.setCameraParam();
    if(MV_OK != nRet){
        printf("There is something wrong with hikcamera parameters setting!\n");
        getchar();
    }
    cameraInitInfo = hikCamera.start_grabbing();
    

    void *pUser = cameraInitInfo.pUser;
    unsigned int nDataSize = cameraInitInfo.nDataSize;
    MV_FRAME_OUT* stImageInfo;
    sensor_msgs::ImagePtr imgMsg;

    int loopRate;
    rosHandle.param("ros_image_publish_rate", loopRate, 100);
    ros::Rate loop_rate(loopRate);
    
    

    while(ros::ok()){

        imgMsg = hikCamera.grabbingOneFrame2ROS();
        hikCamera.freeFrameCache();
        

        imgPub.publish(imgMsg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // hikCamera.stop_grabbing();


    return 0;
}
