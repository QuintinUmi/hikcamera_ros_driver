#ifndef HIKCAMERA_H_
#define HIKCAMERA_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "MvCameraControl.h"

#include "hikcameraDataType.h"


class HikCamera
{
    public:

        HikCamera(ros::NodeHandle &nodeHandle, int cameraIndex);
        ~HikCamera();

        bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);

        CAMERA_INIT_INFO camera_init();

        sensor_msgs::ImagePtr grabbingOneFrame();

        int freeFrameCache();

        void stop_grabbing();


    private:

        ros::NodeHandle rosHandle;

        void *camHandle;
        int camIndex;

        CAMERA_INIT_INFO cameraInitInfo;

        int nRet;
        int width;
        int height;
        int offset_x;
        int offset_y;
        bool frameRateEnable;
        int frameRate;
        int exposureTime;
        int gainAuto;
};


#endif 
