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

        int setCameraParam();

        CAMERA_INIT_INFO start_grabbing();

        sensor_msgs::ImagePtr grabbingOneFrame();

        int freeFrameCache();

        void stop_grabbing();

        void printParam();


    private:

        ros::NodeHandle rosHandle;

        void *camHandle;
        int camIndex;

        CAMERA_INIT_INFO cameraInitInfo;

        int nRet;
        int width;
        int height;
        int Offset_x;
        int Offset_y;
        bool FrameRateEnable;
        int FrameRate;
        int ExposureTime;
        int GainAuto;

        int bayerCvtQuality;
};


#endif 
