#ifndef HIKCAMERA_H_
#define HIKCAMERA_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "MvCameraControl.h"

#include "hikcameraDataType.h"


class HikCamera
{
    public:

        HikCamera();
        HikCamera(ros::NodeHandle &nodeHandle, int cameraIndex);
        HikCamera(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable = true, int FrameRate = 80, int ExposureTime = 5000, 
                    int GainAuto = 2, int bayerCvtQuality = 1);
        ~HikCamera();


        bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);


        CAMERA_INIT_INFO camera_init();


        int setCameraParam();
        int setCameraParam(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable = true, int FrameRate = 80, int ExposureTime = 5000, 
                    int GainAuto = 2, int bayerCvtQuality = 1);


        bool setCameraIntrinsics(ros::NodeHandle &nodeHandle);
        bool setCameraIntrinsics(cv::String cameraIntrinsicsPath);
        bool setCameraIntrinsics(cv::Mat cameraMatrix, cv::Mat disCoffes);


        CAMERA_INIT_INFO start_grabbing();

        sensor_msgs::ImagePtr grabbingOneFrame2ROS(bool undistortion = false);

        cv::Mat grabbingOneFrame2Mat(bool undistortion = false);

        cv::Mat getNewCameraMatrix();

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

        cv::String cameraIntrinsicsPath;
        bool undistortion;

        cv::Mat cameraMatrix;
        cv::Mat disCoffes;
        cv::Mat newCameraMatrix;
};


#endif 
