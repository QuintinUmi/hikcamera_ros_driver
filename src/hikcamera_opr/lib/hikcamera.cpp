#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>

#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"        

#include "image_transport/image_transport.h"

#include "MvCameraControl.h"
#include "CameraParams.h"

#include "hikcamera.h"


using namespace hikcamera_opr;
using namespace livox_ros;


HikCamera::HikCamera(){}
HikCamera::HikCamera(ros::NodeHandle &nodeHandle, int cameraIndex)
{
    this->rosHandle = nodeHandle;
    this->camIndex = cameraIndex;
    this->nRet = MV_OK;
    
    this->rosHandle.param("width", this->hikcamera_param.width, 1280);
    this->rosHandle.param("height", this->hikcamera_param.height, 1024);
    this->rosHandle.param("Offset_x", this->hikcamera_param.Offset_x, 0);
    this->rosHandle.param("Offset_y", this->hikcamera_param.Offset_y, 0);
    this->rosHandle.param("FrameRateEnable", this->hikcamera_param.FrameRateEnable, false);
    this->rosHandle.param("FrameRate", this->hikcamera_param.FrameRate, 80);


    this->rosHandle.param("TriggerMode", this->hikcamera_param.TriggerMode, 0);
    this->rosHandle.param("LineSelector", this->hikcamera_param.LineSelector, 0);
    this->rosHandle.param("LineMode", this->hikcamera_param.LineMode, 0);
    this->rosHandle.param("LineSource", this->hikcamera_param.LineSource, 5);

    this->rosHandle.param("StrobeEnable", this->hikcamera_param.StrobeEnable, true);
    this->rosHandle.param("StrobeLineDelay", this->hikcamera_param.StrobeLineDelay, 0);
    this->rosHandle.param("StrobeLinePreDelay", this->hikcamera_param.StrobeLineDelay, 0);


    this->rosHandle.param("ExposureAuto", this->hikcamera_param.ExposureAuto, 0);
    this->rosHandle.param("ExposureTimeUpper", this->hikcamera_param.ExposureTimeUpper, 35000);
    this->rosHandle.param("ExposureTimeLower", this->hikcamera_param.ExposureTimeLower, 25000);
    this->rosHandle.param("ExposureTime", this->hikcamera_param.ExposureTime, 30000);

    this->rosHandle.param("GainAuto", this->hikcamera_param.GainAuto, 0);
    this->rosHandle.param("Gain", this->hikcamera_param.Gain, (float)15.0);


    this->rosHandle.param("BayerCvtQuality", this->hikcamera_param.bayerCvtQuality, 1);


    this->rosHandle.param("camera_instrinsics_path_yaml", this->cameraIntrinsicsPath, cv::String("~/caliberation_param.yaml"));
    this->rosHandle.param("undistortion", this->undistortion, false);
    this->rosHandle.param("alpha", this->alpha, 0.0);
    this->rosHandle.param("interpolation", this->interpolation, 1);
    

    this->setCameraIntrinsics(this->cameraIntrinsicsPath);
    

    // if(this->undistortion)
    // {
    //     // cv::FileStorage fs(this->cameraIntrinsicsPath, cv::FileStorage::READ);
    //     // fs["cameraMatrix"] >> this->cameraMatrix;
    //     // fs["disCoffes"] >> this->disCoffes;
    //     this->setCameraIntrinsics(this->rosHandle);
    // }

    // rosHandle.param("ros-publication-rate", this->rosPublicationRate, 100);
}
HikCamera::HikCamera(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable, int FrameRate, int ExposureTime, 
                    int GainAuto, int bayerCvtQuality, bool undistortion, double alpha)
{
    this->hikcamera_param.width = width;
    this->hikcamera_param.height = height;
    this->hikcamera_param.Offset_x = Offset_x;
    this->hikcamera_param.Offset_y = Offset_y;
    this->hikcamera_param.FrameRateEnable = FrameRateEnable;
    this->hikcamera_param.FrameRate = FrameRate;
    this->hikcamera_param.ExposureTime = ExposureTime;
    this->hikcamera_param.GainAuto = GainAuto;
    this->hikcamera_param.bayerCvtQuality = bayerCvtQuality;
    this->undistortion = undistortion;
    this->alpha = alpha;
}
HikCamera::~HikCamera()
{
    stop_grab();
}

bool HikCamera::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

CAMERA_INFO HikCamera::camera_init()
{
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    }

    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            } 
            PrintDeviceInfo(pDeviceInfo);            
        }  
    } 
    else
    {
        printf("Find No Devices!\n");
    }

    // printf("Please Intput camera index: ");
    // unsigned int nIndex = 0;
    // scanf("%d", &nIndex);
    unsigned int nIndex = camIndex;

    if (nIndex >= stDeviceList.nDeviceNum)
    {
        printf("Intput error!\n");
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&camHandle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(camHandle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    }
		
    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
    if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
    {
        int nPacketSize = MV_CC_GetOptimalPacketSize(camHandle);
        if (nPacketSize > 0)
        {
            nRet = MV_CC_SetIntValue(camHandle,"GevSCPSPacketSize",nPacketSize);
            if(nRet != MV_OK)
            {
                printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
            }
        }
        else
        {
            printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
        }
    }
		
    // 设置触发模式为off
    // set trigger mode as off
    nRet = MV_CC_SetEnumValue(camHandle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
    }

    cameraInfo.pUser = camHandle;

    return cameraInfo;
}


int HikCamera::setCameraParam()
{
    int nRet = MV_OK;

    // 设置 Width
    nRet = MV_CC_SetIntValue(camHandle, "Width", this->hikcamera_param.width);
    if (nRet != MV_OK) {
        printf("Error setting Width: %d\n", nRet);
        return nRet;
    }

    // 设置 Height
    nRet = MV_CC_SetIntValue(camHandle, "Height", this->hikcamera_param.height);
    if (nRet != MV_OK) {
        printf("Error setting Height: %d\n", nRet);
        return nRet;
    }

    // 设置 OffsetX
    nRet = MV_CC_SetIntValue(camHandle, "OffsetX", this->hikcamera_param.Offset_x);
    if (nRet != MV_OK) {
        printf("Error setting OffsetX: %d\n", nRet);
        return nRet;
    }

    // 设置 OffsetY
    nRet = MV_CC_SetIntValue(camHandle, "OffsetY", this->hikcamera_param.Offset_y);
    if (nRet != MV_OK) {
        printf("Error setting OffsetY: %d\n", nRet);
        return nRet;
    }


    // 设置 FrameRateEnable
    nRet = MV_CC_SetBoolValue(camHandle, "AcquisitionFrameRateEnable", this->hikcamera_param.FrameRateEnable);
    if (nRet != MV_OK) {
        printf("Error setting FrameRateEnable: %d\n", nRet);
        return nRet;
    }

    // 设置 FrameRate
    nRet = MV_CC_SetFloatValue(camHandle, "AcquisitionFrameRate", this->hikcamera_param.FrameRate);
    if (nRet != MV_OK) {
        printf("Error setting FrameRate: %d\n", nRet);
        return nRet;
    }



    // 设置 TriggerMode
    nRet = MV_CC_SetEnumValue(camHandle, "TriggerMode", this->hikcamera_param.TriggerMode);
    if (nRet != MV_OK) {
        printf("Error setting TriggerMode: %d\n", nRet);
        return nRet;
    }

    if(this->hikcamera_param.TriggerMode == 1)
    {
        // 设置 LineSelector
        nRet = MV_CC_SetEnumValue(camHandle, "LineSelector", this->hikcamera_param.LineSelector);
        if (nRet != MV_OK) {
            printf("Error setting LineSelector: %d\n", nRet);
            return nRet;
        }

        if(this->hikcamera_param.LineSelector == 2)
        {
            nRet = MV_CC_SetEnumValue(camHandle, "LineMode", this->hikcamera_param.LineMode);//仅line2需要设置
            if(nRet != MV_OK)
            {
                printf("Error setting LineMode: %d\n", nRet);
                return nRet;
            }
        }
        
        // nRet = MV_CC_SetEnumValue(camHandle, "LineSource", this->hikcamera_param.LineSource);
        // if(nRet != MV_OK)
        // {
        //     printf("Error setting LineSource: %d\n", nRet);
        //     return nRet;
        // }

        // // 设置 StrobeLineDelay
        // nRet = MV_CC_SetIntValue(camHandle, "LineDelay", this->hikcamera_param.StrobeLineDelay);
        // if (nRet != MV_OK) {
        //     printf("Error setting StrobeLineDelay: %d\n", nRet);
        //     return nRet;
        // }

        // // 设置 StrobeLinePreDelay
        // nRet = MV_CC_SetIntValue(camHandle, "LinePreDelay", this->hikcamera_param.StrobeLinePreDelay);
        // if (nRet != MV_OK) {
        //     printf("Error setting StrobeLinePreDelay: %d\n", nRet);
        //     return nRet;
        // }

        // // 设置 StrobeEnable
        // nRet = MV_CC_SetBoolValue(camHandle, "StrobeEnable", this->hikcamera_param.StrobeEnable);
        // if (nRet != MV_OK) {
        //     printf("Error setting StrobeEnable: %d\n", nRet);
        //     return nRet;
        // }
    }
    


    // 设置 ExposureAuto
    nRet = MV_CC_SetEnumValue(camHandle, "ExposureAuto", this->hikcamera_param.ExposureAuto);
    if (nRet != MV_OK) {
        printf("Error setting ExposureAuto: %d\n", nRet);
        return nRet;
    }

    if(this->hikcamera_param.ExposureAuto != 0)
    {
        // 设置 ExposureTimeUpper
        nRet = MV_CC_SetIntValue(camHandle, "ExposureTimeUpper", this->hikcamera_param.ExposureTimeUpper);
        if (nRet != MV_OK) {
            printf("Error setting ExposureTimeUpper: %d\n", nRet);
            return nRet;
        }

        // 设置 ExposureTimeLower
        nRet = MV_CC_SetIntValue(camHandle, "ExposureTimeLower", this->hikcamera_param.ExposureTimeLower);
        if (nRet != MV_OK) {
            printf("Error setting ExposureTimeLower: %d\n", nRet);
            return nRet;
        }

        // 设置 ExposureTime
        nRet = MV_CC_SetFloatValue(camHandle, "ExposureTime", this->hikcamera_param.ExposureTime);
        if (nRet != MV_OK) {
            printf("Error setting ExposureTime: %d\n", nRet);
            return nRet;
        }

    }


    // 设置 GainAuto
    nRet = MV_CC_SetEnumValue(camHandle, "GainAuto", this->hikcamera_param.GainAuto);
    if (nRet != MV_OK) {
        printf("Error setting GainAuto: %d\n", nRet);
        return nRet;
    }

    // 设置 Gain
    nRet = MV_CC_SetFloatValue(camHandle, "Gain", this->hikcamera_param.Gain);
    if (nRet != MV_OK) {
        printf("Error setting Gain: %d\n", nRet);
        return nRet;
    }


    nRet |= MV_CC_SetBayerCvtQuality(camHandle, hikcamera_param.bayerCvtQuality);


    this->printParam();

    return MV_OK;  
}
int HikCamera::setCameraParam(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable, int FrameRate, int ExposureTime, 
                                int GainAuto, int bayerCvtQuality)
{
    int nRet = MV_OK;

    nRet |= MV_CC_SetIntValue(camHandle, "width", width);
    nRet |= MV_CC_SetIntValue(camHandle, "height", height);
    nRet |= MV_CC_SetIntValue(camHandle, "Offset_x", Offset_x);
    nRet |= MV_CC_SetIntValue(camHandle, "Offset_y", Offset_y);
    nRet |= MV_CC_SetBoolValue(camHandle, "FrameRateEnable", FrameRateEnable);
    nRet |= MV_CC_SetFloatValue(camHandle, "FrameRate", FrameRate);
    nRet |= MV_CC_SetFloatValue(camHandle, "ExposureTime", ExposureTime);
    nRet |= MV_CC_SetEnumValue(camHandle, "GainAuto", GainAuto);

    nRet |= MV_CC_SetBayerCvtQuality(camHandle, bayerCvtQuality);

    this->printParam();

    return MV_OK;
}


bool HikCamera::setCameraIntrinsics(ros::NodeHandle &nodeHandle)
{
    this->rosHandle.param("camera_instrinsics_path_yaml", this->cameraIntrinsicsPath, cv::String("~/caliberation_param.yaml"));
    cv::FileStorage fs(this->cameraIntrinsicsPath, cv::FileStorage::READ);
    
    int imageWidth, imageHeight;
    fs["imageWidth"] >> imageWidth;
    fs["imageHeight"] >> imageHeight;
    this->imageSize = cv::Size(imageWidth, imageHeight);
    fs["cameraMatrix"] >> this->cameraMatrix;
    fs["disCoffes"] >> this->disCoffes;
    
    std::cout << this->cameraMatrix << std::endl << this->disCoffes << std::endl << this->imageSize << std::endl << this->alpha << std::endl;
    this->newCameraMatrix = cv::getOptimalNewCameraMatrix(this->cameraMatrix, this->disCoffes, this->imageSize, this->alpha);
    cv::initUndistortRectifyMap(this->cameraMatrix, this->disCoffes, cv::Mat(), this->newCameraMatrix, this->imageSize, CV_32FC2, this->map1, this->map2);

    fs.release();

    return true;
}
bool HikCamera::setCameraIntrinsics(cv::String cameraIntrinsicsPath)
{
    cv::FileStorage fs(this->cameraIntrinsicsPath, cv::FileStorage::READ);
    
    int imageWidth, imageHeight;
    fs["imageWidth"] >> imageWidth;
    fs["imageHeight"] >> imageHeight;
    this->imageSize = cv::Size(imageWidth, imageHeight);
    fs["cameraMatrix"] >> this->cameraMatrix;
    fs["disCoffes"] >> this->disCoffes;
    
    std::cout << cameraIntrinsicsPath << std::endl  << imageWidth << std::endl << imageHeight << std::endl << this->cameraMatrix << std::endl << this->disCoffes << std::endl;
    printf("test-------------------------------------------------------------\n");
    this->newCameraMatrix = cv::getOptimalNewCameraMatrix(this->cameraMatrix, this->disCoffes, this->imageSize, this->alpha);
    cv::initUndistortRectifyMap(this->cameraMatrix, this->disCoffes, cv::Mat(), this->newCameraMatrix, this->imageSize, CV_32FC2, this->map1, this->map2);

    std::cout << "newCameraMatrix: \n" << this->newCameraMatrix << std::endl;
    std::cout << "newImageSize:" << this->newImageSize << std::endl;

    fs.release();

    return true;
}
bool HikCamera::setCameraIntrinsics(int imageWidth, int imageHeight, cv::Mat cameraMatrix, cv::Mat disCoffes)
{
    this->imageSize = cv::Size(imageWidth, imageHeight);
    this->cameraMatrix = cameraMatrix;
    this->disCoffes = disCoffes;

    this->newCameraMatrix = cv::getOptimalNewCameraMatrix(this->cameraMatrix, this->disCoffes, this->imageSize, this->alpha);
    cv::initUndistortRectifyMap(this->cameraMatrix, this->disCoffes, cv::Mat(), this->newCameraMatrix, this->imageSize, CV_32FC2, this->map1, this->map2);

    std::cout << "newCameraMatrix:" << this->newCameraMatrix << std::endl;
    std::cout << "newImageSize:" << this->newImageSize << std::endl;

    return true;
}




CAMERA_INFO HikCamera::start_grab()
{
    int Ret = MV_OK;
    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(camHandle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_Startgrab fail! nRet [%x]\n", nRet);
    }

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(camHandle, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    }

    MV_FRAME_OUT_INFO_EX stFrameInfo = {0};
    memset(&stFrameInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);

    // if (NULL == pData)
    // {
    //     return NULL;
    // }

    unsigned int nDataSize = stParam.nCurValue;

    cameraInfo.nDataSize = nDataSize;
    cameraInfo.stImageInfo.pBufAddr = pData;
    cameraInfo.stImageInfo.stFrameInfo = stFrameInfo;

    // printf("%p\n", handle);

    return cameraInfo;
}


// sensor_msgs::ImagePtr HikCamera::grabOneFrame2ROS_sync(GPRMC_TIME_STAMP *GPRMC_ptr)
// {
//     void* pUser = cameraInfo.pUser;

//     // Get payload size
//     MVCC_INTVALUE stParam;
//     memset(&stParam, 0, sizeof(MVCC_INTVALUE));
//     int nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
//     if (MV_OK != nRet) {
//         printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
//         return sensor_msgs::ImagePtr(); 
//     }

//     MV_FRAME_OUT stImageInfo = {0};
//     nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 1000); // Timeout set to 1000 ms
//     if (nRet != MV_OK) {
//         printf("Get Image fail! nRet [0x%x]\n", nRet);
//         return sensor_msgs::ImagePtr(); 
//     }

//     auto time_pc_clk = std::chrono::high_resolution_clock::now();
//     int64_t b = GPRMC_ptr->low;
//     // printf("b:%d\n", b);

//     double time_pc = b / 1000000000.0;
//     ros::Time rcv_time = ros::Time(time_pc);
    

//     // Log retrieved frame information
//     std::string debug_msg;
//     debug_msg = "GetOneFrame,nFrameNum[" +
//                   std::to_string(stImageInfo.stFrameInfo.nFrameNum) + "], FrameTime: [" +
//                   std::to_string(rcv_time.toSec()) + "], Width: [" +
//                   std::to_string(stImageInfo.stFrameInfo.nWidth) + "], Height: [" +
//                   std::to_string(stImageInfo.stFrameInfo.nHeight) + "], nFrameLen: [" +
//                   std::to_string(stImageInfo.stFrameInfo.nFrameLen);
//       ROS_INFO_STREAM(debug_msg.c_str());


//     if (stImageInfo.pBufAddr != NULL) {
//         nRet = MV_CC_FreeImageBuffer(pUser, &stImageInfo);
//         if (nRet != MV_OK) {
//             printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
//         }
//     }

//     // Convert Bayer GB format to BGR format
//     cv::Mat imgBayerGB(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
//     cv::Mat imgBGR;
//     cv::cvtColor(imgBayerGB, imgBGR, cv::COLOR_BayerGB2BGR);

//     // Apply undistortion if required
//     cv::Mat cvImageOutput;
//     if (this->undistortion) {
//         cv::remap(imgBGR, cvImageOutput, this->map1, this->map2, this->interpolation);
//     } else {
//         cvImageOutput = imgBGR;
//     }

//     // Convert OpenCV image to ROS message
//     sensor_msgs::ImagePtr pRosImg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvImageOutput).toImageMsg();
//     pRosImg->header.stamp = rcv_time;

//     // Update camera initialization info
//     cameraInfo.pUser = pUser;
//     cameraInfo.stImageInfo = stImageInfo;
//     cameraInfo.pImageCache = stImageInfo.pBufAddr;

//     return pRosImg;
// }
sensor_msgs::ImagePtr HikCamera::grabOneFrame2ROS()
{
    void* pUser = cameraInfo.pUser;

    MV_FRAME_OUT stImageInfo = {0};
    nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 1000); // Timeout set to 1000 ms
    if (nRet != MV_OK) {
        printf("Get Image fail! nRet [0x%x]\n", nRet);
        return sensor_msgs::ImagePtr(); 
    }

    auto time_pc_clk = std::chrono::high_resolution_clock::now();
    double time_pc =
        uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(time_pc_clk.time_since_epoch()).count())
        / 1000000000.0;
    ros::Time rcv_time = ros::Time(time_pc);

    // Log retrieved frame information
    std::string debug_msg;
    debug_msg = "GetOneFrame,nFrameNum[" +
                  std::to_string(stImageInfo.stFrameInfo.nFrameNum) + "], FrameTime: [" +
                  std::to_string(rcv_time.toSec()) + "], Width: [" +
                  std::to_string(stImageInfo.stFrameInfo.nWidth) + "], Height: [" +
                  std::to_string(stImageInfo.stFrameInfo.nHeight) + "], nFrameLen: [" +
                  std::to_string(stImageInfo.stFrameInfo.nFrameLen);
    ROS_INFO_STREAM(debug_msg.c_str());

    if (stImageInfo.pBufAddr != NULL) {
        nRet = MV_CC_FreeImageBuffer(pUser, &stImageInfo);
        if (nRet != MV_OK) {
            printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
        }
    }

    // Convert Bayer GB format to BGR format
    cv::Mat imgBayerGB(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
    cv::Mat imgBGR;
    cv::cvtColor(imgBayerGB, imgBGR, cv::COLOR_BayerGB2BGR);

    // Apply undistortion if required
    cv::Mat cvImageOutput;
    if (this->undistortion) {
        cv::remap(imgBGR, cvImageOutput, this->map1, this->map2, this->interpolation);
    } else {
        cvImageOutput = imgBGR;
    }

    // Convert OpenCV image to ROS message
    sensor_msgs::ImagePtr pRosImg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvImageOutput).toImageMsg();
    pRosImg->header.stamp = rcv_time;

    // Update camera initialization info
    cameraInfo.pUser = pUser;
    cameraInfo.stImageInfo = stImageInfo;
    cameraInfo.pImageCache = stImageInfo.pBufAddr;

    return pRosImg;
}
sensor_msgs::ImagePtr HikCamera::grabOneFrame2ROS(bool undistortion, int interpolation)
{
    void* pUser = cameraInfo.pUser;

    MV_FRAME_OUT stImageInfo = {0};
    nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 1000); // Timeout set to 1000 ms
    if (nRet != MV_OK) {
        printf("Get Image fail! nRet [0x%x]\n", nRet);
        return sensor_msgs::ImagePtr(); 
    }

    auto time_pc_clk = std::chrono::high_resolution_clock::now();
    double time_pc =
        uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(time_pc_clk.time_since_epoch()).count())
        / 1000000000.0;
    ros::Time rcv_time = ros::Time(time_pc);

    // Log retrieved frame information
    std::string debug_msg;
    debug_msg = "GetOneFrame,nFrameNum[" +
                  std::to_string(stImageInfo.stFrameInfo.nFrameNum) + "], FrameTime: [" +
                  std::to_string(rcv_time.toSec()) + "], Width: [" +
                  std::to_string(stImageInfo.stFrameInfo.nWidth) + "], Height: [" +
                  std::to_string(stImageInfo.stFrameInfo.nHeight) + "], nFrameLen: [" +
                  std::to_string(stImageInfo.stFrameInfo.nFrameLen);
      ROS_INFO_STREAM(debug_msg.c_str());

    // Log retrieved frame information
    printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d], nFrameLen[%d]\n", 
           stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight, 
           stImageInfo.stFrameInfo.nFrameNum, stImageInfo.stFrameInfo.nFrameLen);

    if (stImageInfo.pBufAddr != NULL) {
        nRet = MV_CC_FreeImageBuffer(pUser, &stImageInfo);
        if (nRet != MV_OK) {
            printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
        }
    }

    // Convert Bayer GB format to BGR format
    cv::Mat imgBayerGB(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
    cv::Mat imgBGR;
    cv::cvtColor(imgBayerGB, imgBGR, cv::COLOR_BayerGB2BGR);

    // Apply undistortion if required
    cv::Mat cvImageOutput;
    if (undistortion) {
        cv::remap(imgBGR, cvImageOutput, this->map1, this->map2, interpolation);
    } else {
        cvImageOutput = imgBGR;
    }

    // Convert OpenCV image to ROS message
    sensor_msgs::ImagePtr pRosImg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvImageOutput).toImageMsg();
    pRosImg->header.stamp = rcv_time;

    // Update camera initialization info
    cameraInfo.pUser = pUser;
    cameraInfo.stImageInfo = stImageInfo;
    cameraInfo.pImageCache = stImageInfo.pBufAddr;

    return pRosImg;
}


cv::Mat HikCamera::grabOneFrame2Mat()
{

    void* pUser = cameraInfo.pUser;

    MV_FRAME_OUT stImageInfo = {0};
    nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 1000); // Timeout set to 1000 ms
    if (nRet != MV_OK) {
        printf("Get Image fail! nRet [0x%x]\n", nRet);
        return cv::Mat(); 
    }

    auto time_pc_clk = std::chrono::high_resolution_clock::now();
    double time_pc =
    uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(time_pc_clk.time_since_epoch()).count())
    / 1000000000.0;
    ros::Time rcv_time = ros::Time(time_pc);

    std::string debug_msg;
    debug_msg = "GetOneFrame,nFrameNum[" +
                  std::to_string(stImageInfo.stFrameInfo.nFrameNum) + "], FrameTime:" +
                  std::to_string(rcv_time.toSec()) + "], Width:" +
                  std::to_string(stImageInfo.stFrameInfo.nWidth) + "], Height:" +
                  std::to_string(stImageInfo.stFrameInfo.nHeight) + "], nFrameLen:" +
                  std::to_string(stImageInfo.stFrameInfo.nFrameLen);
    ROS_INFO_STREAM(debug_msg.c_str());

    // Log retrieved frame information
    printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d], nFrameLen[%d]\n", 
           stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight, 
           stImageInfo.stFrameInfo.nFrameNum, stImageInfo.stFrameInfo.nFrameLen);

    if (stImageInfo.pBufAddr != NULL) {
        nRet = MV_CC_FreeImageBuffer(pUser, &stImageInfo);
        if (nRet != MV_OK) {
            printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
        }
    }

    // Convert Bayer GB format to BGR format
    cv::Mat imgBayerGB(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
    cv::Mat imgBGR;
    cv::cvtColor(imgBayerGB, imgBGR, cv::COLOR_BayerGB2BGR);

    // Apply undistortion if required
    cv::Mat cvImageOutput;
    if (undistortion) {
        cv::remap(imgBGR, cvImageOutput, this->map1, this->map2, interpolation);
    } else {
        cvImageOutput = imgBGR;
    }

    // Convert OpenCV image to ROS message
    sensor_msgs::ImagePtr pRosImg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvImageOutput).toImageMsg();
    pRosImg->header.stamp = rcv_time;

    // Update camera initialization info
    cameraInfo.pUser = pUser;
    cameraInfo.stImageInfo = stImageInfo;
    cameraInfo.pImageCache = stImageInfo.pBufAddr;

    return cvImageOutput;
}
cv::Mat HikCamera::grabOneFrame2Mat(bool undistortion, int interpolation)
{
    void* pUser = cameraInfo.pUser;

    MV_FRAME_OUT stImageInfo = {0};
    nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 1000); // Timeout set to 1000 ms
    if (nRet != MV_OK) {
        printf("Get Image fail! nRet [0x%x]\n", nRet);
        return cv::Mat(); 
    }

    // Log retrieved frame information
    printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d], nFrameLen[%d]\n", 
           stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight, 
           stImageInfo.stFrameInfo.nFrameNum, stImageInfo.stFrameInfo.nFrameLen);

    if (stImageInfo.pBufAddr != NULL) {
        nRet = MV_CC_FreeImageBuffer(pUser, &stImageInfo);
        if (nRet != MV_OK) {
            printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
        }
    }

    // Convert Bayer GB format to BGR format
    cv::Mat imgBayerGB(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
    cv::Mat imgBGR;
    cv::cvtColor(imgBayerGB, imgBGR, cv::COLOR_BayerGB2BGR);

    // Apply undistortion if required
    cv::Mat cvImageOutput;
    if (undistortion) {
        cv::remap(imgBGR, cvImageOutput, this->map1, this->map2, interpolation);
    } else {
        cvImageOutput = imgBGR;
    }

    // Convert OpenCV image to ROS message
    sensor_msgs::ImagePtr pRosImg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvImageOutput).toImageMsg();

    // Update camera initialization info
    cameraInfo.pUser = pUser;
    cameraInfo.stImageInfo = stImageInfo;
    cameraInfo.pImageCache = stImageInfo.pBufAddr;

    return cvImageOutput;
}


ros::NodeHandle HikCamera::getRosHandler() {
    return this->rosHandle;
}

cv::Mat HikCamera::getNewCameraMatrix()
{
    return this->newCameraMatrix;
}


int HikCamera::freeFrameCache()
{
    free(cameraInfo.pImageCache);
    return MV_OK;
}

void HikCamera::stop_grab()
{
    if(cameraInfo.pUser != NULL)
    {
        void *handle = cameraInfo.pUser;

        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_Stopgrab fail! nRet [%x]\n", nRet);
        }

        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        }
        // if (cameraInfo.pImageCache)
        // {
        //     free(cameraInfo.pImageCache);	
        //     cameraInfo.pImageCache = NULL;
        // }
        // if (cameraInfo.stImageInfo.pBufAddr)
        // {
        //     free(cameraInfo.stImageInfo.pBufAddr);	
        //     cameraInfo.stImageInfo.pBufAddr = NULL;
        // }    
        free(cameraInfo.pUser);
        cameraInfo.pUser = NULL;
    }
    
}




void HikCamera::printParam(){
    printf("width: %d\n", this->hikcamera_param.width);
    printf("height: %d\n", this->hikcamera_param.height);
    printf("Offset_x: %d\n", this->hikcamera_param.Offset_x);
    printf("Offset_y: %d\n", this->hikcamera_param.Offset_y);
    printf("FrameRateEnable: %d\n", this->hikcamera_param.FrameRateEnable);
    printf("FrameRate: %d\n", this->hikcamera_param.FrameRate);

    printf("TriggerMode: %d\n", this->hikcamera_param.TriggerMode);
    printf("LineSelector: %d\n", this->hikcamera_param.LineSelector);
    printf("StrobeEnable: %d\n", this->hikcamera_param.StrobeEnable);
    printf("StrobeLineDelay: %d\n", this->hikcamera_param.StrobeLineDelay);
    printf("StrobeLinePreDelay: %d\n", this->hikcamera_param.StrobeLinePreDelay);

    printf("ExposureAuto: %d\n", this->hikcamera_param.ExposureAuto);
    printf("ExposureTimeUpper: %d\n", this->hikcamera_param.ExposureTimeUpper);
    printf("ExposureTimeLower: %d\n", this->hikcamera_param.ExposureTimeLower);
    printf("ExposureTime: %d\n", this->hikcamera_param.ExposureTime);

    printf("GainAuto: %d\n", this->hikcamera_param.GainAuto);
    printf("Gain: %f\n", this->hikcamera_param.Gain); 

    printf("BayerCvtQuality: %d\n", this->hikcamera_param.bayerCvtQuality);

    printf("undistortion: %d\n", this->undistortion); 
}





int HikCameraSync::initTimeSync(uint32_t freq, uint8_t baudrate_index, uint8_t parity, const std::string& shm_name) {

    freq_ = freq;
    ideal_interval_ = NS(1000000000 / freq_);
    stamp_scale_ = 1.0f;

    is_gps_update_ = false;
    sync_flag_ = false;

    timesync_config_.dev_config.type = kCommDevUart;
    std::strcpy(timesync_config_.dev_config.name, std::string("gprmc_serial").c_str());
    timesync_config_.dev_config.config.uart.baudrate = baudrate_index;
    timesync_config_.dev_config.config.uart.parity = parity;

    timesync_ = TimeSync::GetInstance();
    if (timesync_->InitTimeSync(timesync_config_)) {
      printf("Timesync init fail\n");
      return -1;
    }

    if (timesync_->SetReceiveSyncTimeCb(ReceiveSyncTimeCallback, this)) {
      printf("Set Timesync callback fail\n");
      return -1;
    }

    timesync_->StartTimesync();

    return 0;
}
int HikCameraSync::initCameraSetting(std::string publish_topic)
{
    CAMERA_INFO cameraInfo = this->camera_init();
    int nRet = this->setCameraParam();
    if(MV_OK != nRet){
        printf("Error Occurred in hikcamera parameters setting!\n");
        getchar();
    }

    pub_handler_ = getRosHandler().advertise<sensor_msgs::Image>(publish_topic, 2);

    get_frame_wt_ = std::make_shared<std::thread>(std::bind(&HikCameraSync::GetFrameWorkThread, this));
    queue_process_wt_ = std::make_shared<std::thread>(std::bind(&HikCameraSync::QueueProcessWorkThread, this));

}
int HikCameraSync::startSyncFrameGrab()
{
    SetPublishCb(PublishCallBack, this);

    CAMERA_INFO cameraInfo = this->start_grab();
    seq_ = 0;
    start_get_frame_wt_ = true;
    start_queue_process_wt_ = true;
    
}
int HikCameraSync::stopSyncFrameGrab()
{
    start_get_frame_wt_ = false;
    start_queue_process_wt_ = false;
    exit_get_frame_wt_ = true;
    exit_queue_process_wt_ = true;
    if (get_frame_wt_) {
        get_frame_wt_->join();
        get_frame_wt_ = nullptr;
    }
    if (queue_process_wt_) {
        queue_process_wt_->join();
        queue_process_wt_ = nullptr;
    }

    pub_cb_ = nullptr;
    client_data_ = nullptr;
}


void HikCameraSync::ReceiveSyncTimeCallback(uint64_t gps_time_stamp, 
                                            CLK::time_point gps_rcv_time, 
                                            void *client_data) {

    HikCameraSync *cam_handler = static_cast<HikCameraSync *>(client_data);

    std::lock_guard<std::mutex> lock(cam_handler->sync_mtx_);
    cam_handler->last_gps_rcv_time_ = cam_handler->gps_rcv_time_;
    cam_handler->gps_rcv_time_ = gps_rcv_time;
    cam_handler->last_gps_time_stamp_ = cam_handler->gps_time_stamp_;
    cam_handler->gps_time_stamp_ = gps_time_stamp;
    cam_handler->is_gps_update_ = true;
}
void HikCameraSync::SyncStatusQueryLoop() {

    std::lock_guard<std::mutex> lock(sync_mtx_);
    NS diff = std::chrono::duration_cast<NS>(gps_rcv_time_.time_since_epoch() - last_gps_rcv_time_.time_since_epoch());

    if (CLK::now().time_since_epoch() - gps_rcv_time_.time_since_epoch() > ideal_interval_ * 15) {

        sync_flag_ = 0;
        last_gps_rcv_time_ = gps_rcv_time_;
        gps_rcv_time_ = CLK::time_point();
        return;
    }
    if (-NS(ideal_interval_ * 15) > diff || diff > NS(ideal_interval_ * 15)) {

        sync_flag_ = 0;
        return;
    }

    sync_flag_ = 1;
    stamp_scale_ = NS(1000000000).count() / diff.count();
}


void HikCameraSync::GetFrameWorkThread() {

    static volatile bool is_first_frame = true;
    static CLK::time_point last_image_rcv_time_;

    while (!start_get_frame_wt_) {
        /* waiting to start */
    }

    while(1) {

        void* pUser = cameraInfo.pUser;

        int nRet = MV_OK;

        MV_FRAME_OUT stImageInfo = {0};
        nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 1000); // Timeout set to 1000 ms
        if (nRet != MV_OK) {
            printf("Get Image fail! nRet [0x%x]\n", nRet);
            return; 
        }
        
        auto image_rcv_time = CLK::now();

        if (is_first_frame) {
            ros::Time now_t = ros::Time::now();
            base_time_stamp_ = now_t.toNSec();
            base_time_ = CLK::now();
            seq_ = 0;
            is_first_frame = false;
            last_image_rcv_time_ = image_rcv_time;
        } else {
            NS diff = image_rcv_time - last_image_rcv_time_;
            uint32_t del_seq = (diff + (ideal_interval_ * stamp_scale_) / 2) / (ideal_interval_ * stamp_scale_);
            seq_ += del_seq;
            last_image_rcv_time_ = image_rcv_time;
        }

        // Log retrieved frame information
        std::string debug_msg;
        debug_msg = "GetOneFrame,nFrameNum[" +
                    std::to_string(stImageInfo.stFrameInfo.nFrameNum) + "], FrameRcvTime: [" +
                    std::to_string(std::chrono::duration_cast<NS>(image_rcv_time.time_since_epoch()).count()) + "], Width: [" +
                    std::to_string(stImageInfo.stFrameInfo.nWidth) + "], Height: [" +
                    std::to_string(stImageInfo.stFrameInfo.nHeight) + "], nFrameLen: [" +
                    std::to_string(stImageInfo.stFrameInfo.nFrameLen);
        ROS_INFO_STREAM(debug_msg.c_str());


        if (stImageInfo.pBufAddr != NULL) {
            nRet = MV_CC_FreeImageBuffer(pUser, &stImageInfo);
            if (nRet != MV_OK) {
                printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
            }
        }

        // Convert Bayer GB format to BGR format
        cv::Mat imgBayerGB(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC1, stImageInfo.pBufAddr);
        cv::Mat imgBGR;
        cv::cvtColor(imgBayerGB, imgBGR, cv::COLOR_BayerGB2BGR);

        // Apply undistortion if required
        cv::Mat cvImageOutput;
        if (this->undistortion) {
            cv::remap(imgBGR, cvImageOutput, this->map1, this->map2, this->interpolation);
        } else {
            cvImageOutput = imgBGR;
        }

        {
            std::lock_guard<std::mutex> lock(queue_mtx_);
            queue_.emplace_back(cvImageOutput, seq_, image_rcv_time);
            if (queue_.size() > freq_ * 2) {
                printf("Warning! Out of maximum queue size for process!\n");
                // cvImageOutput;
            }
        }
        

        // Update camera initialization info
        cameraInfo.pUser = pUser;
        cameraInfo.stImageInfo = stImageInfo;
        cameraInfo.pImageCache = stImageInfo.pBufAddr;

        return;
    }
}

void HikCameraSync::QueueProcessWorkThread() {

    static uint64_t last_time_stamp = 0;
    static CLK::time_point last_rcv_time;

    std::unique_lock<std::mutex> sync_lock(sync_mtx_);
    if (!sync_flag_) {
        std::unique_lock<std::mutex> queue_lock(queue_mtx_);
        FramePacket frame_pkg = queue_.front();
        queue_.pop_front(); 
        queue_lock.unlock();
        if (last_time_stamp == 0) {
            last_rcv_time = frame_pkg.rcv_time;
            last_time_stamp = base_time_stamp_;
            return;
        }
        uint64_t time_stamp = last_time_stamp
                            + NS(frame_pkg.rcv_time.time_since_epoch() - last_rcv_time.time_since_epoch()).count() 
                            * stamp_scale_;

        pub_cb_(frame_pkg, time_stamp, this);
        last_time_stamp = time_stamp;
        last_rcv_time = frame_pkg.rcv_time;
           
    } else {
        std::unique_lock<std::mutex> queue_lock(queue_mtx_);
        if(is_gps_update_) {
            NS diff;
            NS min_diff = NS::max();
            uint32_t min_idx = 0;      
            for (int i = 0; i < queue_.size(); i++) {
                FramePacket frame_pkg = queue_[i];
                diff = frame_pkg.rcv_time.time_since_epoch()
                    - (gps_rcv_time_ - offset_serial_time_).time_since_epoch();
                if (diff < NS(0)) {
                    diff = -diff;
                }
                if (diff < min_diff) {
                    min_diff = diff;
                    min_idx = i;
                }
            }

            if (min_diff < ideal_interval_ / 2) {
                FramePacket& frame_pkg = queue_[min_idx];
                frame_pkg.is_sync_base = true;
                frame_pkg.sync_time_stamp = gps_time_stamp_;
            } else {
                queue_cond_var_.wait(queue_lock, [this, queue_size = queue_.size()]{ return queue_.size() > queue_size; });
                FramePacket& frame_pkg = queue_[min_idx + 1];
                diff = frame_pkg.rcv_time.time_since_epoch()
                    - (gps_rcv_time_ - offset_serial_time_).time_since_epoch();
                if (diff < NS(0)) {
                    diff = -diff;
                }
                if (min_diff < ideal_interval_ / 2) {
                    FramePacket& frame_pkg = queue_[min_idx];
                    frame_pkg.is_sync_base = true;
                    frame_pkg.sync_time_stamp = gps_time_stamp_;
                } else {
                    printf("Error in gps time stamp match!\n");
                    return;
                }
            }
            
        }
        sync_lock.unlock();
        
        
        
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void HikCameraSync::PublishCallBack(FramePacket frame_pkt, uint64_t time_stamp, void* client_data) {

    HikCameraSync *cam_handler = static_cast<HikCameraSync *>(client_data);

    ros::Time stamp(time_stamp / 1000000000);
    std::string debug_msg;
        debug_msg = "PublishOneFrame, FrameSeq[" +
            std::to_string(frame_pkt.seq) + "], FrameStamp: [" +
            std::to_string(time_stamp);
        ROS_INFO_STREAM(debug_msg.c_str());

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", *frame_pkt.image).toImageMsg();
    msg->header.seq = frame_pkt.seq;
    msg->header.stamp = stamp;
    cam_handler->pub_handler_.publish(msg);
}