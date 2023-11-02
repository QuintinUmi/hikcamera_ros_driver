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

#include "msg_hikcamera_grabbing.h"


bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
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

CAMERA_INIT_INFO camera_init(unsigned int camIndex = 0){

    int nRet = MV_OK;

    void* handle = NULL;

    CAMERA_INIT_INFO ERROR_STATUS;
    ERROR_STATUS.pUser = NULL;
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        return ERROR_STATUS;
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
        return ERROR_STATUS;
    }

    // printf("Please Intput camera index: ");
    // unsigned int nIndex = 0;
    // scanf("%d", &nIndex);
    unsigned int nIndex = camIndex;

    if (nIndex >= stDeviceList.nDeviceNum)
    {
        printf("Intput error!\n");
        return ERROR_STATUS;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return ERROR_STATUS;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        return ERROR_STATUS;
    }
		
    // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
    if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
    {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
        if (nPacketSize > 0)
        {
            nRet = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
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
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return ERROR_STATUS;
    }

    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        return ERROR_STATUS;
    }

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    // if (MV_OK != nRet)
    // {
    //     printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    //     return NULL;
    // }

    MV_FRAME_OUT_INFO_EX stFrameInfo = {0};
    memset(&stFrameInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);

    // if (NULL == pData)
    // {
    //     return NULL;
    // }

    unsigned int nDataSize = stParam.nCurValue;

    CAMERA_INIT_INFO cameraInitInfo = {0};
    cameraInitInfo.pData = pData;
    cameraInitInfo.pUser = handle;
    cameraInitInfo.nDataSize = nDataSize;
    cameraInitInfo.stImageInfo.pBufAddr = pData;
    cameraInitInfo.stImageInfo.stFrameInfo = stFrameInfo;

    // printf("%p\n", handle);

    return cameraInitInfo;
}



sensor_msgs::ImagePtr grabbingOneFrame(CAMERA_INIT_INFO *cameraInitInfo)
{
    int nRet = MV_OK;

    void* pUser = cameraInitInfo->pUser;
    MV_FRAME_OUT stImageInfo = cameraInitInfo->stImageInfo;
    unsigned int nDataSize = cameraInitInfo->nDataSize;

    // nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 1000);
    nRet = MV_CC_GetOneFrameTimeout(pUser, stImageInfo.pBufAddr, nDataSize, &(stImageInfo.stFrameInfo), 1000);
    if (nRet == MV_OK)
    {
        printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d], nFrameLen[%d]\n", 
        stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nFrameNum, stImageInfo.stFrameInfo.nFrameLen);
        
        // for(int i = 0; i <= stImageInfo.stFrameInfo.nFrameLen; i++) printf("pData[%d] = %d\n", i, stImageInfo.pBufAddr[i]);
    }
    else
    {
        printf("Get Image fail! nRet [0x%x]\n", nRet);
    }



    unsigned char *pDataForRGB = NULL;

    pDataForRGB = (unsigned char*)malloc(stImageInfo.stFrameInfo.nWidth * stImageInfo.stFrameInfo.nHeight * 4 + 2048);
    if (NULL == pDataForRGB)
    {
        printf("Error1");
    }

    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
    stConvertParam.nWidth = stImageInfo.stFrameInfo.nWidth;
    stConvertParam.nHeight = stImageInfo.stFrameInfo.nHeight;
    stConvertParam.pSrcData = stImageInfo.pBufAddr;
    stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLen;
    stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    stConvertParam.pDstBuffer = pDataForRGB;
    stConvertParam.nDstBufferSize = stImageInfo.stFrameInfo.nWidth * stImageInfo.stFrameInfo.nHeight *  4 + 2048;

    nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
    if (MV_OK != nRet)
    {
        printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
    }

    cv::Mat cvImage(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC3, pDataForRGB);
    if (cvImage.empty())  
    {
        printf("Could not open or find the image\n");
    }

    sensor_msgs::ImagePtr pRosImg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cvImage).toImageMsg();

    cameraInitInfo->pUser = pUser;
    cameraInitInfo->stImageInfo = stImageInfo;
    cameraInitInfo->pIamgeCache = pDataForRGB;

    return pRosImg;
}


void stop_grabbing(CAMERA_INIT_INFO cameraInitInfo)
{
    int nRet = MV_OK;
    void *handle = cameraInitInfo.pUser;

    // 停止取流
    // end grab image
    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
    }
    if (cameraInitInfo.pData)
    {
        free(cameraInitInfo.pData);	
        cameraInitInfo.pData = NULL;
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "msg_camera_grabbing");
    ros::NodeHandle rosHandle;

    // ros::Publisher imgPub = rosHandle.advertise<sensor_msgs::Image>("/msg_camera/img", 100);
    ros::Publisher msgPub = rosHandle.advertise<std_msgs::String>("/msg_camera/std_msgs", 100);

    sensor_msgs::Image imgOneFrame;
    CAMERA_INIT_INFO cameraInitInfo = camera_init(0);
    
    image_transport::ImageTransport imgIt(rosHandle);

    //topic name is /camera_front/image_color,the publish message queue size is 1.
    image_transport::Publisher imgPub = imgIt.advertise("/msg_camera/img", 1);

    int nRet = MV_OK;
    void *pUser = cameraInitInfo.pUser;
    unsigned char *pData = cameraInitInfo.pData;
    unsigned int nDataSize = cameraInitInfo.nDataSize;
    MV_FRAME_OUT* stImageInfo;

    cv::Mat cvImage;
    sensor_msgs::ImagePtr imgMsg;

    std_msgs::String test_msg;

    ros::Rate loop_rate(80);

    while(ros::ok()){

        imgMsg = grabbingOneFrame(&cameraInitInfo);
        free(cameraInitInfo.pIamgeCache);

        imgPub.publish(imgMsg);
        ros::spinOnce();
        // loop_rate.sleep();
    }

    stop_grabbing(cameraInitInfo);


    return 0;
}
