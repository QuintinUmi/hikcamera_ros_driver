#ifndef _MSG_HIKCAMERA_GRABBING_H_
#define _MSG_HIKCAMERA_GRABBING_H_

#include "MvCameraControl.h"

typedef struct _CAMERA_INIT_INFO_
{
    void* pUser;
    unsigned char* pData;
    unsigned int nDataSize;
    MV_FRAME_OUT stImageInfo;

    unsigned char* pImageCache;

}CAMERA_INIT_INFO;


#endif 
