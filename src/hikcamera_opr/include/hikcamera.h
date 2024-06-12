#ifndef HIKCAMERA_H_
#define HIKCAMERA_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>

#include "MvCameraControl.h"

#include "hikcameraDataType.h"
#include "shm_handler/shm_handler.h"
#include "timesync/timesync.h"


using namespace shm_handler;
using namespace livox_ros;

namespace hikcamera_opr
{
    class HikCamera
    {
        public:

            struct _HIKCAMERA_PARAM_{

                int width;
                int height;
                int Offset_x;
                int Offset_y;
                bool FrameRateEnable;
                int FrameRate;

                int TriggerMode;
                int LineSelector;
                int LineMode;
                int LineSource;

                bool StrobeEnable;
                int StrobeLineDelay;
                int StrobeLinePreDelay;

                int ExposureAuto;
                int ExposureTimeUpper;
                int ExposureTimeLower;
                int ExposureTime;

                float Gain;
                int GainAuto;

                int bayerCvtQuality;

            };

        public:

            HikCamera();
            HikCamera(ros::NodeHandle &nodeHandle, int cameraIndex);
            HikCamera(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable = true, int FrameRate = 80, int ExposureTime = 5000, 
                        int GainAuto = 2, int bayerCvtQuality = 1, bool undistortion = false, double alpha = 1.0);
            virtual ~HikCamera();


            bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);


            CAMERA_INFO camera_init();


            int setCameraParam();
            int setCameraParam(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable, int FrameRate, int ExposureTime, 
                        int GainAuto, int bayerCvtQuality = 1);


            bool setCameraIntrinsics(ros::NodeHandle &nodeHandle);
            bool setCameraIntrinsics(cv::String cameraIntrinsicsPath);
            bool setCameraIntrinsics(int imageWidth, int imageHeight, cv::Mat cameraMatrix, cv::Mat disCoffes = cv::Mat());


            CAMERA_INFO start_grab();

            sensor_msgs::ImagePtr grabOneFrame2ROS();
            sensor_msgs::ImagePtr grabOneFrame2ROS(bool undistortion, int interpolation = 1);

            cv::Mat grabOneFrame2Mat();
            cv::Mat grabOneFrame2Mat(bool undistortion, int interpolation = 1);

            cv::Mat getNewCameraMatrix();

            int freeFrameCache();

            void stop_grab();

            void printParam();

        protected:
            CAMERA_INFO cameraInfo;

            _HIKCAMERA_PARAM_ hikcamera_param;

            cv::String cameraIntrinsicsPath;
            bool undistortion;
            int interpolation;

            cv::Size imageSize;
            cv::Mat cameraMatrix;
            cv::Mat disCoffes;
            double alpha;
            cv::Mat newCameraMatrix;
            cv::Size newImageSize;
            cv::Mat map1;
            cv::Mat map2;

        private:

            ros::NodeHandle rosHandle;

            void *camHandle;
            int camIndex;

            int nRet;

    };


    class HikCameraSync : public HikCamera {
        
        public:

            typedef struct TIME_STAMP
            {
                int64_t seq;
                int64_t base_time;
            }TIME_STAMP;

            struct FramePacket {
                std::shared_ptr<cv::Mat> image;
                std::chrono::high_resolution_clock::time_point rcv_time;

                FramePacket(const std::shared_ptr<cv::Mat>& img, std::chrono::high_resolution_clock::time_point image_time_stamp)
                    : image(img), rcv_time(image_time_stamp) {}
            };

            HikCameraSync() : HikCamera(), exit_get_frame_wt_(false), start_get_frame_wt_(false) {};
            HikCameraSync(ros::NodeHandle &nodeHandle, int cameraIndex) : HikCamera(nodeHandle, cameraIndex), 
                                                                        exit_get_frame_wt_(false), start_get_frame_wt_(false) {};
            HikCameraSync(int width, int height, int Offset_x, int Offset_y, bool FrameRateEnable = true, int FrameRate = 80, int ExposureTime = 5000, 
                        int GainAuto = 2, int bayerCvtQuality = 1, bool undistortion = false, double alpha = 1.0)
                        : HikCamera(width, height, Offset_x, Offset_y, FrameRateEnable, FrameRate, ExposureTime, 
                        GainAuto, bayerCvtQuality , undistortion, alpha), 
                        exit_get_frame_wt_(false), start_get_frame_wt_(false) {};

            ~HikCameraSync() override = default;

            int initTimeSync(uint8_t baudrate_index, uint8_t parity, const std::string& shm_name);
            int initCameraSetting();
            int startSyncFrameGrab();
            int stopSyncFrameGrab();


        private:
            // std::shared_ptr<UserUart> uart_;
            std::shared_ptr<ShmHandler<TIME_STAMP>> shm_;

            TimeSync *timesync_;
            TimeSyncConfig timesync_config_;
            std::mutex config_mutex_;

            int64_t gps_time_ns_;

            std::shared_ptr<std::thread> get_frame_wt_;
            volatile bool exit_get_frame_wt_;
            volatile bool start_get_frame_wt_;

            std::mutex sync_mtx_;

            static void ReceiveSyncTimeCallback(uint64_t gps_time_ns, void *client_data);
            void GetFrameWorkThread();
    };
}




#endif 
