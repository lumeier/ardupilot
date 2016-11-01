/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <linux/videodev2.h>

#include <AP_HAL/OpticalFlow.h>
#include <AP_Math/AP_Math.h>

#include "AP_HAL_Linux.h"
#include "CameraSensor.h"
#include "Flow_PX4.h"
#include "VideoIn.h"

//Include libraries for OpenCV support
#include <iostream>
#include <fstream>
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "trackFeatures.h"

//RangeFinder
#include <AP_RangeFinder/AP_RangeFinder.h>
//Inetial Sensors
#include <AP_InertialSensor/AP_InertialSensor.h>

// Include header for VIO
#include "vio/VIO.h"
#include "assert.h"
#include "vio/SLAM_includes.h"
#include "vio/matlab_consts.h"
#include "vio/Precision.h"
#include "vio/SLAM_types.h"


class Linux::OpticalFlow_Onboard : public AP_HAL::OpticalFlow {
public:
    void init(AP_HAL::OpticalFlow::Gyro_Cb);
    bool read(AP_HAL::OpticalFlow::Data_Frame& frame);
    FILE * fp;
    std::ofstream log_file;

private:
    //Added for VIO
    Vector3f _vioflow(cv::Mat, int count);
    void _init_vioflow();
    std::vector<FloatType> map;
    std::vector<AnchorPose> anchor_poses;
    RobotState robot_state;
    std::vector<int> update_vec_;
    CameraParameters vertcameraParams;
    DUOParameters cameraParams;
    NoiseParameters noiseParams;
    VIOParameters vioParams;
    double fps;
    int vision_subsample;
    bool auto_subsample;

    void _run_optflow();
    static void *_read_thread(void *arg);
    VideoIn* _videoin;
    VideoIn::Frame _last_video_frame;
    PWM_Sysfs_Base* _pwm;
    CameraSensor* _camerasensor;
    Flow_PX4* _flow;
    pthread_t _thread;
    pthread_mutex_t _mutex;
    bool _initialized;
    bool _data_available;
    bool _crop_by_software;
    bool _shrink_by_software;
    uint32_t _camera_output_width;
    uint32_t _camera_output_height;
    uint32_t _width;
    uint32_t _height;
    uint32_t _format;
    uint32_t _bytesperline;
    uint32_t _sizeimage;
    float _pixel_flow_x_integral;
    float _pixel_flow_y_integral;
    float _gyro_x_integral;
    float _gyro_y_integral;
    uint32_t _integration_timespan;
    uint8_t _surface_quality;
    AP_HAL::OpticalFlow::Gyro_Cb _get_sensors;
    Vector3f _last_gyro_rate;
};
