
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

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP ||\
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE ||\
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#include "OpticalFlow_Onboard.h"

#include <fcntl.h>
#include <linux/v4l2-mediabus.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <deque>



//Gyro & Accel Variables
Vector3f gyro_rate;
Vector2f flow_rate;
Vector3f accel;
Vector3f gyro;
float dt;
uint32_t range;
Vector3f vio_vel;
float tic_predict_old =0.0;
uint16_t range_cm;
//Variables and Parameters for velocity moving average filter
int n_avg=1;
std::deque<double> avg_bin_x;
double avg_x=0;
std::deque<double> avg_bin_y;
double avg_y=0;
std::deque<Vector3f> gyro_delay_cont;
std::deque<Vector3f> accel_delay_cont;
int gyro_delay=2;



#include <AP_AHRS/AP_AHRS.h>
#include "CameraSensor_Mt9v117.h"
#include "GPIO.h"
#include "PWM_Sysfs.h"

#define OPTICAL_FLOW_ONBOARD_RTPRIO 11

extern const AP_HAL::HAL& hal;

// Add own Hal object to plot debug vars:
const AP_HAL::HAL& hal_debug = AP_HAL::get_HAL();
uint32_t now = AP_HAL::millis();


//Simulation start time for Logging & image_counter
uint32_t start_time=0;
uint32_t runtime=0;
uint32_t image_count=0;
uint32_t timestamp=0;
int vio_count=0;

using namespace Linux;
VIO vio;


void OpticalFlow_Onboard::init(AP_HAL::OpticalFlow::Gyro_Cb get_sensors)
{

    //Uncomment to Log Sensor Data and Raw Image Data
    //Empty earlier logfile and write header
    //std::ofstream log_file;
    // log_file.open ("/dev/shm/vio_log.csv");
    // log_file << "timestamp,image_nr,vel.x,vel.y,vel.z,gyro.x,gyro.y,gyro.z,range\n";
    // //log_file.close();
    // printf("csv initialized\n");
    //
    // std::string img_path="/data/ftp/internal_000/vio_logs/images/log_img.bin";
    // fp = fopen(img_path.c_str(), "w");



    vio_vel[0]=0;
    vio_vel[1]=0;
    vio_vel[2]=0;
    printf("Initialize OptFlow");

    //Set Start Time
    start_time=AP_HAL::millis();
    tic_predict_old=start_time/1000.;

    // Initialize VioFlow
    _init_vioflow();

    uint32_t top, left;
    uint32_t crop_width, crop_height;
    uint32_t memtype = V4L2_MEMORY_MMAP;
    unsigned int nbufs = 0;
    int ret;
    pthread_attr_t attr;
    struct sched_param param = {
        .sched_priority = OPTICAL_FLOW_ONBOARD_RTPRIO
    };

    if (_initialized) {
        return;
    }

    _get_sensors = get_sensors;
    _videoin = new VideoIn;
    const char* device_path = HAL_OPTFLOW_ONBOARD_VDEV_PATH;
    memtype = V4L2_MEMORY_MMAP;
    nbufs = HAL_OPTFLOW_ONBOARD_NBUFS;
    // _width = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
    // _height = HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT;
    _width = HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH;
    _height = HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT;
    crop_width = HAL_OPTFLOW_ONBOARD_CROP_WIDTH;
    crop_height = HAL_OPTFLOW_ONBOARD_CROP_HEIGHT;
    top = 0;
    /* make the image square by cropping to YxY, removing the lateral edges */
    left = (HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH -
            HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT) / 2;

    if (device_path == NULL ||
        !_videoin->open_device(device_path, memtype)) {
        AP_HAL::panic("OpticalFlow_Onboard: couldn't open "
                      "video device");
    }

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
    _pwm = new PWM_Sysfs_Bebop(BEBOP_CAMV_PWM);
    _pwm->set_freq(BEBOP_CAMV_PWM_FREQ);
    _pwm->enable(true);

    _camerasensor = new CameraSensor_Mt9v117(HAL_OPTFLOW_ONBOARD_SUBDEV_PATH,
                                             hal.i2c, 0x5D, MT9V117_QVGA,
                                             BEBOP_GPIO_CAMV_NRST,
                                             BEBOP_CAMV_PWM_FREQ);
    if (!_camerasensor->set_format(HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH,
                                   HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT,
                                   V4L2_MBUS_FMT_UYVY8_2X8)) {
        AP_HAL::panic("OpticalFlow_Onboard: couldn't set subdev fmt\n");
    }
    _format = V4L2_PIX_FMT_NV12;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE ||\
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    std::vector<uint32_t> pixel_formats;

    _videoin->get_pixel_formats(&pixel_formats);

    for (uint32_t px_fmt : pixel_formats) {
        if (px_fmt == V4L2_PIX_FMT_NV12 || px_fmt == V4L2_PIX_FMT_GREY) {
            _format = px_fmt;
            break;
        }

        /* if V4L2_PIX_FMT_YUYV format is found we still iterate through
         * the vector since the other formats need no conversions. */
        if (px_fmt == V4L2_PIX_FMT_YUYV) {
            _format = px_fmt;
        }
    }
#endif

    if (!_videoin->set_format(&_width, &_height, &_format, &_bytesperline,
                              &_sizeimage)) {
        AP_HAL::panic("OpticalFlow_Onboard: couldn't set video format");
    }

    if (_format != V4L2_PIX_FMT_NV12 && _format != V4L2_PIX_FMT_GREY &&
        _format != V4L2_PIX_FMT_YUYV) {
        AP_HAL::panic("OpticalFlow_Onboard: format not supported\n");
    }

    if (_width == HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH &&
        _height == HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT) {
        _shrink_by_software = false;
    } else {
        /* here we store the actual camera output width and height to use
         * them later on to software shrink each frame. */
        _shrink_by_software = true;
        _camera_output_width = _width;
        _camera_output_height = _height;

        /* we set these values here in order to the calculations be correct
         * (such as PX4 init) even though we shrink each frame later on. */
        _width = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
        _height = HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT;
        _bytesperline = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
    }

    if (_videoin->set_crop(left, top, crop_width, crop_height)) {
        _crop_by_software = false;
    } else {
        _crop_by_software = true;

        if (!_shrink_by_software) {
            /* here we store the actual camera output width and height to use
             * them later on to software crop each frame. */
            _camera_output_width = _width;
            _camera_output_height = _height;

            /* we set these values here in order to the calculations be correct
             * (such as PX4 init) even though we crop each frame later on. */
            _width = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
            _height = HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT;
            _bytesperline = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
        }
    }

    if (!_videoin->allocate_buffers(nbufs)) {
        AP_HAL::panic("OpticalFlow_Onboard: couldn't allocate video buffers");
    }

    _videoin->prepare_capture();

    /* Use px4 algorithm for optical flow */
    _flow = new Flow_PX4(_width, _bytesperline,
                         HAL_FLOW_PX4_MAX_FLOW_PIXEL,
                         HAL_FLOW_PX4_BOTTOM_FLOW_FEATURE_THRESHOLD,
                         HAL_FLOW_PX4_BOTTOM_FLOW_VALUE_THRESHOLD);

    /* Create the thread that will be waiting for frames
     * Initialize thread and mutex */
    ret = pthread_mutex_init(&_mutex, NULL);
    if (ret != 0) {
        AP_HAL::panic("OpticalFlow_Onboard: failed to init mutex");
    }

    ret = pthread_attr_init(&attr);
    if (ret != 0) {
        AP_HAL::panic("OpticalFlow_Onboard: failed to init attr");
    }
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    ret = pthread_create(&_thread, &attr, _read_thread, this);
    if (ret != 0) {
        AP_HAL::panic("OpticalFlow_Onboard: failed to create thread");
    }

    _initialized = true;
}

bool OpticalFlow_Onboard::read(AP_HAL::OpticalFlow::Data_Frame& frame)
{
    bool ret;

    pthread_mutex_lock(&_mutex);
    if (!_data_available) {
        ret = false;
        goto end;
    }
    frame.pixel_flow_x_integral = _pixel_flow_x_integral;
    frame.pixel_flow_y_integral = _pixel_flow_y_integral;
    frame.gyro_x_integral = _gyro_x_integral;
    frame.gyro_y_integral = _gyro_y_integral;
    frame.delta_time = _integration_timespan;
    frame.quality = _surface_quality;
    _integration_timespan = 0;
    _pixel_flow_x_integral = 0;
    _pixel_flow_y_integral = 0;
    _gyro_x_integral = 0;
    _gyro_y_integral = 0;
    _data_available = false;
    ret = true;
end:
    pthread_mutex_unlock(&_mutex);
    return ret;
}

void *OpticalFlow_Onboard::_read_thread(void *arg)
{
    OpticalFlow_Onboard *optflow_onboard = (OpticalFlow_Onboard *) arg;

    optflow_onboard->_run_optflow();
    return NULL;
}


Vector3f OpticalFlow_Onboard::_vioflow(cv::Mat vertcam_frame,int count)
{
  Vector3f vel_output;
  double att_output[4];
  //Initialize
    VIOMeasurements meas;
    int sample_step_pred=1;
    int sample_step_upd=1;

    std::vector<FloatType> z_all_l(matlab_consts::numTrackFeatures * 2, 0.0);
    std::vector<FloatType> z_all_r(matlab_consts::numTrackFeatures * 2, 0.0);
    std::vector<cv::Point2f> features_l(matlab_consts::numTrackFeatures);
    std::vector<cv::Point2f> features_r(matlab_consts::numTrackFeatures);
    std::vector<FloatType> delayedStatus(matlab_consts::numTrackFeatures);
    //**********************************************************************
    // SLAM prediciton
    //**********************************************************************
    //Gyro will be integrated later, no prediction before that
    //Accelerometer is not used, so no prediction

    //Get measurements from Gyro
    if (!(count % sample_step_pred))
    {

    meas.gyr[0]=gyro_delay_cont[0].x;
    meas.gyr[1]=gyro_delay_cont[0].y;
    meas.gyr[2]=gyro_delay_cont[0].z;
    meas.acc[0]=accel.x;
    meas.acc[1]=accel.y;
    meas.acc[2]=accel.z;
    meas.sonar=0.1;

    float tic_predict = AP_HAL::millis()/1000.;
    dt=tic_predict-tic_predict_old;
    tic_predict_old=tic_predict;

    for (int i_pred=0;i_pred<1;i_pred++)
    {
      vio.predict(meas,dt);
    }

    }
    //*********************************************************************
    // Point tracking
    //*********************************************************************

    if (!(count % sample_step_upd))
    {
    int tic_flow = AP_HAL::millis();
    trackFeatures(vertcam_frame,vertcam_frame,features_l,features_r,update_vec_,stereo);
    int duration_flow = (AP_HAL::millis() - tic_flow);

    for (int i = 0; i < features_l.size(); i++) {
        if (i<48){

      }
        z_all_l[2*i + 0] = features_l[i].x;
        z_all_l[2*i + 1] = features_l[i].y;

        //Dummy: There is no right image
        z_all_r[2*i + 0] = features_r[i].x;
        z_all_r[2*i + 1] = features_r[i].y;
    }
    // //*********************************************************************
    // // SLAM update
    // //*********************************************************************

    meas.sonar=range_cm/100.;
    if (meas.sonar<0.1)
      {meas.sonar=0.1;}

    vio.setParams(cameraParams, noiseParams, vioParams);
    vio.update(update_vec_, z_all_l, z_all_r, robot_state, map, anchor_poses, delayedStatus,meas);
    //in camera x direction
    avg_bin_x.push_back(robot_state.vel[0]);
    if (avg_bin_x.size()>n_avg)
    {avg_bin_x.pop_front();}

    avg_bin_y.push_back(robot_state.vel[1]);
    if (avg_bin_y.size()>n_avg)
    {avg_bin_y.pop_front();}

    double sum_x=0;
    double sum_y=0;
    for(int i=0;i<avg_bin_x.size();i++)
    {
      sum_x=sum_x+avg_bin_x[i];
      sum_y=sum_y+avg_bin_y[i];

    }
    avg_x=sum_x/avg_bin_x.size();
    avg_y=sum_y/avg_bin_y.size();

    }

    vel_output[0]=-avg_y;
    vel_output[1]=avg_x;
    vel_output[2]=robot_state.vel[2];

    timestamp=AP_HAL::millis()-start_time;

    //Uncomment to activate Logging to csv File
    // Logging: timestamp, image #, Accel.x, Accel.y, Accel.z, Gyro.x, Gyro.y, Gyro.z, Rangefinder
    //printf("Log: Timestamp: %d image#: %d Accel: %lf %lf %lf Gyro: %lf %lf %lf Range: %d\n",timestamp,image_count,accel.x,accel.y,accel.z,gyro.x,gyro.y,gyro.z,meas.sonar);
    //std::ofstream log_file;
    //log_file.open ("/data/ftp/internal_000/vio_logs/vio_log.csv",std::ios_base::app);
    //log_file << "timestamp,image_nr,acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,range\n";

    // //Calculate Optical Flow velocities
    float fx=409.72;
    float fy=306.48;

    float flow_x_trans=flow_rate.x/fx*meas.sonar/dt*4;
    float flow_x_rot=gyro_delay_cont[0].x*meas.sonar;
    float flow_y_trans=flow_rate.y/fy*meas.sonar/dt*4;
    float flow_y_rot=gyro_delay_cont[0].y*meas.sonar;
    float flow_x=flow_x_trans-flow_x_rot;
    float flow_y=flow_y_trans-flow_y_rot;
    float flow_x_ekf=(0.95*vel_output[0]/meas.sonar+gyro_delay_cont[0].y)*fx;
    float flow_y_ekf=(-0.95*vel_output[1]/meas.sonar+gyro_delay_cont[0].x)*fy;


    // log_file << std::to_string(timestamp)+","+std::to_string(image_count)+","+std::to_string(vel_output[0])+","+
    //             std::to_string(vel_output[1])+","+std::to_string(vel_output[2])+","+std::to_string(gyro.x)+","+
    //             std::to_string(gyro.y)+","+std::to_string(gyro.z)+","+std::to_string(accel.x)+","+
    //             std::to_string(accel.y)+","+std::to_string(accel.z)+","+std::to_string(flow_x)+","+std::to_string(flow_y)+
    //             ","+std::to_string(flow_x_trans)+","+std::to_string(flow_y_trans)+","+std::to_string(meas.sonar)+"\n";

    // log_file << std::to_string(timestamp)+","+std::to_string(image_count)+","+std::to_string(vel_output[0])+","+
    //             std::to_string(vel_output[1])+","+std::to_string(vel_output[2])+","+std::to_string(gyro.x)+","+
    //             std::to_string(gyro.y)+","+std::to_string(gyro.z)+","+std::to_string(accel.x)+","+
    //             std::to_string(accel.y)+","+std::to_string(accel.z)+","+std::to_string(meas.sonar)+"\n";

    //log_file.close();


    vel_output[0]=flow_x_ekf;
    vel_output[1]=flow_y_ekf;

    return vel_output;
}

void OpticalFlow_Onboard::_init_vioflow() {
      // initialize structs
      cameraParams = { {}, {}};
      noiseParams = {};
      vioParams = {};

      // Define Vio Parameters

      noiseParams.process_noise.qv = 10;                  // noise acc
      noiseParams.process_noise.qw = 0.001;               // noise gyro
      noiseParams.process_noise.qwo = 0.000;              // noise gyro bias
      noiseParams.process_noise.qao = 0.000;              // noise acc bias
      noiseParams.process_noise.qR_ci =0.001*0;           // noise q_R_ci
      noiseParams.inv_depth_initial_unc = 0.01;          // noise inverse depth initial uncertainty
      noiseParams.image_noise = 2;                        // image noise

      //noiseParams.gyro_bias_initial_unc= [0.01 0.01 0.01];// gyro bias initial uncertainty vector
      noiseParams.gyro_bias_initial_unc[0]= 0.01;
      noiseParams.gyro_bias_initial_unc[1]= 0.01;
      noiseParams.gyro_bias_initial_unc[2]= 0.01;

      //noiseParams.acc_bias_initial_unc= [0,0,0];          // acc bias initial uncetainty vector
      noiseParams.acc_bias_initial_unc[0]= 0;
      noiseParams.acc_bias_initial_unc[1]= 0;
      noiseParams.acc_bias_initial_unc[2]= 0;


      vioParams.max_ekf_iterations=3;                     // max ekf iterations
      vioParams.delayed_initialization = true;
      vioParams.mono = true;
      vioParams.fixed_feature = false;
      vioParams.RANSAC = true;
      vioParams.full_stereo = false;
      vioParams.delayed_fixing=true;

      //cameraParams.RadialDistortion=[0.0698,-1.2074,3.2751];
      vertcameraParams.RadialDistortion[0]=0.0698;
      vertcameraParams.RadialDistortion[1]=-1.2074;
      vertcameraParams.RadialDistortion[2]=3.2751;
      //cameraParams.FocalLength=[412.3540,306.5525];
      vertcameraParams.FocalLength[0]=412.3540;
      vertcameraParams.FocalLength[1]=306.5525;
      //cameraParams.PrincipalPoint=[155.9669,130.6196];
      vertcameraParams.PrincipalPoint[0]=155.9669;
      vertcameraParams.PrincipalPoint[0]=130.6196;

      vertcameraParams.DistortionModel=0; // PLUMB_BOB=0, ATAN = 1

      cameraParams.CameraParameters1=vertcameraParams;
      cameraParams.CameraParameters2=vertcameraParams;

      fps=30;
      vision_subsample=1;
      if (vision_subsample < 1) {
          auto_subsample = true;
          //("Auto subsamlple: Using every VIO message with images to update, others to predict");
      }

      update_vec_.assign(matlab_consts::numTrackFeatures, 0);
      map.resize(matlab_consts::numTrackFeatures * 3);
      anchor_poses.resize(matlab_consts::numAnchors);

       vio.setParams(cameraParams, noiseParams, vioParams);


}


void OpticalFlow_Onboard::_run_optflow()
{
    hal.console->println("Run Optflow");
    float rate_x, rate_y, rate_z;
    float acc_x,acc_y,acc_z;

    VideoIn::Frame video_frame;
    uint32_t convert_buffer_size = 0, output_buffer_size = 0;
    uint32_t crop_left = 0, crop_top = 0;
    uint32_t shrink_scale = 0, shrink_width = 0, shrink_height = 0;
    uint32_t shrink_width_offset = 0, shrink_height_offset = 0;
    uint8_t *convert_buffer = NULL, *output_buffer = NULL;
    uint8_t qual;

    if (_format == V4L2_PIX_FMT_YUYV) {
        if (_shrink_by_software || _crop_by_software) {
            convert_buffer_size = _camera_output_width * _camera_output_height;
        } else {
            convert_buffer_size = _width * _height;
        }

        convert_buffer = (uint8_t *)malloc(convert_buffer_size);
        if (!convert_buffer) {
            AP_HAL::panic("OpticalFlow_Onboard: couldn't allocate conversion buffer\n");
        }
    }

    if (_shrink_by_software || _crop_by_software) {
        output_buffer_size = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH *
            HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT;

        output_buffer = (uint8_t *)malloc(output_buffer_size);
        if (!output_buffer) {
            if (convert_buffer) {
                free(convert_buffer);
            }

            AP_HAL::panic("OpticalFlow_Onboard: couldn't allocate crop buffer\n");
        }
    }

    if (_shrink_by_software) {
        if (_camera_output_width > _camera_output_height) {
            shrink_scale = (uint32_t) _camera_output_height /
                HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT;
        } else {
            shrink_scale = (uint32_t) _camera_output_width /
                HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH;
        }

        shrink_width = HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH * shrink_scale;
        shrink_height = HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT * shrink_scale;

        shrink_width_offset = (_camera_output_width - shrink_width) / 2;
        shrink_height_offset = (_camera_output_height - shrink_height) / 2;
    } else if (_crop_by_software) {
        crop_left = _camera_output_width / 2 -
           HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH / 2;
        crop_top = _camera_output_height / 2 -
           HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT / 2;
    }




    while(true) {
      now=AP_HAL::millis();

        /* wait for next frame to come */
        if (!_videoin->get_frame(video_frame)) {
            if (convert_buffer) {
               free(convert_buffer);
            }

            if (output_buffer) {
               free(output_buffer);
            }

            AP_HAL::panic("OpticalFlow_Onboard: couldn't get frame\n");
        }
        //hal.console->println("Still alive!!");
        if (_format == V4L2_PIX_FMT_YUYV) {
            VideoIn::yuyv_to_grey((uint8_t *)video_frame.data,
                convert_buffer_size * 2, convert_buffer);

            memset(video_frame.data, 0, convert_buffer_size * 2);
            memcpy(video_frame.data, convert_buffer, convert_buffer_size);
        }

#ifdef OPTICALFLOW_ONBOARD_RECORD_OPENCV_VIDEO
        int tic_fps = AP_HAL::millis();

          //Save raw image_yuv, Uncomment to activate
          //int outfd = open("/data/ftp/internal_000/vio_logs/images/log_img_"+std::to_string(image_count)+".img", O_RDWR);
          //FILE * fp;
          //std::string img_path="/data/ftp/internal_000/vio_logs/images/log_img.bin";
          //fp = fopen(img_path.c_str(), "w");

          //Load Image to Mat
          cv::Mat image_yuv=cv::Mat(HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT + HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT/2,HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH, CV_8UC1, video_frame.data);
          cv::Rect myROI(0,0,HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH,HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT);
          cv::Mat frame_mat = image_yuv(myROI);
          /

          /* read sensor Data data from EKF via the opticalflow driver */
          _get_sensors(rate_x, rate_y, rate_z,acc_x,acc_y,acc_z,range_cm);
          gyro_rate.x = rate_x;
          gyro_rate.y = rate_y;
          gyro_rate.z = rate_z;
          gyro.x=rate_x;
          gyro.y=rate_y;
          gyro.z=0;//rate_z;
          gyro_delay_cont.push_back(gyro);

          if (gyro_delay_cont.size()>gyro_delay)
          {gyro_delay_cont.pop_front();}

          accel.x=acc_x;
          accel.y=acc_y;
          accel.z=acc_z;
          accel_delay_cont.push_back(accel);

          if (accel_delay_cont.size()>gyro_delay)
          {accel_delay_cont.pop_front();}


          vio_vel=_vioflow(frame_mat,vio_count);
          vio_count++;
          image_count++;

#endif

    int tic_update = AP_HAL::millis();
        if (_shrink_by_software) {
            /* shrink_8bpp() will shrink a selected area using the offsets,
             * therefore, we don't need the crop. */
            VideoIn::shrink_8bpp((uint8_t *)video_frame.data, output_buffer,
                                 _camera_output_width, _camera_output_height,
                                 shrink_width_offset, shrink_width,
                                 shrink_height_offset, shrink_height,
                                 shrink_scale, shrink_scale);
            memset(video_frame.data, 0, _camera_output_width * _camera_output_height);
            memcpy(video_frame.data, output_buffer, output_buffer_size);
        } else if (_crop_by_software) {
            VideoIn::crop_8bpp((uint8_t *)video_frame.data, output_buffer,
                               _camera_output_width,
                               crop_left, HAL_OPTFLOW_ONBOARD_OUTPUT_WIDTH,
                               crop_top, HAL_OPTFLOW_ONBOARD_OUTPUT_HEIGHT);

            memset(video_frame.data, 0, _camera_output_width * _camera_output_height);
            memcpy(video_frame.data, output_buffer, output_buffer_size);
        }

        /* if it is at least the second frame we receive
         * since we have to compare 2 frames */
        if (_last_video_frame.data == NULL) {
            _last_video_frame = video_frame;
            continue;
        }

        /* compute gyro data and video frames
         * get flow rate to send it to the opticalflow driver
         */
        qual = _flow->compute_flow((uint8_t*)_last_video_frame.data,
                                   (uint8_t *)video_frame.data,
                                   video_frame.timestamp -
                                   _last_video_frame.timestamp,
                                   &flow_rate.x, &flow_rate.y);
        qual=255;

        int duration_update = (AP_HAL::millis() - tic_update);

        /* fill data frame for upper layers */
        pthread_mutex_lock(&_mutex);
        _pixel_flow_x_integral += flow_rate.x /
                                  HAL_FLOW_PX4_FOCAL_LENGTH_MILLIPX;
        _pixel_flow_y_integral += flow_rate.y /
                                  HAL_FLOW_PX4_FOCAL_LENGTH_MILLIPX;
        _integration_timespan += video_frame.timestamp -
                                 _last_video_frame.timestamp;
        _gyro_x_integral       += (gyro_rate.x + _last_gyro_rate.x) / 2.0f *
                                  (video_frame.timestamp -
                                  _last_video_frame.timestamp);
        _gyro_y_integral       += (gyro_rate.y + _last_gyro_rate.y) / 2.0f *
                                  (video_frame.timestamp -
                                  _last_video_frame.timestamp);

        _pixel_flow_x_integral = vio_vel[0];
        _pixel_flow_y_integral = vio_vel[1];
       _gyro_x_integral       = gyro_delay_cont[0].x;
       _gyro_y_integral       = gyro_delay_cont[0].y;
       _integration_timespan += 15;


        _surface_quality = qual;
        _data_available = true;
        pthread_mutex_unlock(&_mutex);

        /* give the last frame back to the video input driver */
        _videoin->put_frame(_last_video_frame);
        _last_video_frame = video_frame;
        _last_gyro_rate = gyro_rate;
    }

    if (convert_buffer) {
        free(convert_buffer);
    }

    if (output_buffer) {
        free(output_buffer);
    }
}
#endif
