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



//Feature Tracker Initializations
int n_features = 5;
cv::Point2f point_a(0.f, 0.f);
cv::Point2f motion(0.f, 0.f);
cv::Point2f motion_old(0.f, 0.f);
cv::Point2f motion_print(0.f, 0.f);
cv::Point midpoint(160,120);
std::vector<cv::Point2f> features_l_old(n_features,point_a);
std::vector<cv::Point2f> features_old(n_features,point_a);
std::vector<int> status(n_features,2);
int n_active=0;

//Gyro & Accel Variables
Vector3f accel;
Vector3f gyro;
uint32_t range;
Vector3f vio_vel;
double tic_predict_old =0.0;
float hagl;
//Variables and Parameters for velocity moving average filter
int n_avg=1;
std::deque<double> avg_bin_x;
double avg_x=0;
std::deque<double> avg_bin_y;
double avg_y=0;
std::deque<Vector3f> gyro_delay_cont;
int gyro_delay=6;



#include <AP_AHRS/AP_AHRS.h>
#include "CameraSensor_Mt9v117.h"
#include "GPIO.h"
#include "PWM_Sysfs.h"

#define OPTICAL_FLOW_ONBOARD_RTPRIO 11

extern const AP_HAL::HAL& hal;

//Initialize Rangefinder and SerialManager
 // static AP_SerialManager serial_manager;
 // static RangeFinder sonar {serial_manager};
//static AP_InertialSensor &inertial_sensors;
//AP_InertialSensor &inertial_sensors = *AP_InertialSensor::get_instance();
// Add own Hal object to plot debug vars:
const AP_HAL::HAL& hal_debug = AP_HAL::get_HAL();
uint32_t now = AP_HAL::millis();


//Simulation start time for Logging & image_counter
uint32_t start_time=0;
uint32_t runtime=0;
uint32_t image_count=0;
uint32_t timestamp=0;
int vio_count=0;


#ifdef OPTICALFLOW_ONBOARD_RECORD_OPENCV_VIDEO
  //cv::Size size_vid = cv::Size(320,240);
  //cv::VideoWriter vertcam("/data/ftp/internal_000/vert_cam.avi",cv::VideoWriter::fourcc('D','I','V','X'),10,size_vid,false);
//  cv::VideoWriter vertcam("/data/ftp/internal_000/vert_cam.avi",CV_FOURCC('D','I','V','X'),10,size_vid,false);
#endif

using namespace Linux;
VIO vio;


void OpticalFlow_Onboard::init(AP_HAL::OpticalFlow::Gyro_Cb get_gyro)
{
  //Empty earlier logfile and write header
    std::ofstream log_file;
    log_file.open ("/data/ftp/internal_000/vio_logs/vio_log.csv");
    log_file << "timestamp,image_nr,vel.x,vel.y,vel.z,gyro.x,gyro.y,gyro.z,range\n";
    log_file.close();

    vio_vel[0]=0;
    vio_vel[1]=0;
    vio_vel[2]=0;
    printf("Initialize OptFlow");

    //Set Start Time
    start_time=AP_HAL::millis();
    tic_predict_old=start_time/1000.;
    //init Inertial Sensors
    //inertial_sensors.init(200);


    //Initialize Range Finder
    _init_rangefinder();

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

    _get_gyro = get_gyro;
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
    //Debug: Load Image 5 for debug purpose (Fixed Image)
    //vertcam_frame= cv::imread("/data/ftp/internal_000/bebop_vertcam_5.bmp",0);

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

    //inertial_sensors.update();
    //inertial_sensors.wait_for_sample();

    //accel = inertial_sensors.get_accel(0);
    //gyro =  inertial_sensors.get_gyro(0);
    //
    // meas.gyr[0]=gyro.x;
    // meas.gyr[1]=gyro.y;
    // meas.gyr[2]=gyro.z;
    //
    meas.gyr[0]=gyro_delay_cont[0].x;
    meas.gyr[1]=gyro_delay_cont[0].y;
    meas.gyr[2]=gyro_delay_cont[0].z;
    //
    meas.acc[0]=accel.x;
    meas.acc[1]=accel.y;
    meas.acc[2]=accel.z;


    // meas.gyr[0]=-0.00001;
    // meas.gyr[1]=-0.00001;
    // meas.gyr[2]=-0.00001;
    // //
    // meas.acc[0]=0.40;
    // meas.acc[1]=-0.25;
    // meas.acc[2]=-10.19;

    // meas.gyr[0]=-0.0699;
    // meas.gyr[1]=-0.0343;
    // meas.gyr[2]=-0.0357;
    // //
    // meas.acc[0]=0.2943;
    // meas.acc[1]=0.0026;
    // meas.acc[2]=-11.044;

    meas.sonar=0.1;


    //Print Acceleration measurements
    //_print_inertial_sensors(inertial_sensors);

    //double dt = 1/60.;

    double tic_predict = AP_HAL::millis()/1000.;
    double dt=tic_predict-tic_predict_old;
    tic_predict_old=tic_predict;
    //double dt= 0.025;
    printf("dt= %lf\n", dt);
    for (int i_pred=0;i_pred<1;i_pred++)
    {
      //printf("$$$$$$$- Sensor Prediction -$$$$$$$$$\n");
      vio.predict(meas,dt);
    }
    // int duration_predict = (AP_HAL::millis() - tic_predict);

    }
    //printf("Duration predict: %d ms\n",duration_predict);
    //printf("Prediction Ended\n");

//    printf("Optical Flow: Camera Focal length2: %lf \n", cameraParams.CameraParameters1.FocalLength[0]);

    //*********************************************************************
    // Point tracking
    //*********************************************************************
    // int tic_feature_tracking = AP_HAL::millis();

    if (!(count % sample_step_upd))
    {
    int stereo = 0;

    //Print update_vector for debugging
    //hal.console->printf("update_vec_ before tracking :"
    //
    // for (int i = 0; i < features_l.size(); i++) {
    //     if (i<45){
    //     hal.console->printf("%d",update_vec_[i]);
    //     hal.console->printf(" ");
    //   }}
    //   hal.console->println(" ");





    trackFeatures(vertcam_frame,vertcam_frame,features_l,features_r,update_vec_,stereo);

    for (int i = 0; i < features_l.size(); i++) {
        if (i<48){

      }
        z_all_l[2*i + 0] = features_l[i].x;
        z_all_l[2*i + 1] = features_l[i].y;

        //Should be removed: right image not needed for VioFlow
        z_all_r[2*i + 0] = features_r[i].x;
        z_all_r[2*i + 1] = features_r[i].y;
    }
    //int duration_feature_tracking = (AP_HAL::millis() - tic_feature_tracking);




    //printf("Duration Feature Tracking %d ms\n",duration_feature_tracking);

    // //*********************************************************************
    // // SLAM update
    // //*********************************************************************
    //Should be removed: Parameters don't have to be set before every update

    //Get Rangefinder measurements
    // sonar.update();
    // meas.sonar=sonar.distance_cm()/100.;
    //     printf("sonar measurement before: %d\n",sonar.distance_cm());
    // if (meas.sonar<0.1)
    //   {meas.sonar=0.1;}

    meas.sonar=hagl;
    if (meas.sonar<0.1)
      {meas.sonar=0.1;}
    //printf("sonar measurement: %lf\n",meas.sonar);



    vio.setParams(cameraParams, noiseParams, vioParams);

    int tic_update = AP_HAL::millis();
    vio.update(update_vec_, z_all_l, z_all_r, robot_state, map, anchor_poses, delayedStatus,meas);
    int duration_update = (AP_HAL::millis() - tic_update);

    //moving average on velocities

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



    // // Debug: Print Robot Position
    // printf("Robot Velocity: X: ");
    // printf("%f",robot_state.vel[0]);
    // printf(" Y: ");
    // printf("%f",robot_state.vel[1]);
    // printf(" Z: ");
    // printf("%f\n",robot_state.vel[2]);

    }

    //Transform velocities from Camera to IMU frame (-90 deg rotation around common z axis)
    // vel_output[0]=-robot_state.vel[1];
    // vel_output[1]=robot_state.vel[0];
    // vel_output[2]=robot_state.vel[2];

    vel_output[0]=-avg_y;
    vel_output[1]=avg_x;
    vel_output[2]=robot_state.vel[2];


    //printf("Duration update: %d ms\n",duration_update);


    // Logging: timestamp, image #, Accel.x, Accel.y, Accel.z, Gyro.x, Gyro.y, Gyro.z, Rangefinder
    timestamp=AP_HAL::millis()-start_time;
    //printf("Log: Timestamp: %d image#: %d Accel: %lf %lf %lf Gyro: %lf %lf %lf Range: %d\n",timestamp,image_count,accel.x,accel.y,accel.z,gyro.x,gyro.y,gyro.z,meas.sonar);
    std::ofstream log_file;
    log_file.open ("/data/ftp/internal_000/vio_logs/vio_log.csv",std::ios_base::app);
    //log_file << "timestamp,image_nr,acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,range\n";
    log_file << std::to_string(timestamp)+","+std::to_string(image_count)+","+std::to_string(vel_output[0])+","+
                std::to_string(vel_output[1])+","+std::to_string(vel_output[2])+","+std::to_string(gyro.x)+","+
                std::to_string(gyro.y)+","+std::to_string(gyro.z)+","+std::to_string(meas.sonar)+"\n";
    log_file.close();




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
      noiseParams.inv_depth_initial_unc = 0.001;          // noise inverse depth initial uncertainty
      noiseParams.image_noise = 1;                        // image noise

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

    //  printf("Optical Flow: Camera Focal length: %lf \n", vertcameraParams.FocalLength[0]);




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

void OpticalFlow_Onboard::_init_rangefinder()
{
  // // Setup parameter for Bebop Rangefinder
  // AP_Param::set_object_value(&sonar, sonar.var_info, "_TYPE", RangeFinder::RangeFinder_TYPE_BEBOP);
  // AP_Param::set_object_value(&sonar, sonar.var_info, "_PIN", -1);
  // AP_Param::set_object_value(&sonar, sonar.var_info, "_SCALING", 1.0);

  // // initialise sensor, delaying to make debug easier
  // hal.scheduler->delay(200);
  //  sonar.init();
  // printf("RangeFinder: %d devices detected\n", sonar.num_sensors());
}

// void OpticalFlow_Onboard::_print_inertial_sensors(AP_InertialSensor ins_print)
// {
//   Vector3f accel;
//   Vector3f gyro;
//   ins_print.update();
//   ins_print.wait_for_sample();
//
//
//   // read samples from ins
//   ins_print.update();
//
//   accel = ins_print.get_accel(0);
//   gyro =  ins_print.get_gyro(0);
//
//   printf("Accel (%d) : X:%6.2f Y:%6.2f Z:%6.2f norm:%5.2f\n", ins_print.get_accel_health(0), accel.x, accel.y, accel.z, accel.length());
//   printf("Gyro (%d) : X:%6.2f Y:%6.2f Z:%6.2f\n", ins_print.get_gyro_health(0), gyro.x, gyro.y, gyro.z);
// }

void OpticalFlow_Onboard::_run_optflow()
{
    hal.console->println("Run Optflow");
    float rate_x, rate_y, rate_z;
    Vector3f gyro_rate;
    Vector2f flow_rate;
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



      while (now>AP_HAL::millis()-10)
      {
      int wait = 10-(AP_HAL::millis()-now);
      if (wait<0)
      {wait =0;}
      printf("Wait for %d ms\n",wait);
      //delay(wait);
      usleep(wait*1000);
      }
      //printf("Finished waiting\n");
      now=AP_HAL::millis();


        /* wait for next frame to come */
        //hal.console->println("Before segfault");
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
        // Save image before shrinking/cropping
        int tic_fps = AP_HAL::millis();

        //hal.console->println(tic_fps);


          //Load Image to Mat
          cv::Mat image_yuv=cv::Mat(HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT + HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT/2,HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH, CV_8UC1, video_frame.data);
          cv::Rect myROI(0,0,HAL_OPTFLOW_ONBOARD_SENSOR_WIDTH,HAL_OPTFLOW_ONBOARD_SENSOR_HEIGHT);
          cv::Mat frame_mat = image_yuv(myROI);
          cv::imwrite("/data/ftp/internal_000/vio_logs/images/log_img_"+std::to_string(image_count)+".bmp", frame_mat);
          //hal.console->println("Next Step is vioflow");

          /* read gyro data from EKF via the opticalflow driver */
          _get_gyro(rate_x, rate_y, rate_z,hagl);
          //printf("Height: %lf\n",hagl);
          gyro_rate.x = rate_x;
          gyro_rate.y = rate_y;
          gyro_rate.z = rate_z;
          gyro.x=rate_x;
          gyro.y=rate_y;
          gyro.z=rate_z;
          gyro_delay_cont.push_back(gyro);

          if (gyro_delay_cont.size()>gyro_delay)
          {gyro_delay_cont.pop_front();}


          vio_vel=_vioflow(frame_mat,vio_count);
          vio_count++;
        // Debug: Track Features in Image, draw Circles around Features and Save Image
        //
        //   //Track Features
        //   //std::vector<int> status(n_features,2);
        //   features_old=features_l_old;
        //   int stereo = 0;
        //   trackFeatures(frame_mat,frame_mat,features_l_old,features_l_old,status,stereo);
        //
        //   //Mark Features in image_count
        //   int i=0;
        //   while (i<n_features)
        //    {
        //       //hal.console->printf("Marking Feature #");
        //       //hal.console->println(i+1);
        //       cv::circle(frame_mat,features_l_old[i],5,CV_RGB(0,0,0),1);
        //       i++;
        //     }
        //
        //
        //     i=0;
        //     motion_old=motion;
        //     motion.x=0;
        //     motion.y=0;
        //     n_active=0;
        //
        //
        //     while (i<n_features)
        //      {
        //        if (status[i]==0)
        //        {
        //          status[i]=2;
        //        }
        //        else if (status[i]==1)
        //        {
        //          motion.x=motion.x+(features_l_old[i].x-features_old[i].x);
        //          motion.y=motion.y+(features_l_old[i].y-features_old[i].y);
        //          n_active++;
        //        }
        //        else if (status[i]==2)
        //        {
        //         status[i]=1;
        //        }
        //         i++;
        //       }
        //
        //       motion.x=2*motion.x/n_active;
        //       motion.y=2*motion.y/n_active;
        //       n_active=0;
        //       motion_print.x=-(motion.x+motion_old.x)/2+midpoint.x;
        //       motion_print.y=-(motion.y+motion_old.y)/2+midpoint.y;
        //
        //       cv::arrowedLine(frame_mat,midpoint,motion_print,CV_RGB(0,0,0),3);
        //       runtime=AP_HAL::millis()-start_time;
        //       cv::rectangle(frame_mat,cv::Point(1,239),cv::Point(319,220),CV_RGB(255,255,255),-1);
        //       cv::putText(frame_mat,"Time: "+std::to_string(runtime)+"ms",cv::Point(120,235),cv::FONT_HERSHEY_PLAIN,1,CV_RGB(0,0,0),1);
        //
        //

          //hal_debug.console->println("Saved Picture!!");
          image_count++;

#endif


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




// #ifdef OPTICALFLOW_ONBOARD_RECORD_VIDEO
//         int fd = open(OPTICALFLOW_ONBOARD_VIDEO_FILE, O_CREAT | O_WRONLY
//                 | O_APPEND, S_IRUSR | S_IWUSR | S_IRGRP |
//                 S_IWGRP | S_IROTH | S_IWOTH);
//         if (fd != -1) {
// 	        write(fd, video_frame.data, _sizeimage);
// #ifdef OPTICALFLOW_ONBOARD_RECORD_METADATAS
//             struct PACKED {
//                 uint32_t timestamp;
//                 float x;
//                 float y;
//                 float z;
//             } metas = { video_frame.timestamp, rate_x, rate_y, rate_z};
//             write(fd, &metas, sizeof(metas));
// #endif
// 	        close(fd);
//         }
// #endif

        /* compute gyro data and video frames
         * get flow rate to send it to the opticalflow driver
         */
        qual = _flow->compute_flow((uint8_t*)_last_video_frame.data,
                                   (uint8_t *)video_frame.data,
                                   video_frame.timestamp -
                                   _last_video_frame.timestamp,
                                   &flow_rate.x, &flow_rate.y);

    //  printf("flow_rate %lf|%lf  Gyro: %lf %lf\n",flow_rate.x,flow_rate.y,gyro_rate.x,gyro_rate.y);


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

       _gyro_x_integral       = vio_vel[0];
       _gyro_y_integral       = vio_vel[1];
       _integration_timespan += 10;
      //printf("Velocity Sent: %lf %lf\n",_gyro_x_integral,_gyro_y_integral);


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
