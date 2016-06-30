/*
VioFlow by Lukas Meier
 */

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

//Added libraries for OpenCV support
#include <iostream>
#include <fstream>
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "trackFeatures.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
cv::Mat input_img;


//Feature Tracker Initializations
int n_features = 25;
int stereo = 0;
cv::Point2f point_a(0.f, 0.f);

std::vector<int> status(n_features,2);
std::vector<cv::Point2f> features_l(n_features,point_a);
std::vector<cv::Point2f> features_r = features_l;
cv::Mat img_l;
cv::Mat img_r;
int i=0;



std::vector<unsigned char> statusRight;
std::vector<float> error;

void setup()
{

    hal.console->println("VioFlow Example sketch skeleton - Setup");
    input_img=cv::imread("/data/ftp/internal_000/input.png",0);
    img_l = input_img;
    img_r = img_l;
        hal.console->println("Still alive!!!!");
    cv::Size input_size = input_img.size();
    //cv::Point2f point_a(160.2f, 60.f);
//Print the size of input image
    hal.console->printf("Width: ");
    hal.console->println(input_size.width);
    hal.console->printf("Height: ");
    hal.console->println(input_size.height);


//Track Features
    trackFeatures(img_l,img_r,features_l,features_r,status,stereo);

}

void loop()
{
  //Draw circle on Feature Positions
    if (i<n_features)
     {
        hal.console->printf("Marking Feature #");
        hal.console->println(i+1);
        cv::circle(input_img,features_l[i],5,CV_RGB(0,0,0),1);
     i++;
      }
    else if (i==n_features){
        i++;
        cv::imwrite("/data/ftp/internal_000/output.bmp",input_img);
        hal.console->printf("Image saved");
      }
 //hal.scheduler->delay(100);
}

AP_HAL_MAIN();
