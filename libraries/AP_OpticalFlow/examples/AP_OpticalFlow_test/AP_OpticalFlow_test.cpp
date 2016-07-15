/*
 *  Example of AP_OpticalFlow library.
 *  Code by Randy Mackay. DIYDrones.com
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

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class DummyVehicle {
public:
    AP_GPS gps;
    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;
    AP_SerialManager serial_manager;
    RangeFinder sonar {serial_manager};
    AP_AHRS_NavEKF ahrs{ins, barometer, gps, sonar, EKF, EKF2,
                        AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF};
    NavEKF EKF{&ahrs, barometer, sonar};
    NavEKF2 EKF2{&ahrs, barometer, sonar};
};

static DummyVehicle vehicle;
static OpticalFlow optflow(vehicle.ahrs);

void setup()
{
  hal.console->println("OpticalFlow library test ver 1.6");

    // flowSensor initialization  & first update
    optflow.init();
    optflow.update();
    hal.scheduler->delay(11000);

    if (!optflow.healthy()) {
        hal.console->print("Failed to initialise PX4Flow ");
    }

    hal.scheduler->delay(100);
}

void loop()
{
  // optflow.update() will print optical flow data if debug messages are activated
    optflow.update();
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
