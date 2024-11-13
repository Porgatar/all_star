#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

// #include "tf2/exceptions.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

#define X                     0
#define Y                     1
#define LEFT                  0
#define RIGHT                 1
#define EPSILON               0.017453
// #define MAX_CAMERA_POS        3.141592
// #define MIN_CAMERA_POS        -3.141592
#define MAX_THRUSTERS_POS     0.785398
#define MIN_THRUSTERS_POS     -0.785398
#define MAX_THRUSTERS_THRUST  5000
#define MIN_THRUSTERS_THRUST  -5000

class AquabotNode : public rclcpp::Node {

  public:
    AquabotNode();

  private:
    //  - - - - - Commands Loops - - - - - //
    void  _placeholderCallback();

    //  - - - - - Commands Publisher  - - - - - //
    // Thrusters
    void  _setThrusterPos(double [2]);
    void  _setThrusterThrust(int [2]);

    // sensors
    void  _setCameraPos(double);

    //  - - - - - Commands Subscribers  - - - - - //
    // Sensors
    void  _gpsPosCallback(const sensor_msgs::msg::NavSatFix::SharedPtr);
    void  _getGpsPos(double [2]);
    void  _imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr);
    void  _getImuData(double [2], double *, double *);

    //  - - - - - Main Variables  - - - - - //
    double  _gpsPos[2];
    double  _targetGpsPos[2];

    double  _acceleration[2];
    double  _targetAcceleration[2];

    double  _angularAcceleration;
    double  _targetAngularAcceleration;

    double  _rotation;
    double  _targetRotation;

    //  - - - - - Publishers  - - - - - //
    // Thrusters
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _thrusterPos[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _thrusterThrust[2];

    // Camera
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _cameraPos;

    // -  - - - - Subscribers  - - - - - //
    // Sensors
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  _gpsSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        _imuSub;

    //  - - - - - Loops - - - - - //
    rclcpp::TimerBase::SharedPtr  _placeholderCallbackTimer;

    //  - - - - - Mutexes  - - - - - //
    std::mutex  _cameraMutex;
    std::mutex  _gpsMutex;
    std::mutex  _imuMutex;
    std::mutex  _thrusterPosMutex;
    std::mutex  _thrusterThrustMutex;
};
