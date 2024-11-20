#pragma once

// main node dep
#include <rclcpp/rclcpp.hpp>
#include <mutex>

// topics type dep
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ros_gz_interfaces/msg/param_vec.hpp"


// image handling dep
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <zbar.h>

using namespace std::chrono_literals;

#define X                     0
#define Y                     1
#define Z                     2
#define W                     3
#define LEFT                  0
#define RIGHT                 1
#define RANGE                 0
#define BEARING               1
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
    void  _targetStanCallback();
    void  _imageProcessorCallback();

    //  - - - - - Commands Publisher  - - - - - //
    // Thrusters
    void  _setThrusterPos(double [2]);
    void  _setThrusterThrust(int [2]);

    // sensors
    void  _setCameraPos(double);

    //  - - - - - Commands Subscribers  - - - - - //
    // Sensors
    void  _gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr);
    void  _imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr);
    void  _criticalWindTurbinDataCallback(const ros_gz_interfaces::msg::ParamVec::SharedPtr);
    void  _imageDataCallback(const sensor_msgs::msg::Image::SharedPtr);

    //  - - - - - Commands Getters  - - - - - //
    void  _getGpsData(double [2]);
    void  _getImuData(double [2], double [3], double [3]);
    void  _getCriticalWindTurbinData(double [2]);
    void  _getImageData(cv::Mat &);
    void  _getAvoidanceOrientation(double &);

    //  - - - - - Commands Setters  - - - - - //
    void  _setAvoidanceOrientation(const double &);

    //  - - - - - Commands Utils  - - - - - //
    void  _degToMeter(double [2]);
    void  _meterToDeg(double [2]);

    //  - - - - - Main Variables  - - - - - //
    double  _gpsOrigin[2];
    double  _gpsPos[2];
    double  _targetGpsPos[2];

    double  _acceleration[2];
    double  _angularVelocity[3];
    double  _orientation[4]; // on purpose^^
    double  _targetOrientation[3];
    double  _avoidanceOrientation;

    double  _criticalWindTurbin[2];

    int _statmentTrip;
    cv::Mat _lastFrame;

    //  - - - - - Publishers  - - - - - //
    // Thrusters
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _thrusterPos[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _thrusterThrust[2];

    // Camera
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _cameraPos;

    // -  - - - - Subscribers  - - - - - //
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr      _gpsSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            _imuSub;
    rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr _criticalWindTurbinSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          _imageSub;

    //  - - - - - Loops - - - - - //
    rclcpp::TimerBase::SharedPtr  _targetStanCallbackTimer;
    rclcpp::TimerBase::SharedPtr  _imageProcessorCallbackTimer;

    //  - - - - - Mutexes  - - - - - //
    std::mutex  _thrusterPosMutex;
    std::mutex  _thrusterThrustMutex;
    std::mutex  _cameraMutex;
    std::mutex  _lastFrameMutex;
    std::mutex  _gpsMutex;
    std::mutex  _gpsOriginMutex;
    std::mutex  _imuMutex;
    std::mutex  _criticalWindTurbinMutex;
    std::mutex  _avoidanceOrientationMutex;
};
