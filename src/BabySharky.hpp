#pragma once

// main node dep
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <list>
#include <array>

// topics type dep
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ros_gz_interfaces/msg/param_vec.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

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
#define RX                    3
#define RY                    4
#define RZ                    5
#define RW                    6
#define LEFT                  0
#define RIGHT                 1
#define RANGE                 0
#define BEARING               1
#define QR_DETECTOR           1
#define OBSTACLE_DETECTOR     2
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
    void  _mainProcessorCallback();

    //  - - - - - Commands Publisher  - - - - - //
    // Thrusters
    void  _setThrusterPos(double [2]);
    void  _setThrusterThrust(int [2]);

    // sensors
    void  _setCameraPos(double);

    //  - - - - - Commands Subscribers  - - - - - //
    void  _gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr);
    void  _imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr);
    void  _imageDataCallback(const sensor_msgs::msg::Image::SharedPtr);
    void  _windTurbinDataCallback(const geometry_msgs::msg::PoseArray::SharedPtr);
    void  _criticalWindTurbinDataCallback(const ros_gz_interfaces::msg::ParamVec::SharedPtr);
    void  _globalStateDataCallback(const std_msgs::msg::UInt32::SharedPtr);

    //  - - - - - Commands Getters  - - - - - //
    void  _getGpsData(double [2]);
    void  _getTargetGpsData(double [2]);
    void  _getImuData(double [2], double [3], double [3]);
    void  _getImageData(cv::Mat &);
    void  _getAvoidanceTarget(double [2]);
    void  _getWindTurbinData(std::list<std::array<double, 3>> &);
    void  _getCriticalWindTurbinData(double [2]);
    void  _getGlobalState(int &);
    void  _getTripState(int &);
    void  _getCameraState(int &);
    void  _getTargetOrientation(double &);
    void  _getLastQrCode(std::string &);

    //  - - - - - Commands Setters  - - - - - //
    void  _setTargetGpsData(const double [2]);
    void  _setAvoidanceTarget(const double [2]);
    void  _setGlobalState(const int &);
    void  _setTripState(const int &);
    void  _setCameraState(const int &);
    void  _setTargetOrientation(const double &);
    void  _setLastQrCode(const std::string &);

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
    double  _targetOrientation;
    double  _avoidanceTarget[2];

    std::list<std::array<double, 7>>  _windTurbinGpsPos;
    double                            _criticalWindTurbin[2];

    int _globalState;
    int _statementTrip;
    int _cameraState;

    cv::Mat _lastFrame;

    std::string _lastQrCode;

    //  - - - - - Publishers  - - - - - //
    // Thrusters
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _thrusterPos[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _thrusterThrust[2];

    // Camera
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _cameraPos;

    // WindTurbin Checkup
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   _windTurbinCheckup;

    // -  - - - - Subscribers  - - - - - //
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr      _gpsSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            _imuSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          _imageSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr    _windTurbinSub;
    rclcpp::Subscription<ros_gz_interfaces::msg::ParamVec>::SharedPtr _criticalWindTurbinSub;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr            _globalStateSub;

    //  - - - - - Loops - - - - - //
    rclcpp::TimerBase::SharedPtr  _targetStanCallbackTimer;
    rclcpp::TimerBase::SharedPtr  _imageProcessorCallbackTimer;
    rclcpp::TimerBase::SharedPtr  _mainProcessorCallbackTimer;

    //  - - - - - Mutexes  - - - - - //
    std::mutex  _thrusterPosMutex;
    std::mutex  _thrusterThrustMutex;
    std::mutex  _cameraMutex;
    std::mutex  _lastFrameMutex;
    std::mutex  _gpsMutex;
    std::mutex  _targetGpsMutex;
    std::mutex  _gpsOriginMutex;
    std::mutex  _imuMutex;
    std::mutex  _windTurbinMutex;
    std::mutex  _criticalWindTurbinMutex;
    std::mutex  _avoidanceTargetMutex;
    std::mutex  _globalStateMutex;
    std::mutex  _tripStateMutex;
    std::mutex  _cameraStateMutex;
};
