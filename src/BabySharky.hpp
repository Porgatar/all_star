#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

#define LEFT                  0
#define RIGHT                 1
#define MAX_CAMERA_POS        6.283185
//30718
#define EPSILON               0.017453
//3
#define MAX_THRUSTERS_POS     0.785398
//16339
#define MIN_THRUSTERS_POS     -0.785398
//16339
#define MAX_THRUSTERS_THRUST  5000
#define MIN_THRUSTERS_THRUST  -5000

class AquabotNode : public rclcpp::Node {

  public:
    AquabotNode();

  private:
    //  - - - - - Commands Loops - - - - - //
    void  _targetFollower();

    //  - - - - - Commands Publisher  - - - - - //
    // Thrusters
    void  _setThrusterPos(double [2]);
    // void  _setThrusterThrust(int [2]);

    // sensors
    void  _setCameraPos(double);

    //  - - - - - Commands Subscribers  - - - - - //
    // Sensors
    void  _getGpsPos(const std_msgs::msg::Float64::SharedPtr);

    //  - - - - - Main Variables  - - - - - //
    double _gpsPos;

    //  - - - - - Publishers  - - - - - //
    // Thrusters
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _thrusterPos[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _thrusterThrust[2];

    // Camera
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  _cameraPos;

    // -  - - - - Subscribers  - - - - - //
    // Sensors
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _gpsSub;

    //  - - - - - Loops - - - - - //
    rclcpp::TimerBase::SharedPtr  _targetFollowerTimer;

    //  - - - - - Mutexes  - - - - - //
    std::mutex  _sensorMutex;
    std::mutex  _thrusterPosMutex;
    std::mutex  _thrusterThrustMutex;
};
