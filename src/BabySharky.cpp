#include "BabySharky.hpp"

AquabotNode::AquabotNode() : Node("all_star") {

    RCLCPP_INFO(this->get_logger(), "Hello world from baby-sharky node in cpp!");

    //    -   -   -   -   -   Publishers    -   -   -   -   -   //
    // Thrusters
    this->_thrusterPos[LEFT] = this->create_publisher<std_msgs::msg::Float64> \
        ("/aquabot/thrusters/left/pos", \
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable)
    );
    this->_thrusterThrust[LEFT] = this->create_publisher<std_msgs::msg::Float64> \
        ("/aquabot/thrusters/left/thrust", \
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable)
    );
    this->_thrusterPos[RIGHT] = this->create_publisher<std_msgs::msg::Float64> \
        ("/aquabot/thrusters/right/pos", \
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable)
    );
    this->_thrusterThrust[RIGHT] = this->create_publisher<std_msgs::msg::Float64> \
        ("/aquabot/thrusters/right/thrust", \
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable)
    );

    // Camera
    this->_cameraPos = this->create_publisher<std_msgs::msg::Float64> \
        ("/aquabot/thrusters/main_camera_sensor/pos", \
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable)
    );
    this->_gpsPos = 0;

    //    -   -   -   -   -   Subscription    -   -   -   -   -   //
    this->_gpsSub = this->create_subscription<std_msgs::msg::Float64> \
        ("/aquabot/sensors/gps/gps/fix", \
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), \
        std::bind(&AquabotNode::_getGpsPos, this, std::placeholders::_1)
    );

    // loops
    this->_targetFollowerTimer = this->create_wall_timer(1s, std::bind(&AquabotNode::_targetFollower, this));
}

//  -   -   -   -   -   Main services   -   -   -   -   -   //

void    AquabotNode::_targetFollower() {

    // thrusters test
    static double   TargetPos[2] = {MIN_THRUSTERS_POS, MAX_THRUSTERS_POS};
    static double   TargetCameraPos = 0;
    static int      TargetThrust[2] = {MIN_THRUSTERS_THRUST, MIN_THRUSTERS_THRUST};

    TargetPos[LEFT] += EPSILON * 5;
    TargetPos[RIGHT] -= EPSILON * 5;
    TargetCameraPos += EPSILON * 10;
    TargetThrust[LEFT] += 500;
    TargetThrust[RIGHT] += 500;
    // this->_setThrusterPos(TargetPos);
    // this->_setThrusterThrust(TargetThrust);
    this->_setCameraPos(TargetCameraPos);
}

//  -   -   -   -   -   Thrusters   -   -   -   -   -   //

void    AquabotNode::_setThrusterPos(double NewTargetPos[2]) {

    std::lock_guard<std::mutex> lock(this->_thrusterPosMutex);
    static double               CurrentTargetPos[2] = {0, 0};
    double                      NewPos;
    int                         i;
    std_msgs::msg::Float64      msg;

    i = LEFT;
    while (i <= RIGHT) {

        NewPos = std::max(MIN_THRUSTERS_POS, std::min(MAX_THRUSTERS_POS, NewTargetPos[i]));
        if (std::abs(CurrentTargetPos[i] - NewPos) > EPSILON) {

            msg.data = NewPos;
            this->_thrusterPos[i]->publish(msg);
            CurrentTargetPos[i] = NewPos;
        }
        i++;
    }
}

void    AquabotNode::_setThrusterThrust(int NewTargetThrust[2]) {

    std::lock_guard<std::mutex> lock(this->_thrusterThrustMutex);
    static int                  CurrentTargetThrust[2] = {0, 0};
    int                         NewThrust;
    int                         i;
    std_msgs::msg::Float64      msg;

    i = LEFT;
    while (i <= RIGHT) {

        NewThrust = std::max(MIN_THRUSTERS_THRUST, std::min(MAX_THRUSTERS_THRUST, NewTargetThrust[i]));
        if (CurrentTargetThrust[i] != NewThrust) {

            msg.data = NewThrust;
            this->_thrusterThrust[i]->publish(msg);
            CurrentTargetThrust[i] = NewThrust;
        }
        i++;
    }
}

//  -   -   -   -   -   Sensors   -   -   -   -   -   //

void    AquabotNode::_setCameraPos(double NewPos) {

    std::lock_guard<std::mutex> lock(this->_sensorMutex);
    static double               CurrentTargetPos = 0;
    std_msgs::msg::Float64      msg;

    // while (NewPos > MAX_CAMERA_POS)
    //     NewPos -= MAX_CAMERA_POS * 2;
    // while (NewPos < MIN_CAMERA_POS)
    //     NewPos += MAX_CAMERA_POS * 2;
    if (abs(CurrentTargetPos - NewPos) > EPSILON) {

        msg.data = NewPos;
        this->_cameraPos->publish(msg);
        CurrentTargetPos = NewPos;
        RCLCPP_INFO(this->get_logger(), "Camera pos set to %f", NewPos);
    }
}

void    AquabotNode::_getGpsPos(const std_msgs::msg::Float64::SharedPtr msg) {

    std::lock_guard<std::mutex>lock(this->_sensorMutex);
    this->_gpsPos = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received GPS position: %f", msg->data);
}
