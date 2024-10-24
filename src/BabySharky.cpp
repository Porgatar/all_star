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

void    AquabotNode::_targetFollower() {

    // thrusters test
    static double   TargetPos[2] = {MIN_THRUSTERS_POS, MAX_THRUSTERS_POS};
    static int      TargetThrust[2] = {MIN_THRUSTERS_THRUST, MIN_THRUSTERS_THRUST};

    TargetPos[LEFT] += EPSILON * 5;
    TargetPos[RIGHT] -= EPSILON * 5;
    TargetThrust[LEFT] += 500;
    TargetThrust[RIGHT] += 500;
    this->_setThrusterPos(TargetPos);
    this->_setThrusterThrust(TargetThrust);
}

//  -   -   -   -   -   Thrusters   -   -   -   -   -   //

static double clamp(const double value, const double min, const double max) {

    return (std::max(min, std::min(max, value)));
}

void    AquabotNode::_setThrusterPos(double NewTargetPos[2]) {

    std::lock_guard<std::mutex> lock(this->_thrusterPosMutex);
    static double               CurrentTargetPos[2] = {0, 0};
    std_msgs::msg::Float64      msg;
    double                      NewPos;
    int                         i;

    i = LEFT;
    while (i <= RIGHT) {

        NewPos = clamp(NewTargetPos[i], MIN_THRUSTERS_POS, MAX_THRUSTERS_POS);
        if (std::abs(CurrentTargetPos[i] - NewPos) > EPSILON) {

            msg.data = NewPos;
            CurrentTargetPos[i] = NewPos;
            this->_thrusterPos[i]->publish(msg);
        }
        i++;
    }
}

//  rework in progress...
void    AquabotNode::_setThrusterThrust(int NewTargetThrust[2]) {

    std::lock_guard<std::mutex> lock(this->_thrusterThrustMutex);
    static int                  CurrentTargetThrust[2] = {0, 0};
    std_msgs::msg::Float64      msg;
    int                         NewThrust;
    int                         i;

    i = LEFT;
    while (i <= RIGHT) {

        NewThrust = clamp(NewTargetThrust[i], MIN_THRUSTERS_THRUST, MAX_THRUSTERS_THRUST);
        if (CurrentTargetThrust[i] != NewThrust) {

            msg.data = NewThrust;
            CurrentTargetThrust[i] = NewThrust;
            this->_thrusterThrust[i]->publish(msg);
        }
        i++;
    }
}

//  rework in progress...
void    AquabotNode::_setCameraPos(double NewPos) {

    std_msgs::msg::Float64   msg;

    while (NewPos > MAX_CAMERA_POS)
        NewPos -= MAX_CAMERA_POS;
    while (NewPos < 0)
        NewPos += MAX_CAMERA_POS;
    msg.data = NewPos;
    this->_cameraPos->publish(msg);
}

void AquabotNode::_getGpsPos(const std_msgs::msg::Float64::SharedPtr msg) {

    std::lock_guard<std::mutex>lock(this->_sensorMutex);
    this->_gpsPos = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received GPS position: %f", msg->data);
}
