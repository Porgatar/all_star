#include "BabySharky.hpp"

AquabotNode::AquabotNode() : Node("all_star") {

    RCLCPP_INFO(this->get_logger(), "Hello world from baby-sharky node in cpp!");

    // placeholder targets...
    this->_gpsPos[X] = 0;
    this->_gpsPos[Y] = 0;
    this->_targetGpsPos[X] = -4.97;
    this->_targetGpsPos[Y] = 48.04;

    this->_acceleration[X] = 0;
    this->_acceleration[Y] = 0;
    this->_targetAcceleration[X] = 10.1;
    this->_targetAcceleration[Y] = 10.1;

    this->_angularVelocity = 0;
    this->_targetAngularVelocity = 10.1;

    this->_orientation = 0;
    this->_targetOrientation = EPSILON * 45;

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

    //    -   -   -   -   -   Subscription    -   -   -   -   -   //
    this->_gpsSub = this->create_subscription<sensor_msgs::msg::NavSatFix> \
        ("/aquabot/sensors/gps/gps/fix", \
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), \
        std::bind(&AquabotNode::_gpsDataCallback, this, std::placeholders::_1)
    );
    this->_imuSub = this->create_subscription<sensor_msgs::msg::Imu> \
        ("/aquabot/sensors/imu/imu/data", \
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), \
        std::bind(&AquabotNode::_imuDataCallback, this, std::placeholders::_1)
    );
    this->_pingerSub = this->create_subscription<ros_gz_interfaces::msg::ParamVec> \
        ("/aquabot/sensors/acoustics/receiver/range_bearing", \
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), \
        std::bind(&AquabotNode::_pingerDataCallback, this, std::placeholders::_1)
    );

    // callback loops
    this->_placeholderCallbackTimer = this->create_wall_timer(1s, std::bind(&AquabotNode::_placeholderCallback, this));
}

//  -   -   -   -   -   Main services   -   -   -   -   -   //

// test function
void    AquabotNode::_placeholderCallback() {

    static double                   TargetCameraPos = 0;
    static double                   TargetPos[2] = {MIN_THRUSTERS_POS, MAX_THRUSTERS_POS};
    static int                      TargetThrust[2] = {MAX_THRUSTERS_THRUST, MAX_THRUSTERS_THRUST};
    // double                          currentAcceleration[2];
    // double                          currentAngularVelocity;
    // double                          currentOrientation;


    // RCLCPP_INFO(this->get_logger(), "distance to parkour x = %f, y = %f", this->_targetGpsPos[X] - this->_gpsPos[X], this->_targetGpsPos[Y] - this->_gpsPos[Y]);
    // this->_getImuData(currentAcceleration, currentAngularVelocity, currentOrientation);
    // RCLCPP_INFO(this->get_logger(), "x %f, y %f, z %f, z rot %f", currentAcceleration[X], currentAcceleration[Y], currentAngularVelocity, currentOrientation);
    TargetCameraPos += EPSILON * 10;
    this->_setThrusterPos(TargetPos);
    this->_setThrusterThrust(TargetThrust);
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

// rework needed for optimisation...
void    AquabotNode::_setCameraPos(double NewPos) {

    std::lock_guard<std::mutex> lock(this->_cameraMutex);
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
        // RCLCPP_INFO(this->get_logger(), "Camera pos set to %f", NewPos);
    }
}

void    AquabotNode::_gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(this->_gpsMutex);

    this->_gpsPos[X] = msg->longitude;
    this->_gpsPos[Y] = msg->latitude;
}

void    AquabotNode::_getGpsData(double gpsPos[2]) {

    std::lock_guard<std::mutex> lock(this->_gpsMutex);

    gpsPos[X] = this->_gpsPos[X];
    gpsPos[Y] = this->_gpsPos[Y];
}

void    AquabotNode::_imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(this->_imuMutex);

    this->_acceleration[X] = msg->linear_acceleration.x;
    this->_acceleration[Y] = msg->linear_acceleration.y;
    this->_angularVelocity = msg->angular_velocity.z;
    this->_orientation = msg->orientation.z;
}

void    AquabotNode::_getImuData(double acceleration[2], double & angularVelocity, double & orientation) {

    std::lock_guard<std::mutex> lock(this->_imuMutex);

    acceleration[X] = this->_acceleration[X];
    acceleration[Y] = this->_acceleration[Y];
    angularVelocity = this->_angularVelocity;
    orientation = this->_orientation;
}

void    AquabotNode::_pingerDataCallback(const ros_gz_interfaces::msg::ParamVec::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(this->_pingerMutex);
    double range;
    double elevation;
    double bearing;

    for (const auto &param : msg->params) {

        if (param.name == "range")
            range = param.value.double_value;
        else if (param.name == "elevation")
            elevation = param.value.double_value;
        else if (param.name == "bearing")
            bearing = param.value.double_value;
    }
    RCLCPP_INFO(this->get_logger(), "range: %f, elevation: %f, bearing: %f", range, elevation, bearing);
}

void    AquabotNode::_getPingerData() {

    std::lock_guard<std::mutex> lock(this->_pingerMutex);
}
