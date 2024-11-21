#include "BabySharky.hpp"

AquabotNode::AquabotNode() : Node("all_star") {

    RCLCPP_INFO(this->get_logger(), "Hello world from baby-sharky node in cpp!");

    // placeholder targets...
    this->_gpsOrigin[X] = 0;
    this->_gpsOrigin[Y] = 0;
    this->_gpsPos[X] = 0;
    this->_gpsPos[Y] = 0;
    this->_targetGpsPos[X] = 0;
    this->_targetGpsPos[Y] = 0;

    this->_acceleration[X] = 0;
    this->_acceleration[Y] = 0;

    this->_angularVelocity[X] = 0;
    this->_angularVelocity[Y] = 0;
    this->_angularVelocity[Z] = 0;

    this->_orientation[X] = 0;
    this->_orientation[Y] = 0;
    this->_orientation[Z] = 0;
    this->_orientation[W] = 0;
    this->_targetOrientation[X] = 0;
    this->_targetOrientation[Y] = 0;
    this->_targetOrientation[Z] = 0;

    this->_criticalWindTurbin[RANGE] = 0;
    this->_criticalWindTurbin[BEARING] = 0;

    this->_statementTrip = 0;
    this->_lastFrame = 0;

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
    this->_criticalWindTurbinSub = this->create_subscription<ros_gz_interfaces::msg::ParamVec> \
        ("/aquabot/sensors/acoustics/receiver/range_bearing", \
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), \
        std::bind(&AquabotNode::_criticalWindTurbinDataCallback, this, std::placeholders::_1)
    );
    this->_imageSub = this->create_subscription<sensor_msgs::msg::Image> \
        ("/aquabot/sensors/cameras/main_camera_sensor/image_raw", \
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), \
        std::bind(&AquabotNode::_imageDataCallback, this, std::placeholders::_1)
    );

    // callback loops
    this->_targetStanCallbackTimer = this->create_wall_timer(1ms, std::bind(&AquabotNode::_targetStanCallback, this));
    this->_imageProcessorCallbackTimer = this->create_wall_timer(32ms, std::bind(&AquabotNode::_imageProcessorCallback, this));
}

//  -   -   -   -   -   Thrusters Publisher   -   -   -   -   -   //

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

//  -   -   -   -   -   Sensors Publisher   -   -   -   -   -   //

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

//  -   -   -   -   -   Sensors Subscribers  -   -   -   -   -   //

void    AquabotNode::_gpsDataCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(this->_gpsMutex);

    if (!this->_gpsOrigin[X]) {

        this->_gpsOrigin[X] = msg->longitude;
        this->_gpsOrigin[Y] = msg->latitude;
    }
    this->_gpsPos[X] = msg->longitude;
    this->_gpsPos[Y] = msg->latitude;
}

void    AquabotNode::_imuDataCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(this->_imuMutex);

    this->_acceleration[X] = msg->linear_acceleration.x;
    this->_acceleration[Y] = msg->linear_acceleration.y;
    this->_angularVelocity[X] = msg->angular_velocity.x;
    this->_angularVelocity[Y] = msg->angular_velocity.y;
    this->_angularVelocity[Z] = msg->angular_velocity.z;
    this->_orientation[X] = msg->orientation.x;
    this->_orientation[Y] = msg->orientation.y;
    this->_orientation[Z] = msg->orientation.z;
    this->_orientation[W] = msg->orientation.w;
}

void    AquabotNode::_criticalWindTurbinDataCallback(const ros_gz_interfaces::msg::ParamVec::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(this->_criticalWindTurbinMutex);

    for (const auto &param : msg->params) {

        if (param.name == "range")
            this->_criticalWindTurbin[RANGE] = param.value.double_value;
        else if (param.name == "bearing")
            this->_criticalWindTurbin[BEARING] = param.value.double_value;
    }
}

void    AquabotNode::_imageDataCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(this->_lastFrameMutex);

    this->_lastFrame = cv_bridge::toCvCopy(msg, "bgr8")->image;
}

//  -   -   -   -   -   Sensors Getters  -   -   -   -   -   //

void    AquabotNode::_getGpsData(double gpsPos[2]) {

    std::lock_guard<std::mutex> lock(this->_gpsMutex);

    gpsPos[X] = this->_gpsPos[X];
    gpsPos[Y] = this->_gpsPos[Y];
    this->_degToMeter(gpsPos);
}

void    AquabotNode::_getImuData(double acceleration[2], double angularVelocity[3], double orientation[3]) {

    std::lock_guard<std::mutex> lock(this->_imuMutex);
    double                      rx = this->_orientation[X];
    double                      ry = this->_orientation[Y];
    double                      rz = this->_orientation[Z];
    double                      rw = this->_orientation[W];

    if (acceleration) { // not actual speed

        acceleration[X] = this->_acceleration[X];
        acceleration[Y] = this->_acceleration[Y];
    }
    if (angularVelocity) {

        angularVelocity[X] = this->_angularVelocity[X];
        angularVelocity[Y] = this->_angularVelocity[Y];
        angularVelocity[Z] = this->_angularVelocity[Z];
    }
    if (orientation) {

        orientation[X] = atan2(2.0 * (rw * rx + ry * rz), 1.0 - 2.0 * (rx * rx + ry * ry));
        orientation[Y] = asin(2.0 * (rw * ry - rz * rx));
        orientation[Z] = atan2(2.0 * (rw * rz + rx * ry), 1.0 - 2.0 * (ry * ry + rz * rz));
    }
}

void    AquabotNode::_getCriticalWindTurbinData(double criticalWindTurbin[2]) {

    std::lock_guard<std::mutex> lock(this->_criticalWindTurbinMutex);

    criticalWindTurbin[RANGE] = this->_criticalWindTurbin[RANGE];
    criticalWindTurbin[BEARING] = this->_criticalWindTurbin[BEARING];
}

void    AquabotNode::_getImageData(cv::Mat &frame) {

    std::lock_guard<std::mutex> lock(this->_lastFrameMutex);

    frame = this->_lastFrame.clone();
}

void    AquabotNode::_getAvoidanceOrientation(double & angle) {

    std::lock_guard<std::mutex> lock(this->_avoidanceOrientationMutex);

    angle = this->_avoidanceOrientation;
}

//  -   -   -   -   -   Sensors Setters  -   -   -   -   -   //

void    AquabotNode::_setAvoidanceOrientation(const double & angle) {

    std::lock_guard<std::mutex> lock(this->_avoidanceOrientationMutex);

    this->_avoidanceOrientation = angle;
}

//  -   -   -   -   -   utils  -   -   -   -   -   //

void    AquabotNode::_degToMeter(double gpsPos[2]) {

    std::lock_guard<std::mutex> lock(this->_gpsOriginMutex);
    double                      newGpsPos[2];

    newGpsPos[X] = (gpsPos[X] - this->_gpsOrigin[X]) * 111320 * cos(gpsPos[Y] * M_PI / 180);
    newGpsPos[Y] = (gpsPos[Y] - this->_gpsOrigin[Y]) * 111320;
    gpsPos[X] = newGpsPos[X];
    gpsPos[Y] = newGpsPos[Y];
}

void    AquabotNode::_meterToDeg(double gpsPos[2]) {

    std::lock_guard<std::mutex> lock(this->_gpsOriginMutex);
    double                      newGpsPos[2];

    newGpsPos[Y] = this->_gpsOrigin[Y] + gpsPos[Y] / 111320;
    newGpsPos[X] = this->_gpsOrigin[X] + gpsPos[X] / (111320 * cos(newGpsPos[Y] * M_PI / 180));
    gpsPos[Y] = newGpsPos[Y];
    gpsPos[X] = newGpsPos[X];
}
