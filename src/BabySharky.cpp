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

    this->_criticalWindTurbin[RANGE] = 0;
    this->_criticalWindTurbin[BEARING] = 0;

    this->_globalState = 0;
    this->_statmentTrip = 0;
    this->_cameraState = 0;

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

    // Camera
    this->_windTurbinCheckup = this->create_publisher<std_msgs::msg::String> \
        ("/vrx/windturbinesinspection/windturbine_checkup", \
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
    this->_imageSub = this->create_subscription<sensor_msgs::msg::Image> \
        ("/aquabot/sensors/cameras/main_camera_sensor/image_raw", \
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), \
        std::bind(&AquabotNode::_imageDataCallback, this, std::placeholders::_1)
    );
    this->_windTurbinSub = this->create_subscription<geometry_msgs::msg::PoseArray> \
        ("/aquabot/ais_sensor/windturbines_positions", \
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), \
        std::bind(&AquabotNode::_windTurbinDataCallback, this, std::placeholders::_1)
    );
    this->_criticalWindTurbinSub = this->create_subscription<ros_gz_interfaces::msg::ParamVec> \
        ("/aquabot/sensors/acoustics/receiver/range_bearing", \
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), \
        std::bind(&AquabotNode::_criticalWindTurbinDataCallback, this, std::placeholders::_1)
    );
    this->_globalStateSub = this->create_subscription<std_msgs::msg::UInt32> \
        ("/vrx/windturbinesinspection/current_phase", \
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), \
        std::bind(&AquabotNode::_globalStateDataCallback, this, std::placeholders::_1)
    );

    // callback loops
    this->_targetStanCallbackTimer = this->create_wall_timer(1ms, std::bind(&AquabotNode::_targetStanCallback, this));
    this->_imageProcessorCallbackTimer = this->create_wall_timer(64ms, std::bind(&AquabotNode::_imageProcessorCallback, this));
    this->_mainProcessorCallbackTimer = this->create_wall_timer(1s, std::bind(&AquabotNode::_mainProcessorCallback, this));
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

void    AquabotNode::_imageDataCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(this->_lastFrameMutex);

    this->_lastFrame = cv_bridge::toCvCopy(msg, "bgr8")->image;
}

void    AquabotNode::_windTurbinDataCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(this->_windTurbinMutex);
    std::array<double, 7>       data;

    this->_windTurbinGpsPos.clear();
    for (size_t i = 0; i < msg->poses.size(); i++) {

        data[X] = msg->poses[i].position.x;
        data[Y] = msg->poses[i].position.y;
        data[Z] = msg->poses[i].position.z;
        data[RX] = msg->poses[i].orientation.x;
        data[RY] = msg->poses[i].orientation.y;
        data[RZ] = msg->poses[i].orientation.z;
        data[RW] = msg->poses[i].orientation.w;
        this->_windTurbinGpsPos.push_back(data);
    }
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

void    AquabotNode::_globalStateDataCallback(const std_msgs::msg::UInt32::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(this->_globalStateMutex);

    this->_globalState = msg->data;
}

//  -   -   -   -   -   Sensors Getters  -   -   -   -   -   //

void    AquabotNode::_getGpsData(double gpsPos[2]) {

    std::lock_guard<std::mutex> lock(this->_gpsMutex);

    gpsPos[X] = this->_gpsPos[X];
    gpsPos[Y] = this->_gpsPos[Y];
    this->_degToMeter(gpsPos);
}

void    AquabotNode::_getTargetGpsData(double gpsPos[2]) {

    std::lock_guard<std::mutex> lock(this->_targetGpsMutex);

    gpsPos[X] = this->_targetGpsPos[X];
    gpsPos[Y] = this->_targetGpsPos[Y];
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

void    AquabotNode::_getImageData(cv::Mat &frame) {

    std::lock_guard<std::mutex> lock(this->_lastFrameMutex);

    frame = this->_lastFrame.clone();
}

void    AquabotNode::_getAvoidanceTarget(double target[2]) {

    std::lock_guard<std::mutex> lock(this->_avoidanceTargetMutex);

    target[RANGE] = this->_avoidanceTarget[RANGE];
    target[BEARING] = this->_avoidanceTarget[BEARING];
}

void    AquabotNode::_getWindTurbinData(std::list<std::array<double, 3>> &pos) {

    std::lock_guard<std::mutex> lock(this->_windTurbinMutex);
    std::array<double, 3>       data;
    double                      degree[2];
    double                      rx, ry, rz, rw;

    pos.clear();
    for (const std::array<double, 7> entry : this->_windTurbinGpsPos) {

        rx = entry[RX], ry = entry[RY], rz = entry[RZ], rw = entry[RW];
        degree[X] = entry[Y]; // it works like this so dont touch it !
        degree[Y] = entry[X];
        this->_degToMeter(degree);
        data[X] = degree[X];
        data[Y] = degree[Y];
        data[Z] = atan2(2.0 * (rw * rz + rx * ry), 1.0 - 2.0 * (ry * ry + rz * rz));
        pos.push_back(data);
    }
}

void    AquabotNode::_getCriticalWindTurbinData(double criticalWindTurbin[2]) {

    std::lock_guard<std::mutex> lock(this->_criticalWindTurbinMutex);

    criticalWindTurbin[RANGE] = this->_criticalWindTurbin[RANGE];
    criticalWindTurbin[BEARING] = this->_criticalWindTurbin[BEARING];
}

void    AquabotNode::_getGlobalState(int &state) {

    std::lock_guard<std::mutex> lock(this->_globalStateMutex);

    state = this->_globalState;
}

void    AquabotNode::_getTripState(int &state) {

    std::lock_guard<std::mutex> lock(this->_tripStateMutex);

    state = this->_statmentTrip;
}

void    AquabotNode::_getCameraState(int &state) {

    std::lock_guard<std::mutex> lock(this->_cameraStateMutex);

    state = this->_cameraState;
}

//  -   -   -   -   -   Setters -   -   -   -   -   //

void    AquabotNode::_setAvoidanceTarget(const double target[2]) {

    std::lock_guard<std::mutex> lock(this->_avoidanceTargetMutex);

    this->_avoidanceTarget[RANGE] = target[RANGE];
    this->_avoidanceTarget[BEARING] = target[BEARING];
}

void    AquabotNode::_setGlobalState(const int &state) {

    std::lock_guard<std::mutex> lock(this->_globalStateMutex);

    this->_globalState = state;
}

void    AquabotNode::_setTripState(const int &state) {

    std::lock_guard<std::mutex> lock(this->_tripStateMutex);

    this->_statmentTrip = state;
}

void    AquabotNode::_setCameraState(const int &state) {

    std::lock_guard<std::mutex> lock(this->_cameraStateMutex);

    this->_cameraState = state;
}

void    AquabotNode::_setTargetGpsData(const double gpsPos[2]) {

    std::lock_guard<std::mutex> lock(this->_targetGpsMutex);

    this->_targetGpsPos[X] = gpsPos[X];
    this->_targetGpsPos[Y] = gpsPos[Y];
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
