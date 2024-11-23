#include "BabySharky.hpp"

#define SEARCH      1
#define RALLY       2
#define STABILIZE   3
#define TURN_AROUND 4
#define FINISHED    5

void    AquabotNode::_setCameraToTarget(void) {

    double  boatPos[2];
    double  boatOrientation[3];
    double  targetPos[2];
    double  targetOrientation;

    this->_getGpsData(boatPos);
    this->_getTargetGpsData(targetPos);
    this->_getTargetOrientation(targetOrientation);
    targetPos[X] += std::cos(targetOrientation) * 10;
    targetPos[Y] += std::sin(targetOrientation) * 10;
    this->_getImuData(0, 0, boatOrientation);
    targetOrientation = atan2(targetPos[Y] - boatPos[Y], targetPos[X] - boatPos[X]) - boatOrientation[Z];
    this->_setCameraPos(targetOrientation);
}

void    AquabotNode::_mainProcessorCallback() {

    int     GlobalState;

    this->_getGlobalState(GlobalState);
    switch (GlobalState) {

        case SEARCH: {

            static std::chrono::time_point<std::chrono::steady_clock>   timer;
            static int      currentTurbin = 0;
            static double   additionalZ = 0;
            static bool     avoidance = 0;
            int             TripState;

            this->_getTripState(TripState);
            // RCLCPP_INFO(this->get_logger(), "%d", TripState);
            switch (TripState) {

                case -1: {


                    std::list<std::array<double, 3>>    windTurbines;
                    double                              targetPos[2];

                    this->_getWindTurbinData(windTurbines);
                    std::list<std::array<double, 3>>::iterator  it = windTurbines.begin();
                    std::advance(it, currentTurbin);

                    if (it == windTurbines.end())
                        return ;
                    targetPos[X] = (*it)[X];
                    targetPos[Y] = (*it)[Y];
                    targetPos[X] -= std::cos((*it)[Z] + additionalZ) * 12;
                    targetPos[Y] -= std::sin((*it)[Z] + additionalZ) * 12;
                    this->_setTargetGpsData(targetPos);
                    this->_setTargetOrientation((*it)[Z]);
                    this->_setTripState(0);
                    RCLCPP_INFO(this->get_logger(), "setting wt target");
                    return ;
                }
                case 0: {

                    this->_setCameraPos(0);
                    return ;
                }
                case 1: {

                    double  targetPos[2];
                    double  obstacle[2];
                    double  boatPos[2];
                    double  boatOrientation[3];
                    double  targetDistance;

                    this->_setCameraPos(0);
                    this->_getGpsData(boatPos);
                    this->_getTargetGpsData(targetPos);
                    this->_getAvoidanceTarget(obstacle);
                    targetDistance = std::sqrt(std::pow(targetPos[X] - boatPos[X], 2) + std::pow(targetPos[Y] - boatPos[Y], 2));
                    // if (obstacle[RANGE] < 1.0) // temp
                    //     return ;
                    if (obstacle[RANGE] > 70.0 || obstacle[RANGE] < 1.0 || obstacle[RANGE] > targetDistance - 10) {// || std::abs(obstacle[RANGE] - targetDistance) < 10.0) {

                        // RCLCPP_INFO(this->get_logger(), "rejected obstacle avoidance at %fm, %fr !", obstacle[RANGE], obstacle[BEARING]);
                        return ;
                    }
                    if (avoidance) {

                        RCLCPP_INFO(this->get_logger(), "already in avoidance");
                        return ;
                    }
                    avoidance = true;
                    this->_getImuData(0, 0, boatOrientation);
                    boatOrientation[Z] -= obstacle[BEARING];
                    targetPos[X] = boatPos[X] + std::cos(boatOrientation[Z]) * obstacle[RANGE];
                    targetPos[Y] = boatPos[Y] + std::sin(boatOrientation[Z]) * obstacle[RANGE];
                    this->_setTargetGpsData(targetPos);
                    this->_setTripState(0);
                    RCLCPP_INFO(this->get_logger(), "accepted obstacle avoidance at %fm, %fr !", obstacle[RANGE], obstacle[BEARING]);
                    return ;
                }
                case 2: {

                    return ;
                }
                case 3: {

                    return ;
                }
                case 4: {

                    static std::string  lastCode;
                    std::string         code;

                    this->_setCameraToTarget();
                    this->_getLastQrCode(code);
                    RCLCPP_INFO(this->get_logger(), "%lds passer", (std::chrono::steady_clock::now() - timer).count());
                    if (!code.empty() && code != lastCode) {

                        RCLCPP_INFO(this->get_logger(), "New QR Code detected: %s", code.c_str());

                        std_msgs::msg::String   msg;

                        msg.data = code.c_str();
                        this->_windTurbinCheckup->publish(msg);
                        lastCode = code;
                        currentTurbin++;
                        this->_setTripState(-1);
                    }
                    else if ((std::chrono::steady_clock::now() - timer).count() > 3) {

                        RCLCPP_INFO(this->get_logger(), "3s passer");

                        additionalZ += EPSILON * 45;
                        this->_setTripState(-1);
                    }
                    return ;
                }
                case 5: {

                    double  targetPos[2];
                    double  orientation;

                    if (avoidance) {

                        avoidance = false;
                        this->_setTripState(-1);
                        return ;
                    }
                    this->_getTargetOrientation(orientation);
                    this->_getTargetGpsData(targetPos);
                    targetPos[X] += std::cos(orientation) * 10;
                    targetPos[Y] += std::sin(orientation) * 10;
                    this->_setTargetGpsData(targetPos);
                    this->_setTripState(4);
                    this->_setCameraToTarget();
                    timer = std::chrono::steady_clock::now();
                    return ;
                }
            }
            return ;
        }
        case RALLY: {

            RCLCPP_INFO(this->get_logger(), "RALLY PHASE !");

            double  targetPos[2];
            double  tmpPos[2];

            targetPos[X] = 0;
            targetPos[Y] = 0;
            this->_getTargetGpsData(tmpPos);
            if (tmpPos[X] != targetPos[X] && tmpPos[Y] != targetPos[Y]) {

                // RCLCPP_INFO(this->get_logger(), "pos = %f, %f", targetPos[X], targetPos[Y]);
                this->_setTargetGpsData(targetPos);
                this->_setTripState(0);
            }
            return ;
        }
        default:
            return ;
    }
}
