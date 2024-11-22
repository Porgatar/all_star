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

            // RCLCPP_INFO(this->get_logger(), "SEARCH PHASE !");

            std::list<std::array<double, 3>>    windTurbines;
            static int                          currentTurbin = 0;
            double                              targetPos[2];
            int                                 TripState;

            this->_getTripState(TripState);
            switch (TripState) {

                case -1: {

                    RCLCPP_INFO(this->get_logger(), "-1");

                    this->_getWindTurbinData(windTurbines);
                    std::list<std::array<double, 3>>::iterator  it = windTurbines.begin();
                    std::advance(it, currentTurbin);

                    if (it == windTurbines.end())
                        return ;
                    targetPos[X] = (*it)[X];
                    targetPos[Y] = (*it)[Y];
                    targetPos[X] -= std::cos((*it)[Z]) * 30;
                    targetPos[Y] -= std::sin((*it)[Z]) * 30;
                    this->_setTargetGpsData(targetPos);
                    this->_setTargetOrientation((*it)[Z]);
                    this->_setTripState(0);
                    break ;
                }
                case 0: {

                    RCLCPP_INFO(this->get_logger(), "0");

                    break ;
                }
                case 1: {

                    RCLCPP_INFO(this->get_logger(), "1");

                    this->_setCameraPos(0);
                    break ;
                }
                case 2: {

                    RCLCPP_INFO(this->get_logger(), "2");

                    break ;
                }
                case 3: {

                    RCLCPP_INFO(this->get_logger(), "3");

                    break ;
                }
                case 4: {

                    RCLCPP_INFO(this->get_logger(), "4");

                    static std::string  lastCode;
                    std::string         code;

                    this->_setCameraToTarget();
                    this->_getLastQrCode(code);
                    if (!code.empty() && code != lastCode) {

                        RCLCPP_INFO(this->get_logger(), "New QR Code detected: %s", code.c_str());

                        std_msgs::msg::String   msg;

                        msg.data = code.c_str();
                        this->_windTurbinCheckup->publish(msg);
                        lastCode = code;
                        currentTurbin++;
                        this->_setTripState(-1);
                    }
                    break ;
                }
                case 5: {

                    RCLCPP_INFO(this->get_logger(), "5");

                    double  orientation;

                    this->_getTargetOrientation(orientation);
                    this->_getTargetGpsData(targetPos);
                    targetPos[X] += std::cos(orientation) * 20;
                    targetPos[Y] += std::sin(orientation) * 20;
                    this->_setTargetGpsData(targetPos);
                    this->_setTripState(4);
                    this->_setCameraToTarget();
                    break ;
                }
            }
            break ;
        }
        case RALLY: {

            RCLCPP_INFO(this->get_logger(), "RALLY PHASE !");

            double  targetPos[2];
            double  tmpPos[2];

            targetPos[X] = 0;
            targetPos[Y] = 0;
            this->_getTargetGpsData(tmpPos);
            if (tmpPos[X] != targetPos[X] && tmpPos[Y] != targetPos[Y]) {

                RCLCPP_INFO(this->get_logger(), "pos = %f, %f", targetPos[X], targetPos[Y]);
                this->_setTargetGpsData(targetPos);
                this->_setTripState(0);
            }
            break ;
        }
        default:
            return ;
    }
}
