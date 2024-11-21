#include "BabySharky.hpp"

#define SEARCH      1
#define RALLY       2
#define STABILIZE   3
#define TURN_AROUND 4
#define FINISHED    5

void    AquabotNode::_mainProcessorCallback() {

    int     GlobalState;
    double  targetPos[2];
    double  tmpPos[2];

    this->_getGlobalState(GlobalState);
    switch (GlobalState) {

        case SEARCH: {

            RCLCPP_INFO(this->get_logger(), "SEARCH PHASE !");

            static int                          current = 0;
            std::list<std::array<double, 3>>    windTurbines;
            double                              BoatPos[2];
            double                              distance;

            this->_getGpsData(BoatPos);
            this->_getWindTurbinData(windTurbines);

            std::list<std::array<double, 3>>::iterator  it = windTurbines.begin();
            std::advance(it, current);

            targetPos[X] = (*it)[X];
            targetPos[Y] = (*it)[Y];
            targetPos[X] -= std::cos((*it)[Z]) * 20;
            targetPos[Y] -= std::sin((*it)[Z]) * 20;
            distance = std::sqrt(std::pow(BoatPos[X] - targetPos[X], 2) + std::pow(BoatPos[Y] - targetPos[Y], 2));
            RCLCPP_INFO(this->get_logger(), "distance = %f, rot = %f", distance, (*it)[Z]);
            if (distance < 25.0) {

                current++;
                return ;
            }
            this->_getTargetGpsData(tmpPos);
            if (tmpPos[X] != targetPos[X] && tmpPos[Y] != targetPos[Y]) {

                RCLCPP_INFO(this->get_logger(), "pos = %f, %f", targetPos[X], targetPos[Y]);
                this->_setTargetGpsData(targetPos);
                this->_setTripState(0);
            }
            break ;
        }
        case RALLY: {

            RCLCPP_INFO(this->get_logger(), "RALLY PHASE !");
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