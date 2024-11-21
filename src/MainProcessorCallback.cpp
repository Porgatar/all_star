#include "BabySharky.hpp"

#define SEARCH      1
#define RALLY       2
#define STABILIZE   3
#define TURN_AROUND 4
#define FINISHED    5

void    AquabotNode::_mainProcessorCallback() {

    int GlobalState;

    this->_getGlobalState(GlobalState);
    switch (GlobalState) {

        case SEARCH: {

            static int                          current = 0;
            std::list<std::array<double, 3>>    windTurbines;
            double                              BoatPos[2];
            double                              tmpPos[2];
            double                              distance;

            this->_getGpsData(BoatPos);
            this->_getWindTurbinData(windTurbines);

            std::list<std::array<double, 3>>::iterator  it = windTurbines.begin();
            std::advance(it, current);
            if (windTurbines.size() && it == windTurbines.end())
                this->_setGlobalState(RALLY);

            distance = std::sqrt(std::pow(BoatPos[X] - (*it)[X], 2) + std::pow(BoatPos[Y] - (*it)[Y], 2));
            RCLCPP_INFO(this->get_logger(), "distance = %f", distance);
            if (distance < 15.0) {

                current++;
                return ;
            }
            tmpPos[X] = (*it)[X];
            tmpPos[Y] = (*it)[Y];
            RCLCPP_INFO(this->get_logger(), "pos = %f, %f", tmpPos[X], tmpPos[Y]);
            this->_setTargetGpsData(tmpPos);
            break ;
        }
        case RALLY: {

            RCLCPP_INFO(this->get_logger(), "visited all Windturbins !");
            break ;
        }
        default:
            return ;
    }
}