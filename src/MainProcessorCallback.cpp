#include "BabySharky.hpp"

#define SEARCH      1
#define RALLY       2
#define STABILIZE   3
#define TURN_AROUND 4
#define FINISHED    5

void    AquabotNode::_mainProcessorCallback() {

    int     GlobalState;

    this->_getGlobalState(GlobalState);
    switch (GlobalState) {

        case SEARCH: {

            RCLCPP_INFO(this->get_logger(), "SEARCH PHASE !");

            std::list<std::array<double, 3>>    windTurbines;
            static int                          currentTurbin = 0;
            double                              targetPos[2];
            int                                 TripState;

            this->_getTripState(TripState);
            switch (TripState) {

                case -1: {

                    RCLCPP_INFO(this->get_logger(), "searching next target...");

                    this->_getWindTurbinData(windTurbines);
                    std::list<std::array<double, 3>>::iterator  it = windTurbines.begin();
                    std::advance(it, currentTurbin);

                    targetPos[X] = (*it)[X];
                    targetPos[Y] = (*it)[Y];
                    targetPos[X] -= std::cos((*it)[Z]) * 15;
                    targetPos[Y] -= std::sin((*it)[Z]) * 15;
                    this->_setTargetGpsData(targetPos);
                    this->_setTargetOrientation((*it)[Z]);
                    this->_setTripState(0);
                    break ;
                }
                case 1: {

                    RCLCPP_INFO(this->get_logger(), "moving to target...");

                    this->_setCameraState(OBSTACLE_DETECTOR);
                    break ;
                }
                case 2: {

                    RCLCPP_INFO(this->get_logger(), "rotating camera to target...");

                    this->_setCameraState(QR_DETECTOR);
                    break ;
                }
                case 4: {

                    RCLCPP_INFO(this->get_logger(), "stabilization...");
                    break ;
                }
            }
            // RCLCPP_INFO(this->get_logger(), "distance = %f, rot = %f", distance, (*it)[Z]);
            // if (distance < 15.0) {

            //     this->_setCameraState(QR_DETECTOR);
            //     return ;
            // }
            // if (distance < 5.0) {

            //     this->_setCameraState(OBSTACLE_DETECTOR);
            //     current++;
            //     return ;
            // }
            // if ()
            // this->_getTargetGpsData(tmpPos);
            // if (tmpPos[X] != targetPos[X] && tmpPos[Y] != targetPos[Y]) {

            //     RCLCPP_INFO(this->get_logger(), "pos = %f, %f", targetPos[X], targetPos[Y]);
            //     this->_setTargetGpsData(targetPos);
            //     this->_setTripState(0);
            // }
            break ;
        }
        case RALLY: {

            RCLCPP_INFO(this->get_logger(), "RALLY PHASE !");
            // targetPos[X] = 0;
            // targetPos[Y] = 0;
            // this->_getTargetGpsData(tmpPos);
            // if (tmpPos[X] != targetPos[X] && tmpPos[Y] != targetPos[Y]) {

            //     RCLCPP_INFO(this->get_logger(), "pos = %f, %f", targetPos[X], targetPos[Y]);
            //     this->_setTargetGpsData(targetPos);
            //     this->_setTripState(0);
            // }
            break ;
        }
        default:
            return ;
    }
}
