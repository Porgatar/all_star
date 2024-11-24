#include "BabySharky.hpp"

#define STANDBY     0
#define SEARCH      1
#define RALLY       2
#define STABILIZE   3
#define TURN_AROUND 4
#define FINISHED    5

void    AquabotNode::_setCameraToTarget(const double & additionalZ) {

    double  boatPos[2];
    double  boatOrientation[3];
    double  targetPos[2];
    double  targetOrientation;

    this->_getGpsData(boatPos);
    this->_getTargetGpsData(targetPos);
    this->_getTargetOrientation(targetOrientation);
    targetPos[X] += std::cos(targetOrientation + additionalZ) * 10;
    targetPos[Y] += std::sin(targetOrientation + additionalZ) * 10;
    this->_getImuData(0, 0, boatOrientation);
    targetOrientation = atan2(targetPos[Y] - boatPos[Y], targetPos[X] - boatPos[X]) - boatOrientation[Z];
    this->_setCameraPos(targetOrientation);
}

void    AquabotNode::_stabilize(void) {

    RCLCPP_INFO(this->get_logger(), "stabilizing...");
}

void    AquabotNode::_turnAround(void) {

    RCLCPP_INFO(this->get_logger(), "Suite au 'je sais pas encore quoi mettre'");
    RCLCPP_INFO(this->get_logger(), "Ce Bateau n'iras malheureusement pas plus loin...");
    this->_finished();
}

void    AquabotNode::_finished(void) {

    RCLCPP_INFO(this->get_logger(), "Merci d'avoir selectionner la Team 42 all star !");
    RCLCPP_INFO(this->get_logger(), "Nous esperons que vous avez passer un agreable moment pendant ce dernier voyage de maintenance a bord de notre BabySharky_V0.1");
    RCLCPP_INFO(this->get_logger(), "Shuting down...");

    rclcpp::shutdown();
}

void    AquabotNode::_mainProcessorCallback(void) {

    static int  lastState = -1;
    int         GlobalState;

    this->_getGlobalState(GlobalState);
    switch (GlobalState) {

        case STANDBY: {

            if (GlobalState != lastState)
                RCLCPP_INFO(this->get_logger(), "STANDBY !");
            break ;
        }
        case SEARCH: {

            if (GlobalState != lastState)
                RCLCPP_INFO(this->get_logger(), "SEARCH PHASE !");
            this->_search();
            break ;
        }
        case RALLY: {

            if (GlobalState != lastState)
                RCLCPP_INFO(this->get_logger(), "RALLY PHASE !");
            this->_rally();
            break ;
        }
        case STABILIZE: {

            if (GlobalState != lastState)
                RCLCPP_INFO(this->get_logger(), "STABILIZE PHASE !");
            this->_stabilize();
            break ;
        }
        case TURN_AROUND: {

            if (GlobalState != lastState)
                RCLCPP_INFO(this->get_logger(), "TURN_AROUND PHASE !");
            this->_turnAround();
            break ;
        }
        case FINISHED: {

            if (GlobalState != lastState)
                RCLCPP_INFO(this->get_logger(), "FINISHED PHASE !");
            this->_finished();
            break ;
        }
        default: {

            RCLCPP_INFO(this->get_logger(), "UNKNOWN PHASE !");
            break ;
        }
    }
    lastState = GlobalState;
}
