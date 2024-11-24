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
    targetPos[X] += std::cos(targetOrientation + additionalZ) * 15;
    targetPos[Y] += std::sin(targetOrientation + additionalZ) * 15;
    this->_getImuData(0, 0, boatOrientation);
    targetOrientation = atan2(targetPos[Y] - boatPos[Y], targetPos[X] - boatPos[X]) - boatOrientation[Z];
    this->_setCameraPos(targetOrientation);
}

void    AquabotNode::_search(void) {

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
            targetPos[X] -= std::cos((*it)[Z] + additionalZ) * 15;
            targetPos[Y] -= std::sin((*it)[Z] + additionalZ) * 15;
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
            if (obstacle[RANGE] > 70.0 || obstacle[RANGE] < 1.0 || obstacle[RANGE] > targetDistance - 5) {// || std::abs(obstacle[RANGE] - targetDistance) < 10.0) {

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

            this->_setCameraToTarget(additionalZ);
            return ;
        }
        case 3: {

            this->_setCameraToTarget(additionalZ);
            return ;
        }
        case 4: {

            static std::string  lastCode;
            std::string         code;
            long int            elapsed_seconds;

            this->_setCameraToTarget(additionalZ);
            this->_getLastQrCode(code);
            elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - timer).count();

            if (!code.empty() && code != lastCode) {

                RCLCPP_INFO(this->get_logger(), "New QR Code detected: %s", code.c_str());

                std_msgs::msg::String   msg;

                msg.data = code.c_str();
                this->_windTurbinCheckup->publish(msg);
                lastCode = code;
                currentTurbin++;
                additionalZ = 0;
                this->_setTripState(-1);
            }
            else if (elapsed_seconds > 1) {

                RCLCPP_INFO(this->get_logger(), "1s passer");

                additionalZ += EPSILON * 90;
                if (additionalZ > EPSILON * 271)
                    additionalZ = 0;
                this->_setTripState(-1);
            }
            return ;
        }
        case 5: {

            double  targetPos[2];
            double  orientation;

            this->_setCameraToTarget(additionalZ);
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
            timer = std::chrono::steady_clock::now();
            return ;
        }
    }
}

void    AquabotNode::_rally(void) {

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
}

void    AquabotNode::_stabilize(void) {

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
}

void    AquabotNode::_turnAround(void) {

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
}

void    AquabotNode::_finished(void) {

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
}

void    AquabotNode::_mainProcessorCallback(void) {

    static int  lastState = -1;
    int         GlobalState;

    this->_getGlobalState(GlobalState);
    switch (GlobalState) {

        case STANDBY:
            break ;
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
