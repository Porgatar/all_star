#include "BabySharky.hpp"

void    AquabotNode::_search(void) {

    static rclcpp::Time timer;
    static int          currentTurbin = 0;
    static double       additionalZ = 0;
    static bool         avoidance = false;
    int                 TripState;

    this->_getTripState(TripState);
    switch (TripState) {

        case -1: {

            std::list<std::array<double, 3>>    windTurbines;
            double                              targetPos[2];

            this->_getWindTurbinData(windTurbines);
            std::list<std::array<double, 3>>::iterator  it = windTurbines.begin();
            std::advance(it, currentTurbin);

            if (it == windTurbines.end()) {

                currentTurbin = 0;
                return ;
            }
            targetPos[X] = (*it)[X];
            targetPos[Y] = (*it)[Y];
            targetPos[X] -= std::cos((*it)[Z] + additionalZ) * 15;
            targetPos[Y] -= std::sin((*it)[Z] + additionalZ) * 15;
            this->_setTargetGpsData(targetPos);
            this->_setTargetOrientation((*it)[Z]);
            this->_setTripState(0);
            RCLCPP_INFO(this->get_logger(), "setting new windturbin target");
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
            if (obstacle[RANGE] > 70.0 || obstacle[RANGE] < 1.0 || obstacle[RANGE] > targetDistance - 5)
                return ;
            avoidance = true;
            this->_getImuData(0, 0, boatOrientation);
            boatOrientation[Z] -= obstacle[BEARING];
            targetPos[X] = boatPos[X] + std::cos(boatOrientation[Z]) * (obstacle[RANGE] * 3);
            targetPos[Y] = boatPos[Y] + std::sin(boatOrientation[Z]) * (obstacle[RANGE] * 3);
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
            double              elapsed_seconds;

            this->_setCameraToTarget(additionalZ);
            this->_getLastQrCode(code);
            elapsed_seconds = (this->get_clock()->now() - timer).seconds();

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
            else if (elapsed_seconds > 3) {

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
            timer = this->get_clock()->now();
            return ;
        }
    }
}