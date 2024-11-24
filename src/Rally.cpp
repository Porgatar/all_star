#include "BabySharky.hpp"

void    AquabotNode::_rally(void) {

    static bool     targetSet = false;
    static bool     avoidance = false;
    static double   additionalZ = 0;
    int             TripState;

    if (!targetSet) {

        this->_setTripState(-1);
        targetSet = true;
    }
    this->_getTripState(TripState);
    switch (TripState) {

        case -1: {

            double  criticalWindTurbin[2];
            double  targetPos[2];
            double  boatOrientation[3];

            this->_getCriticalWindTurbinData(criticalWindTurbin);
            this->_getGpsData(targetPos);
            this->_getImuData(0, 0, boatOrientation);
            boatOrientation[Z] += criticalWindTurbin[BEARING];
            targetPos[X] += std::cos(boatOrientation[Z]) * criticalWindTurbin[RANGE] - (std::cos(boatOrientation[Z]) * 10);
            targetPos[Y] += std::sin(boatOrientation[Z]) * criticalWindTurbin[RANGE] - (std::cos(boatOrientation[Z]) * 10);
            this->_setTargetGpsData(targetPos);
            this->_setTargetOrientation(boatOrientation[Z]);
            this->_setTripState(0);
            RCLCPP_INFO(this->get_logger(), "setting new windturbin target at %fm, %fr", criticalWindTurbin[RANGE], criticalWindTurbin[BEARING]);
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
            if (obstacle[RANGE] > 50.0 || obstacle[RANGE] < 1.0 || obstacle[RANGE] > targetDistance - 5)
                return ;
            avoidance = true;
            this->_getImuData(0, 0, boatOrientation);
            boatOrientation[Z] -= obstacle[BEARING];
            targetPos[X] = boatPos[X] + std::cos(boatOrientation[Z]) * (obstacle[RANGE] * 3);
            targetPos[Y] = boatPos[Y] + std::sin(boatOrientation[Z]) * (obstacle[RANGE] * 3);
            this->_setTargetGpsData(targetPos);
            this->_setTargetOrientation(boatOrientation[Z]);
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

            this->_setCameraToTarget(additionalZ);
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
            return ;
        }
    }
}
