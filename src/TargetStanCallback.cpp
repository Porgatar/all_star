#include "BabySharky.hpp"

static double	dist(double ax, double ay, double bx, double by)
{
	return (sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay)));
}

void	AquabotNode::_targetStanCallback() {

 // --------STATE----------- //
	std::lock_guard<std::mutex> lock(this->_tripStateMutex);
 // --------THRUST---------- //
	int		Thrust[2];
	double	setThrusterPos[2];
 // --------GPS------------- //
	double	gpsPos[2];
	double	targetGpsPos[2]; // target
 // --------IMU_DATA-------- //
	double	orientation[3];
	double	acceleration[2];
	double	angularVelocity[3];
 // --------GETTEUR--------- //
	this->_getGpsData(gpsPos);
	this->_getTargetGpsData(targetGpsPos);
	this->_getImuData(acceleration, angularVelocity, orientation);
 // --------UTILS_VAR------- //
	double	distance =  dist(gpsPos[0], gpsPos[1], targetGpsPos[0], targetGpsPos[1]);
	double	orientationTarget = atan2(targetGpsPos[1] - gpsPos[1], targetGpsPos[0] - gpsPos[0]);
	double	delta_orientation = -1 * (orientationTarget - orientation[Z]);
 // --------PRINTEUR-------- //
	// if (this->_statementTrip < 3)
		// RCLCPP_INFO(this->get_logger(), "distance = %f statment %d", distance, this->_statementTrip);
	// RCLCPP_INFO(this->get_logger(), "delta_o %f", delta_orientation);
	// RCLCPP_INFO(this->get_logger(), "gpspos = %f;%f", gpsPos[0], gpsPos[1]);
	// RCLCPP_INFO(this->get_logger(), "accel %f %f velo %f", acceleration[0], acceleration[1], angularVelocity);
	// RCLCPP_INFO(this->get_logger(), "targer_o = %f o = %f resultat = %f", orientationTarget, orientation, orientationTarget - orientation);

	if (this->_statementTrip == 0) { // premiere fase du trajet -> stab sur l'orientation

		if (delta_orientation <  5 * EPSILON && delta_orientation > 5 * -EPSILON) {

			this->_statementTrip++;
			return ;

		}

		int	delta_power = (int)(((delta_orientation) / EPSILON) * 10);

		if (delta_orientation > 40 * EPSILON) {

			setThrusterPos[LEFT] = delta_orientation;
			setThrusterPos[RIGHT] = delta_orientation * -1;

			Thrust[LEFT] = delta_power;
			Thrust[RIGHT] = 0;

		} else if (delta_orientation < 40 * -EPSILON) {

			setThrusterPos[LEFT] = delta_orientation * -1;
			setThrusterPos[RIGHT] = delta_orientation;

			Thrust[LEFT] = 0;
			Thrust[RIGHT] = delta_power * -1;

		} else {

			if (delta_orientation < 0) {

				setThrusterPos[LEFT] = delta_orientation * -1;
				setThrusterPos[RIGHT] = delta_orientation;

				Thrust[LEFT] = (int)(delta_power * -0.3 * abs(angularVelocity[Z]));
				Thrust[RIGHT] = delta_power * -1;

			} else {

				setThrusterPos[LEFT] = delta_orientation;
				setThrusterPos[RIGHT] = delta_orientation * -1;

				Thrust[LEFT] = delta_power;
				Thrust[RIGHT] = (int)(delta_power * 0.3 * abs(angularVelocity[Z]));

			}

		}
		this->_setThrusterPos(setThrusterPos);
		this->_setThrusterThrust(Thrust);

	} else if (this->_statementTrip == 1) { // seconde fase -> deplacment sur site

		if (distance < 6)	{
			this->_statementTrip++;
			return ;
		}

		if (delta_orientation > 10 * EPSILON || delta_orientation < 10 * -EPSILON) {

			this->_statementTrip = 0;
			return ;

		} else if (delta_orientation > 2 * EPSILON || delta_orientation < 2 * -EPSILON || distance < 15) {

			if (delta_orientation < 0) {

				setThrusterPos[LEFT] = delta_orientation ;
				setThrusterPos[RIGHT] = (delta_orientation) * -0.1;
				Thrust[LEFT] = 1900;
				Thrust[RIGHT] = 2000;

			} else {

				setThrusterPos[LEFT] = (delta_orientation) * 0.1;
				setThrusterPos[RIGHT] = delta_orientation;
				Thrust[LEFT] = 2000;
				Thrust[RIGHT] = 1900;


			}
			this->_setThrusterPos(setThrusterPos);
			this->_setThrusterThrust(Thrust);
			return ;

		}

		if (delta_orientation < 0) {

			setThrusterPos[LEFT] = 0 ;
			setThrusterPos[RIGHT] = (delta_orientation) * -0.01;

		} else {

			setThrusterPos[LEFT] = (delta_orientation) * 0.01;
			setThrusterPos[RIGHT] = 0;

		}
		this->_setThrusterPos(setThrusterPos);

		Thrust[LEFT] = 5000;
		Thrust[RIGHT] = 5000;
		this->_setThrusterThrust(Thrust);

	} else if (this->_statementTrip == 2) { // troisieme fase -> frein

		if (distance < 5.9) {

			if (((abs(acceleration[0]) + abs(acceleration[1])) * 0.5) < 0.6) {

				this->_statementTrip++;
				return ;

			}

			Thrust[LEFT] = -5000;
			Thrust[RIGHT] = -5000;
			this->_setThrusterThrust(Thrust);

		} else {

			setThrusterPos[LEFT] = 0;
			setThrusterPos[RIGHT] = 0;
			this->_setThrusterPos(setThrusterPos);

			Thrust[LEFT] = -5000;
			Thrust[RIGHT] = -5000;
			this->_setThrusterThrust(Thrust);

		}

	} else if (this->_statementTrip == 3) { // quatrieme fase -> goto point petite vitesse

		if (distance < 2) {

			this->_statementTrip = 5;
			return ;

		} else {

			Thrust[LEFT] = (int)(30 * (distance)* 0.1);
			Thrust[RIGHT] = (int)(30  * (distance)* 0.1);
			this->_setThrusterThrust(Thrust);

			setThrusterPos[LEFT] = delta_orientation;
			setThrusterPos[RIGHT] = delta_orientation;
			this->_setThrusterPos(setThrusterPos);

		}
	} else if (this->_statementTrip == 4) { // cinquieme fase -> stab sur site

		if (distance < 9.99) {

			setThrusterPos[LEFT] = delta_orientation * -1;
			setThrusterPos[RIGHT] = delta_orientation * -1;

			Thrust[LEFT] = (int)(-300 * abs(distance - 10));
			Thrust[RIGHT] = (int)(-300 * abs(distance - 10));

		} else if (distance > 10.01) {

			setThrusterPos[LEFT] = delta_orientation;
			setThrusterPos[RIGHT] = delta_orientation;

			Thrust[LEFT] = (int)(300 * (distance - 10));
			Thrust[RIGHT] = (int)(300 * (distance - 10));

		}

		this->_setThrusterPos(setThrusterPos);
		this->_setThrusterThrust(Thrust);

	} else if (this->_statementTrip == 5) { // standby

		setThrusterPos[LEFT] = 0;
		setThrusterPos[RIGHT] = 0;
		Thrust[LEFT] = 0;
		Thrust[RIGHT] = 0;
		this->_setThrusterPos(setThrusterPos);
		this->_setThrusterThrust(Thrust);

	}
}
