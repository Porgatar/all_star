#include "BabySharky.hpp"

static double	dist(double ax, double ay, double bx, double by)
{
	return (sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay)));
}

void	AquabotNode::_targetStanCallback() {

 // --------THRUST---------- //
	int		Thrust[2];
	double	setThrusterPos[2];
 // --------GPS------------- //
	double	gpsPos[2];
	double	targetGpsPos[2] = {70, -60}; // target
 // --------IMU_DATA-------- //
	double	orientation;
	double	acceleration[2];
	double	angularVelocity;
 // --------GETTEUR--------- //-
	this->_getGpsData(gpsPos);
	this->_getImuData(acceleration, angularVelocity, orientation);
 // --------UTILS_VAR------- //
	double	distance =  dist(gpsPos[0], gpsPos[1], targetGpsPos[0], targetGpsPos[1]);
	double	orientationTarget = atan2(targetGpsPos[1] - gpsPos[1], targetGpsPos[0] - gpsPos[0]);
	double	delta_orientation = -1 * (orientationTarget - orientation);
 // --------PRINTEUR-------- //
	// if (this->_statmentTrip < 3)
		RCLCPP_INFO(this->get_logger(), "distance = %f statment %d", distance, this->_statmentTrip);
	// RCLCPP_INFO(this->get_logger(), "delta_o %f", delta_orientation);
	// RCLCPP_INFO(this->get_logger(), "gpspos = %f;%f", gpsPos[0], gpsPos[1]);
	// RCLCPP_INFO(this->get_logger(), "accel %f %f velo %f", acceleration[0], acceleration[1], angularVelocity);
	// RCLCPP_INFO(this->get_logger(), "targer_o = %f o = %f resultat = %f", orientationTarget, orientation, orientationTarget - orientation);


	if (this->_statmentTrip == 0) { // premiere fase du trajet -> stab sur l'orientation

		if (delta_orientation < 5 * EPSILON && delta_orientation > 5 * -EPSILON) {

			this->_statmentTrip++;
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

				Thrust[LEFT] = (int)(delta_power * -0.3);
				Thrust[RIGHT] = delta_power * -1;

			} else {

				setThrusterPos[LEFT] = delta_orientation;
				setThrusterPos[RIGHT] = delta_orientation * -1;

				Thrust[LEFT] = delta_power;
				Thrust[RIGHT] = (int)(delta_power * 0.3);

			}

		}
		this->_setThrusterPos(setThrusterPos);
		this->_setThrusterThrust(Thrust);

	} else if (this->_statmentTrip == 1) { // seconde fase -> deplacment sur site

		if (distance < 16)	{
			this->_statmentTrip++;
			return ;
		}

		if (delta_orientation > 10 * EPSILON || delta_orientation < 10 * -EPSILON) {

			this->_statmentTrip = 0;
			return ;

		} else if (delta_orientation > 2 * EPSILON || delta_orientation < 2 * -EPSILON || distance < 25) {

			if (delta_orientation < 0) {

				setThrusterPos[LEFT] = delta_orientation ;
				setThrusterPos[RIGHT] = (delta_orientation) * -0.1;
				Thrust[LEFT] = 1800;
				Thrust[RIGHT] = 2000;

			} else {

				setThrusterPos[LEFT] = (delta_orientation) * 0.1;
				setThrusterPos[RIGHT] = delta_orientation;
				Thrust[LEFT] = 2000;
				Thrust[RIGHT] = 1800;


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

	} else if (this->_statmentTrip == 2) { // troisieme fase -> frein

		if (distance < 15) {

			if (((abs(acceleration[0]) + abs(acceleration[1])) * 0.5) < 0.6) {

				this->_statmentTrip++;
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

	} else if (this->_statmentTrip == 3) { // quatrieme fase -> goto point petite vitesse

		if (distance < 12) {

			this->_statmentTrip++;
			return ;

		} else {

			Thrust[LEFT] = (int)(30 * distance * 0.1);
			Thrust[RIGHT] = (int)(30  * distance * 0.1);
			this->_setThrusterThrust(Thrust);

			setThrusterPos[LEFT] = delta_orientation;
			setThrusterPos[RIGHT] = delta_orientation;
			this->_setThrusterPos(setThrusterPos);

		}
	} else if (this->_statmentTrip == 4) { // cinquieme fase -> stab sur site

		setThrusterPos[LEFT] = 45 * -EPSILON ;
		setThrusterPos[RIGHT] = 45 * EPSILON;
		this->_setThrusterPos(setThrusterPos);

		int	counter_push = (int)(2500 * (distance - 10));
		if (abs(distance - 10) < 0.5)
			counter_push = (int)(counter_push * 0.1);
		if (delta_orientation < 0) {

			Thrust[LEFT] = (int)(counter_push * 0.7);
			Thrust[RIGHT] = counter_push;

		} else {

			Thrust[LEFT] = counter_push;
			Thrust[RIGHT] = (int)(counter_push * 0.7);

		}
		this->_setThrusterThrust(Thrust);

		// RCLCPP_INFO(this->get_logger(), "delta_o %f", delta_orientation);
		// this->_statmentTrip++;

	}
}
