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
	double	targetGpsPos[2] = {0, 30}; // target
 // --------IMU_DATA-------- //
	double	orientation[3];
	double	acceleration[2];
	double	angularVelocity[3];
 // --------OBSTACLE-------- //
	double	avoidanceOrientation;
 // --------GETTEUR--------- //
	this->_getGpsData(gpsPos);
	this->_getImuData(acceleration, angularVelocity, orientation);
	this->_getAvoidanceOrientation(avoidanceOrientation);
 // --------UTILS_VAR------- //
	double	distance =  dist(gpsPos[0], gpsPos[1], targetGpsPos[0], targetGpsPos[1]);
	double	orientationTarget = atan2(targetGpsPos[1] - gpsPos[1], targetGpsPos[0] - gpsPos[0]);
	double	delta_orientation = -1 * (orientationTarget - orientation[Z]);
	if (this->_statementTrip == 1)
		delta_orientation += avoidanceOrientation * 5;
 // --------PRINTEUR-------- //
	// if (this->_statementTrip < 3)
		RCLCPP_INFO(this->get_logger(), "distance = %f statment %d", distance, this->_statementTrip);
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

		if (distance < 16)	{
			this->_statementTrip++;
			return ;
		}

		if (delta_orientation > 10 * EPSILON || delta_orientation < 10 * -EPSILON) {

			this->_statementTrip = 0;
			return ;

		} else if (delta_orientation > 2 * EPSILON || delta_orientation < 2 * -EPSILON || distance < 25) {

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

		if (distance < 15) {

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

		if (distance < 12) {

			this->_statementTrip++;
			return ;

		} else {

			Thrust[LEFT] = (int)(30 * (distance - 10)* 0.1);
			Thrust[RIGHT] = (int)(30  * (distance - 10)* 0.1);
			this->_setThrusterThrust(Thrust);

			setThrusterPos[LEFT] = delta_orientation;
			setThrusterPos[RIGHT] = delta_orientation;
			this->_setThrusterPos(setThrusterPos);

		}
	} else if (this->_statementTrip == 4) { // cinquieme fase -> stab sur site

		if ((acceleration[0] + acceleration[1]) * 0.5 < 0.01 && distance - 10 < 0.2 && distance - 10 > -0.2 && delta_orientation < 2 * EPSILON && delta_orientation > 2 * -EPSILON) {

			this->_statementTrip++;
			return ;

		}

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


	} else if (this->_statementTrip == 5) { // sixieme fase -> tour de l'eolienne

		// RCLCPP_INFO(this->get_logger(), "delta_o %f", delta_orientation);
		// this->_statementTrip++;

		// if (delta_orientation > 10 * EPSILON || delta_orientation < 10 * -EPSILON) {

		// 	this->_statementTrip--;
		// 	return ;

		// }

		// if (delta_orientation > 25 * EPSILON || delta_orientation < 25 * -EPSILON) {
		// 	setThrusterPos[LEFT] = delta_orientation * -1;
		// 	setThrusterPos[RIGHT] = delta_orientation * -1;
		// 	Thrust[LEFT] = (int)(300 * (distance - 10));
		// 	Thrust[RIGHT] = (int)(300 * (distance - 10));
		// 	this->_setThrusterPos(setThrusterPos);
		// 	this->_setThrusterThrust(Thrust);
		// 	return ;

		// }
		// setThrusterPos[LEFT] = 45 * -EPSILON;
		// setThrusterPos[RIGHT] = delta_orientation;
		// this->_setThrusterPos(setThrusterPos);

		// Thrust[LEFT] = 90;
		// Thrust[RIGHT] = (int)(300 * (distance - 10));
		// this->_setThrusterThrust(Thrust);

		// setThrusterPos[LEFT] = 45 * EPSILON;
		// setThrusterPos[RIGHT] = 0;
		// this->_setThrusterPos(setThrusterPos);

		// Thrust[LEFT] = -50;
		// Thrust[RIGHT] = (int)(50 * (distance - 10));
		// this->_setThrusterThrust(Thrust);

		// if (delta_orientation > 35 * EPSILON || delta_orientation < 35 * -EPSILON || distance > 10.5 || distance < 9.5) {

		// 	this->_statementTrip--;
		// 	return ;

		// }

		// setThrusterPos[LEFT] = 45 * -EPSILON;
		// setThrusterPos[RIGHT] = delta_orientation;
		// this->_setThrusterPos(setThrusterPos);

		// Thrust[LEFT] = 20;
		// Thrust[RIGHT] = (int)(300 * (distance - 10));
		// this->_setThrusterThrust(Thrust);
		// 	Thrust[RIGHT] = ;

		// } else {

		// 	Thrust[LEFT] = (int)(50 * (distance - 10));
		// 	Thrust[RIGHT] = (int)(50 * (distance - 10));
		// }

		// setThrusterPos[LEFT] = 45 * - EPSILON;
		// if (distance - 10 < 0)
		// 	setThrusterPos[RIGHT] = -delta_orientation;
		// else
		// 	setThrusterPos[RIGHT] = delta_orientation;
		// this->_setThrusterPos(setThrusterPos);

		// Thrust[LEFT] = 40;
		// Thrust[RIGHT] = (int)(300 * (distance - 10));

		// Thrust[RIGHT] = std::max(-300, std::min(Thrust[RIGHT], 300));
		// this->_setThrusterThrust(Thrust);

		// setThrusterPos[LEFT] = 45 * -EPSILON;
		// setThrusterPos[RIGHT] = 45 * EPSILON;
		// this->_setThrusterPos(setThrusterPos);

		// if (delta_orientation > 0) {

		// 	if (distance < 10) {
		// 		Thrust[LEFT] = 150;
		// 		Thrust[RIGHT] = (int)(-50 * abs(distance - 10));
		// 	} else {
		// 		Thrust[LEFT] = 150;
		// 		Thrust[RIGHT] = (int)(50 * abs(distance - 10));
		// 	}

		// } else {

		// 	if (distance < 10) {
		// 		Thrust[LEFT] = (int)(-50 * abs(distance - 10));
		// 		Thrust[RIGHT] = -10;
		// 	} else {
		// 		Thrust[LEFT] = (int)(50 * abs(distance - 10));
		// 		Thrust[RIGHT] = 10;
		// 	}

		// }

		// Thrust[LEFT] = 40;
		// Thrust[RIGHT] = (int)(300 * (distance - 10));

		// Thrust[RIGHT] = std::max(-300, std::min(Thrust[RIGHT], 300));
		// this->_setThrusterThrust(Thrust);

		if (delta_orientation < 5 * EPSILON && delta_orientation > 5 * -EPSILON) {
			setThrusterPos[LEFT] = 45 * -EPSILON; // interieur
			setThrusterPos[RIGHT] = 45 * EPSILON; // exterieur

		}
		setThrusterPos[LEFT] = 45 * -EPSILON;
		setThrusterPos[RIGHT] = 45 * EPSILON;
		this->_setThrusterPos(setThrusterPos);

		if (delta_orientation > 0) {

			if (distance < 10) {
				Thrust[LEFT] = 150;
				Thrust[RIGHT] = (int)(-50 * abs(distance - 10));
			} else {
				Thrust[LEFT] = 150;
				Thrust[RIGHT] = (int)(50 * abs(distance - 10));
			}

		} else {

			if (distance < 10) {
				Thrust[LEFT] = (int)(-50 * abs(distance - 10));
				Thrust[RIGHT] = -10;
			} else {
				Thrust[LEFT] = (int)(50 * abs(distance - 10));
				Thrust[RIGHT] = 10;
			}

		}

		Thrust[LEFT] = 40;
		Thrust[RIGHT] = (int)(300 * (distance - 10));

		this->_setThrusterThrust(Thrust);





		// // Fixation des positions des thrusters
		// setThrusterPos[LEFT] = 45 * -EPSILON; // Orientation fixe du moteur gauche
		// setThrusterPos[RIGHT] = 45 * EPSILON; // Orientation fixe du moteur droit
		// this->_setThrusterPos(setThrusterPos);

		// // Définition des coefficients pour la modulation dynamique
		// double base_power = 1000; // Puissance minimale pour garantir le mouvement

		// // Ajustement des poussées en fonction de la distance et de la direction
		// if (delta_orientation < 0) { // Sens horaire
		// 	if (distance < 10.01) { // Trop proche de l'éolienne
		// 		Thrust[LEFT] = (int)base_power;                             // Force de base pour tourner
		// 		Thrust[RIGHT] = (int)(200 * (distance - 10));                  // Recule pour augmenter la distance
		// 	} else { // Trop loin de l'éolienne
		// 		Thrust[LEFT] = (int)(base_power * (distance - 10));       // Augmente la poussée pour rapprocher
		// 		Thrust[RIGHT] = (int)base_power;                            // Force de base pour tourner
		// 	}
		// } else { // Sens antihoraire
		// 	if (distance < 10.01) { // Trop proche de l'éolienne
		// 		Thrust[LEFT] = (int)(200 * (distance - 10));;                   // Recule pour augmenter la distance
		// 		Thrust[RIGHT] = (int)base_power;                            // Force de base pour tourner
		// 	} else { // Trop loin de l'éolienne
		// 		Thrust[LEFT] = (int)base_power;                             // Force de base pour tourner
		// 		Thrust[RIGHT] = (int)(base_power * (distance - 10));      // Augmente la poussée pour rapprocher
		// 	}
		// }
		// // Appliquer les poussées aux moteurs
		// this->_setThrusterThrust(Thrust);






	}
}
