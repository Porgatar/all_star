#include "BabySharky.hpp"

// #define size_one_block 64 // taille un block dans la map 2D
// #define dist_max 100000 // hard limit
// static void ft_raycasting(char **map, int mapX, int mapY, float player_pos[2], float orientation) {
//     int dof, mx, my; // raport raycasting/map
//     float rx, ry, ra, xo, yo, vx, vy; // les float pour les point
//     float px = player_pos[0];
//     float py = player_pos[1];
//     float disV = dist_max, disH = dist_max; // distance vertival et horizontal
//     float Tan;
// 	int	i = 0;

// 	ra = orientation;

//     // raycast verticale
//     while (i < 1)
// 	{
// 		dof = 0;
// 		Tan = tan(ra);
// 		if (cosf(ra) > 0.001)  // Vers la droite
// 		{
// 			rx = (((int)px >> 6) << 6) + size_one_block;
// 			ry = (px - rx) * Tan + py;
// 			xo = size_one_block;
// 			yo = -xo * Tan;
// 		}
// 		else if (cosf(ra) < -0.001)  // Vers la gauche
// 		{
// 			rx = (((int)px >> 6) << 6) - 0.0001;
// 			ry = (px - rx) * Tan + py;
// 			xo = -size_one_block;
// 			yo = -xo * Tan;
// 		}
// 		else  // Regard droit ou gauche
// 		{
// 			rx = px;
// 			ry = py;
// 			dof = 8;
// 		}
// 		while (dof < 8) {
// 			mx = (int)(rx) >> 6;
// 			my = (int)(ry) >> 6;
// 			if (mx >= 0 && mx < mapX && my >= 0 && my < mapY && data->map[my][mx] == '1')
// 			{
// 				disV = cosf(ra) * (rx - px) - sinf(ra) * (ry - py);  // Calcul de la distance
// 				vx = rx;
// 				vy = ry;
// 				break;
// 			}
// 			else
// 			{
// 				rx += xo;
// 				ry += yo;
// 				dof++;
// 				if (dof >= dist_max) break;
// 			}
// 		}

// 		// raycast horizontal
// 		dof = 0;
// 		Tan = 1.0 / Tan;
// 		if (sinf(ra) > 0.001)  // Vers le haut
// 		{
// 			ry = (((int)py >> 6) << 6) - 0.0001;
// 			rx = (py - ry) * Tan + px;
// 			yo = -size_one_block;
// 			xo = -yo * Tan;
// 		}
// 		else if (sinf(ra) < -0.001)  // Vers le bas
// 		{
// 			ry = (((int)py >> 6) << 6) + size_one_block;
// 			rx = (py - ry) * Tan + px;
// 			yo = size_one_block;
// 			xo = -yo * Tan;
// 		}
// 		else  // Droit ou gauche
// 		{
// 			rx = px;
// 			ry = py;
// 			dof = 8;
// 		}
// 		while (dof < 8) {
// 			mx = (int)(rx) >> 6;
// 			my = (int)(ry) >> 6;
// 			if (mx >= 0 && mx < mapX && my >= 0 && my < mapY && data->map[my][mx] == '1')
// 			{
// 				disH = cosf(ra) * (rx - px) - sinf(ra) * (ry - py);  // Calcul de la distance
// 				break;
// 			}
// 			else
// 			{
// 				rx += xo;
// 				ry += yo;
// 				dof++;
// 				if (dof >= dist_max) break;
// 			}
// 		}

// 		// Comparer disH et disV pour afficher le point d'impact le plus proche
// 		if (disV < disH) { // on tape un wall Vertical (contourner en consequence)
// 			rx = vx;
// 			ry = vy;
// 		}

// 		i++;
// 	}
// }

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
	double	targetGpsPos[2] = {-70, 60}; // target
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
	// 	RCLCPP_INFO(this->get_logger(), "distance = %f statment %d", distance, this->_statmentTrip);
	// RCLCPP_INFO(this->get_logger(), "delta_o %f", delta_orientation);
	// RCLCPP_INFO(this->get_logger(), "gpspos = %f;%f", gpsPos[0], gpsPos[1]);
	RCLCPP_INFO(this->get_logger(), "accel %f %f velo %f", acceleration[0], acceleration[1], angularVelocity);
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
				// Thrust[LEFT] = 0;
				Thrust[RIGHT] = delta_power * -1;

			} else {

				setThrusterPos[LEFT] = delta_orientation;
				setThrusterPos[RIGHT] = delta_orientation * -1;

				Thrust[LEFT] = delta_power;
				Thrust[RIGHT] = (int)(delta_power * 0.3);
				// Thrust[RIGHT] = 0;

			}

		}
		this->_setThrusterPos(setThrusterPos);
		this->_setThrusterThrust(Thrust);

	} else if (this->_statmentTrip == 1) { // seconde fase -> deplacment sur site

		if (distance < 6)	{
			this->_statmentTrip++;
			return ;
		}

		if (delta_orientation > 10 * EPSILON || delta_orientation < 10 * -EPSILON) {

			this->_statmentTrip = 0;
			return ;

		}

		if (distance < 12) {

			setThrusterPos[LEFT] = 0;
			setThrusterPos[RIGHT] = 0;

		} else if (delta_orientation < 0) {

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

	} else if (this->_statmentTrip == 2) { // troisieme fase -> arriver a moin de 15 metre

		if (distance < 2) {

			this->_statmentTrip++;
			return ;

		} else if (distance < 5) {

			setThrusterPos[LEFT] = delta_orientation;
			setThrusterPos[RIGHT] = delta_orientation;
			this->_setThrusterPos(setThrusterPos);

			Thrust[LEFT] = (int)(30 * distance * 0.1);
			Thrust[RIGHT] = (int)(30  * distance * 0.1);
			this->_setThrusterThrust(Thrust);

		} else {

			setThrusterPos[LEFT] = 0;
			setThrusterPos[RIGHT] = 0;
			this->_setThrusterPos(setThrusterPos);

			Thrust[LEFT] = (int)(-5000 * distance * 0.01);
			Thrust[RIGHT] = (int)(-5000 * distance * 0.01);
			this->_setThrusterThrust(Thrust);

		}

	} else if (this->_statmentTrip == 3) { // quatrieme fase -> stab sur le point

		RCLCPP_INFO(this->get_logger(), "delta_o %f", delta_orientation);
		this->_statmentTrip++;

	}
}
