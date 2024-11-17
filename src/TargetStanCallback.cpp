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

double degToRad(double angle) {
    return angle * M_PI / 180.0;
}

double rad_to_deg(double radians) {
    return radians * (180.0 / M_PI);
}

static double	dist(double ax, double ay, double bx, double by)
{
	return (sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay)));
}

void	AquabotNode::_targetStanCallback() {

	double	gpsPos[2];
	this->_getGpsData(gpsPos);
	// RCLCPP_INFO(this->get_logger(), "gpspos = %f;%f", gpsPos[0], gpsPos[1]);

	double	targetGpsPos[2] = {-70, 60};

	double	setThrusterPos[2];

	double	acceleration[2];
	double	angularVelocity;
	double	orientation;
	this->_getImuData(acceleration, angularVelocity, orientation);

	int		Thrust[2];

	double orientationTarget = atan2(targetGpsPos[1] - gpsPos[1], targetGpsPos[0] - gpsPos[0]);
	// RCLCPP_INFO(this->get_logger(), "targer_o = %f o = %f resultat = %f", orientationTarget, orientation, orientationTarget - orientation);

	double distance =  dist(gpsPos[0], gpsPos[1], targetGpsPos[0], targetGpsPos[1]);
	// RCLCPP_INFO(this->get_logger(), "distance = %f", distance);

	if (distance < 0.000003) // moin de ... metre -> procedure de detection des eolienne
	{
		setThrusterPos[LEFT] = 0.785398;
		setThrusterPos[RIGHT] = -0.785398;
		this->_setThrusterPos(setThrusterPos);

		Thrust[LEFT] = 0;
		Thrust[RIGHT] = 0;
		this->_setThrusterThrust(Thrust);
		return ;
	}

	double delta_orientation = -1 * (orientationTarget - orientation);

	// RCLCPP_INFO(this->get_logger(), "accel %f %f velo %f", acceleration[0], acceleration[1], angularVelocity);
	if (delta_orientation > 10 * EPSILON || delta_orientation < 10 * -EPSILON)
	{

		setThrusterPos[LEFT] = delta_orientation;
		setThrusterPos[RIGHT] = delta_orientation * -1;
		this->_setThrusterPos(setThrusterPos);

		// double delta_power;
		if (delta_orientation > 0) { // produit en croix (100->70)
			Thrust[LEFT] = 0;
			Thrust[RIGHT] = (int)((100 * ((delta_orientation * -1) / EPSILON)) * 0.05);
		}
		else {
			Thrust[LEFT] = (int)((100 * (delta_orientation / EPSILON)) * 0.05);
			Thrust[RIGHT] = 0;
		}

		// Thrust[LEFT] = (int)delta_power;
		// Thrust[RIGHT] = 0;
		this->_setThrusterThrust(Thrust);

	}
	else {
		if (delta_orientation > EPSILON || delta_orientation < -EPSILON)
		{

			if (delta_orientation < 0) {
				setThrusterPos[LEFT] = -EPSILON * 0.8;
				setThrusterPos[RIGHT] = -EPSILON * 0.8;
			}
			else {
				setThrusterPos[LEFT] = EPSILON * 0.8;
				setThrusterPos[RIGHT] = EPSILON * 0.8;
			}
			this->_setThrusterPos(setThrusterPos);
			Thrust[LEFT] = 2000;
			Thrust[RIGHT] = 2000;
		}
		else
		{
			setThrusterPos[LEFT] = 0;
			setThrusterPos[RIGHT] = 0;
			this->_setThrusterPos(setThrusterPos);
			Thrust[LEFT] = 2000;
			Thrust[RIGHT] = 2000;
		}
		this->_setThrusterThrust(Thrust);
	}
}
