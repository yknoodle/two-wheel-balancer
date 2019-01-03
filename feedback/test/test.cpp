#include  <iostream>
#include "LSM303DLHC.h"
#include "CodeClock.h"
#include <unistd.h>
#include <string>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <cmath>
#include "LowPassFilter.hpp"
int main (int argc, char const* argv[])
{
	LSM303DLHC lms303;
	CodeClock dt_o;
	std::cout << "sensor creation successful\n";
	std::cout << I2Cdev::initialize() << "\n";

	// start the sensor with default settings
	lms303.initialize();

	// set the full acceleration scale to +/-2g ( best resolution )
	lms303.setAccelFullScale(2);

	// set the magnetometer to continuously convert readings
	lms303.setMagMode(0b00000000);

	// set the magnetometer gain to 1100 ( best resolution )
	lms303.setMagGain(1100);

	// declare up and north vectors
	Eigen::Vector3d u, n;
	Eigen::Vector3d apx, str, bow;
	/* apx, str, bow
	** apx - vector pointing upwards from board regardless of pitch
	** str - vector pointing to the rear regardless of pitch
	** bow - vector pointing to the front regardless of pitch
	*/
	Eigen::Vector3d x(1,0,0);
	Eigen::Vector3d y(0,1,0);
	Eigen::Vector3d z(0,0,1);

	float f_cyc = atof(argv[1]);
	LowPassFilter x_lf(f_cyc), y_lf(f_cyc), z_lf(f_cyc);
	dt_o.start(16000);

	float x_mag, y_mag, z_mag;
	float x_acc, y_acc, z_acc;
	float acc_abs, mag_abs;
	float pit, rol, yaw, dir;
	bool b_dir;
	Eigen::Vector3d v_dir;

	for (;;)
	{

		// get new cycle duration
		dt_o.clock();

		// get new accelertion values
		x_acc = ((float)x_lf.update( lms303.getAccelerationX(), (float)dt_o.lastClock()/1000000 ) ) / 32767 * 2 * 9.81;
		y_acc = ((float)y_lf.update( lms303.getAccelerationY(), (float)dt_o.lastClock()/1000000 ) ) / 32767 * 2 * 9.81;
		z_acc = ((float)z_lf.update( lms303.getAccelerationZ(), (float)dt_o.lastClock()/1000000 ) ) / 32767 * 2 * 9.81;
		acc_abs = pow( ( pow(x_acc,2) + pow(y_acc,2) + pow(z_acc,2) ),0.5 );

		// assign acceleration values for "up" vector
		u(0) = x_acc/acc_abs;
		u(1) = y_acc/acc_abs;
		u(2) = z_acc/acc_abs;

		// get new magnetometer values
		x_mag = ((float)lms303.getMagX());
		y_mag = ((float)lms303.getMagY());
		z_mag = ((float)lms303.getMagZ());
		mag_abs = pow( ( pow(x_mag,2) + pow(y_mag,2) + pow(z_mag,2) ),0.5 );

		// assign magnetometer values for "north" vector
		n(0) = x_mag/mag_abs;
		n(1) = y_mag/mag_abs;
		n(2) = z_mag/mag_abs;

		str = ( y.cross(u) ).normalized(); // normalizing ensures the vector magnitude is 1
		bow = -str;
		apx = ( str.cross(y) ).normalized();

		// check if the board is rolling left or right, b_dir is false if the board rolls left
		v_dir = ( apx.cross(u) ).normalized();
		b_dir = v_dir.dot(bow)>0 && !std::isnan(b_dir);

		// math to get pitch and roll values
		pit = 90 - acos( u.dot(x) ) / M_PI * 180;
		dir = ( b_dir ? 1 :-1 );
 		rol = dir * acos( z.dot(u) / fabs( z.dot(u) ) * apx.dot(u) ) / M_PI * 180;
		rol = std::isnan(rol) ? 0 : rol;
		if (std::isnan(rol)) break;
		std::cout
		<< " ------------------------------- \n"
		<< "pit: " << pit << ", "
		<< "rol: " << rol << ", "
		<< "clk: " << dt_o.lastClock() << ", "
		<< "\n";
		usleep(8000);
	}

	return 0;
}
