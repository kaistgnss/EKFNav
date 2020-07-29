#include <iostream>
#include "../include/novatel/gnss.h"
#include "LCEKF.h"
#include "HgDataParser.h"

using namespace gnss;

int main(int argc, char **argv) {
	HgDataParser IMU;
	LCEKF Filter;

	RunGnssThread(); // Start Reading GNSS Data
	IMU.ReadPort();
	Filter.Configure();

	double t, dualheading;
	bool Is_Dualheading;
	double t_ins = 0;
	double t_gnss = 0;
	double t_ins_prev = 0;
	int num_update = 0;

	while (1) {
		IMU.ReadIMU();
		PositionEcef best_xyz = GNSS::getInstance()->GetBestxyz();
		Heading Dualheading = GNSS::getInstance()->GetDualheading();
		Is_Dualheading = GNSS::getInstance()->GetIsDualheading();

		// Get Time
		t_ins = IMU.time; // ins time
		t_gnss = GNSS::getInstance()->GetTimeBestpos(); // gnss time
		boost::posix_time::ptime present_time(
		boost::posix_time::microsec_clock::universal_time());
		boost::posix_time::time_duration duration(present_time.time_of_day());
		t = (duration.total_milliseconds()) / 1000.0; // process time

		// Get Measurement
		Vector3d w = Vector3d(IMU.gyro[0],IMU.gyro[1],IMU.gyro[2]);
		Vector3d a = Vector3d(IMU.acc[0],IMU.acc[1],IMU.acc[2]);
		Vector3d pos = Vector3d(best_xyz.x_position, best_xyz.y_position, best_xyz.z_position);
		Vector3d vel = Vector3d(best_xyz.x_velocity, best_xyz.y_velocity, best_xyz.z_velocity);
		Vector3d pos_SD = Vector3d(best_xyz.x_standard_deviation,best_xyz.y_standard_deviation,best_xyz.z_standard_deviation);
		Vector3d vel_SD = Vector3d(best_xyz.x_velocity_standard_deviation,best_xyz.y_velocity_standard_deviation,best_xyz.z_velocity_standard_deviation);
		dualheading = Dualheading.heading/180*PI;
		if (dualheading > PI)
			dualheading -= 2*PI;

		// Loosely Coupled Kalman Filter
		if (t_ins_prev < t_ins && pos(0) != 0){
			num_update++;
			if (num_update == 1)
				Filter.Initialization(w, a, pos, vel, pos_SD, vel_SD, dualheading, Is_Dualheading);
			if (num_update > 1)
				Filter.Update(t_gnss, t_ins, w, a, pos, vel, pos_SD, vel_SD, dualheading, Is_Dualheading);
			Filter.PrintStatus();
			//printf("\n IMU Time : %8.6f, IMU DATA : %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f",t_ins,a(0),a(1),a(2),w(0),w(1),w(2));
		}

		boost::this_thread::sleep(boost::posix_time::milliseconds(10)); // While Hz
		t_ins_prev = t_ins;
	}
	return 0;
}


