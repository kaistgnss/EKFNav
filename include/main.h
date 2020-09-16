#include "../include/novatel/gnss.h"
#include "LCEKF.h"
#include "HgDataParser.h"

double t, dualheading, dualheading_std;
bool Is_Dualheading;
double t_ins = 0;
double t_gnss = 0;
double t_ins_prev = 0;
double t_gnss_prev = 0;
int num_update = 0;
int num_ins = 0;
int num_ins_prev;
double init_time = 0;

void Initialize();
void Loop();

HgDataParser IMU;
LCEKF Filter;

void Initialize(){
	remove("/home/namgihun/EKFNav/Ground.gps");
	remove("/home/namgihun/EKFNav/Output_imu.txt");
	remove("/home/namgihun/EKFNav/State.txt");
	remove("/home/namgihun/EKFNav/True_Position.txt");
	remove("/home/namgihun/EKFNav/Covariance.txt");

	RunGnssThread(); // Start Reading GNSS Data
	IMU.ReadPort();
	Filter.Configure();
}

void Loop(){
	IMU.ReadIMU();

	//PositionEcef best_xyz = GNSS::getInstance()->GetBestxyz();
	PositionEcef best_xyz = GNSS::getInstance()->GetPdpxyz();
	PositionEcef usr_xyz = GnssCore::getInstance()->GetUsrxyz();
	Heading Dualheading = GNSS::getInstance()->GetDualheading();
	Is_Dualheading = GNSS::getInstance()->GetIsDualheading();

	// Get Time
	t_ins = IMU.time; // ins time
	t_gnss = GNSS::getInstance()->GetTimeBestpos(); // gnss time

	/*
	if (t_gnss > t_gnss_prev){
		printf("main usr xyz : %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f \n",usr_xyz.x_position,usr_xyz.y_position,usr_xyz.z_position,usr_xyz.x_velocity,usr_xyz.y_velocity,usr_xyz.z_velocity);
		printf("maint xyz SD : %8.6f, %8.6f, %8.6f, %8.6f, %8.6f, %8.6f \n",usr_xyz.x_standard_deviation,usr_xyz.y_standard_deviation,usr_xyz.z_standard_deviation,usr_xyz.x_velocity_standard_deviation,usr_xyz.y_velocity_standard_deviation,usr_xyz.z_velocity_standard_deviation);
	}
	*/

	// Get Measurement
	Vector3d w = Vector3d(IMU.gyro[0],IMU.gyro[1],IMU.gyro[2]);
	Vector3d a = Vector3d(IMU.acc[0],IMU.acc[1],IMU.acc[2]);
	Vector3d pos = Vector3d(best_xyz.x_position, best_xyz.y_position, best_xyz.z_position);
	Vector3d vel = Vector3d(best_xyz.x_velocity, best_xyz.y_velocity, best_xyz.z_velocity);
	Vector3d pos_SD = Vector3d(best_xyz.x_standard_deviation,best_xyz.y_standard_deviation,best_xyz.z_standard_deviation);
	Vector3d vel_SD = Vector3d(best_xyz.x_velocity_standard_deviation,best_xyz.y_velocity_standard_deviation,best_xyz.z_velocity_standard_deviation);
	dualheading = Dualheading.heading/180*PI;
	dualheading_std = Dualheading.heading_std;
	//printf("std dualheading : %8.6f \n", Dualheading.heading_std);
	if (dualheading > PI)
		dualheading -= 2*PI;

	// Loosely Coupled Kalman Filter
	if (t_ins_prev < t_ins && pos(0) != 0){
		num_update++;
		if (num_update == 1)
			Filter.Initialization(w, a, pos, vel, pos_SD, vel_SD, dualheading, Is_Dualheading);
		if (num_update > 1)
			Filter.Update(t_gnss, t_ins, w, a, pos, vel, pos_SD, vel_SD, dualheading, Is_Dualheading);
	}
	t_ins_prev = t_ins;
	t_gnss_prev = t_gnss;
}
