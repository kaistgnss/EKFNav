/*
 * navutils.h
 *
 *  Created on: 2020. 6. 29.
 *      Author: root
 */

#ifndef INCLUDE_NAVUTILS_H_
#define INCLUDE_NAVUTILS_H_

#define JULIAN_DATE_START_OF_GPS_TIME (2444244.5)
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

double ConvertGPST2JulianDate(unsigned short gps_week, double gps_tow, unsigned char utc_offset);
double ConvertJulianDate2GMST(double JD);
Vector3d ConvertECEF2ENU(Vector3d xyz, double reflat, double reflon, double refalt);
Vector3d ConvertLLA2ECEF(double reflat, double reflon, double refalt);
Matrix3d GetRotationMatrix(double angleRad, uint8_t axis);



#endif /* INCLUDE_NAVUTILS_H_ */
