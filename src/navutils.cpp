#include "navutils.h"
#include "gnss_core.h"

double ConvertGPST2JulianDate(unsigned short gps_week, double gps_tow, unsigned char utc_offset) {
	double julian_date;
//	if (gps_tow < 0.0 || gps_tow > 604800.0)
//		return false;

	julian_date = (gps_week + (gps_tow - utc_offset)/604800.0)*7.0 + JULIAN_DATE_START_OF_GPS_TIME;
	return julian_date;
}

double ConvertJulianDate2GMST(double JD){
	double JD0min = floor(JD) - 0.5;
	double JD0max = floor(JD) + 0.5;
	double JD0;
	if (JD > JD0min)
		JD0 = JD0min;
	if (JD > JD0max)
		JD0 = JD0max;

	double H, D, D0, T, GMST;
	H = (JD - JD0)*24;
	D = JD - 2451545.0;
	D0 = JD0 - 2451545.0;
	T = D/36525;

	GMST = fmod(6.697374558 + 0.06570982441908 * D0 + 1.00273790935*H + 0.000026*T*T, 24)*15;
	return GMST;
}

Vector3d ConvertECEF2ENU(Vector3d xyz, double reflat, double reflon, double refalt){

	Vector3d refxyz = ConvertLLA2ECEF(reflat, reflon, refalt);
	Vector3d diffxyz = xyz - refxyz;

	Matrix3d R1 = GetRotationMatrix(PI/2 + reflon, 3);
	Matrix3d R2 = GetRotationMatrix(PI/2 - reflat, 1);
	Matrix3d R = R2*R1;

	Vector3d enu = R*diffxyz;
	return enu;
}

Vector3d ConvertLLA2ECEF(double reflat, double reflon, double refalt) {
	double slat = sin(reflat);
	double clat = cos(reflat);
	double r_n;
	r_n = EARTH_RADIUS / sqrt(1 - E2_EARTH_WGS84 * slat * slat);

	Vector3d xyz;
	xyz(0) = (r_n + refalt) * clat * cos(reflon);
	xyz(1) = (r_n + refalt) * clat * sin(reflon);
	xyz(2) = ((r_n) * (1 - E2_EARTH_WGS84) + refalt) * slat;

	return xyz;
}

Matrix3d GetRotationMatrix(double angleRad, uint8_t axis){
	Matrix3d R = Matrix3d::Identity(3,3);
	double cang = cos(angleRad);
	double sang = sin(angleRad);

	if (axis == 1) {
		R(1,1) = cang;
		R(2,2) = cang;
		R(1,2) = sang;
		R(2,1) = -sang;
	}

	if (axis == 2) {
		R(0,0) = cang;
		R(2,2) = cang;
		R(0,2) = -sang;
		R(2,0) = sang;
	}

	if (axis == 3){
		R(0,0) = cang;
		R(1,1) = cang;
		R(1,0) = -sang;
		R(0,1) = sang;
	}
	return R;
}
