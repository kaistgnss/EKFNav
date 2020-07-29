#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "gnss_core.h"
using namespace Eigen;

// Constants
//const double EARTH_RADIUS = 6378137.0;        // earth semi-major axis radius (m)
const double ECC2 = 0.0066943799901;         // major eccentricity squared
const double mu = 3.986004418e14;           //WGS84 Earth gravitational constant (m3s-2)
const double J_2 = 1.082627e-3;             //WGS84 Earth second gravitational constant
const double EarthRate = 0.00007292115;      // rotation rate of earth (rad/sec)
const Vector3d Omega_ie = Vector3d{0, 0, EarthRate};

// Constants that are no longer used
// const double Eccentricity = 0.0818191908426; // major eccentricity of earth ellipsoid
// const double Flattening = 0.0033528106650;   // flattening of the ellipsoid
// const double Gravity0 = 9.7803730;           // zeroth coefficient for gravity model
// const double Gravity1 = 0.0052891;           // first coefficient for the gravity model
// const double Gravity2 = 0.0000059;           // second coefficient for the gravity model
// const double GravityNom = 9.81;              // nominal gravity
// const double Schuler2 = 1.533421593170545E-06; // Schuler Frequency (rad/sec) Squared

Vector3d NED2D_Rate(Vector3d v_NED, Vector3d pRef_D);
Vector3f NED2D_Rate(Vector3f v_NED, Vector3d pRef_D);

Vector3d NavRate(Vector3d v_NED, Vector3d pRef_D);
Vector3f NavRate(Vector3f v_NED, Vector3d pRef_D);

Vector3d D2E(Vector3d p_D);

Vector3d E2D(Vector3d p_E);

Vector3d E2NED(Vector3d p_E, Vector3d pRef_D);
Matrix3d TransE2NED(Vector3d pRef_D);
Quaterniond TransE2NED_Quat(Vector3d pRef_D);

Matrix3d Skew(Vector3d w);
Matrix3f Skew(Vector3f w);

Vector3d Quat2Euler(Quaterniond quat);
Vector3f Quat2Euler(Quaternionf quat);

Quaterniond Euler2Quat(Vector3d euler);
Quaternionf Euler2Quat(Vector3f euler);

Matrix3d Quat2DCM(Quaterniond quat);
Matrix3f Quat2DCM(Quaternionf quat);

void EarthRad(double lat, double *Rew, double *Rns);

double WrapToPi(double a);
float WrapToPi(float a);
double WrapTo2Pi(double a);
float WrapTo2Pi(float a);

double Gravity(Vector3d p_E);
double Geocentric_Radius(Vector3d p_E);
