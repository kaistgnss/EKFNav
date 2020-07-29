#pragma once

#include <stdint.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
using namespace std;
using namespace Eigen;

#include "filter_functions.h"

class LCEKF {
  public:
	//LCEKF() {};
    void Configure();
    void PrintStatus();
    void Initialization(Vector3d Meas_omega_ib_b, Vector3d Meas_f_ib_b, Vector3d Meas_r_eb_e, Vector3d Meas_v_eb_e, Vector3d Bestpos_ECEF_SD, Vector3d Bestvel_ECEF_SD, double dualheading, bool is_dualheading);
    bool Initialized() { return initialized_; } // returns whether the INS has been initialized
    void Update(double t_gnss, double t_ins, Vector3d Meas_omega_ib_b, Vector3d Meas_f_ib_b, Vector3d Meas_r_eb_e, Vector3d Meas_v_eb_e, Vector3d Bestpos_ECEF_SD, Vector3d Bestvel_ECEF_SD, double dualheading, bool is_dualheading);

    // Set Configuration
    inline void Set_AccelNoiseSigma(double val) { aNoiseSigma_mps2 = val; }
    inline void Set_AccelMarakov(double val) { aMarkovSigma_mps2 = val; }
    inline void Set_AccelTau(double val) { aMarkovTau_s = val; }
    inline void Set_GyrNoiseSigma(double val) { wNoiseSigma_rps2 = val; }
    inline void Set_GyroMarakov(double val) { wMarkovSigma_rps2 = val; }
    inline void Set_GyroTau(double val) { wMarkovTau_s = val; }

    // Set Initial Covariance
    inline void Set_InitPosSigma(double val) { pErrSigma_Init_m = val; }
    inline void Set_InitVelSigma(double val) { vErrSigma_Init_mps = val; }
    inline void Set_InitOrientSigma(double val) { attErrSigma_Init_rad = val; }
    inline void Set_InitHeadingSigma(double val) { hdgErrSigma_Init_rad = val; }
    inline void Set_InitAccelBiasSigma(double val) { aBiasSigma_Init_mps2 = val; }
    inline void Set_InitRotRateBiasSigma(double val) { wBiasSigma_Init_rps = val; }

    // Get Navigation Estimates and Noise
    inline Vector3d Get_AccelEst() { return Est_f_ib_b_; }
    inline Vector3d Get_AccelBias() { return Est_IMUaBias_; }
    inline Vector3d Get_RotRateEst() { return Est_omega_ib_b_; }
    inline Vector3d Get_RotRateBias() { return Est_IMUwBias_; }
    inline Vector3d Get_OrientEst() { return Est_att_euler_; }
    inline Vector3d Get_PosEst() { return Est_r_eb_e_; }
    inline Vector3d Get_VelEst() { return Est_v_eb_e_; }
    inline double Get_Track() { return atan2(Est_v_eb_e_(1), Est_v_eb_e_(0)); }

    // Get Covariance Estimates
    inline Vector3d Get_CovPos() { return P_matrix_.block(0,0,3,3).diagonal(); }
    inline Vector3d Get_CovVel() { return P_matrix_.block(3,3,3,3).diagonal(); }
    inline Vector3d Get_CovOrient() { return P_matrix_.block(6,6,3,3).diagonal(); }
    inline Vector3d Get_CovAccelBias() { return P_matrix_.block(9,9,3,3).diagonal(); }
    inline Vector3d Get_CovRotRateBias() { return P_matrix_.block(12,12,3,3).diagonal(); }

    // Get Innovation
    inline Vector3d Get_InnovationPos() { return S_matrix_.block(0,0,3,3).diagonal(); }
    inline Vector3d Get_InnovationVel() { return S_matrix_.block(3,3,3,3).diagonal(); }

    // Global variables
    Vector3d Est_IMUaBias_; // acceleration bias
    Vector3d Est_IMUwBias_; // rotation rate bias
    Vector3d Est_att_euler_; // Euler angles - B wrt L (3-2-1) [phi, theta, psi]
    Quaterniond Est_att_quat_; // Quaternion of B wrt L
    Vector3d Est_f_ib_b_; // Estimated acceleration in Body
    Vector3d Est_omega_ib_b_; // Estimated rotation rate in Body
    Vector3d Est_r_eb_e_; // Estimated position in ECEF
    Vector3d Est_v_eb_e_; // Estimated velocity in ECEF

    Matrix<double,15,15> P_matrix_; // Covariance estimate

  private:
    // Model Constants
    const double G = 9.807; // Acceleration due to gravity

    // Initialize flag
    bool initialized_ = false;

    // Timing
    double dt_s_ = 0.001666666666666666666;
    double t_gnss_prev_ = 0;
    double t_ins_prev_ = 0;
    int num_ins_ = 0;
    int num_gnss_ = 0;

    // Sensor variances (as standard deviation) and models (tau)
    double aNoiseSigma_mps2 = 0.0005; // 0.05
    double aMarkovSigma_mps2 = 0.0001; // 0.01
    double aMarkovTau_s = 100.0;

    double wNoiseSigma_rps2 = 0.00175; // 0.00175
    double wMarkovSigma_rps2 = 0.00025; // 0.00025
    double wMarkovTau_s = 50.0;


    // Initial set of covariance
    double attErrSigma_Init_rad = 0.34906;//sqrt(3.0462e-04); // Std dev of initial attitude (phi and theta) error (rad)
    double hdgErrSigma_Init_rad = 3.14159;//sqrt(3.0462e-04); // Std dev of initial Heading (psi) error (rad)
    double vErrSigma_Init_mps = 1; // Std dev of initial velocity error (m/s)
    double pErrSigma_Init_m = 10.0; // Std dev of initial position error (m)
    double aBiasSigma_Init_mps2 = 0.009810; //sqrt(9.6170e-05);// Std dev of initial acceleration bias (m/s^2)
    double wBiasSigma_Init_rps = 0.01745; //sqrt(2.3504e-09);// Std dev of initial rotation rate bias (rad/s)

    double pNoisesigma_NE = 3;
    double pNoisesigma_D = 6;
    double vNoisesigma_NE = 0.5;
    double vNoisesigma_D = 1;

    // Identity matrices
    const Matrix<double,2,2> I2 = Matrix<double,2,2>::Identity();
    const Matrix<double,3,3> I3 = Matrix<double,3,3>::Identity();
    const Matrix<double,5,5> I5 = Matrix<double,5,5>::Identity();
    const Matrix<double,6,6> I6 = Matrix<double,6,6>::Identity();
    const Matrix<double,15,15> I15 = Matrix<double,15,15>::Identity();

    // Kalman Matrices
    Matrix<double,6,15> H_matrix_; // Observation matrix
    Matrix<double,6,6> R_matrix_;// Covariance of the Observation Noise (associated with Measurement_Update())
    Matrix<double,12,12> Rw_; // Covariance of the Noise (associated with System_Update())
    Matrix<double,6,6> S_matrix_; // Innovation covariance

    // Monitoring flag
    bool check_meas = true;
    bool check_update = true;

    // Methods
    void System_Update();
    void Measurement_Update(Vector3d pMeas_ECEF_m, Vector3d vMeas_ECEF_mps, double dualheading, bool is_dualheading);
};
