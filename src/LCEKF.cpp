// State Vector
// Attitude Roll, Pitch, Yaw [3]
// Velocity_NED [3]
// Position_NED [3]
// Accelerometer Biases [3]
// Gyro Biases [3]

#include "LCEKF.h"

int i, j;

void LCEKF::Configure() {
  // Observation matrix
  H_matrix_.setZero();
  H_matrix_.block(0,3,6,6) = I6;

  // Covariance of the Process Noise (associated with TimeUpdate())
  Rw_.setZero();
  Rw_.block(0,0,3,3) = (wNoiseSigma_rps2 * wNoiseSigma_rps2) * I3;
  Rw_.block(3,3,3,3) = (aNoiseSigma_mps2 * aNoiseSigma_mps2) * I3;
  Rw_.block(6,6,3,3) = 2.0 * (aMarkovSigma_mps2 * aMarkovSigma_mps2) / aMarkovTau_s * I3;
  Rw_.block(9,9,3,3) = 2.0 * (wMarkovSigma_rps2 * wMarkovSigma_rps2) / wMarkovTau_s * I3;


  // Initial Innovation Covariance Estimate (S)
  S_matrix_.setZero();

  // Initial Covariance Estimate (P)
  P_matrix_.setZero();
  P_matrix_.block(0,0,2,2) = (attErrSigma_Init_rad * attErrSigma_Init_rad) * I2; // Attitude initial uncertainty (roll, pitch)
  P_matrix_(2,2) = (hdgErrSigma_Init_rad * hdgErrSigma_Init_rad); // Heading initial uncertainty
  P_matrix_.block(3,3,3,3) = (vErrSigma_Init_mps * vErrSigma_Init_mps) * I3; // Velocity initial uncertainty
  P_matrix_.block(6,6,3,3) = (pErrSigma_Init_m * pErrSigma_Init_m) * I3; // Position initial uncertainty
  P_matrix_.block(9,9,3,3) = (aBiasSigma_Init_mps2 * aBiasSigma_Init_mps2) * I3; // Accelerometer bias initial uncertainty
  P_matrix_.block(12,12,3,3) = (wBiasSigma_Init_rps * wBiasSigma_Init_rps) * I3; // Gyro bias initial uncertainty
}

void LCEKF::Initialization(Vector3d Meas_omega_ib_b, Vector3d Meas_f_ib_b, Vector3d Meas_r_eb_e, Vector3d Meas_v_eb_e, Vector3d Bestpos_ECEF_SD, Vector3d Bestvel_ECEF_SD, double dualheading, bool is_dualheading) {
  // Initialize Position and Velocity
  Est_r_eb_e_ = Meas_r_eb_e; // Position in ECEF (m)
  Est_v_eb_e_ = Meas_v_eb_e; // Velocity in ECEF (m/s)

  // Initialize sensor biases
  Est_IMUwBias_ = Meas_omega_ib_b;
  Est_IMUaBias_.setZero();

  // New Specific forces and Rotation Rate
  Est_f_ib_b_ = Meas_f_ib_b; // Accelerometer Measurement Body frame meter per second
  Est_omega_ib_b_ = Meas_omega_ib_b - Est_IMUwBias_; // Gyro Measurement Body frame meter per second

  // Initial attitude, roll and pitch
  Vector3d Est_f = Est_f_ib_b_ / Est_f_ib_b_.norm(); // Normalize to remove the 1g sensitivity

  Est_att_euler_(1) = asin(Est_f(0));
  Est_att_euler_(0) = -asin(Est_f(1) / cos(Est_att_euler_(1)));

  // Initial attitude, heading with GPS heading
  if (!is_dualheading)
	  cout << "\n No dualheading data" << endl;

  Est_att_euler_(2) = dualheading;

  // Euler to quaternion
  Est_att_quat_ = Euler2Quat(Est_att_euler_);

  // set initialized flag
  initialized_ = true;
}

void LCEKF::Update(double t_gnss, double t_ins, Vector3d Meas_omega_ib_b, Vector3d Meas_f_ib_b, Vector3d Meas_r_eb_e, Vector3d Meas_v_eb_e, Vector3d Bestpos_ECEF_SD, Vector3d Bestvel_ECEF_SD, double dualheading, bool is_dualheading) {
  Meas_f_ib_b_ = Meas_f_ib_b;
  Meas_omega_ib_b_ = Meas_omega_ib_b;
  t_gnss_ = t_gnss;
  t_ins_ = t_ins;

  // A-priori accel and rotation rate estimate
  Est_f_ib_b_ = Meas_f_ib_b - Est_IMUaBias_;
  Est_omega_ib_b_ = Meas_omega_ib_b - Est_IMUwBias_;

  // Kalman Time Update (Prediction)
  if (t_ins_prev_ != 0)
	  dt_s_ = t_ins - t_ins_prev_;
  if (Meas_f_ib_b(2) != 0)
  	  System_Update();
  t_ins_prev_ = t_ins;
  // Gps measurement update, if TOW increased
  if ((t_gnss - t_gnss_prev_) > 0) {
    t_gnss_prev_ = t_gnss;
    Meas_r_eb_e_ = Meas_r_eb_e;
    Meas_v_eb_e_ = Meas_v_eb_e;

    R_matrix_(0,0) = Bestvel_ECEF_SD(0) * Bestvel_ECEF_SD(0);
    R_matrix_(1,1) = Bestvel_ECEF_SD(1) * Bestvel_ECEF_SD(1);
    R_matrix_(2,2) = Bestvel_ECEF_SD(2) * Bestvel_ECEF_SD(2);
    R_matrix_(3,3) = Bestpos_ECEF_SD(0) * Bestpos_ECEF_SD(0);
    R_matrix_(4,4) = Bestpos_ECEF_SD(1) * Bestpos_ECEF_SD(1);
    R_matrix_(5,5) = Bestpos_ECEF_SD(2) * Bestpos_ECEF_SD(2);

    // Kalman Measurement Update
    Measurement_Update(Meas_r_eb_e, Meas_v_eb_e, dualheading, is_dualheading);

    // Post-priori accel and rotation rate estimate, biases updated in MeasUpdate()
    Est_f_ib_b_ = Meas_f_ib_b - Est_IMUaBias_;
    Est_omega_ib_b_ = Meas_omega_ib_b - Est_IMUwBias_;
  }

  // Euler angles from quaternion
  Est_att_euler_ = Quat2Euler(Est_att_quat_);
}

void LCEKF::System_Update() {

	  // Attitude Update Using Compensation Filter
	  Vector3d Est_f = Est_f_ib_b_ / Est_f_ib_b_.norm(); // Normalize to remove the 1g sensitivity
	  double PITCH = asin(Est_f(0));
	  double ROLL = -asin(Est_f(1) / cos(PITCH));;

	  double ROLL_prev = Est_att_euler_(0);
	  double PITCH_prev = Est_att_euler_(1);
	  double YAW_prev = Est_att_euler_(2);
	  double acc_angle_ratio = 0.002;

	  // Attitude Update
	  Quaterniond dQuat = Quaterniond(1.0, 0.5*Est_omega_ib_b_(0)*dt_s_, 0.5*Est_omega_ib_b_(1)*dt_s_, 0.5*Est_omega_ib_b_(2)*dt_s_);
	  Est_att_quat_ = (Est_att_quat_ * dQuat).normalized();

	  // Avoid quaternion flips sign
	  if (Est_att_quat_.w() < 0) {
	    Est_att_quat_ = Quaterniond(-Est_att_quat_.w(), -Est_att_quat_.x(), -Est_att_quat_.y(), -Est_att_quat_.z());
	  }
	  Est_att_euler_ = Quat2Euler(Est_att_quat_);

	  //Est_att_euler_(0) = (1.0-acc_angle_ratio) * (ROLL_prev + Est_omega_ib_b_(0) * dt_s_) + acc_angle_ratio * ROLL;
	  //Est_att_euler_(1) = (1.0-acc_angle_ratio) * (PITCH_prev + Est_omega_ib_b_(1) * dt_s_) + acc_angle_ratio * PITCH;
	  //Est_att_euler_(2) = (1.0-acc_angle_ratio) * (YAW_prev + Est_omega_ib_b_(2) * dt_s_) + acc_angle_ratio * dualheading;
	  Est_att_quat_ = Euler2Quat(Est_att_euler_);

	  // Compute DCM (Body to/from NED) Transformations from Quaternion
	  Matrix3d Est_C_NED2B = Quat2DCM(Est_att_quat_);
	  Matrix3d Est_C_B2NED = Est_C_NED2B.transpose();
	  Vector3d p_LLA = E2D(Est_r_eb_e_);
	  Matrix3d Est_C_ECEF2NED_ = TransE2NED(p_LLA);
	  Matrix3d Est_C_NED2ECEF = Est_C_ECEF2NED_.transpose();

	  // Velocity Update
	  Vector3d Gravity_NED = Vector3d(0,0,G);
	  Est_v_eb_e_ += dt_s_ * (Est_C_NED2ECEF * (Est_C_B2NED * Est_f_ib_b_ + Gravity_NED));

	  // Position Update
	  Est_r_eb_e_ += (dt_s_ * Est_v_eb_e_);

	  // Calculate the Jacobian element
	  double R_geo = Geocentric_Radius(Est_r_eb_e_);
	  double mag_r = Est_r_eb_e_.norm();
	  double g0 = Gravity(Est_r_eb_e_);

	  Matrix<double,3,1> Est_ECEF = Est_r_eb_e_;
	  Matrix<double,3,3> F; F = Est_ECEF * Est_ECEF.transpose();

	  Vector3d Est_f_ib_e = - Est_C_NED2ECEF * (Est_C_B2NED * Est_f_ib_b_);

	  // Assemble the Jacobian (state update matrix)
	  Matrix<double,15,15> Fs; Fs.setZero();
	  Fs.block(6,3,3,3) = I3;
	  Fs.block(3,6,3,3) = 2 * g0 / (R_geo * mag_r * mag_r) * F;
	  Fs.block(3,0,3,3) = -2.0 * Est_C_NED2ECEF * Est_C_B2NED * Skew(Est_f_ib_b_); // change
	  Fs.block(3,9,3,3) = -Est_C_NED2ECEF * Est_C_B2NED;
	  Fs.block(3,3,3,3) = -2 * Skew(Omega_ie);
	  Fs.block(0,0,3,3) = -Skew(Est_omega_ib_b_);
	  Fs.block(0,12,3,3) = -0.5 * I3;
	  Fs.block(9,9,3,3) = -1.0 / aMarkovTau_s * I3;
	  Fs.block(12,12,3,3) = -1.0 / wMarkovTau_s * I3;

	  // State Transition Matrix
	  Matrix<double,15,15> PHI_matrix = I15 + Fs * dt_s_;

	  Matrix<double,15,12> Gs; Gs.setZero();
	  Gs.block(0,0,3,3) = -0.5 * I3;
	  Gs.block(3,3,3,3) = - Est_C_NED2ECEF * Est_C_B2NED;
	  Gs.block(9,6,3,3) = I3;
	  Gs.block(12,9,3,3) = I3;

	  // Discrete Process Noise
	  Matrix<double,15,15> Q_matrix; Q_matrix.setZero();
	  Q_matrix = PHI_matrix * dt_s_ * Gs * Rw_ * Gs.transpose();
	  Q_matrix = 0.5 * (Q_matrix + Q_matrix.transpose());

	  // Covariance Time Update
	  P_matrix_ = PHI_matrix * P_matrix_ * PHI_matrix.transpose() + Q_matrix;
	  P_matrix_ = 0.5 * (P_matrix_ + P_matrix_.transpose());

	  num_ins_ ++;
	  SystemSaveStatus();
}

// Measurement Update
void LCEKF::Measurement_Update(Vector3d Meas_r_eb_e, Vector3d Meas_v_eb_e, double dualheading, bool is_dualheading) {
  // Initialize monitoring parameters
  check_meas = true;
  check_update = true;

  Vector3d p_LLA = E2D(Est_r_eb_e_);
  Matrix3d Est_C_ECEF2NED = TransE2NED(p_LLA);

  // Position Error
  Vector3d Delta_p_eb_e = (Meas_r_eb_e - Est_r_eb_e_);// Compute position error double precision, cast to float, apply transformation
  Vector3d Delta_p_ned = Est_C_ECEF2NED * Delta_p_eb_e;
  Delta_p_ned(2) = 0;
  Delta_p_eb_e = Est_C_ECEF2NED.transpose() * Delta_p_ned;

  // Velocity Error
  Vector3d Delta_v_eb_e = (Meas_v_eb_e - Est_v_eb_e_);

  // Create innovation vector
  Matrix<double,6,1> Delta_z; Delta_z.setZero();
  Delta_z.segment(0,3) = Delta_v_eb_e;
  Delta_z.segment(3,3) = Delta_p_eb_e;

  // Innovation covariance
  S_matrix_ = H_matrix_ * P_matrix_ * H_matrix_.transpose() + R_matrix_;

  // Kalman gain
  Matrix<double,15,6> K_matrix; K_matrix.setZero();
  K_matrix = P_matrix_ * H_matrix_.transpose() * S_matrix_.inverse();
  Matrix<double,15,15> P_prev; P_prev.block(0,0,15,15) = P_matrix_;

  // Covariance update, P = (I + K * H) * P * (I + K * H)' + K * R * K'
  Matrix<double,15,15> I_KH = I15 - K_matrix * H_matrix_; // temp
  P_matrix_ = I_KH * P_matrix_ * I_KH.transpose() + K_matrix * R_matrix_ * K_matrix.transpose();

  // Measurement health monitoring
  for(i=0; i++; i<6){
  	if(fabs(Delta_z(i, 1)) - 3 * sqrt(S_matrix_(i, i)) > 0)
  		check_meas = false;
  }
  // Measurement
  for(i=0; i++; i<16){
  	if(P_matrix_(i, i) - P_prev(i, i) >= 0)
  		check_update = false;
  }
  if(check_meas && check_update){
  	// State update, x = K * y
  	Matrix<double,15,1> x = K_matrix * Delta_z;

  	// Pull apart x terms to update the Position, velocity, orientation, and sensor biases
  	Vector3d quatDelta = x.segment(0,3); // Quaternion Delta
  	Vector3d vDeltaEst_ECEF = x.segment(3,3); // Velocity Deltas in ECEF
    Vector3d pDeltaEst_ECEF = x.segment(6,3); // Position Deltas in ECEF
  	Vector3d aBiasDelta = x.segment(9,3); // Accel Bias Deltas
  	Vector3d wBiasDelta = x.segment(12,3); // Rotation Rate Bias Deltas

  	Vector3d pDeltaEst_NED = Est_C_ECEF2NED * pDeltaEst_ECEF;

  	// Position update
  	Est_r_eb_e_ += pDeltaEst_ECEF;

  	// Velocity update
  	Est_v_eb_e_ += vDeltaEst_ECEF;

  	// Attitude correction
  	Quaterniond dQuat = Quaterniond(1.0, quatDelta(0), quatDelta(1), quatDelta(2));
  	Est_att_quat_ = (Est_att_quat_ * dQuat).normalized();
  	if (is_dualheading){
  		Est_att_euler_ = Quat2Euler(Est_att_quat_);
  		Est_att_euler_(2) = dualheading;
  		Est_att_quat_ = Euler2Quat(Est_att_euler_);
  	}
  	if (Est_att_quat_.w() < 0) {
  		 Est_att_quat_ = Quaterniond(Est_att_quat_.w(), Est_att_quat_.x(), Est_att_quat_.y(), Est_att_quat_.z());
  	}

  	// Update biases from states
  	Est_IMUaBias_ += aBiasDelta;
  	Est_IMUwBias_ += wBiasDelta;
  	}
  num_gnss_ ++;

  PrintStatus();

  if (is_dualheading)
	printf("Dualheading angle : %8.6f \n", dualheading/M_PI*180);

  MeasSaveStatus();
  }

void LCEKF::PrintStatus() {
	printf("Est Pos : %8.6f, %8.6f, %8.6f,Real Pos : %8.6f, %8.6f, %8.6f, Est Vel : %8.6f, %8.6f, %8.6f \n",Est_r_eb_e_(0), Est_r_eb_e_(1), Est_r_eb_e_(2), Meas_r_eb_e_(0), Meas_r_eb_e_(1), Meas_r_eb_e_(2), Est_v_eb_e_(0), Est_v_eb_e_(1), Est_v_eb_e_(2));
	printf("Att : %8.6f, %8.6f, %8.6f, System update delta t : %2.8f Num INS : %d, Num GNSS : %d \n \n \n", Est_att_euler_(0)/M_PI*180, Est_att_euler_(1)/M_PI*180, Est_att_euler_(2)/M_PI*180, dt_s_,num_ins_,num_gnss_);
}

void LCEKF::SystemSaveStatus() {
	fstream imudataFile;
	fstream covFile;
	fstream stateFile;
	string buffer;

	imudataFile.open("Output_imu.txt",ios::app);
	covFile.open("Covariance.txt",ios::app);
	stateFile.open("State.txt",ios::app);

	for (i=0; i<3; i++)  {
		imudataFile << Meas_f_ib_b_(i);
		imudataFile << ",";
	}

	for (i=0; i<3; i++)  {
		imudataFile << Meas_omega_ib_b_(i);
		imudataFile << ",";
	}

	covFile << "IMU";
	for (i=0; i<15; i++){
		covFile << ",";
		covFile << setprecision(4) << P_matrix_(i, i);
	}

	stateFile << "IMU,";

	for (i=0; i<3; i++){
		stateFile << setprecision(6) << Est_att_euler_(i)/M_PI*180;
		stateFile << ",";
	}

	for (i=0; i<3; i++){
		stateFile << setprecision(6) << Est_v_eb_e_(i);
		stateFile << ",";
	}

	for (i=0; i<3; i++){
		stateFile << setprecision(12) << Est_r_eb_e_(i);
		stateFile << ",";
	}

	for (i=0; i<3; i++){
		stateFile << setprecision(4) << Est_IMUaBias_(i);
		stateFile << ",";
	}

	for (i=0; i<3; i++){
		stateFile << setprecision(4) << Est_IMUwBias_(i);
		stateFile << ",";
	}

	stateFile << dt_s_;
	stateFile << ",";

	stateFile << setprecision(12) << t_gnss_;

	imudataFile << "\n";
	covFile << "\n";
	stateFile << "\n";

	imudataFile.close();
	covFile.close();
	stateFile.close();
}

void LCEKF::MeasSaveStatus() {
	fstream stateFile;
	fstream covFile;
	fstream truePFile;

	stateFile.open("State.txt",ios::app);
	covFile.open("Covariance.txt",ios::app);
	truePFile.open("True_Position.txt",ios::app);

	covFile << "GNSS";
	for (i=0; i<15; i++){
		covFile << ",";
		covFile << setprecision(4) << P_matrix_(i, i);
	}

	stateFile << "GNSS,";

	for (i=0; i<3; i++){
		stateFile << setprecision(6) << Est_att_euler_(i)/M_PI*180;
		stateFile << ",";
	}

	for (i=0; i<3; i++){
		stateFile << setprecision(6) << Est_v_eb_e_(i);
		stateFile << ",";
	}

	for (i=0; i<3; i++){
		stateFile << setprecision(12) << Est_r_eb_e_(i);
		stateFile << ",";
	}

	for (i=0; i<3; i++){
		stateFile << setprecision(4) << Est_IMUaBias_(i);
		stateFile << ",";
	}

	for (i=0; i<3; i++){
		stateFile << setprecision(4) << Est_IMUwBias_(i);
		stateFile << ",";
	}

	stateFile << dt_s_;
	stateFile << ",";
	stateFile << setprecision(12) << t_gnss_;

	for (i=0;i<3;i++){
		truePFile << setprecision(12) << Meas_r_eb_e_(i);
		truePFile << ",";
	}

	Vector3d pos_lla = E2D(Meas_r_eb_e_);
	for (i=0;i<3;i++){
		truePFile << setprecision(12) << pos_lla(i);
		truePFile << ",";
	}

	for (i=0;i<3;i++){
		truePFile << setprecision(12) << Meas_v_eb_e_(i);
		truePFile << ",";
	}

	for (i=0;i<6;i++){
		truePFile << setprecision(12) << R_matrix_(i,i);
		truePFile << ",";
	}

	covFile << "\n";
	stateFile << "\n";
	truePFile << "\n";

	stateFile.close();
	covFile.close();
	truePFile.close();
}





