#include <stdio.h>
#include <stdint.h>
#include "matops.h"

double c=299792458; //[m/s]
double pos_SD=3;
double vel_SD=0.1;
double Earth_rotation_rate=7.292115e-5; //[rad/s]
double R_0=6378137; //WGS84 Equatorial radius [m]
double mu=3.986004418e14; //WGS84 Earth gravitational constant [m^3s^-2]
double J_2=1.082627e-3; // %WGS84 Earth second gravitational constant
double e=0.0818191908425; //WGS84 eccentricity
double covariancelimit[17]={250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250,250};
/*
double gyro_noise_PSD = 3.3846e-11;
double accel_noise_PSD=3.8468e-1;
double gyro_bias_PSD=1.0e-7;
double accel_bias_PSD=2.0e-2;
*/
double accel_bias_PSD[3]={1.5253e-5, 2.7453e-5, 8.6334e-6};
double gyro_bias_PSD[3]={2.6683e-7, 5.2678e-8, 1.3345e-7};
double accel_noise_PSD[3]={5.0193e-4, 2.7478e-4, 2.6500e-4};
double gyro_noise_PSD[3]={1.0073e-5, 9.6946e-6, 9.9310e-6};

extern "C" void initdmat(dmat *dmatp, int nrl, int nrh, int ncl, int nch);
extern "C" void initdvec(dvec *dvecp, int nel, int neh);
extern "C" void zerodmat(dmat *dmatp);
extern "C" void setdmatrel(dmat *Ap, int relrow, int relcol, double value);
extern "C" void dmatinv(dmat *ATAp, dmat *Asip);
extern "C" void setdvec(dvec *dvecp, int sub, double val);
extern "C" void setdmat(dmat *dmatp, int sub1, int sub2, double val);
extern "C" void transdmat(dmat *matp, dmat *ansp);
extern "C" void dmatxdmat(dmat *mat1p, dmat *mat2p, dmat *ansp);
extern "C" void freedmat(dmat *dmatp);
extern "C" void freedvec(dvec *dvecp);
extern "C" void dmatxdvec(dmat *matp, dvec *vecp, dvec *ansp);
extern "C" void dvecplsdvec(dvec *dvec1p, dvec *dvec2p, dvec *ansp);
extern "C" void dvecxscal(dvec *dvecp, double scal, dvec *ansp);
extern "C" void dmatplsdmat(dmat *mat1p, dmat *mat2p, dmat *ansp);
extern "C" void dmatxscal(dmat *dmatp, double scal, dmat *ansp);
extern "C" void printdmat(dmat *dmatp);
extern "C" void printdvec(dvec *dvecp);

extern int no_const;
extern int no_state;
extern int step;
extern int clear;
int i,j;

void C_b_e_update(double state[], double C_b_e[][3]) {
	double ROLL = state[0];
	double PITCH = state[1];
	double YAW = state[2];

	dmat C_b_n_matrix=ZERODMAT;
	dmat C_n_e_matrix=ZERODMAT;
	dmat C_b_e_matrix=ZERODMAT;

	initdmat(&C_b_n_matrix, 1, 3, 1, 3);
	initdmat(&C_n_e_matrix, 1, 3, 1, 3);
	initdmat(&C_b_e_matrix, 1, 3, 1, 3);

	//Calculate Body to NED frame transformation matrix
	double C11=cos(ROLL)*cos(YAW);
	double C12=-cos(PITCH)*sin(YAW)+sin(PITCH)*sin(ROLL)*cos(YAW);
	double C13=sin(PITCH)*sin(YAW)+cos(PITCH)*sin(ROLL)*cos(YAW);
	double C21=cos(ROLL)*sin(YAW);
	double C22=cos(PITCH)*cos(YAW)+cos(PITCH)*sin(ROLL)*sin(YAW);
	double C23=-sin(PITCH)*cos(YAW)+cos(PITCH)*sin(ROLL)*sin(YAW);
	double C31=-sin(ROLL);
	double C32=sin(PITCH)*cos(ROLL);
	double C33=cos(PITCH)*cos(ROLL);

	setdmat(&C_b_n_matrix, 1, 1, C11);
	setdmat(&C_b_n_matrix, 1, 2, C12);
	setdmat(&C_b_n_matrix, 1, 3, C13);
	setdmat(&C_b_n_matrix, 2, 1, C21);
	setdmat(&C_b_n_matrix, 2, 2, C22);
	setdmat(&C_b_n_matrix, 2, 3, C23);
	setdmat(&C_b_n_matrix, 3, 1, C31);
	setdmat(&C_b_n_matrix, 3, 2, C32);
	setdmat(&C_b_n_matrix, 3, 3, C33);

	//Calculate NED to ECEF frame transformation matrix
	double lambda=atan2(state[7], state[6]);

	double k1=sqrt(1-e*e)*fabs(state[8]);
	double k2=e*e*R_0;
	double beta=sqrt(state[6]*state[6]+state[7]*state[7]);
	double E=(k1-k2)/beta;
	double F=(k1-k2)/beta;

	double P=(4/3)*(E*F+1);
	double Q=2*(E*E-F*F);
	double D=P*P*P+Q*Q;
	double V=pow(sqrt(D)-Q, 1/3)-pow(sqrt(D)+Q, 1/3);
	double G=0.5*(sqrt(E*E+V)+E);
	double T=sqrt(G*G+(F-V*G)/(2*G-E))-G;

	double L=(state[8]/fabs(state[8]))*atan((1-T*T)/(2*T*sqrt(1-e*e)));
	double height=(beta-R_0*T)*cos(L)+(state[8]-state[8]/fabs(state[8]))*R_0*sqrt(1-e*e)*sin(L);

	setdmat(&C_n_e_matrix, 1, 1, -sin(L)*cos(lambda));
	setdmat(&C_n_e_matrix, 1, 2, -sin(lambda));
	setdmat(&C_n_e_matrix, 1, 3, -cos(L)*cos(lambda));
	setdmat(&C_n_e_matrix, 2, 1, -sin(L)*sin(lambda));
	setdmat(&C_n_e_matrix, 2, 2, cos(lambda));
	setdmat(&C_n_e_matrix, 2, 3, -cos(L)*sin(lambda));
	setdmat(&C_n_e_matrix, 3, 1, cos(L));
	setdmat(&C_n_e_matrix, 3, 2, 0);
	setdmat(&C_n_e_matrix, 3, 3, -sin(L));

	//Calculate Body to ECEF frame transformation matrix
	dmatxdmat(&C_n_e_matrix, &C_b_n_matrix,&C_b_e_matrix);

	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++)
			C_b_e[i][j]=C_b_e_matrix.mat[i+1][j+1];
	}

	freedmat(&C_b_n_matrix);
	freedmat(&C_n_e_matrix);
	freedmat(&C_b_e_matrix);

}// C_b_e_update


void initialization(double IMU_data[], double state[], double C_b_e[][3], double dualheading) {
	double acc_x=IMU_data[3];
	double acc_y=IMU_data[4];
	double acc_z=IMU_data[5];

	double ROLL;
	double PITCH;
	double YAW;

	PITCH = atan((acc_x)/sqrt(acc_y * acc_y + acc_z * acc_z));
	ROLL = atan2(-acc_y,-acc_z);
	//PITCH = atan((acc_y)/sqrt(acc_x * acc_x + acc_z * acc_z));
	YAW = dualheading/180*M_PI;
	printf("initial att : %f %f %f",ROLL/M_PI*180,PITCH/M_PI*180,YAW);
	printf("\n IMU_DATA : %f, %f, %f, %f, %f, %f",IMU_data[0],IMU_data[1],IMU_data[2],IMU_data[3],IMU_data[4],IMU_data[5]);

	state[0] = ROLL;
	state[1] = PITCH;
	state[2] = YAW;

	dmat C_b_n_matrix=ZERODMAT;
	dmat C_n_e_matrix=ZERODMAT;
	dmat C_b_e_matrix=ZERODMAT;

	initdmat(&C_b_n_matrix, 1, 3, 1, 3);
	initdmat(&C_n_e_matrix, 1, 3, 1, 3);
	initdmat(&C_b_e_matrix, 1, 3, 1, 3);

	//Calculate Body to NED frame transformation matrix
	double C11=cos(ROLL)*cos(YAW);
	double C12=-cos(PITCH)*sin(YAW)+sin(PITCH)*sin(ROLL)*cos(YAW);
	double C13=sin(PITCH)*sin(YAW)+cos(PITCH)*sin(ROLL)*cos(YAW);
	double C21=cos(ROLL)*sin(YAW);
	double C22=cos(PITCH)*cos(YAW)+cos(PITCH)*sin(ROLL)*sin(YAW);
	double C23=-sin(PITCH)*cos(YAW)+cos(PITCH)*sin(ROLL)*sin(YAW);
	double C31=-sin(ROLL);
	double C32=sin(PITCH)*cos(ROLL);
	double C33=cos(PITCH)*cos(ROLL);

	setdmat(&C_b_n_matrix, 1, 1, C11);
	setdmat(&C_b_n_matrix, 1, 2, C12);
	setdmat(&C_b_n_matrix, 1, 3, C13);
	setdmat(&C_b_n_matrix, 2, 1, C21);
	setdmat(&C_b_n_matrix, 2, 2, C22);
	setdmat(&C_b_n_matrix, 2, 3, C23);
	setdmat(&C_b_n_matrix, 3, 1, C31);
	setdmat(&C_b_n_matrix, 3, 2, C32);
	setdmat(&C_b_n_matrix, 3, 3, C33);

	//Calculate NED to ECEF frame transformation matrix
	double lambda=atan2(state[7], state[6]);

	double k1=sqrt(1-e*e)*fabs(state[8]);
	double k2=e*e*R_0;
	double beta=sqrt(state[6]*state[6]+state[7]*state[7]);
	double E=(k1-k2)/beta;
	double F=(k1-k2)/beta;

	double P=(4/3)*(E*F+1);
	double Q=2*(E*E-F*F);
	double D=P*P*P+Q*Q;
	double V=pow(sqrt(D)-Q, 1/3)-pow(sqrt(D)+Q, 1/3);
	double G=0.5*(sqrt(E*E+V)+E);
	double T=sqrt(G*G+(F-V*G)/(2*G-E))-G;

	double L=(state[8]/fabs(state[8]))*atan((1-T*T)/(2*T*sqrt(1-e*e)));
	double height=(beta-R_0*T)*cos(L)+(state[8]-state[8]/fabs(state[8]))*R_0*sqrt(1-e*e)*sin(L);

	setdmat(&C_n_e_matrix, 1, 1, -sin(L)*cos(lambda));
	setdmat(&C_n_e_matrix, 1, 2, -sin(lambda));
	setdmat(&C_n_e_matrix, 1, 3, -cos(L)*cos(lambda));
	setdmat(&C_n_e_matrix, 2, 1, -sin(L)*sin(lambda));
	setdmat(&C_n_e_matrix, 2, 2, cos(lambda));
	setdmat(&C_n_e_matrix, 2, 3, -cos(L)*sin(lambda));
	setdmat(&C_n_e_matrix, 3, 1, cos(L));
	setdmat(&C_n_e_matrix, 3, 2, 0);
	setdmat(&C_n_e_matrix, 3, 3, -sin(L));

	//Calculate Body to ECEF frame transformation matrix
	dmatxdmat(&C_n_e_matrix, &C_b_n_matrix,&C_b_e_matrix);

	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++)
			C_b_e[i][j]=C_b_e_matrix.mat[i+1][j+1];
	}

	freedmat(&C_b_n_matrix);
	freedmat(&C_n_e_matrix);
	freedmat(&C_b_e_matrix);

} // function : Initialization

void Attitude_update(double state[],double C_b_e[][3]){

	dmat C_b_e_matrix=ZERODMAT;
	dmat C_b_n_matrix=ZERODMAT;
	dmat C_e_n_matrix=ZERODMAT;

	initdmat(&C_b_e_matrix, 1, 3, 1, 3);
	initdmat(&C_b_n_matrix, 1, 3, 1, 3);
	initdmat(&C_e_n_matrix, 1, 3, 1, 3);

	//Calculate NED to ECEF frame transformation matrix
	double lambda=atan2(state[7], state[6]);

	double k1=sqrt(1-e*e)*fabs(state[8]);
	double k2=e*e*R_0;
	double beta=sqrt(state[6]*state[6]+state[7]*state[7]);
	double E=(k1-k2)/beta;
	double F=(k1-k2)/beta;

	double P=(4/3)*(E*F+1);
	double Q=2*(E*E-F*F);
	double D=P*P*P+Q*Q;
	double V=pow(sqrt(D)-Q, 1/3)-pow(sqrt(D)+Q, 1/3);
	double G=0.5*(sqrt(E*E+V)+E);
	double T=sqrt(G*G+(F-V*G)/(2*G-E))-G;

	double L=(state[8]/fabs(state[8]))*atan((1-T*T)/(2*T*sqrt(1-e*e)));
	double height=(beta-R_0*T)*cos(L)+(state[8]-state[8]/fabs(state[8]))*R_0*sqrt(1-e*e)*sin(L);

	setdmat(&C_b_e_matrix, 1, 1, C_b_e[0][0]);
	setdmat(&C_b_e_matrix, 1, 2, C_b_e[0][1]);
	setdmat(&C_b_e_matrix, 1, 3, C_b_e[0][2]);
	setdmat(&C_b_e_matrix, 2, 1, C_b_e[1][0]);
	setdmat(&C_b_e_matrix, 2, 2, C_b_e[1][1]);
	setdmat(&C_b_e_matrix, 2, 3, C_b_e[1][2]);
	setdmat(&C_b_e_matrix, 3, 1, C_b_e[2][0]);
	setdmat(&C_b_e_matrix, 3, 2, C_b_e[2][1]);
	setdmat(&C_b_e_matrix, 3, 3, C_b_e[2][2]);

	//calculate ECEF to NED coordinate transformation matrix
	setdmat(&C_e_n_matrix, 1, 1, -sin(L)*cos(lambda));
	setdmat(&C_e_n_matrix, 1, 2, -sin(L)*sin(lambda));
	setdmat(&C_e_n_matrix, 1, 3, cos(L));
	setdmat(&C_e_n_matrix, 2, 1, -sin(lambda));
	setdmat(&C_e_n_matrix, 2, 2, cos(lambda));
	setdmat(&C_e_n_matrix, 2, 3, 0);
	setdmat(&C_e_n_matrix, 3, 1, -cos(L)*cos(lambda));
	setdmat(&C_e_n_matrix, 3, 2, -cos(L)*sin(lambda));
	setdmat(&C_e_n_matrix, 3, 3, -sin(L));

	//transform attitude
	dmatxdmat(&C_e_n_matrix, &C_b_e_matrix, &C_b_n_matrix);

	state[0]= asin(-C_b_n_matrix.mat[3][1]); //roll
	state[1]= asin(C_b_n_matrix.mat[3][2]/cos(state[0])); //pitch
	state[2]= asin(C_b_n_matrix.mat[2][1]/cos(state[0])); //yaw

	freedmat(&C_b_n_matrix);
	freedmat(&C_e_n_matrix);
	freedmat(&C_b_e_matrix);

	//double C11=cos(ROLL)*cos(YAW);
	//double C12=-cos(PITCH)*sin(YAW)+sin(PITCH)*sin(ROLL)*cos(YAW);
	//double C13=sin(PITCH)*sin(YAW)+cos(PITCH)*sin(ROLL)*cos(YAW);
	//double C21=cos(ROLL)*sin(YAW);
	//double C22=cos(PITCH)*cos(YAW)+cos(PITCH)*sin(ROLL)*sin(YAW);
	//double C23=-sin(PITCH)*cos(YAW)+cos(PITCH)*sin(ROLL)*sin(YAW);
	//double C31=-sin(ROLL);
	//double C32=sin(PITCH)*cos(ROLL);
	//double C33=cos(PITCH)*cos(ROLL);
}

void ECEF_to_NED(double est_r_e[], double est_r_n[], double C_b_e[][3]){
//input
//est_r_e [3] ECEF position vector
//est_v_e [3] ECEF velocity vector
//est_r_n [3] NED position vector
//C_b_e [3][3] body to ECEF transformation matrix
//
//output
//est_r_n [3] NED position vector

	double L;
	double lambda;
	double height;

	//dvec r_e=ZERODVEC; //ECEF position vector
	//dvec v_e=ZERODVEC; //ECEF velocity vector
	//dvec v_n=ZERODVEC;

	//dmat C_b_e_matrix=ZERODMAT;
	//dmat C_b_n_matrix=ZERODMAT;
	//dmat C_e_n_matrix=ZERODMAT;

	//initdvec(&r_e, 1, 3);
	//initdvec(&v_e, 1, 3);
	//initdvec(&v_n, 1, 3);

	//initdmat(&C_b_e_matrix, 1, 3, 1, 3);
	//initdmat(&C_b_n_matrix, 1, 3, 1, 3);
	//initdmat(&C_e_n_matrix, 1, 3, 1, 3);

	//for(int i=1; i<4; i++)
	//	setdvec(&r_e, i, est_r_e[i-1]);

	lambda=atan2(est_r_e[1], est_r_e[0]);

	double k1=sqrt(1-e*e)*fabs(est_r_e[2]);
	double k2=e*e*R_0;
	double beta=sqrt(est_r_e[0]*est_r_e[0]+est_r_e[1]*est_r_e[1]);
	double E=(k1-k2)/beta;
	double F=(k1+k2)/beta;

	double P=(4/3)*(E*F+1);
	double Q=2*(E*E-F*F);
	double D=P*P*P+Q*Q;
	double V=pow(sqrt(D)-Q, 1/3)-pow(sqrt(D)+Q, 1/3);
	double G=0.5*(sqrt(E*E+V)+E);
	double T=sqrt(G*G+(F-V*G)/(2*G-E))-G;

	L=(est_r_e[2]/fabs(est_r_e[2]))*atan((1-T*T)/(2*T*sqrt(1-e*e)));
	height=(beta-R_0*T)*cos(L)+(est_r_e[2]-est_r_e[2]/fabs(est_r_e[2]))*R_0*sqrt(1-e*e)*sin(L);

	//calculate ECEF to NED coordinate transformation matrix
	//setdmat(&C_e_n_matrix, 1, 1, -sin(L)*cos(lambda));
	//setdmat(&C_e_n_matrix, 1, 2, -sin(L)*sin(lambda));
	//setdmat(&C_e_n_matrix, 1, 3, cos(L));
	//setdmat(&C_e_n_matrix, 2, 1, -sin(lambda));
	//setdmat(&C_e_n_matrix, 2, 2, cos(lambda));
	//setdmat(&C_e_n_matrix, 2, 3, 0);
	//setdmat(&C_e_n_matrix, 3, 1, -cos(L)*cos(lambda));
	//setdmat(&C_e_n_matrix, 3, 2, -cos(L)*sin(lambda));
	//setdmat(&C_e_n_matrix, 3, 3, -sin(L));

	//transform velocity
	//dmatxdvec(&C_e_n_matrix, &v_e, &v_n);

	//transform attitude
	//dmatxdmat(&C_e_n_matrix, &C_b_e_matrix, C_b_n_matrix);

	est_r_n[0]=L;
	est_r_n[1]=lambda;
	est_r_n[2]=height;
}

void System_Update(double IMU_data[], double state[], double **P, double C_b_e[][3], double tor, double dualheading){
//input
//IMU_data[6] measurement vector accelerometer(3) and gyro measurement(3)
//state[no_state] state vector attitude(3), velocity(3), position(3), IMU bias(6)
//P [no_state][no_state]
//C_b_e[3][3] transformation matrix from body to ECEF
//
//output
//state [no_state] vector with updated attitude, velocity, position, IMU bias
//P updated covariance matrix
//C_b_n updated body to ECEF transformation matrix



	double delta; //earth rotation
	double mag_r; //distance from center of the Earth
	double mag_alpha; //magnitude of attitude increment
	double z_scale; // constant for gravity calculation

	//matops header for matrix calculation

	//Define vector and matrix
	dvec a_e=ZERODVEC; //ECEF acceleration vector
	dvec est_v_e=ZERODVEC; //ECEF velocity vector
	dvec est_r_e=ZERODVEC; //ECEF position vector
	dvec a_b=ZERODVEC; //Body frame accerleration
	dvec est_v_e_old=ZERODVEC; //old ECEF velocity
	dvec est_r_e_old=ZERODVEC; //old ECEF position
	dvec g=ZERODVEC; //gravity vector
	dvec gamma=ZERODVEC; //vector for gravity calculation
	dvec alpha=ZERODVEC; //attitude increment
	dvec tempvec1=ZERODVEC; //empty vector for calculation
	dvec tempvec2=ZERODVEC;

	dmat I=ZERODMAT; //identity matrix
	dmat C_earth=ZERODMAT; //earth rotation transformation matrix
	dmat Alpha=ZERODMAT; //skew symmetric matrix of attitude increment
	dmat Alpha2=ZERODMAT; //Alpha^2
	dmat old_C_b_e=ZERODMAT; //transformation matrix before update
	dmat C_new_old=ZERODMAT; //transformation matrix
	dmat ave_C_b_e=ZERODMAT; //average body to ECEF transformation  matrix
	dmat C=ZERODMAT; //Updated body to ECEF transformation matrix
	dmat Omega=ZERODMAT; //skew symmetry of earth rotation rate
	dmat phi_matrix=ZERODMAT; //system update matrix
	dmat Q_matrix=ZERODMAT; //system noise matrix
	dmat P_matrix_old=ZERODMAT; //old covariance matrix
	dmat P_matrix=ZERODMAT; //covariance matrix
	dmat skew_earth=ZERODMAT; //skew symmetric matrix of earth rotation
	dmat tempmat1=ZERODMAT; //empty matrix for calculation
	dmat tempmat2=ZERODMAT; //empty matrix for calculation
	dmat tempmat3=ZERODMAT; //empty matrix for calculation
	dmat tempmat4=ZERODMAT; //empty matrix for calculation
	dmat tempmat5=ZERODMAT; //empty matrix for calculation
	dmat tempmat6=ZERODMAT; //empty matrix for calculation

	//Initialize defined vector and matrix
	initdvec(&a_e, 1, 3);
	initdvec(&est_v_e, 1, 3);
	initdvec(&est_r_e, 1, 3);
	initdvec(&a_b, 1, 3);
	initdvec(&est_v_e_old, 1, 3);
	initdvec(&est_r_e_old, 1, 3);
	initdvec(&g, 1, 3);
	initdvec(&gamma, 1, 3);
	initdvec(&alpha, 1, 3);
	initdvec(&tempvec1, 1, 3);
	initdvec(&tempvec2, 1, 3);

	initdmat(&I, 1, 3, 1, 3);
	initdmat(&C_earth, 1, 3, 1, 3);
	initdmat(&Alpha, 1, 3, 1, 3);
	initdmat(&Alpha2, 1, 3, 1, 3);
	initdmat(&C_new_old, 1, 3, 1, 3);
	initdmat(&ave_C_b_e, 1, 3, 1, 3);
	initdmat(&C, 1, 3, 1, 3);
	initdmat(&Omega, 1, 3, 1, 3);
	initdmat(&phi_matrix, 1, no_state, 1, no_state);
	initdmat(&Q_matrix, 1, no_state, 1, no_state);
	initdmat(&P_matrix_old, 1, no_state, 1, no_state);
	initdmat(&P_matrix, 1, no_state, 1, no_state);
	initdmat(&old_C_b_e, 1, 3, 1, 3);
	initdmat(&skew_earth, 1, 3, 1, 3);
	initdmat(&tempmat1, 1, 3, 1, 3);
	initdmat(&tempmat2, 1, 3, 1, 3);
	initdmat(&tempmat3, 1, 3, 1, 3);
	initdmat(&tempmat4, 1, no_state, 1, no_state);
	initdmat(&tempmat5, 1, no_state, 1, no_state);
	initdmat(&tempmat6, 1, no_state, 1, no_state);

	//Save input at vector or matrix
	for(int i=1; i<4; i++){
		setdvec(&a_b, i, IMU_data[i+2]); /////////////// before i-1
		setdvec(&est_v_e_old, i, state[2+i]);
		setdvec(&est_r_e_old, i, state[5+i]);
	}

	for(int i=1; i<no_state+1; i++){
		for(int j=1; j<no_state+1; j++)
			setdmat(&P_matrix_old, i, j, P[i-1][j-1]);
	}

	setdmat(&old_C_b_e, 1, 1, C_b_e[0][0]);
	setdmat(&old_C_b_e, 1, 2, C_b_e[0][1]);
	setdmat(&old_C_b_e, 1, 3, C_b_e[0][2]);
	setdmat(&old_C_b_e, 2, 1, C_b_e[1][0]);
	setdmat(&old_C_b_e, 2, 2, C_b_e[1][1]);
	setdmat(&old_C_b_e, 2, 3, C_b_e[1][2]);
	setdmat(&old_C_b_e, 3, 1, C_b_e[2][0]);
	setdmat(&old_C_b_e, 3, 2, C_b_e[2][1]);
	setdmat(&old_C_b_e, 3, 3, C_b_e[2][2]);

	//Earth rotation vector skew symmetry calculation
	setdmat(&Omega, 1, 1, 0);
	setdmat(&Omega, 1, 2, -Earth_rotation_rate);
	setdmat(&Omega, 1, 3, 0);
	setdmat(&Omega, 2, 1, Earth_rotation_rate);
	setdmat(&Omega, 2, 2, 0);
	setdmat(&Omega, 2, 3, 0);
	setdmat(&Omega, 3, 1, 0);
	setdmat(&Omega, 3, 2, 0);
	setdmat(&Omega, 3, 3, 0);

	//Identity matrix
 	setdmat(&I, 1, 1, 1);
	setdmat(&I, 1, 2, 0);
	setdmat(&I, 1, 3, 0);
	setdmat(&I, 2, 1, 0);
	setdmat(&I, 2, 2, 1);
	setdmat(&I, 2, 3, 0);
	setdmat(&I, 3, 1, 0);
	setdmat(&I, 3, 2, 0);
	setdmat(&I, 3, 3, 1);

	//ATTITUDE UPDATE

	delta=Earth_rotation_rate*tor;

	setdmat(&C_earth, 1, 1, cos(delta));
	setdmat(&C_earth, 1, 2, sin(delta));
	setdmat(&C_earth, 1, 3, 0);
	setdmat(&C_earth, 2, 1, -sin(delta));
	setdmat(&C_earth, 2, 2, cos(delta));
	setdmat(&C_earth, 2, 3, 0);
	setdmat(&C_earth, 3, 1, 0);
	setdmat(&C_earth, 3, 2, 0);
	setdmat(&C_earth, 3, 3, 1); //cos, sin function header

	//Calculate attitude change
	setdvec(&alpha, 1, IMU_data[0]*tor);
	setdvec(&alpha, 2, IMU_data[1]*tor);
	setdvec(&alpha, 3, IMU_data[2]*tor);

	//Attitude update

	double acc_x=IMU_data[3];
	double acc_y=IMU_data[4];
	double acc_z=IMU_data[5];

	double ROLL;
	double PITCH;
	double YAW;
	ROLL = atan2(-acc_y,-acc_z);
	PITCH = atan((acc_x)/sqrt(acc_y * acc_y + acc_z * acc_z));

	if (dualheading != 0)
		YAW = dualheading/180*M_PI;

	double ROLL_prev = state[0];
	double PITCH_prev = state[1];
	double YAW_prev = state[2];

	//state[0] += alpha.vec[1];
	//state[1] += alpha.vec[2];
	double acc_angle_ratio = 0.0002;

	state[2] += alpha.vec[3];

	state[0] = (1-acc_angle_ratio) * (ROLL_prev + alpha.vec[1]) + acc_angle_ratio * ROLL;
	state[1] = (1-acc_angle_ratio) * (PITCH_prev + alpha.vec[2]) + acc_angle_ratio * PITCH;


	if (dualheading != 0){
		state[2] = 0.98 * (YAW_prev + alpha.vec[3]) + 0.02 * YAW;
	}
	mag_alpha=sqrt(alpha.vec[1]*alpha.vec[1]+alpha.vec[2]*alpha.vec[2]+alpha.vec[3]*alpha.vec[3]);

	//skew symmetric matrix calculation
	setdmat(&Alpha, 1, 1, 0);
	setdmat(&Alpha, 1, 2, -alpha.vec[3]);
	setdmat(&Alpha, 1, 3, alpha.vec[2]);
	setdmat(&Alpha, 2, 1, alpha.vec[3]);
	setdmat(&Alpha, 2, 2, 0);
	setdmat(&Alpha, 2, 3, -alpha.vec[1]);
	setdmat(&Alpha, 3, 1, -alpha.vec[2]);
	setdmat(&Alpha, 3, 2, alpha.vec[1]);
	setdmat(&Alpha, 3, 3, 0);

	dmatxdmat(&Alpha, &Alpha, &Alpha2);

	if (mag_alpha > 1.0e-8){
		double c1=sin(mag_alpha)/mag_alpha;
		double c2=(1-cos(mag_alpha))/(mag_alpha*mag_alpha);

		for(int i=1; i<4; i++){
			for(int j=1; j<4; j++)
				setdmat(&C_new_old, i, j, I.mat[i][j]+c1*Alpha.mat[i][j]+c2*Alpha2.mat[i][j]);
		}
	}
	else{
		for(int i=1; i<4; i++){
			for(int j=1; j<4; j++)
				setdmat(&C_new_old, i, j, I.mat[i][j]+Alpha.mat[i][j]);
		}
	}

	//Update transformation matrix
	dmatxdmat(&old_C_b_e, &C_new_old, &tempmat1);
	dmatxdmat(&C_earth, &tempmat1, &C);

	//ACCELERLATION FRAME TRANSFORMATION

	//skew symmetric matrix calculation
	setdmat(&skew_earth, 1, 1, 0);
	setdmat(&skew_earth, 1, 2, -delta);
	setdmat(&skew_earth, 1, 3, 0);
	setdmat(&skew_earth, 2, 1, delta);
	setdmat(&skew_earth, 2, 2, 0);
	setdmat(&skew_earth, 2, 3, 0);
	setdmat(&skew_earth, 3, 1, 0);
	setdmat(&skew_earth, 3, 2, 0);
	setdmat(&skew_earth, 3, 3, 0);

	if (mag_alpha > 1.0e-8){
		dmatxdmat(&skew_earth, &old_C_b_e, &tempmat1);
		double c3=(1-cos(mag_alpha))/(mag_alpha*mag_alpha);
		double c4=(1-sin(mag_alpha)/(mag_alpha))/(mag_alpha*mag_alpha);

		for(int i=1; i<4; i++){
			for(int j=1; j<4; j++)
				setdmat(&tempmat2, i, j, I.mat[i][j]+c3*Alpha.mat[i][j]+c4*Alpha2.mat[i][j]);
		}
		dmatxdmat(&old_C_b_e, &tempmat2, &tempmat3);
		for(int i=1; i<4; i++){
				for(int j=1; j<4; j++)
					setdmat(&ave_C_b_e, i, j, tempmat3.mat[i][j]-0.5*tempmat1.mat[i][j]);
		}
	}
	else{
		for(int i=1; i<4; i++){
			for(int j=1; j<4; j++)
				setdmat(&ave_C_b_e, i, j, old_C_b_e.mat[i][j]-0.5*tempmat1.mat[i][j]);
		}
	}

	//Transform acceleration to ECEF frame
	dmatxdvec(&ave_C_b_e, &a_b, &a_e);

	//VELOCITY UPDATE

	//Gravity vector calculation
	///*
	mag_r=sqrt(est_r_e_old.vec[1]*est_r_e_old.vec[1]+est_r_e_old.vec[2]*est_r_e_old.vec[2]+est_r_e_old.vec[3]*est_r_e_old.vec[3]);

	if(mag_r==0){
		for(int i=1; i<4; i++){
			setdvec(&g, i, 0);
		}
	}
	else{
		z_scale=5*(est_r_e_old.vec[3]/mag_r)*(est_r_e_old.vec[3]/mag_r);

		setdvec(&gamma, 1, (1-z_scale)*est_r_e_old.vec[1]);
		setdvec(&gamma, 2, (1-z_scale)*est_r_e_old.vec[2]);
		setdvec(&gamma, 3, (3-z_scale)*est_r_e_old.vec[3]);

		double c5=-mu/(mag_r*mag_r*mag_r);
		double c6=1.5*J_2*(R_0/mag_r)*(R_0/mag_r);

		for(int i=1; i<4; i++)
			setdvec(&gamma, i, c5*(est_r_e_old.vec[i]+c6*gamma.vec[i]));

		setdvec(&g, 1, gamma.vec[1]+Earth_rotation_rate*Earth_rotation_rate*est_r_e_old.vec[1]);
		setdvec(&g, 2, gamma.vec[2]+Earth_rotation_rate*Earth_rotation_rate*est_r_e_old.vec[2]);
		setdvec(&g, 3, gamma.vec[3]);
	}
	//*/

	//Velocity update

	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++)
			setdmat(&tempmat1, i, j, skew_earth.mat[i][j]/tor);
	}
	dmatxdvec(&tempmat1, &est_v_e_old, &tempvec1);

	for(int i=1; i<4; i++)
		setdvec(&est_v_e, i, est_v_e_old.vec[i]+tor*(a_e.vec[i]-2*tempvec1.vec[i]+g.vec[i]));

	//POSITION UPDATE

	for(int i=1; i<4; i++)
		setdvec(&est_r_e, i, est_r_e_old.vec[i]+(est_v_e.vec[i]+est_v_e_old.vec[i])*tor*0.5);

	//Save updated state and transformation matrix

	for(int i=1; i<4; i++){
		//state[i-1]=ave_C_b_e.mat[3][i];
		state[2+i]=est_v_e.vec[i];
		state[5+i]=est_r_e.vec[i];
		for(int j=1; j<4; j++)
			C_b_e[i-1][j-1]=C.mat[i][j];
	}

	//COVARIANCE UPDATE

	double r_e[3];
	double r_n[3];

	//Using updated state
	for(int i=1; i<4; i++)
		r_e[i-1]=state[5+i];

	ECEF_to_NED(r_e, r_n, C_b_e);

	//Set system update matrix,phi as I
	for(int i=1; i<no_state+1; i++){
		for(int j=1; j<no_state+1; j++)
			if(i==j)
				setdmat(&phi_matrix, i, j, 1);
			else
				setdmat(&phi_matrix, i, j, 0);
	}

	//Determine system update matrix, phi_matrix
	//[1:3][1:3]
	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++)
			setdmat(&phi_matrix, i, j, phi_matrix.mat[i][j]-Omega.mat[i][j]*tor);
	}

	//[1:3][13:15]
	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++)
			setdmat(&phi_matrix, i, 12+j, C.mat[i][j]*tor);
	}

	//[4:6][1:3]
	dmatxdvec(&C, &a_b, &a_e);
	setdmat(&tempmat1, 1, 1, 0);
	setdmat(&tempmat1, 1, 2, -a_e.vec[3]);
	setdmat(&tempmat1, 1, 3, a_e.vec[2]);
	setdmat(&tempmat1, 2, 1, a_e.vec[3]);
	setdmat(&tempmat1, 2, 2, 0);
	setdmat(&tempmat1, 2, 3, -a_e.vec[1]);
	setdmat(&tempmat1, 3, 1, -a_e.vec[2]);
	setdmat(&tempmat1, 3, 2, a_e.vec[1]);
	setdmat(&tempmat1, 3, 3, 0);

	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++)
			setdmat(&phi_matrix, 3+i, j, -tor*tempmat1.mat[i][j]);
	}


	//[4:6][4:6]
	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++)
			setdmat(&phi_matrix, 3+i, 3+j, phi_matrix.mat[3+i][3+j]-2*Omega.mat[i][j]*tor);
	}

	//[4:6][7:9]
	double geocentric_radius=R_0/sqrt(1-(e*sin(r_n[0]))*(e*sin(r_n[0])))*sqrt(cos(r_n[0])*cos(r_n[0])+(1-e*e)*(1-e*e)*sin(r_n[0])*sin(r_n[0]));
	mag_r=sqrt(r_e[0]*r_e[0]+r_e[1]*r_e[1]+r_e[2]*r_e[2]);
	double c1=1/(geocentric_radius*mag_r);

	//Gravity calculation

	if(mag_r==0){
		for(int i=1; i<4; i++){
			setdvec(&g, i, 0);
		}
	}
	else{
		double z_scale=5*(r_e[2]/mag_r)*(r_e[2]/mag_r);

		setdvec(&gamma, 1, (1-z_scale)*r_e[0]);
		setdvec(&gamma, 2, (1-z_scale)*r_e[1]);
		setdvec(&gamma, 3, (3-z_scale)*r_e[2]);

		double c5=-mu/(mag_r*mag_r*mag_r);
		double c6=1.5*J_2*(R_0/mag_r)*(R_0/mag_r);

		for(int i=1; i<4; i++)
			setdvec(&gamma, i, c5*(r_e[i-1]+c6*gamma.vec[i]));

		setdvec(&g, 1, gamma.vec[1]+Earth_rotation_rate*Earth_rotation_rate*r_e[0]);
		setdvec(&g, 2, gamma.vec[2]+Earth_rotation_rate*Earth_rotation_rate*r_e[1]);
		setdvec(&g, 3, gamma.vec[3]);
	}

	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++)
			setdmat(&tempmat1, i, j, g.vec[i]*r_e[j-1]);
	}

	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++)
			setdmat(&phi_matrix, 3+i, 6+j, -tor*2*c1*tempmat1.mat[i][j]);
	}

	//[4:6][10:12]
	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++)
			setdmat(&phi_matrix, 3+i, 9+j, C.mat[i][j]*tor);
	}

	//[7:9][4:6]
	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++){
			if(i==j)
				setdmat(&phi_matrix, 6+i, 3+j, tor);
			else
				setdmat(&phi_matrix, 6+i, 3+j, 0);
		}
	}

	//Determine approximate system noise covariance matrix, Q_matrix
	for(int i=1; i<no_state+1; i++){
			for(int j=1; j<no_state+1; j++)
				setdmat(&Q_matrix, i, j, 0);
	}
	int k;
	//[1:3][1:3]
	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++){
			if(i==j){
				k = -1.5 * (i-1) * (i-1) + 2.5 * (i-1) +1;
				setdmat(&Q_matrix, i, j, gyro_noise_PSD[k] * tor);
			}
			else
				setdmat(&Q_matrix, i, j, 0);
		}
	}

	//[4:6][4:6]
	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++){
			if(i==j){
				k = -1.5 * (i-1) * (i-1) + 2.5 * (i-1) +1;
				setdmat(&Q_matrix, 3+i, 3+j, accel_noise_PSD[k] * tor);
			}
			else
				setdmat(&Q_matrix, 3+i, 3+j, 0);
		}
	}

	//[10:12][10:12]
	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++){
			if(i==j){
				k = -1.5 * (i-1) * (i-1) + 2.5 * (i-1) +1;
				setdmat(&Q_matrix, 9+i, 9+j, accel_bias_PSD[k] * tor);
			}
			else
				setdmat(&Q_matrix, 9+i, 9+j, 0);
		}
	}


	//[13:15][13:15]
	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++){
			if(i==j){
				k = -1.5 * (i-1) * (i-1) + 2.5 * (i-1) +1;
				setdmat(&Q_matrix, 12+i, 12+j, gyro_bias_PSD[k] * tor);
			}
			else
				setdmat(&Q_matrix, 12+i, 12+j, 0);
		}
	}
	//printdmat(&Q_matrix);
	//Covariance system update

	for(int i=1; i<no_state+1; i++){
		for(int j=1; j<no_state+1; j++)
			setdmat(&tempmat4, i, j, P_matrix_old.mat[i][j]+0.5*Q_matrix.mat[i][j]);
	}
	transdmat(&phi_matrix, &tempmat5);
	dmatxdmat(&tempmat4, &tempmat5, &tempmat6);
	dmatxdmat(&phi_matrix, &tempmat6, &tempmat4);

	for(int i=1; i<no_state+1; i++){
		for(int j=1; j<no_state+1; j++)
			setdmat(&P_matrix, i, j, tempmat4.mat[i][j]+0.5*Q_matrix.mat[i][j]);
	}

	//Check covariance diagonal terms to prevent infinite growth of variance
	int check=0;
	for(int i=1; i<no_state+1; i++){
		if(P_matrix.mat[i][i]>covariancelimit[i-1])
			check--;
	}

	if(check==0){
		transdmat(&P_matrix, &tempmat4);
		dmatplsdmat(&P_matrix, &tempmat4, &tempmat5);
		dmatxscal(&tempmat5, 0.5, &P_matrix);
	}
	else
		dmatxscal(&P_matrix_old, 1, &P_matrix);

	for(int i=1; i<no_state+1; i++){
		for(int j=1; j<no_state+1; j++)
			P[i-1][j-1]=P_matrix.mat[i][j];
	}

	//Attitude_update(state,C_b_e);
	//printdmat(&phi_matrix);

	freedvec(&a_e);
	freedvec(&est_v_e);
	freedvec(&est_r_e);
	freedvec(&a_b);
	freedvec(&est_v_e_old);
	freedvec(&est_r_e_old);
	freedvec(&g);
	freedvec(&gamma);
	freedvec(&alpha);
	freedvec(&tempvec1);

	freedmat(&I);
	freedmat(&C_earth);
	freedmat(&Alpha);
	freedmat(&Alpha2);
	freedmat(&C_new_old);
	freedmat(&ave_C_b_e);
	freedmat(&C);
	freedmat(&Omega);
	freedmat(&phi_matrix);
	freedmat(&Q_matrix);
	freedmat(&P_matrix_old);
	freedmat(&P_matrix);
	freedmat(&old_C_b_e);
	freedmat(&skew_earth);
	freedmat(&tempmat1);
	freedmat(&tempmat2);
	freedmat(&tempmat3);
	freedmat(&tempmat4);
	freedmat(&tempmat5);
	freedmat(&tempmat6);

	//C_b_e_update(state, C_b_e);
} //function : System Update

void Measurement_Update(double state[], double best_state[], double best_state_SD[], double **P, double C_b_e[][3], double tor) {
//input
//state [15] estimated ECEF state vector attitude[3], velocity[3], position[3], IMU bias[6], clock bias and drift
//best_state[6] receiver best position[3] and best velocity[3]
//best_state_SD[6] receiver best position and velocity SD
//P [15][15] estimated state covariance, using LS clock bias solution directly
//tor time interval
//
//output
//state [15] measurement updated ECEF state vector
//P [15][15] measurement updated covariance matrix

	dvec est_r_e_old=ZERODVEC; //ECEF position vector
	dvec est_v_e_old=ZERODVEC; //ECEF velocity vector
	dvec delta_z=ZERODVEC; //measurement vector
	dvec est_x_propagated=ZERODVEC; //state error vector;
	dvec est_x_new=ZERODVEC; //state error vector;
	dvec tempvec1=ZERODVEC; //empty vector for calculation
	dvec tempvec2=ZERODVEC; //empty vector for calculation
	dvec tempvec3=ZERODVEC; //empty vector for calculation
	dvec tempvec4=ZERODVEC; //empty vector for calculation

	dmat est_C_b_e_old=ZERODMAT; //body to ECEF transformation matrix before update
	dmat est_C_b_e=ZERODMAT; //body to ECEF transformation matrix
	dmat H_matrix=ZERODMAT; //measurement matrix
	dmat R_matrix=ZERODMAT; //measurement noise matrix
	dmat K_matrix=ZERODMAT; //Kalman Gain matrix
	dmat P_matrix_old=ZERODMAT; //covariance matrix before update
	dmat P_matrix=ZERODMAT; //covariance matrix
	dmat S_matrix=ZERODMAT; //innovation covariance matrix
	dmat I=ZERODMAT; //identity matrix
	dmat tempmat1=ZERODMAT; //empty matrix for calculation
	dmat tempmat2=ZERODMAT; //empty matrix for calculation
	dmat tempmat3=ZERODMAT; //empty matrix for calculation
	dmat tempmat4=ZERODMAT; //empty matrix for calculation
	dmat tempmat5=ZERODMAT; //empty matrix for calculation
	dmat tempmat6=ZERODMAT; //empty matrix for calculation

	initdvec(&est_r_e_old, 1, 3);
	initdvec(&est_v_e_old, 1, 3);
	initdvec(&delta_z, 1, 6);
	initdvec(&est_x_propagated, 1, 15); //without clock bias and drift
	initdvec(&est_x_new, 1, 15);
	initdvec(&tempvec1, 1, 3);
	initdvec(&tempvec2, 1, 3);
	initdvec(&tempvec3, 1, 3);
	initdvec(&tempvec4, 1, 15);

	initdmat(&est_C_b_e_old, 1, 3, 1, 3);
	initdmat(&est_C_b_e, 1, 3, 1, 3);
	initdmat(&H_matrix, 1, 6, 1, 15);
	initdmat(&R_matrix, 1, 6, 1, 6);
	initdmat(&K_matrix, 1, 15, 1, 6);
	initdmat(&P_matrix_old, 1, 15, 1, 15);
	initdmat(&P_matrix, 1, 15, 1, 15);
	initdmat(&S_matrix, 1, 6, 1, 6);

	initdmat(&I, 1, 15, 1, 15);

	initdmat(&tempmat1, 1, 15, 1, 6);
	initdmat(&tempmat2, 1, 15, 1, 6);
	initdmat(&tempmat3, 1, 6, 1, 6);
	initdmat(&tempmat4, 1, 3, 1, 3);
	initdmat(&tempmat5, 1, 15, 1, 15);
	initdmat(&tempmat6, 1, 3, 1, 3);


	//save input as vector or matrix
	for(int i=1; i<4; i++){
		setdvec(&est_r_e_old, i, state[5+i]);
		setdvec(&est_v_e_old, i, state[2+i]);
		for(int j=1; j<4; j++)
			setdmat(&est_C_b_e_old, i, j, C_b_e[i-1][j-1]);
	}

	for(int i=1; i<16; i++){
		setdvec(&est_x_propagated, i, 0);
		for(int j=1; j<16; j++){
			setdmat(&P_matrix_old, i, j, P[i-1][j-1]);
			if(i==j)
				setdmat(&I, i, j, 1);
			else
				setdmat(&I, i, j, 0);
		}
	}

	//Set measurement matrix
	for(int i=1; i<7; i++){
		for(int j=1; j<16; j++)
			setdmat(&H_matrix, i, j, 0);
	}

	for(int i=1; i<4; i++){
		for(int j=1; j<4; j++){
			if(i==j){
				setdmat(&H_matrix, i, 6+j, -1);
				setdmat(&H_matrix, 3+i, 3+j, -1);
			}
		}
	}

	//Set measurement noise matrix
	for(int i=1; i<7; i++){
			for(int j=1; j<7; j++){
				if(i==j)
					setdmat(&R_matrix, i, j, best_state_SD[i-1]*best_state_SD[i-1]);
				else
					setdmat(&R_matrix, i, j, 0);
			}
		}

	//Calculate Kalman Gain
	transdmat(&H_matrix, &tempmat1);
	dmatxdmat(&P_matrix_old, &tempmat1, &tempmat2);
	dmatxdmat(&H_matrix, &tempmat2, &tempmat3);
	dmatplsdmat(&tempmat3, &R_matrix, &S_matrix);

	//Singularity Check about covariance of innovation
	//Calculate determinant of innovation covariance, D
	int check_singularity=1;
	double S[6][6];

	for(int i=0; i<6; i++){
		for(int j=0; j<6; j++)
			S[i][j]=S_matrix.mat[i+1][j+1];
	}

	double ratio, Det;

	for(int i=0; i<6; i++){
		for(int j=0; j<6; j++){
			if(j>i){
				ratio=S[j][i]/S[i][i];
				for(int k=0; k<6; k++)
					S[j][k]-=ratio*S[i][k];
			}
		}
	}

	Det=1;
	for(int i=0; i<6; i++)
		Det*=S[i][i];

	if(fabs(Det)==0)
		check_singularity=0;
	dmatinv(&S_matrix, &tempmat3);
	dmatxdmat(&tempmat2, &tempmat3, &K_matrix);

	//Calculate measurement innovation
	for(int i=1; i<4; i++){
		setdvec(&delta_z, i, best_state[i-1]-est_r_e_old.vec[i]);
		setdvec(&delta_z, i+3, best_state[i+2]-est_v_e_old.vec[i]);
	}

	//Covariance measurement update
	dmatxdmat(&K_matrix, &H_matrix, &tempmat5);
	dmatxscal(&tempmat5, -1, &tempmat5);
	dmatplsdmat(&I, &tempmat5, &tempmat5);

	dmatxdmat(&tempmat5, &P_matrix_old, &P_matrix);

	//Check measurement health
	int check_meas=1;

	for(int i=1; i<7; i++){
		if(fabs(delta_z.vec[i])-3*sqrt(S_matrix.mat[i][i])>0)
			check_meas=0;
	}

	//Check update process health
	int check_update=1;

	for(int i=1; i<16; i++){
		if(P_matrix.mat[i][i]-P_matrix_old.mat[i][i]>=0){
			check_update=0;
			printf(" \n p increase : %d %f %f ",i,P_matrix.mat[i][i],P_matrix_old.mat[i][i]);
		}

	}

	//check_meas=1;

	printf("\n measurement info : %d %d %d  ",check_meas,check_update,check_singularity);
	if(check_meas&&check_update&&check_singularity){
		//State measurement update when measurement update is healthy
		dmatxdvec(&K_matrix, &delta_z, &tempvec4);
		dvecplsdvec(&est_x_propagated, &tempvec4, &est_x_new);

		setdmat(&tempmat4, 1, 1, 0);
		setdmat(&tempmat4, 1, 2, -est_x_new.vec[3]);
		setdmat(&tempmat4, 1, 3, est_x_new.vec[2]);
		setdmat(&tempmat4, 2, 1, est_x_new.vec[3]);
		setdmat(&tempmat4, 2, 2, 0);
		setdmat(&tempmat4, 2, 3, -est_x_new.vec[1]);
		setdmat(&tempmat4, 3, 1, -est_x_new.vec[2]);
		setdmat(&tempmat4, 3, 2, est_x_new.vec[1]);
		setdmat(&tempmat4, 3, 3, 0);

		dmatxdmat(&tempmat4, &est_C_b_e_old, &tempmat6);
		dmatxscal(&tempmat6, -1, &tempmat6);
		dmatplsdmat(&est_C_b_e_old, &tempmat6, &est_C_b_e);

		for(int i=1; i<4; i++){
			state[i-1] += est_x_new.vec[i];
			state[5+i]=est_r_e_old.vec[i]-est_x_new.vec[6+i];
			state[2+i]=est_v_e_old.vec[i]-est_x_new.vec[3+i];
			for(int j=1; j<4; j++)
				C_b_e[i-1][j-1]=est_C_b_e.mat[i][j];
		}

		for(int i=1; i<7; i++)
			state[8+i]=state[8+i]+est_x_new.vec[9+i];

		transdmat(&P_matrix, &tempmat5);
		dmatplsdmat(&tempmat5, &P_matrix, &P_matrix);
		dmatxscal(&P_matrix, 0.5, &P_matrix);

		for(int i=1; i<16; i++){
			for(int j=1; j<16; j++)
				P[i-1][j-1]=P_matrix.mat[i][j];
		}
	}

	double C_b_e_copy[3][3];

	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			C_b_e_copy[i][j] = C_b_e[i][j];

	//double ratio, Det;

	for(int i=0; i<3; i++){
		for(int j=0; j<3; j++){
			if(j>i){
				ratio=C_b_e_copy[j][i]/C_b_e_copy[i][i];
				for(int k=0; k<3; k++)
					C_b_e_copy[j][k]-=ratio*C_b_e_copy[i][k];
			}
		}
	}

	Det=1;
	for(int i=0; i<3; i++)
		Det*=C_b_e_copy[i][i];

	printf("\n Det : %f",Det);

	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			C_b_e[i][j] *= pow(Det,-0.3333333333333333333333333);


	//Unhealthy case covariance and state will not be updated

	//printdvec(&est_x_new);
	//printdmat(&K_matrix);
	//printdvec(&delta_z);

	//Attitude_update(state,C_b_e);

	freedvec(&est_r_e_old);
	freedvec(&est_v_e_old);
	freedvec(&delta_z);
	freedvec(&est_x_propagated);
	freedvec(&est_x_new);
	freedvec(&tempvec1);
	freedvec(&tempvec2);
	freedvec(&tempvec3);
	freedvec(&tempvec4);

	freedmat(&est_C_b_e_old);
	freedmat(&est_C_b_e);
	freedmat(&H_matrix);
	freedmat(&R_matrix);
	freedmat(&K_matrix);
	freedmat(&P_matrix_old);
	freedmat(&P_matrix);
	freedmat(&S_matrix);
	freedmat(&I);
	freedmat(&tempmat1);
	freedmat(&tempmat2);
	freedmat(&tempmat3);
	freedmat(&tempmat4);
	freedmat(&tempmat5);
	freedmat(&tempmat6);

	//C_b_e_update(state, C_b_e);
}



