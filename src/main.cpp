#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <ctime>
#include "matops.h"
#include "gnss_main.h"
#include "HgDataParser.h"
#include "Serial.h"
#include "navconst.h"
#include "signa.h"
#include "config.h"
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <termio.h>
#include <iostream>
#include <time.h>
#include "filter_function.h"
//#include "boost/date_time/posix_time/posix_time.hpp"
//#include <boost/thread.hpp>

#define BUFFER_SIZE 2048
//Select the IMU used
//#define HG1120
#define HG4930

extern NavParameter NavParam;
extern NavSystem NavSys;

using namespace std;

int step = 1;
int clear = 1;

int no_const = 1; // 1 : GPS, 2 : GPS + GLONASS, 3 : GPS + GLONASS + GALILEO
int no_state = 15;

double dualheading;

int main(int argc, char **argv) {
	remove("/home/namgihun/gnss_INS_fusion_loosely/acc_info.txt");
	remove("/home/namgihun/gnss_INS_fusion_loosely/Ground.gps");
	remove("/home/namgihun/gnss_INS_fusion_loosely/Output_imu.txt");
	remove("/home/namgihun/gnss_INS_fusion_loosely/State.txt");
	remove("/home/namgihun/gnss_INS_fusion_loosely/True_Position.txt");
	remove("/home/namgihun/gnss_INS_fusion_loosely/Covariance.txt");
	double state[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //attitude[3],velocity[3],position[3],IMU bias[6],clock bias and clock drift

	double C_b_e[3][3]={
			{-0.7766,0.0141,-0.6299},
			{0.0175,0.9998,0.0009},
			{0.6298,-0.0103,-0.7767}
	};

	int i,j;

	double **P= new double*[no_state]; //define a row of covariance double pointer

	for(int i=1; i<no_state+1; i++){
		P[i-1]=new double[no_state]; //define column for each row - P[no_state][no_state]
		memset(P[i-1], 0, sizeof(int)*no_state); //initialize the memory space as zero
	}

	for(int i=1; i<4; i++){
		P[i-1][i-1]=3.0462e-04;
		P[i+2][i+2]=0.01;
		P[i+5][i+5]=100;
		P[i+8][i+8]=9.6170e-05;
		P[i+11][i+11]=2.3504e-09;
	}

	char comPort[32] = "/dev/ttyUSB0";
	int baudrate = 1000000;

	int status=0;
	//Connect to defined serial port
	int SerialHandle;
	status = serial_init(&SerialHandle, comPort);
	if (status!=0)
{
		printf("failed to open Com port:%s\n",comPort);
		return status;
}
	status = serial_configure(SerialHandle, baudrate, SER_PARITY_NO, 1, 8);
	if (status!=0)
{
		printf("failed to configure Com port:%s\tstatus:%d\n",comPort,status);
		return status;
}

	printf("Input COM:\t%s\t%d\n",comPort,baudrate);

	//Allocate the read buffer
	UINT8 ReadBuffer[BUFFER_SIZE*2] = {0};
	memset(ReadBuffer, 0, BUFFER_SIZE*2);
	UINT8 *ReadBufferAct = ReadBuffer;
	UINT8 *ReadBufferEnd = ReadBuffer;

	int BytesRead = 0;
	//UINT x = 0;
	status = 2;

#ifdef HG4930
	#define INERTIAL_MESSAGE_LEN 44
#else
	#define INERTIAL_MESSAGE_LEN 50
#endif

#ifdef HG4930
	HgDataParser::HG4930InertialMessage Message;
#else
	//Data structure for HG1120 data defined in .dll
	HgDataParser::HG1120InertialMessage Message;
#endif

	bool IMU_health;
	bool Gyro_health;
	bool Gyro_health2;

	Message.ZeroMessage();
	///////////////////////////////////////////////////////// IMU Reading Part

///*
	Reading_GNSS(); // Start Reading GNSS Data

	double tor_gnss=0.5; //time interval
	int no_meas;

	while (state[6]==0){
		state[6]=NavSys.RefPos_ECEF[0]; // 처음 gps 데이터 수신할떄까지 멈춤.
		state[7]=NavSys.RefPos_ECEF[1];
		state[8]=NavSys.RefPos_ECEF[2];
	}

	//state[6]=0;
//*/
	double LS_position[3];
	for (int i=0;i<3;i++)
		LS_position[i]=NavSys.RefPos_ECEF[i];

	double tor = 0.01;
	double IMU[6];
	double IMU_data[6]={0,0,0,0,0,0};
	double xyz[3];
	double lla[3];

	int NumData_prev=0;
	int num_gnss=0;
	int num_ins=0;
	double end_prev;

	clock_t start, end;

	while (1) {

		fstream imudataFile;
		fstream covFile;
		fstream stateFile;

		string buffer;
		imudataFile.open("Output_imu.txt",ios::app);
		covFile.open("Covariance.txt",ios::app);
		stateFile.open("State.txt",ios::app);
///*
		memcpy(ReadBuffer, ReadBufferAct, (ReadBufferEnd - ReadBufferAct));
		ReadBufferEnd = ReadBuffer + (ReadBufferEnd - ReadBufferAct);
		ReadBufferAct = ReadBuffer;

		if (!serial_read(SerialHandle, (char*)ReadBufferEnd, BUFFER_SIZE, &BytesRead))
		{
			printf("\nFailed to read Data!\n");
		}
		else
		{
			// Move the pointer to the end of the Read buffer
			ReadBufferEnd += (int)BytesRead;

			while (ReadBufferAct <= ReadBufferEnd - INERTIAL_MESSAGE_LEN)
			{
				if (*(UINT8*)ReadBufferAct == 0x0E)
				{
					// Switch Message Code - Call appropriate .dll functions
					switch (*(UINT8*)(ReadBufferAct + 1))
					{
					#ifdef HG4930
					case 0x01:
					{status = HgDataParser::GetHG4930X01ControlMessage(ReadBufferAct, 0, &Message.ControlMessage); ReadBufferAct += 19;

					break; }
					case 0x02:
					{status = HgDataParser::GetHG4930X02InertialMessage(ReadBufferAct, 0, &Message); ReadBufferAct += 43;

					break; }
					#else
					case 0x04:
					{status = HgDataParser::GetHG1120X04ControlMessage(ReadBufferAct, 0, &Message.ControlMessage); ReadBufferAct += 25; break; }
					case 0x05:
					{status = HgDataParser::GetHG1120X05InertialMessage(ReadBufferAct, 0, &Message); ReadBufferAct += 49; break; }
					case 0x0C:
					{status = HgDataParser::GetHG1120X0CControlMessage(ReadBufferAct, 0, &Message.ControlMessage); ReadBufferAct += 25; break; }
					case 0x0D:
					{status = HgDataParser::GetHG1120X0DInertialMessage(ReadBufferAct, 0, &Message); ReadBufferAct += 49; break; }
					#endif
					default:
					{ status = 2; }
					}

					IMU[0]=-Message.ControlMessage.AngularRate[1]; ////////////////// 1 2 0 - + -
					IMU[1]=Message.ControlMessage.AngularRate[2];
					IMU[2]=-Message.ControlMessage.AngularRate[0];
					IMU[3]=-Message.ControlMessage.LinearAcceleration[1]; ///// cahnge need?????
					IMU[4]=Message.ControlMessage.LinearAcceleration[2];
					IMU[5]=-Message.ControlMessage.LinearAcceleration[0];
					IMU_health=Message.ControlMessage.StatusWord.AccelerometerHealth;
					Gyro_health=Message.ControlMessage.StatusWord.GyroHealth;
					Gyro_health2=Message.ControlMessage.StatusWord.GyroHealth2;

					if (!IMU_health && !Gyro_health && !Gyro_health2){
						num_ins++;

						IMU_data[0] = IMU[0] - state[13];
						IMU_data[1] = IMU[1] - state[12];
						IMU_data[2] = IMU[2] - state[14];
						IMU_data[3] = IMU[3] - state[9];
						IMU_data[4] = IMU[4] - state[10];
						IMU_data[5] = IMU[5] - state[11];


						tor=0.00000001;

						if (num_ins > 1 && num_gnss > 0){
							//start=clock();
							//tor=(double)(start-end_prev)/ CLOCKS_PER_SEC;
							//end=clock();
							//end_prev=end;
							//if (tor > 0.0099)
								//num_ins--;
						//num_ins--;
						/////////////////////// tor fix
							tor=0.00166666666666666666666666666666;
						///////////////////////
						//if (tor >0.0099){
							//tor =0.016666666;
						//	num_ins++;
							System_Update(IMU_data, state, P, C_b_e, tor, dualheading);
						//}
						}

						for (i=0; i<6; i++)  {
							imudataFile << IMU[i];
							imudataFile << ",";
						}

						covFile << "IMU";
						for (i=0; i<15; i++){
							covFile << ",";
							covFile << setprecision(4) << P[i][i];
						}

						stateFile << "IMU,";
						for (i=0; i<15; i++){
							if (i>5 && i<9){
								stateFile << setprecision(12) << state[i];
								stateFile << ",";
							} else if (i<3){
								stateFile << setprecision(6) << state[i]/M_PI*180;
								stateFile << ",";
							} else {
								stateFile << setprecision(4) << state[i];
								stateFile << ",";
							}
						}
						for (i=0; i<3; i++){
							stateFile << setprecision(4) << C_b_e[i][i];
							stateFile << ",";
						}
						stateFile << tor;
						stateFile << ",";

						for (i=0;i<3;i++)
							xyz[i]=state[i+6];
						ConvertECEF2LLA(xyz,lla);

						for (i=0; i<3; i++){
							stateFile << setprecision(12) << lla[i];
							stateFile << ",";
						}

						stateFile << setprecision(12) << NavSys.sTimeCurrent;

						imudataFile << "\n";
						covFile << "\n";
						stateFile << "\n";

						//printf("\n Rawdata : %f %f %f %f %f %f %d %f",IMU[0],IMU[1],IMU[2],IMU[3],IMU[4],IMU[5],num_ins,tor);
						//printf("\n IMU_x : %f",IMU[3]);
						//printf("\n IMUdata : %f %f %f %f %f %f %d %f",IMU_data[0],IMU_data[1],IMU_data[2],IMU_data[3],IMU_data[4],IMU_data[5],num_ins,tor);
						//printf("\n system : Velocity, Position : %f  %f  %f  %f  %f  %f",state[3],state[4],state[5],state[6],state[7],state[8]);
						//printf("\n %f  %f  %f  %f  %f  %f %d %f\n",state[9],state[10],state[11],state[12],state[13],state[14],num_ins,tor);
					}
				}

				//Move the read buffer pointer
				ReadBufferAct++;
			} // !ReadFile

			//Sleep(1);
		}

		imudataFile.close();
		covFile.close();
		stateFile.close();

//*/
///*
	if(NavSys.NumData>num_gnss){
		num_gnss++;

	fstream stateFile;
	fstream covFile;
	fstream truePFile;

	stateFile.open("State.txt",ios::app);
	covFile.open("Covariance.txt",ios::app);
	truePFile.open("True_Position.txt",ios::app);


	double est_pos[4];
	double est_lla[3];
	//while (dualheading ==0 && num_gnss<2)
		//printf("\n dualheading is zero");

		if (num_gnss == 1){
			initialization(IMU_data, state, C_b_e,dualheading);
		}

	double best_state[6];
	double best_state_SD[6];

	for (i=0;i<3;i++){
		best_state[i] = NavSys.RefPos_ECEF[i];
		best_state[i+3] = NavSys.RefVel_ECEF[i];
		best_state_SD[i] = NavSys.RefPos_ECEF_SD[i];
		best_state_SD[i+3] = NavSys.RefVel_ECEF_SD[i];
		//best_state_SD[i+3] = 0.1;
	}


	if (num_gnss >1){
		Measurement_Update(state, best_state, best_state_SD, P, C_b_e, tor_gnss);
		//Measurement_Update(gnss_measurement, state, P, C_b_e,SvNum,step*tor_gnss,no_meas);
	}

	covFile << "GNSS";
	for (i=0; i<15; i++){
		covFile << ",";
		covFile << setprecision(4) << P[i][i];
	}
	stateFile << "GNSS,";
	for (i=0; i<15; i++){
		if (i>5 && i<9){
			stateFile << setprecision(12) << state[i];
			stateFile << ",";
		} else if (i<3){
			stateFile << setprecision(6) << state[i]/M_PI*180;
			stateFile << ",";
		} else {
			stateFile << setprecision(4) << state[i];
			stateFile << ",";
		}
	}
	for (i=0; i<3; i++){
		stateFile << setprecision(4) << C_b_e[i][i];
		stateFile << ",";
	}
	stateFile << tor_gnss;
	stateFile << ",";

	for (i=0;i<3;i++)
		xyz[i]=state[i+6];
	ConvertECEF2LLA(xyz,lla);

	for (i=0; i<3; i++){
		stateFile << setprecision(12) << lla[i];
		stateFile << ",";
	}

	stateFile << setprecision(12) << NavSys.sTimeCurrent;

	for (i=0;i<3;i++){
		truePFile << setprecision(12) << NavSys.RefPos_ECEF[i];
		truePFile << ",";
	}
	for (i=0;i<3;i++){
		truePFile << setprecision(12) << NavSys.RefPos_LLA[i];
		truePFile << ",";
	}

	for (i=0;i<3;i++){
		truePFile << setprecision(12) << NavSys.RefVel_ECEF[i];
		truePFile << ",";
	}

	for (i=0;i<6;i++){
		truePFile << setprecision(12) << best_state_SD[i];
		truePFile << ",";
	}

	covFile << "\n";
	stateFile << "\n";
	truePFile << "\n";

	stateFile.close();
	covFile.close();
	truePFile.close();
	printf("\n IMU RAW DATA : %f, %f, %f, %f, %f, %f",IMU[0],IMU[1],IMU[2],IMU[3],IMU[4],IMU[5]);
	printf("\n IMU DATA : %f, %f, %f, %f, %f, %f",IMU_data[0],IMU_data[1],IMU_data[2],IMU_data[3],IMU_data[4],IMU_data[5]);
	printf("\n IMU_BIAS : %f, %f, %f, %f, %f, %f",state[12],state[13],state[14],state[9],state[10],state[11]);
	printf("\n num_gnss : %d num_ins : %d ",num_gnss,num_ins);
	printf("\n Velocity, Position : %f  %f  %f  %f  %f  %f %f %f %f",state[3],state[4],state[5],state[6],state[7],state[8],lla[0],lla[1],lla[2]);
	printf("\n Real position : %f  %f  %f %f %f %f \n",NavSys.RefPos_ECEF[0],NavSys.RefPos_ECEF[1],NavSys.RefPos_ECEF[2],NavSys.RefPos_LLA[0]/M_PI*180,NavSys.RefPos_LLA[1]/M_PI*180,NavSys.RefPos_LLA[2]);
	printf("\n Real Velocity : %f %f %f",NavSys.RefVel_ECEF[0],NavSys.RefVel_ECEF[1],NavSys.RefVel_ECEF[2]);
	printf("\n Real SD : %f, %f, %f, %f, %f, %f ",NavSys.RefPos_ECEF_SD[0],NavSys.RefPos_ECEF_SD[1],NavSys.RefPos_ECEF_SD[2],NavSys.RefVel_ECEF_SD[0],NavSys.RefVel_ECEF_SD[1],NavSys.RefVel_ECEF_SD[2]);
	printf("\n attitude : %f %f %f dual_heading : %f \n",state[0]/M_PI*180,state[1]/M_PI*180,state[2]/M_PI*180,dualheading);
	double Error_ECEFx = state[6]-NavSys.RefPos_ECEF[0];
	double Error_ECEFy = state[7]-NavSys.RefPos_ECEF[1];
	double Error_ECEFz = state[8]-NavSys.RefPos_ECEF[2];
	double Vertical_Error = NavSys.RefPos_LLA[2] - lla[2];
	double Horizontal_Error = sqrt(Error_ECEFx * Error_ECEFx + Error_ECEFy * Error_ECEFy + Error_ECEFz * Error_ECEFz - Vertical_Error * Vertical_Error);
	//printf("\n %f %f %f \n",Error_ECEFx,Error_ECEFy,Error_ECEFz);
	printf("\n Vertical Error : %f, Horizontal Error : %f \n",Vertical_Error,Horizontal_Error);


	} // gnss if문
//*/
} // while문

}


