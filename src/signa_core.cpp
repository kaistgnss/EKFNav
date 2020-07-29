/*
 * signa_core.c
 *
 *  Created on: 2019. 6. 3.
 *      Author: gnss
 */

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <fstream>

#include "oem4.h"
#include "config.h"
#include "navconst.h"
#include "signa.h"
#include "matops.h"



NavParameter NavParam;
NavSystem NavSys;
NavEphData NavEph;

extern int timeutc[6];
extern double bestPos[3];
extern char LogDirName[50];
extern char start_time[50];
extern int Save_Mode;
extern int ready_surveyedpos;

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

float psrdop;
uint8_t numsat[2];
//extern double bestVel[2];
extern int timeutc[6];

void SIGNA_core(OEM4_RANGEB_MSG *msg, NavEphData *NavEph){

	// Status
	//static NavParameter NavParam;	//
	static int init;
	//static NavSystem NavSys;
	int i;
	if (NavSys.RefPos_LLA[0] == 0){
		LoadRefXYZ(&NavSys);
		NavSys.sEpochSinceStart = 0;
	}
	LoadRefXYZ(&NavSys);
	/*
	if (init == 0){
		while (NavSys.RefPos_LLA[0] = 0) {
			LoadRefXYZ(&NavSys);
		}
			init = 1;
			printf("initiation!!");
			NavSys.sEpochSinceStart = 0;
		//}
		//return;
	}
	 //*/
	// 시스템 변수 초기화
	InitializeNavParam(&NavSys, &NavParam);

	// 측정치 업데이트
	UpdateNavMeasurement(&NavSys, &NavParam, msg);

	// isEphCurr chk. DQM 추가 시 수정 예정
	for (i=0; i<kNumSatellite; i++){
		switch (GetConstFlag(i)){
		case FlagGPS :
			if (NavEph->GPS[i].prn == i+1)
				NavSys.sIsEphCurrOn[i] = 1;
			break;
		case FlagGLO :
			if (NavEph->GLO[i-kNumGPS].prn == i+1)
				NavSys.sIsEphCurrOn[i] = 1;
			break;
		}
	}
	// Continuous SV check
	for (i=0; i<kNumSatellite; i++){
		if (fabsf(NavParam.mLockTime[i] - NavParam.mLockTimePrev[i]) == 0.5){ // not continue
			NavSys.sContinuousEpoch[i]++;
//			printf("[PRN] %2i [ContEpoch] %3i \n", i+1, NavParam.sContinuousEpoch[i]);
		} else {
			NavSys.sContinuousEpoch[i] = 1;;
			NavParam.mCSC1_count[i] = 1;
			NavParam.mCSC2_count[i] = 1;
		}
	}

	// Estimation
	EstimateSvParam(&NavSys, &NavParam, NavEph);

	int corrflag = 0;
	for (i=0; i<kNumSatellite; i++){
			if (NavSys.sIsMeasurementOn[i] == 1 && NavSys.sIsEphCurrOn[i] == 1
					&& NavParam.SvPRC1[i] != 0 && NavParam.mCSC1_count[i] == 200){
				corrflag = 1;
			}
	}
	if (Save_Mode) {
		if (corrflag == 1) {
			FILE *fp_corrlog;
			char CorrLogName[50];
			sprintf(CorrLogName, "%s/Corr_%s.txt", LogDirName, start_time);
			fp_corrlog = fopen(CorrLogName, "a");

			fprintf(fp_corrlog, "%.1f", timeutc[3]*1e4f + timeutc[4]*1e2f + timeutc[5]/1000.0);

			for (i=0; i<kNumSatellite; i++){
				if (NavSys.sIsMeasurementOn[i] == 1 && NavSys.sIsEphCurrOn[i] == 1
						&& NavParam.SvPRC1[i] != 0 && NavParam.mCSC1_count[i] == 200){
					fprintf(fp_corrlog, ",%d,%.2f", i+1, NavParam.SvPRC1[i]);
				}
			}
			fprintf(fp_corrlog, "\n");
			fclose (fp_corrlog);
		}
	}
	// DetermineSvStatus
	//DetermineSvStatus(&NavSys, &NavParam, NavEph);

	SaveCurrentSvParam(&NavSys, &NavParam);

	//EstimateIonoTropo(&NavSys, &NavParam);

	//if (NavSys.NumSvGPS > 6)
		EstimateUsrPos(&NavSys, &NavParam);

	//printf("\n dw :: %f %f %f \n", NavSys.UsrPos_ECEF[0], NavSys.UsrPos_ECEF[1], NavSys.UsrPos_ECEF[2]);


	/*
	for (i = 0; i < kNumSatellite; i++) {
		if (NavSys.sIsMeasurementOn[i] == 1 && NavSys.sIsEphCurrOn[i] == 1) {
			NavParam.mCode[i] += SPEED_OF_LIGHT*NavParam.SvClockOffset[i]-NavParam.tropo_correction[i]-NavParam.iono_correction[i];
		} // if
	} // for
	//*/
} // function: SIGNA_proc

void EstimateIonoTropo(NavSystem *NavSys, NavParameter *NavParam){
	int i;
	double EL, AZ;
	double EL_semi; // elevation in semicircles
	double psi; // earth centered angle in semicircles
	double lat, lon;
	double phi_U, lambda_U; //user latitude, longitude in semicircles
	double phi_I, phi_I_rad; // subionospheric latitude in semicircles
	double lambda_I, lambda_I_rad;
	double phi_m; // subionospheric latitude in semicircles
	double t; //Local time in secs
	double F; // slant Factor
	lat=NavSys->RefPos_LLA[0];
	lon=NavSys->RefPos_LLA[1];
	phi_U=lat/M_PI;
	lambda_U=lon/M_PI;
	double a0,a1,a2,a3,b0,b1,b2,b3;
	double AMP,PER,x; // sum
	double delt;
	double Pr = 1013.25; // Pressure [mbar]
	double Tr = 291.15; // temperature [K]
	double Hr = 50; // numerical constants for the algorithm [mbar]
	double P, T, H;
	double h_a[9];
	double B_a[9];

	double tp,B,e;
	double h=NavSys->RefPos_LLA[2];
	int idx = 0;

	for (i=0;i<9;i++)
		h_a[i] = i * 500;

	B_a[0] = 1.156;
	B_a[1] = 1.079;
	B_a[2] = 1.006;
	B_a[3] = 0.938;
	B_a[4] = 0.874;
	B_a[5] = 0.813;
	B_a[6] = 0.757;
	B_a[7] = 0.654;
	B_a[8] = 0.563;


	a0 = NavParam->a0;
	a1 = NavParam->a1;
	a2 = NavParam->a2;
	a3 = NavParam->a3;
	b0 = NavParam->b0;
	b1 = NavParam->b1;
	b2 = NavParam->b2;
	b3 = NavParam->b3;

	P = Pr * pow(1-0.0000226*h,5.225);
	T = Tr - 0.0065 * h;
	H = Hr * exp(-0.0006396*h);

	for (i=0; i<9; i++)
		if (h_a[i] < h)
			idx+=1;

	idx-=1;

	//printf("\n 1111111111111111111 %d %f 11111111111111111111111111 \n", idx, h);

	tp = (h - h_a[idx]) / (h_a[idx+1] - h_a[idx]);
	B  = (1 - t) * B_a[idx] + t * B_a[idx+1];
	e  = 0.01 * H * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

	for (i=0; i<kNumSatellite; i++){
		if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1){
		EL=NavParam->SvELrad[i];
		AZ=NavParam->SvAZrad[i];
		EL_semi=EL/M_PI;
		//EL_semi=EL*M_PI;
		psi=0.0137/(EL_semi + 0.11) - 0.022;
		phi_I = phi_U+psi*cos(AZ);

		if (phi_I>0.416)
			phi_I=0.416;
		else if (phi_I<-0.416)
			phi_I=-0.416;

		phi_I_rad = phi_I * M_PI;
		lambda_I = lambda_U+(psi * sin(AZ) / cos(phi_I_rad));
		lambda_I_rad = lambda_I * M_PI;
		phi_m = phi_I + 0.064 * cos((lambda_I-1.617)*M_PI);
		t = (4.32 * 10000) * lambda_I + fmod(NavSys->sTimeCurrent, 86400);
		//t = (4.32 * 10000) * lambda_I + fmod(NavParam->TimeTx[i], 86400);
		//t = (4.32 * 10000) * lambda_I + fmod(80000, 86400);
		if (t>86400)
			t-=86400;
		else if (t<0)
			t+=86400;

		F = 1 + 16 * pow((0.53 - EL_semi),3);

		AMP = a0 + a1 * phi_m + a2 * phi_m * phi_m + a3 * phi_m * phi_m * phi_m;
		if (AMP<0)
			AMP=0;
		PER = b0 + b1 * phi_m + b2 * phi_m * phi_m + b3 * phi_m * phi_m * phi_m;
		if (PER<72000)
			PER = 72000;

		x = 2 * M_PI * (t-50400)/PER;

		if (fabs(x)>1.57)
			NavParam->iono_correction[i] = SPEED_OF_LIGHT*F*5e-09;
		else
			NavParam->iono_correction[i] = SPEED_OF_LIGHT*F*(5e-09 + AMP * (1-x*x/2+x*x*x*x/24));


		//NavParam->tropo_correction[i] = 2.312/sin(sqrt(EL_semi * EL_semi  + 0.001904)) + 0.084/sin(sqrt(EL_semi * EL_semi + 0.6854 * 0.001));
		NavParam->tropo_correction[i] = 2.312/sin(sqrt(EL * EL  + 0.001904)) + 0.084/sin(sqrt(EL * EL + 0.6854 * 0.001));
		//NavParam->tropo_correction[i] = (0.002277/sin(EL)) * (P-(B/(tan(EL) * tan(EL)))+(0.002277/sin(EL))*(1255/T+0.05)*e);
		//printf("\n 111111111111 %f %f 11111111111111 \n",tan(EL),sin(EL)/cos(EL));
		}
	}



}//function : EstimateIonoTropo

void InitializeNavParam(NavSystem *NavSys, NavParameter *NavParam){

	int i;

	for (i=0; i<kNumSatellite; i++){
		// NavParam
		NavSys->sIsMeasurementOn[i] = 0;
		NavSys->sIsEphCurrOn[i] = 0;

		// NavParam
		NavParam->mCode[i] = 0;
		NavParam->mPhase[i] = 0;
		NavParam->mDoppler[i] = 0;
		NavParam->mCN0[i] = 0;
		NavParam->mLockTime[i] = 0;

		NavParam->TimeTx_Raw[i] = 0;
		NavParam->TimeTx[i] = 0;
		NavParam->SvClockOffset[i] = 0;
		NavParam->SvClockDrift[i] = 0;
		NavParam->SvPosition[i][0] = 0;
		NavParam->SvPosition[i][1] = 0;
		NavParam->SvPosition[i][2] = 0;
		NavParam->SvAZrad[i] = 0;
		NavParam->SvELrad[i] = 0;
		NavParam->SvPRC1[i] = 0;
		NavParam->SvPRC2[i] = 0;
		NavParam->SvRRC1[i] = 0;
		NavParam->SvRRC2[i] = 0;

	}
} // function: InitializeNavParam

void UpdateNavMeasurement(NavSystem *NavSys, NavParameter *NavParam, OEM4_RANGEB_MSG *msg) {

	int i;
	uint16_t idx;
	uint32_t number_of_obs = msg->obs;
//	number_of_obs = MIN(number_of_obs, 40);
	NavSys->sTimeCurrent = (double) msg->header.millisecs / 1000.0;
	NavSys->sEpochSinceStart++;

//printf("# obs, %d\n", msg->obs);
	for (i=0; i<number_of_obs; i++){
	//	printf("%d [const]%d [freq]%d\n",i, msg->ch[i].status.satellite_system, msg->ch[i].status.frequency);
		// L1 데이터만 저장
		if (msg->ch[i].status.frequency != 0)
			continue;

		// Constellation 별 저장
		switch (msg->ch[i].status.satellite_system) {
		case 0: // GPS
			idx = msg->ch[i].prn - 1;

			// 현재 측정치 업로드
			NavParam->mCode[idx] 		= msg->ch[i].psr;
			NavParam->mCode_std[idx]	= msg->ch[i].psr_std;
			NavParam->mPhase[idx] 		= msg->ch[i].adr;
			NavParam->mDoppler[idx] 		= msg->ch[i].dopp;
			NavParam->mCN0[idx] 			= msg->ch[i].C_No;
			NavParam->mLockTime[idx] 	= msg->ch[i].locktime;
			NavParam->mFrequency[idx]	= Freq_L1_GPS;
			NavParam->mWaveLength[idx]  = SPEED_OF_LIGHT/(double)NavParam->mFrequency[idx];
			NavParam->mCode_rate[idx]	= -NavParam->mWaveLength[idx] * NavParam->mDoppler[idx];
			NavParam->mCode_rate_std[idx] = NavParam->mWaveLength[idx] * NavParam->mWaveLength[idx] * msg->ch[i].adr_std;
			/*
			NavParam->a0[idx] = ion.a0[i];
			NavParam->a1[idx] = ion.a1[i];
			NavParam->a2[idx] = ion.a2[i];
			NavParam->a3[idx] = ion.a3[i];
			NavParam->b0[idx] = ion.b0[i];
			NavParam->b1[idx] = ion.b1[i];
			NavParam->b2[idx] = ion.b2[i];
			NavParam->b3[idx] = ion.b3[i];
			*/
			NavSys->sIsMeasurementOn[idx] = 1;
			break;

		case 1: // GLONASS
			idx = msg->ch[i].prn - 6;

			NavParam->mCode[idx] 		= msg->ch[i].psr;
			NavParam->mCode_std[idx]	= msg->ch[i].psr_std;
			NavParam->mPhase[idx] 		= msg->ch[i].adr;
			NavParam->mDoppler[idx] 		= msg->ch[i].dopp;
			NavParam->mCN0[idx] 			= msg->ch[i].C_No;
			NavParam->mLockTime[idx] 	= msg->ch[i].locktime;
			NavParam->mFrequency[idx]	= Freq_L1_GLO + (msg->ch[i].filler - 7) * FreqStepGLO;
			NavParam->mWaveLength[idx]  = SPEED_OF_LIGHT/(double)NavParam->mFrequency[idx];
			NavParam->mCode_rate[idx]	= -NavParam->mWaveLength[idx] * NavParam->mDoppler[idx];
			NavParam->mCode_rate_std[idx] = NavParam->mWaveLength[idx] * NavParam->mWaveLength[idx] * msg->ch[i].adr_std;

			NavSys->sIsMeasurementOn[idx] = 1;
			break;
		case 3: // GALILEO
			break;
		default :
			break;
		}
	}
} // function: UpdateNavMeasurement

void LoadRefXYZ(NavSystem *NavSys){
//	FILE *fp;
//	fp = fopen("RefLLA.dbs","r");
//
//	if (fp == 0){
//		printf("ERR : Failed to load surveyed location file.\n");
//		return;
//	}

	// LLA 형식 Reference Station XYZ 읽기
//	double latd, latm, lats, lond, lonm, lons;
//	fscanf(fp, "%lf	%lf	%lf", &latd, &latm, &lats);
//	fscanf(fp, "%lf %lf %lf", &lond, &lonm, &lons);
//	fscanf(fp, "%lf", &NavSys->RefPos_LLA[2]);
//	NavSys->RefPos_LLA[0] = (latd + latm/60.0 + lats/3600.0) * d2r; // Latitude
//	NavSys->RefPos_LLA[1] = (lond + lonm/60.0 + lons/3600.0) * d2r; // Longitude

	NavSys->RefPos_LLA[0] = bestPos[0]; // Latitude
	NavSys->RefPos_LLA[1] = bestPos[1]; // Longitude
	NavSys->RefPos_LLA[2] = bestPos[2]; // Longitude

//	printf("-- Position[ECEF] %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f \n",
	//		NavSys->RefPos_LLA[0], NavSys->RefPos_LLA[1], NavSys->RefPos_LLA[2], bestPos[0],bestPos[1],bestPos[2]);

	// LLA to ECEF로 변경
	/*
	double rn;
	double sinlat, coslat, sinlon, coslon, alt;
	if (ready_surveyedpos > 0){
	sinlat = sin(NavSys->RefPos_LLA[0]);
	coslat = cos(NavSys->RefPos_LLA[0]);
	sinlon = sin(NavSys->RefPos_LLA[1]);
	coslon = cos(NavSys->RefPos_LLA[1]);
	alt = NavSys->RefPos_LLA[2];

	rn = A_WGS84 / sqrt(1 - E2_Earth * sinlat * sinlat);
	NavSys->RefPos_ECEF[0] = (rn + alt) * coslat * coslon;
	NavSys->RefPos_ECEF[1] = (rn + alt) * coslat * sinlon;
	NavSys->RefPos_ECEF[2] = (rn * (1 - E2_Earth) + alt) * sinlat;
	}
	//*/
} // function: LoadRefXYZ

void EstimateSvParam(NavSystem *NavSys, NavParameter *NavParam, NavEphData *NavEph){

	int i;
	unsigned char NumSv = 0;

	for (i=0; i<kNumSatellite; i++){
		if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1){
			// Clock offset 적용 전 Transmission Time 추정
			//NavParam->TimeTx_Raw[i] = NavSys->sTimeCurrent - NavParam->mCode[i]/SPEED_OF_LIGHT;
			NavParam->TimeTx_Raw[i] = NavSys->sTimeCurrent - NavParam->mCode[i]/SPEED_OF_LIGHT;

			// 위성 시계 오차, 궤도 추정
			switch (GetConstFlag(i)){
			case FlagGPS:
				EstimateSvClock(NavParam, NavEph, i, FlagGPS);

				NavParam->TimeTx[i] = NavParam->TimeTx_Raw[i] - NavParam->SvClockOffset[i];
				NavParam->TransitTime[i] = NavSys->sTimeCurrent - NavParam->TimeTx[i];

				EstimateSvOrbit(NavParam, NavEph, i, FlagGPS);
				break;

			case FlagGLO:
				EstimateSvClock(NavParam, NavEph, i, FlagGLO);

				NavParam->TimeTx[i] = NavParam->TimeTx_Raw[i] - NavParam->SvClockOffset[i];
				NavParam->TransitTime[i] = NavSys->sTimeCurrent - NavParam->TimeTx[i];

				EstimateSvOrbit(NavParam, NavEph, i, FlagGLO);
				break;
			}

			// 위성 El, Az 계산
			EstimateElAz(NavSys, NavParam, i);

			// Code-Carrier Smoothing
			EstimateCSC(NavParam, i);

			// Correction 계산
			EstimateCorr(NavSys, NavParam, i);
		}
	}
	NavSys->NumData++;
} // function: EstimateSvParam

void KeplerEquation(double *Ek, double Mk, double ecc) {
	int max_iteration = 10;
	double diff_Ek = 10.0;
	double newEk;
	int i = 0;

	*Ek = Mk;
	while (diff_Ek > 1.0e-15 && i < max_iteration){
		newEk = Mk + (ecc * sin(*Ek));
		diff_Ek = fabs(newEk - *Ek);

		*Ek = newEk;
		i++;
	}
} // function: KeplerEquation

void EstimateSvClock(NavParameter *NavParam, NavEphData *NavEph, unsigned char i, unsigned char FlagConstellation) {

	double tk;

	if (FlagConstellation == FlagGPS){
		double n0, nk, Mk;
		double Ek, rel_t;
		double A, M0, deltaN, ecc, af0, af1, af2, TOC, Tgd;

		A 		= NavEph->GPS[i].A;
		M0 		= NavEph->GPS[i].M0;
		deltaN = NavEph->GPS[i].deltaN;
		ecc 	= NavEph->GPS[i].ecc;
		af0 	= NavEph->GPS[i].af0;
		af1 	= NavEph->GPS[i].af1;
		af2 	= NavEph->GPS[i].af2;
		TOC		= NavEph->GPS[i].TOC;
		Tgd 	= NavEph->GPS[i].Tgd;

		NavParam->af0[i] = af0;
		NavParam->af1[i] = af1;
		NavParam->af2[i] = af2;
		NavParam->TOC[i] = TOC;

		tk = NavParam->TimeTx_Raw[i] - TOC;

		// 상대성 효과에 의한 delay
		n0 = sqrt(Mu_WGS84 / (A * A * A));
		nk = n0 + deltaN;
		Mk = M0 + nk * tk;
		KeplerEquation(&Ek, Mk, ecc);
		rel_t = F_relative * ecc * sqrt(A) * sin(Ek);

		NavParam->SvClockOffset[i] = af0 + (af1 * tk) + (af2 * tk * tk) + rel_t + Tgd;
		NavParam->SvClockDrift[i]  = af1 + af2 * tk;
	}
	else if (FlagConstellation == FlagGLO){
		double e_time, taun, gamma;
		e_time = NavEph->GLO[i-kNumGPS].e_time;
		taun 	= NavEph->GLO[i-kNumGPS].taun;
		gamma  = NavEph->GLO[i-kNumGPS].gamma;

		tk = NavParam->TimeTx_Raw[i] - e_time;
		NavParam->SvClockOffset[i] = -taun + (gamma * tk);
	}
} // function: EstimateSvClock

void EstimateSvOrbit(NavParameter *NavParam, NavEphData *NavEph, unsigned char i, unsigned char FlagConstellation){

	if (FlagConstellation == FlagGPS){
		double tk, n0, nk, Mk;
		double Ek, nuk, phik;
		double Ekdot, nukdot, ikdot, ukdot, rkdot, omegakdot, xodot, yodot;
		double A, ecc, w, M0, N, cic, cis, crc, crs, cuc, cus,
				deltaN, i0, idot, omega0, omegadot, TOE,
				uk, rk, ik, xo, yo, omegak;
		double xp_temp, yp_temp, zp_temp, tau, xv_temp, yv_temp, zv_temp;

		A 			= NavEph->GPS[i].A;
		ecc			= NavEph->GPS[i].ecc;
		M0 			= NavEph->GPS[i].M0;
		N 			= NavEph->GPS[i].N;
		cic 		= NavEph->GPS[i].cic;
		cis			= NavEph->GPS[i].cis;
		crc 		= NavEph->GPS[i].crc;
		crs 		= NavEph->GPS[i].crs;
		cuc 		= NavEph->GPS[i].cuc;
		cus 		= NavEph->GPS[i].cus;
		deltaN 	= NavEph->GPS[i].deltaN;
		i0 			= NavEph->GPS[i].i0;
		idot 		= NavEph->GPS[i].idot;
		omega0 	= NavEph->GPS[i].omega0;
		omegadot 	= NavEph->GPS[i].omegadot;
		w 			= NavEph->GPS[i].w;
		TOE			= NavEph->GPS[i].TOE;

		NavParam->A[i] = A;
		NavParam->ecc[i] = ecc;
		NavParam->M0[i] = M0;
		NavParam->N[i] = N;
		NavParam->cic[i] = cic;
		NavParam->cis[i] = cis;
		NavParam->crc[i] = crc;
		NavParam->crs[i] = crs;
		NavParam->cuc[i] = cuc;
		NavParam->cus[i] = cus;
		NavParam->deltaN[i] = deltaN;
		NavParam->i0[i] = i0;
		NavParam->idot[i] = idot;
		NavParam->omega0[i] = omega0;
		NavParam->omegadot[i] = omegadot;
		NavParam->w[i] = w;
		NavParam->TOE[i] = TOE;

		tk = NavParam->TimeTx[i] - TOE;

		n0 = sqrt(Mu_WGS84 / (A * A * A));
		nk = n0 + deltaN;
		Mk = M0 + nk * tk;
		KeplerEquation(&Ek, Mk, ecc);

		nuk = atan2(sqrt(1 - ecc * ecc) * sin(Ek) / (1 - ecc * cos(Ek)), (cos(Ek) - ecc) / (1 - ecc * cos(Ek)));
		phik = nuk + w;

		uk = cus * sin(2*phik) + cuc * cos(2*phik) + phik;
		rk = crs * sin(2*phik) + crc * cos(2*phik) + A * (1 - ecc * cos(Ek));
		ik = cis * sin(2*phik) + cic * cos(2*phik) + i0 + idot * tk;

		xo = rk * cos(uk);
		yo = rk * sin(uk);
		omegak = omega0 - (OmegaDot_WGS84 * TOE) + (omegadot - OmegaDot_WGS84) * tk;

		xp_temp = xo * cos(omegak) - yo * cos(ik) * sin(omegak);
		yp_temp = xo * sin(omegak) + yo * cos(ik) * cos(omegak);
		zp_temp = yo * sin(ik);

		Ekdot  =  nk/(1 - ecc * cos(Ek));
		nukdot =  Ekdot * sqrt(1 - ecc * ecc) / (1 - ecc * cos(Ek));
		ikdot  =  idot + 2 * nukdot * (cis * cos(2 * phik) - cic * sin(2 * phik));
		ukdot  =  nukdot + 2 * nukdot * (cus * cos(2 * phik) - cuc * sin(2 * phik));
		rkdot  =  ecc * A * Ekdot * sin(Ek) + 2 * nukdot * (crs * cos(2 * phik) - crc * sin(2 * phik));
		omegakdot = omegadot - OmegaDot_WGS84;
		xodot  =  rkdot * cos(uk) - rk * ukdot * sin(uk);
		yodot  =  rkdot * sin(uk) + rk * ukdot * cos(uk);
		// Earth Rotation
		tau = NavParam->TransitTime[i];
//		sTimeCurrent - NavParam->TimeTx[i];

		NavParam->SvPosition[i][0] =  xp_temp * cos(tau * OmegaDot_WGS84) + yp_temp * sin(tau * OmegaDot_WGS84);
		NavParam->SvPosition[i][1] = -xp_temp * sin(tau * OmegaDot_WGS84) + yp_temp * cos(tau * OmegaDot_WGS84);
		NavParam->SvPosition[i][2] =  zp_temp;
		//NavParam->SvPosition[i][0] = xp_temp;
		//NavParam->SvPosition[i][1] = yp_temp;
		//NavParam->SvPosition[i][2] = zp_temp;

		xv_temp =  -xo * omegakdot * sin(omegak) + xodot * cos(omegak) - yodot * sin(omegak) * cos(ik) - yo * (omegakdot * cos(omegak) * cos(ik) - (ikdot) * sin(omegak) * sin(ik));
		yv_temp =  xo * omegakdot * cos(omegak) + xodot * sin(omegak) + yodot * cos(omegak) * cos(ik) - yo * (omegakdot * sin(omegak) * cos(ik) + (ikdot) * cos(omegak) * sin(ik));
		zv_temp =  yo * ikdot * cos(ik) + yodot * sin(ik);
		NavParam->SvVelocity[i][0] =  xv_temp * cos(tau * OmegaDot_WGS84) + yv_temp * sin(tau * OmegaDot_WGS84);
		NavParam->SvVelocity[i][1] = -xv_temp * sin(tau * OmegaDot_WGS84) + yv_temp * cos(tau * OmegaDot_WGS84);
		NavParam->SvVelocity[i][2] =  zv_temp;
		//NavParam->SvVelocity[i][0] = xv_temp;
		//NavParam->SvVelocity[i][1] = yv_temp;
		//NavParam->SvVelocity[i][2] = zv_temp;
		//NavParam->SvVelocity[i][0] = xodot
	}
	else if (FlagConstellation == FlagGLO) {
		double tk, xp, yp, zp, xv, yv, zv, xa, ya, za; // Eph 변수 저장용
		double step; // step size
		double n, res; // iteration 횟수, 나머지
		int s;
		/* Runge Kutta 변수 */
		double xp1, xp2, xp3, xp4;
		double yp1, yp2, yp3, yp4;
		double zp1, zp2, zp3, zp4;
		double xv1, xv2, xv3, xv4;
		double yv1, yv2, yv3, yv4;
		double zv1, zv2, zv3, zv4;
		double xvd1, xvd2, xvd3, xvd4;
		double yvd1, yvd2, yvd3, yvd4;
		double zvd1, zvd2, zvd3, zvd4;
		double r, g, h, k, tau;
		double xp_temp, yp_temp, zp_temp, xv_temp, yv_temp, zv_temp;

		xp = NavEph->GLO[i-kNumGPS].posx;
		yp = NavEph->GLO[i-kNumGPS].posy;
		zp = NavEph->GLO[i-kNumGPS].posz;
		xv = NavEph->GLO[i-kNumGPS].velx;
		yv = NavEph->GLO[i-kNumGPS].vely;
		zv = NavEph->GLO[i-kNumGPS].velz;
		xa = NavEph->GLO[i-kNumGPS].accx;
		ya = NavEph->GLO[i-kNumGPS].accy;
		za = NavEph->GLO[i-kNumGPS].accz;

		/* Transmission time 계산 */
		tk = NavParam->TimeTx[i] - NavEph->GLO[i-kNumGPS].e_time;

		step = 60.0 * tk/fabs(tk); // tk의 부호 반영

		/* iteration 횟수 계산 */
		n = floor(fabs(tk / step));
		res = fmod(tk, step);
		if (res != 0.0)
			n++;

		// Runge Kutta
		xp_temp = xp; yp_temp = yp; zp_temp = zp;
		xv_temp = xv; yv_temp = yv; zv_temp = zv;

		for (s=0; s<n; s++){
			// 나머지가 0이 아닌 경우, 마지막의 step size를 나머지의 크기로 변경
			if (res != 0.0 && s == n-1)
				step = res;

			// step 1
			xp1 = xp_temp;
			yp1 = yp_temp;
			zp1 = zp_temp;
			xv1 = xv_temp;
			yv1 = yv_temp;
			zv1 = zv_temp;
			SvMotionEq_GLO(&xvd1, &yvd1, &zvd1, &xp1, &yp1, &zp1, &xv1, &yv1, &zv1, &xa, &ya, &za);

			// step 2
			xp2 = xp_temp + xv1 * step/2.0;
			yp2 = yp_temp + yv1 * step/2.0;
			zp2 = zp_temp + zv1 * step/2.0;
			xv2 = xv_temp + xvd1 * step/2.0;
			yv2 = yv_temp + yvd1 * step/2.0;
			zv2 = zv_temp + zvd1 * step/2.0;
			SvMotionEq_GLO(&xvd2, &yvd2, &zvd2, &xp2, &yp2, &zp2, &xv2, &yv2, &zv2, &xa, &ya, &za);

			// step 3
			xp3 = xp_temp + xv2 * step/2.0;
			yp3 = yp_temp + yv2 * step/2.0;
			zp3 = zp_temp + zv2 * step/2.0;
			xv3 = xv_temp + xvd2 * step/2.0;
			yv3 = yv_temp + yvd2 * step/2.0;
			zv3 = zv_temp + zvd2 * step/2.0;
			SvMotionEq_GLO(&xvd3, &yvd3, &zvd3, &xp3, &yp3, &zp3, &xv3, &yv3, &zv3, &xa, &ya, &za);

			// step 4
			xp4 = xp_temp + xv3 * step;
			yp4 = yp_temp + yv3 * step;
			zp4 = zp_temp + zv3 * step;
			xv4 = xv_temp + xvd3 * step;
			yv4 = yv_temp + yvd3 * step;
			zv4 = zv_temp + zvd3 * step;
			SvMotionEq_GLO(&xvd4, &yvd4, &zvd4, &xp4, &yp4, &zp4, &xv4, &yv4, &zv4, &xa, &ya, &za);

			xp_temp = xp_temp + (xv1 + 2*xv2 + 2*xv3 + xv4) * step/6.0;
			yp_temp = yp_temp + (yv1 + 2*yv2 + 2*yv3 + yv4) * step/6.0;
			zp_temp = zp_temp + (zv1 + 2*zv2 + 2*zv3 + zv4) * step/6.0;
			xv_temp = xv_temp + (xvd1 + 2*xvd2 + 2*xvd3 + xvd4) * step/6.0;
			yv_temp = yv_temp + (yvd1 + 2*yvd2 + 2*yvd3 + yvd4) * step/6.0;
			zv_temp = zv_temp + (zvd1 + 2*zvd2 + 2*zvd3 + zvd4) * step/6.0;
		}
		// Earth Rotation
		tau = NavParam->TransitTime[i];
//		sTimeCurrent - NavParam->TimeTx[i];
		xp_temp =  xp_temp * cos(tau * OmegaDot_PZ90) + yp_temp * sin(tau * OmegaDot_PZ90);
		yp_temp = -xp_temp * sin(tau * OmegaDot_PZ90) + yp_temp * cos(tau * OmegaDot_PZ90);
		xv_temp =  xv_temp * cos(tau * OmegaDot_PZ90) + yv_temp * sin(tau * OmegaDot_PZ90);
		yv_temp = -xv_temp * sin(tau * OmegaDot_PZ90) + yv_temp * cos(tau * OmegaDot_PZ90);

		//	Datum 변환 (PZ-90 to WGS84)
		NavParam->SvPosition[i][0] = xp_temp - 0.36;
		NavParam->SvPosition[i][1] = yp_temp + 0.08;
		NavParam->SvPosition[i][2] = zp_temp + 0.18;
		NavParam->SvVelocity[i][0] = xv_temp;
		NavParam->SvVelocity[i][1] = yv_temp;
		NavParam->SvVelocity[i][2] = zv_temp;
	}
} // function: EstimateSvOrbit

void SvMotionEq_GLO(double *xvd, double *yvd, double *zvd, double *xp, double *yp, double *zp,
					double *xv, double *yv, double *zv, double *xa, double *ya, double *za){
	double r, g, h, k;

	r = sqrt((*xp)*(*xp) + (*yp)*(*yp) + (*zp)*(*zp));
	g = -Mu_PZ90 / (r*r*r);
	h = J2_PZ90 * 1.5 * (A_PZ90 * A_PZ90 / (r*r));
	k = 5 * (*zp)*(*zp) / (r*r);

	*xvd = g*(*xp)*(1-h*(k-1)) + (*xa) + OmegaDot_PZ90*OmegaDot_PZ90*(*xp) + 2*OmegaDot_PZ90*(*yv);
	*yvd = g*(*yp)*(1-h*(k-1)) + (*ya) + OmegaDot_PZ90*OmegaDot_PZ90*(*yp) - 2*OmegaDot_PZ90*(*xv);
	*zvd = g*(*zp)*(1-h*(k-3)) + (*za);
} // function: SvMotionEq_GLO

void EstimateElAz(NavSystem *NavSys, NavParameter *NavParam, unsigned char i){
	double dx, dy, dz, slat, slon, r;
	double enu[3];

	// XYZ to ENU 변환
	dx = NavParam->SvPosition[i][0] - NavSys->RefPos_ECEF[0];
	dy = NavParam->SvPosition[i][1] - NavSys->RefPos_ECEF[1];
	dz = NavParam->SvPosition[i][2] - NavSys->RefPos_ECEF[2];

	slat = PI/2.0 - NavSys->RefPos_LLA[0];
	slon = PI/2.0 + NavSys->RefPos_LLA[1];

	enu[0] = cos(slon)*dx + sin(slon)*dy;
	enu[1] = -cos(slat)*sin(slon)*dx + cos(slat)*cos(slon)*dy + sin(slat)*dz;
	enu[2] = sin(slat)*sin(slon)*dx - sin(slat)*cos(slon)*dy + cos(slat)*dz;

	r = sqrt(enu[0]*enu[0] + enu[1]*enu[1]);
	NavParam->SvELrad[i] = atan2(enu[2], r);
	NavParam->SvAZrad[i] = atan2(enu[0], enu[1]);
} // function: EstimateElAz

void EstimateCSC(NavParameter *NavParam, unsigned int i){
	double Code, Phase;
	uint8_t n1, n2;
	double temp;
	double CSC1, CSC2;
	double PhasePrev, CSC1Prev, CSC2Prev;
	float WaveLength;

	Code = NavParam->mCode[i];
	Phase = NavParam->mPhase[i];
	PhasePrev = NavParam->mPhasePrev[i];
	CSC1Prev = NavParam->mCSC1_prev[i];
	CSC2Prev = NavParam->mCSC2_prev[i];
	WaveLength = NavParam->mWaveLength[i];

	n1 = NavParam->mCSC1_count[i];
	n2 = NavParam->mCSC2_count[i];

	// 100s Smoothing
	if (n1 == 1){
		CSC1 = Code;
		n1++;
	} else {
		n1 = MIN(n1, kSmoothLen);
		temp = (Phase - PhasePrev)*WaveLength;
		CSC1 = Code/n1 + (CSC1Prev - temp) * (n1-1)/n1;
		if (n1 < kSmoothLen)
			n1++;
	}
	NavParam->mCSC1_count[i] = n1;
	NavParam->mCSC1[i] = CSC1;

	// 30s Smoothing
	if (n2 == 1){
		CSC2 = Code;
		n2++;
	} else {
		n2 = MIN(n2, kSmoothLen2);
		temp = (Phase - PhasePrev)*WaveLength;
		CSC2 = Code/n2 + (CSC2Prev - temp) * (n2-1)/n2;
		if (n2 < kSmoothLen2)
			n2++;
	}
	NavParam->mCSC2_count[i] = n2;
	NavParam->mCSC2[i] = CSC2;
} // function: EstimateCSC

void EstimateCorr(NavSystem *NavSys, NavParameter *NavParam, unsigned char i){
	double CSC1, CSC2, dx, dy, dz, r, ClockOffset;

	CSC1 			= NavParam->mCSC1[i];
	CSC2 			= NavParam->mCSC2[i];
	dx 				= NavParam->SvPosition[i][0] - NavSys->RefPos_ECEF[0];
	dy				= NavParam->SvPosition[i][1] - NavSys->RefPos_ECEF[1];
	dz 				= NavParam->SvPosition[i][2] - NavSys->RefPos_ECEF[2];
	ClockOffset 	= NavParam->SvClockOffset[i];

	r  = sqrt(dx*dx + dy*dy + dz*dz);

	if (NavParam->mCSC1_count[i] == kSmoothLen)
		NavParam->SvPRC1[i] = CSC1 - r + ClockOffset*SPEED_OF_LIGHT;

	if (NavParam->mCSC2_count[i] == kSmoothLen2)
		NavParam->SvPRC2[i] = CSC2 - r + ClockOffset*SPEED_OF_LIGHT;

	NavParam->SvRRC1[i] = (NavParam->SvPRC1[i] - NavParam->SvPRC1Prev[i]) / 2.0;
	NavParam->SvRRC2[i] = (NavParam->SvPRC2[i] - NavParam->SvPRC2Prev[i]) / 2.0;

	NavParam->SvPRC[i] = NavParam->mCode[i] - r + ClockOffset*SPEED_OF_LIGHT;
//	printf("%2i %8.2f %8.2f %8.2f %8.2f\n", i+1, NavParam->SvPRC[i], NavParam->mCode[i],
//			r, ClockOffset*SPEED_OF_LIGHT);

} // function: EstimateCorr

void SaveCurrentSvParam(NavSystem *NavSys, NavParameter *NavParam){
	uint8_t i;

	NavSys->sTimePrevious = NavSys->sTimeCurrent;

	for (i=0;i<kNumSatellite;i++){
		if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1){
			NavParam->mCodePrev[i] = NavParam->mCode[i];
			NavParam->mPhasePrev[i] = NavParam->mPhase[i];
			NavParam->mLockTimePrev[i] = NavParam->mLockTime[i];

			NavParam->mCSC1_prev[i] = NavParam->mCSC1[i];
			NavParam->mCSC2_prev[i] = NavParam->mCSC2[i];
			NavParam->SvPRC1Prev[i] = NavParam->SvPRC1[i];
			NavParam->SvPRC2Prev[i] = NavParam->SvPRC2[i];
		} else {
			NavParam->mCodePrev[i] = NavParam->mCode[i];
			NavParam->mPhasePrev[i] = NavParam->mPhase[i];
			NavParam->mLockTimePrev[i] = NavParam->mLockTime[i];

			NavParam->mCSC1_prev[i] = NavParam->mCSC1[i];
			NavParam->mCSC2_prev[i] = NavParam->mCSC2[i];
			NavParam->SvPRC1Prev[i] = NavParam->SvPRC1[i];
			NavParam->SvPRC2Prev[i] = NavParam->SvPRC2[i];
		}
	}
} // function: SaveCurrentParam

unsigned char GetConstFlag(unsigned char i){
	// Return constellation flag
	// 0 : GPS
	// 1 : GLO
	// 9 : Not identified
	unsigned char prn;

	prn = i+1;
	if (prn >= PRN_GPS_min && prn <= PRN_GPS_max)
		return 0;
	else if (prn >= PRN_GLO_min && prn <= PRN_GLO_max)
		return 1;
	else
		return 9;

} // function: GetConstFlag

void ConvertECEF2LLA(double xyz[3],double xlla[3]){
	int i;
	double rhosqrd, rho, templat, tempalt,
			rhoerror, zerror, slat, clat, q, r_n, drdl, aa, bb, cc, dd, invdet;

	xlla[1] = atan2(xyz[1], xyz[0])*r2d;


	rhosqrd = xyz[0]*xyz[0] + xyz[1]*xyz[1];
	rho = sqrt(rhosqrd);
	templat = atan2(xyz[2], rho);
	tempalt = sqrt(rhosqrd + xyz[2]*xyz[2]) - A_WGS84;
	rhoerror = 1000.0;
	zerror   = 1000.0;

	while ((fabs(rhoerror) > 1e-6) || (fabs(zerror) > 1e-6))
	{
		slat = sin(templat);
		clat = cos(templat);
		q = 1 - E2_Earth*slat*slat;
		r_n = A_WGS84/sqrt(q);
		drdl = r_n*E2_Earth*slat*clat/q;

		rhoerror = (r_n + tempalt)*clat - rho;
		zerror   = (r_n*(1 - E2_Earth) + tempalt)*slat - xyz[2];

		aa = drdl*clat - (r_n + tempalt)*slat;
		bb = clat;
		cc = (1 - E2_Earth)*(drdl*slat + r_n*clat);
		dd = slat;

		invdet = 1.0/(aa*dd - bb*cc);
		templat = templat - invdet*(+dd*rhoerror -bb*zerror);
		tempalt = tempalt - invdet*(-cc*rhoerror +aa*zerror);
	}

	xlla[0] = templat*r2d;
	xlla[2] = tempalt;
} // function: ConvertECEF2LLA

void DetermineSvStatus(NavSystem *NavSys, NavParameter *NavParam, NavEphData *NavEph){
	unsigned int i;

	NavSys->NumSv = 0;
	NavSys->NumSvGPS = 0;
	NavSys->NumSvGLO = 0;
	for (i = 0; i < kNumSatellite; i++) {
		if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1 && NavParam->SvELrad[i] > (MaskAngle * d2r)) {
			switch (GetConstFlag(i)) {
			case FlagGPS:
				if (ConstellationMode == 0 || ConstellationMode == 2) {
					NavSys->NumSv++;
					NavSys->NumSvGPS++;
					NavSys->sSvStatus[i] = 1; // Good
				}
				break;
			case FlagGLO:
				if (ConstellationMode == 1 || ConstellationMode == 2) {
					NavSys->NumSv++;
					NavSys->NumSvGLO++;
					NavSys->sSvStatus[i] = 1; // Good
				}
				break;
			} // switch
		} // if
		else
			NavSys->sSvStatus[i] = 0; // Bad
	} // for

	// Set Current Constellation Status
	if (NavSys->NumSvGLO == 0){
		NavSys->sConstellationFlag = 0;
		NavSys->NumConstellation = 1;
	} else if (NavSys->NumSvGPS == 0) {
		NavSys->sConstellationFlag = 1;
		NavSys->NumConstellation = 1;
	} else if ((NavSys->NumSvGPS > 0) && (NavSys->NumSvGLO > 0)) {
		NavSys->sConstellationFlag = 2;
		NavSys->NumConstellation = 2;
	}
} // function: DetermineSvStatus

void EstimateUsrPos(NavSystem *NavSys, NavParameter *NavParam) {

	//unsigned char NumSv, NumConstellation;
	double drho;
	int i, j, k;
	unsigned char SvRow = 1;
	double el;
	double Norm_dX = 100;

	double sigma;
	double sigTotal;
	double Re = 6378.1363;
	double xair = 0.2;
	double tauair = 100;
	double vair = 0;
	double dI = 0.004;
	double hI = 350;
	double sigPr, Fpp, sigIono, sigAir, sigTropo, sigTot;

	dmat G = ZERODMAT;
	dmat sigmaMatrix = ZERODMAT;
	dmat W = ZERODMAT;
	dmat transG = ZERODMAT;
	dmat transGxW = ZERODMAT;
	dmat transGxWxG = ZERODMAT;
	dmat invtransGxWxG = ZERODMAT;
	dmat H = ZERODMAT;
	dvec residual = ZERODVEC;
	dvec dX = ZERODVEC;
	int NumSv;
	int NumConstellation;

	NumSv = NavSys->NumSvGPS;

	NumConstellation = NavSys->NumConstellation;
	NumConstellation =1;

	initdmat(&G, 1, NumSv, 1, 3+NumConstellation); //G
	initdmat(&sigmaMatrix, 1, NumSv, 1, NumSv); //sigmaMatrix
	initdmat(&W, 1, NumSv, 1, NumSv);
	initdmat(&transG, 1, 3+NumConstellation, 1, NumSv);
	initdmat(&transGxW, 1, 3+NumConstellation, 1, NumSv);
	initdmat(&transGxWxG, 1, 3+NumConstellation, 1, 3+NumConstellation);
	initdmat(&invtransGxWxG, 1, 3+NumConstellation, 1, 3+NumConstellation);
	initdmat(&H, 1, 3+NumConstellation, 1, NumSv);
	initdvec(&residual, 1, NumSv);
	initdvec(&dX, 1, 3+NumConstellation);
	zerodmat(&sigmaMatrix);

	// S matrix
	for (i = 0; i < kNumSatellite-24; i++) {
//		if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1 &&
//				GndData.SvPRC1[i] != 0.0 && NavParam->SvELrad[i] > (5*d2r)){
		//if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1) {
		if (NavSys->sSvStatus[i] == 1){ // Good

			el = NavParam->SvELrad[i];

			sigPr = 0.16 + 1.07 * exp(-el / 15.5);
			Fpp = 1 / sqrt(1 - ((Re * cos(el) / (Re + hI))*(Re * cos(el) / (Re + hI))));
			sigIono = dI * (xair + 2 * tauair * vair) * Fpp;
			sigAir = 0.16 + 1.07 * exp(-el / 15.5);
			sigTropo = 0;
			sigTotal= sigPr*sigPr + sigIono*sigIono + sigAir*sigAir + sigTropo*sigTropo;

			//setdmat(&sigmaMatrix, SvRow, SvRow, sigTotal);	//x
			setdmat(&sigmaMatrix, SvRow, SvRow, 1);	//x

			SvRow++;
		}
	}
	// Get W by inverting sigmaMatrix

	dmatinv(&sigmaMatrix, &W);

	double dx, dy, dz, r;

	// Calculate Position
	while(Norm_dX > 0.001) {
		SvRow = 1;
		for (i = 0; i < kNumSatellite-24; i++) {
			//if (NavSys->sIsMeasurementOn[i] == 1 && NavSys->sIsEphCurrOn[i] == 1) {
			if (NavSys->sSvStatus[i] == 1){

				dx = NavParam->SvPosition[i][0] - NavSys->UsrPos_ECEF[0];
				dy = NavParam->SvPosition[i][1] - NavSys->UsrPos_ECEF[1];
				dz = NavParam->SvPosition[i][2] - NavSys->UsrPos_ECEF[2];
				r = sqrt(dx*dx + dy*dy + dz*dz);

				// update G matrix
				setdmat(&G, SvRow, 1, -1*dx/r);	//x
				setdmat(&G, SvRow, 2, -1*dy/r);	//x
				setdmat(&G, SvRow, 3, -1*dz/r);	//x
				NavSys->NumConstellation = 1;

				switch (NavSys->NumConstellation) {
				case 1:
					setdmat(&G, SvRow, 4, 1);
					//					drho = NavParam->mCode[i] - GndData.SvPRC1[i] -
//							(r - NavParam->SvClockOffset[i]*SPEED_OF_LIGHT
//									+ NavSys->UsrClockOffset);
//					drho = NavParam->mCode[i]+SPEED_OF_LIGHT*NavParam->SvClockOffset[i]-NavParam->tropo_correction[i]-NavParam->iono_correction[i] - (r + NavSys->UsrClockOffset);
					drho = NavParam->mCSC1[i]+SPEED_OF_LIGHT*NavParam->SvClockOffset[i]-NavParam->tropo_correction[i]-NavParam->iono_correction[i] - (r + NavSys->UsrClockOffset);

					//NavParam->mCSC1[i] - GndData.SvPRC1[i] -
					break;
				case 2:
					if (GetConstFlag(i) == FlagGPS) {
						setdmat(&G, SvRow, 4, 1);
						setdmat(&G, SvRow, 5, 0);
					} else if (GetConstFlag(i) == FlagGLO) {
						setdmat(&G, SvRow, 4, 1);
						setdmat(&G, SvRow, 5, 1);
					}
//					drho = NavParam->mCode[i] - GndData.SvPRC1[i] -
//							(r - NavParam->SvClockOffset[i]*SPEED_OF_LIGHT
//									+ NavSys->UsrClockOffset + NavSys->ISTB1);
					drho = (r - NavParam->SvClockOffset[i]*SPEED_OF_LIGHT
							+ NavSys->UsrClockOffset + NavSys->ISTB1);

					//NavParam->mCSC1[i] - GndData.SvPRC1[i] -
					break;
				}

				setdvec(&residual, SvRow, drho);	//range
				SvRow++;
			}
		}

		// Positioning
		transdmat(&G, &transG);
		dmatxdmat(&transG, &W, &transGxW);
		dmatxdmat(&transGxW, &G, &transGxWxG);
		dmatinv(&transGxWxG, &invtransGxWxG);
		dmatxdmat(&invtransGxWxG, &transGxW, &H);	//H=inv(G'WG)G'W
		dmatxdvec(&H,&residual,&dX); // After residual calculation

		NavSys->UsrPos_ECEF[0] = NavSys->UsrPos_ECEF[0] + dX.vec[1];
		NavSys->UsrPos_ECEF[1] = NavSys->UsrPos_ECEF[1] + dX.vec[2];
		NavSys->UsrPos_ECEF[2] = NavSys->UsrPos_ECEF[2] + dX.vec[3];
		NavSys->UsrClockOffset = NavSys->UsrClockOffset + dX.vec[4];

		if (NavSys->sConstellationFlag == FlagGPSGLO)
			NavSys->ISTB1 = NavSys->ISTB1 + dX.vec[5];

		Norm_dX = sqrt(dX.vec[1]*dX.vec[1] + dX.vec[2]*dX.vec[2] + dX.vec[3]*dX.vec[3]);
	}

	// DOP
	dmat transGxG = ZERODMAT;
	dmat invtransGxG = ZERODMAT;
	initdmat(&transGxG, 1, 3+NumConstellation, 1, 3+NumConstellation);
	initdmat(&invtransGxG, 1, 3+NumConstellation, 1, 3+NumConstellation);

	dmatxdmat(&transG, &G, &transGxG);
	dmatinv(&transGxG, &invtransGxG);

	NavSys->HDOP = sqrt(invtransGxG.mat[1][1] + invtransGxG.mat[2][2]);
	NavSys->VDOP = sqrt(invtransGxG.mat[3][3]);
	NavSys->PDOP = sqrt(invtransGxG.mat[1][1] + invtransGxG.mat[2][2] + invtransGxG.mat[3][3]);

	freedmat(&transGxG);
	freedmat(&invtransGxG);

	freedmat(&G);
	freedmat(&sigmaMatrix);
	freedmat(&W);
	freedmat(&transG);
	freedmat(&transGxW);
	freedmat(&transGxWxG);
	freedmat(&invtransGxWxG);
	freedmat(&H);
	freedvec(&residual);
	freedvec(&dX);
} // function: EstimateUsrPos








