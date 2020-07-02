/*
 * gnss_core.cpp
 *
 *  Created on: 2020. 6. 19.
 *      Author: root
 */

#include "novatel/gnss.h"
#include "gnss_core.h"
#include "navutils.h"
#include <cmath>
using namespace Eigen;
using namespace std;

/* Singleton Constructor */
GnssCore* GnssCore::inst = nullptr;
GnssCore* GnssCore::getInstance(){
	if (inst == nullptr)
		inst = new GnssCore();
	return inst;
}

void GnssCore::ProcessRange(double timeRangeHeader){
	uint16_t i;
	timeCurrent_ = timeRangeHeader;

	InitializeGnssFilter();

	bool isCN0[NUMBER_OF_SATELLITES];
	bool isElGood[NUMBER_OF_SATELLITES];
	for (i=0; i<NUMBER_OF_SATELLITES; i++){
		isCN0[i] = false;
		isElGood[i] = false;
	}
	numSvForPos_ = 0;
	for (i=0; i<NUMBER_OF_SATELLITES; i++){
		if (CheckSvForProcess(i)){
			// Clock offset 적용 전 Transmission Time 추정
			timeTxRaw_[i] = timeCurrent_ - mCurrentCode_[i]/SPEED_OF_LIGHT;

			// 위성 시계 오차, 궤도 추정
			CalcSvClockOffset(i);

			timeTx_[i] = timeTxRaw_[i] - svClockOffset_[i];
			timeTransit_[i] = timeCurrent_ - timeTx_[i];

			CalcSvOrbit(i);

			// 위성 El, Az 계산
			CalcElAz(i);
			if (svElRad_[i] * R2D > EL_MASK_IN_DEGREE)
				isGoodForPos_[i] = true;
			else
				continue;

			CalcIonoDelay(i);

			CalcTropoDelay(i);

			if (USE_SMOOTHING)
				CalcCarrierSmoothedCode(i);

			// Correction 계산
//			EstimateCorr(NavSys, NavParam, i);
			numSvForPos_++;
		}

		if (svElRad_[i] * R2D > EL_MASK_IN_DEGREE)
			isElGood[i] = true;
		if (mCN0_[i] > CN0_MINIMUM)
			isCN0[i] = true;
		if (isMeasurementOn_[i] || isCurrentEphemOn_[i])
		printf("[PRN] %2i\t[EL] %5.2f\t[AZ] %7.2f\t[CN0] %5.2f\t [SCnt] %3i [Flag] %i%i%i%i%i\n",
				i, svElRad_[i]*R2D, svAzRad_[i]*R2D, mCN0_[i], mCountOnCSC1_[i],
				isMeasurementOn_[i], isCurrentEphemOn_[i], isEphemHealthGood_[i],
				isElGood[i], isCN0[i]);

	}
	CalcLeastSquaredPosition();

	ClearGnssFilter();
}
bool GnssCore::CheckSvForProcess(unsigned char i){

	if (isMeasurementOn_[i] &
		isCurrentEphemOn_[i] &
		mCN0_[i] > CN0_MINIMUM &
		isEphemHealthGood_[i])
		return true;
	else
		return false;

}

void GnssCore::InitializeGnssFilter(){

	Position bestpos = GNSS::getInstance()->GetBestpos();
	usrBestposLLA_[0] = bestpos.latitude * D2R;
	usrBestposLLA_[1] = bestpos.longitude * D2R;
	usrBestposLLA_[2] = bestpos.height + bestpos.undulation;

	Vector3d ecef = ConvertLLA2ECEF(usrBestposLLA_[0], usrBestposLLA_[1], usrBestposLLA_[2]);
	usrBestposECEF_[0] = ecef(0);
	usrBestposECEF_[1] = ecef(1);
	usrBestposECEF_[2] = ecef(2);

	usrPositionECEF_[0] = usrBestposECEF_[0];
	usrPositionECEF_[1] = usrBestposECEF_[1];
	usrPositionECEF_[2] = usrBestposECEF_[2];
	usrPositionLLA_[0] = usrBestposLLA_[0];
	usrPositionLLA_[1] = usrBestposLLA_[1];
	usrPositionLLA_[2] = usrBestposLLA_[2];

	// Measurement
	RangeData 	obsSet;
	for (uint8_t i = 0; i < NUMBER_OF_SATELLITES; i++) {
		if (isMeasurementOn_[i] & isCurrentEphemOn_[i]){
			switch (GetConstFlag(i)) {
			case FLAG_GPS:
				obsSet 					= obsGpsL1_[i];
				mFrequency_[i] 			= FREQ_GPS_L1;
				break;
			case FLAG_GLO:
				obsSet						= obsGloL1_[i-INDEX_GLO_MIN];
				mFrequency_[i] 			= FREQ_GLO_L1 + (obsGloL1_[i-INDEX_GLO_MIN].glonass_frequency - 7) * FREQ_GLO_STEP;
				break;
			case FLAG_GAL:
				obsSet						= obsGalE5a_[i-INDEX_GAL_MIN];
				mFrequency_[i] 			= FREQ_GAL_E5a;
				break;
			case FLAG_BDS:
				obsSet 					= obsBdsB1I_[i-INDEX_BDS_MIN];
				mFrequency_[i] 			= FREQ_BDS_B1I;
				break;
			default:
				break;
			}
			mCurrentCode_[i] 			= obsSet.pseudorange;
			mCurrentPhase_[i] 		= obsSet.accumulated_doppler;
			mCurrentLockTime_[i] 	= obsSet.locktime;
			mCodeRate_[i]				= mWaveLength_[i] * obsSet.doppler;
			mCN0_[i]					= obsSet.carrier_to_noise;
			mWaveLength_[i] 			= SPEED_OF_LIGHT/mFrequency_[i];

		}
	}
} // function: InitializeGnssFilter

void GnssCore::CalcSvClockOffset(unsigned char i){
	double tk;

	switch (GetConstFlag(i)){
	case FLAG_GPS:
		double n0, nk, Mk;
		double Ek, rel_t;
		double A, M0, deltaN, ecc, af0, af1, af2, TOC, Tgd;

		A 		= currentGpsEphemerides_[i].semi_major_axis;
		M0 		= currentGpsEphemerides_[i].anomoly_reference_time;
		deltaN = currentGpsEphemerides_[i].mean_motion_difference;
		ecc 	= currentGpsEphemerides_[i].eccentricity;
		af0 	= currentGpsEphemerides_[i].clock_aligning_param_0;
		af1 	= currentGpsEphemerides_[i].clock_aligning_param_1;
		af2 	= currentGpsEphemerides_[i].clock_aligning_param_2;
		TOC		= currentGpsEphemerides_[i].sv_clock_correction;
		Tgd 	= currentGpsEphemerides_[i].group_delay_difference;

		tk = timeTxRaw_[i] - TOC;

		// 상대성 효과에 의한 delay
		n0 = sqrt(MU_WGS84 / (A * A * A));
		nk = n0 + deltaN;
		Mk = M0 + nk * tk;
		KeplerEquation(&Ek, Mk, ecc);
		rel_t = F_RELATIVE * ecc * sqrt(A) * sin(Ek);

		svClockOffset_[i] = af0 + (af1 * tk) + (af2 * tk * tk) + rel_t - Tgd;
		svClockDrift_[i]  = af1 + af2 * tk;
		break;
	case FLAG_GLO:
		double e_time, taun, gamma;
		e_time = currentGloEphemerides_[i-NUMBER_OF_GPS].e_time;
		taun 	= currentGloEphemerides_[i-NUMBER_OF_GPS].taun;
		gamma  = currentGloEphemerides_[i-NUMBER_OF_GPS].gamma;

		tk = timeTxRaw_[i] - e_time;

		svClockOffset_[i] = -taun + (gamma * tk);
		break;
	default:
		break;
	}
} // function: CalcSvClockOffset

void GnssCore::KeplerEquation(double *Ek, double Mk, double ecc) {
	int max_iteration = 20;
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

void GnssCore::CalcSvOrbit(unsigned char i){

	double n0, nk, Mk;
	double Ek, nuk, phik;
	double Ekdot, nukdot, ikdot, ukdot, rkdot, omegakdot, xodot, yodot;
	double A, ecc, w, M0, N, cic, cis, crc, crs, cuc, cus,
			deltaN, i0, idot, omega0, omegadot, TOE,
			uk, rk, ik, xo, yo, omegak;

	double tk, xp_temp, yp_temp, zp_temp, tau, xv_temp, yv_temp, zv_temp;
	double xp, yp, zp, xv, yv, zv, xa, ya, za; // Eph 변수 저장용
	double step; // step size
	double n, res; // iteration 횟수, 나머지
	int s;
	/* Runge Kutta 변수 */
	double xp1, xp2, xp3, xp4, yp1, yp2, yp3, yp4, zp1, zp2, zp3, zp4;
	double xv1, xv2, xv3, xv4, yv1, yv2, yv3, yv4, zv1, zv2, zv3, zv4;
	double xvd1, xvd2, xvd3, xvd4, yvd1, yvd2, yvd3, yvd4, zvd1, zvd2, zvd3, zvd4;
	double r, g, h, k;
	double xpe, ype, zpe, xve, yve, zve, xae, yae, zae;

	switch (GetConstFlag(i)){
	case FLAG_GPS:

		A 			= currentGpsEphemerides_[i].semi_major_axis;
		ecc			= currentGpsEphemerides_[i].eccentricity;
		M0 			= currentGpsEphemerides_[i].anomoly_reference_time;
		N 			= currentGpsEphemerides_[i].corrected_mean_motion;
		cic 		= currentGpsEphemerides_[i].inclination_cosine;
		cis			= currentGpsEphemerides_[i].inclination_sine;
		crc 		= currentGpsEphemerides_[i].orbit_radius_cosine;
		crs 		= currentGpsEphemerides_[i].orbit_radius_sine;
		cuc 		= currentGpsEphemerides_[i].latitude_cosine;
		cus 		= currentGpsEphemerides_[i].latitude_sine;
		deltaN 	= currentGpsEphemerides_[i].mean_motion_difference;
		i0 			= currentGpsEphemerides_[i].inclination_angle;
		idot 		= currentGpsEphemerides_[i].inclination_angle_rate;
		omega0 	= currentGpsEphemerides_[i].right_ascension;
		omegadot 	= currentGpsEphemerides_[i].right_ascension_rate;
		w 			= currentGpsEphemerides_[i].omega;
		TOE			= currentGpsEphemerides_[i].time_of_ephemeris;

		tk = timeTx_[i] - TOE;

		n0 = sqrt(MU_WGS84 / (A * A * A));
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
		omegak = omega0 - (OMEGADOT_WGS84 * TOE) + (omegadot - OMEGADOT_WGS84) * tk;

		xp_temp = xo * cos(omegak) - yo * cos(ik) * sin(omegak);
		yp_temp = xo * sin(omegak) + yo * cos(ik) * cos(omegak);
		zp_temp = yo * sin(ik);

		Ekdot  =  nk/(1 - ecc * cos(Ek));
		nukdot =  Ekdot * sqrt(1 - ecc * ecc) / (1 - ecc * cos(Ek));
		ikdot  =  idot + 2 * nukdot * (cis * cos(2 * phik) - cic * sin(2 * phik));
		ukdot  =  nukdot + 2 * nukdot * (cus * cos(2 * phik) - cuc * sin(2 * phik));
		rkdot  =  ecc * A * Ekdot * sin(Ek) + 2 * nukdot * (crs * cos(2 * phik) - crc * sin(2 * phik));
		omegakdot = omegadot - OMEGADOT_WGS84;
		xodot  =  rkdot * cos(uk) - rk * ukdot * sin(uk);
		yodot  =  rkdot * sin(uk) + rk * ukdot * cos(uk);

		// Earth Rotation
		tau = timeTransit_[i];
		svPosition_[i][0] =  xp_temp * cos(tau * OMEGADOT_WGS84) + yp_temp * sin(tau * OMEGADOT_WGS84);
		svPosition_[i][1] = -xp_temp * sin(tau * OMEGADOT_WGS84) + yp_temp * cos(tau * OMEGADOT_WGS84);
		svPosition_[i][2] =  zp_temp;

		xv_temp =  -xo * omegakdot * sin(omegak) + xodot * cos(omegak) - yodot * sin(omegak) * cos(ik) - yo * (omegakdot * cos(omegak) * cos(ik) - (ikdot) * sin(omegak) * sin(ik));
		yv_temp =  xo * omegakdot * cos(omegak) + xodot * sin(omegak) + yodot * cos(omegak) * cos(ik) - yo * (omegakdot * sin(omegak) * cos(ik) + (ikdot) * cos(omegak) * sin(ik));
		zv_temp =  yo * ikdot * cos(ik) + yodot * sin(ik);
		svVelocity_[i][0] =  xv_temp * cos(tau * OMEGADOT_WGS84) + yv_temp * sin(tau * OMEGADOT_WGS84);
		svVelocity_[i][1] = -xv_temp * sin(tau * OMEGADOT_WGS84) + yv_temp * cos(tau * OMEGADOT_WGS84);
		svVelocity_[i][2] =  zv_temp;
		break;
	case FLAG_GLO:

		xpe = currentGloEphemerides_[i-INDEX_GLO_MIN].posx;
		ype = currentGloEphemerides_[i-INDEX_GLO_MIN].posy;
		zpe = currentGloEphemerides_[i-INDEX_GLO_MIN].posz;
		xve = currentGloEphemerides_[i-INDEX_GLO_MIN].velx;
		yve = currentGloEphemerides_[i-INDEX_GLO_MIN].vely;
		zve = currentGloEphemerides_[i-INDEX_GLO_MIN].velz;
		xae = currentGloEphemerides_[i-INDEX_GLO_MIN].accx;
		yae = currentGloEphemerides_[i-INDEX_GLO_MIN].accy;
		zae = currentGloEphemerides_[i-INDEX_GLO_MIN].accz;

		/* Transmission time 계산 */
		tk = timeTx_[i] - currentGloEphemerides_[i-INDEX_GLO_MIN].e_time;

//		// Coordinate transformation to an inertial reference frame
		double thetaG0, thetaGe;
		unsigned short gps_week;
		double gps_tow;
		gps_week = currentGloEphemerides_[i-INDEX_GLO_MIN].e_week;
		gps_tow = (double) currentGloEphemerides_[i-INDEX_GLO_MIN].e_time;
		unsigned char utc_offset;
		utc_offset = 27;
		double julianDate;

		julianDate = ConvertGPST2JulianDate(gps_week, gps_tow, utc_offset);
		thetaG0 = ConvertJulianDate2GMST(julianDate);

		thetaGe = thetaG0 + OMEGADOT_PZ90*(gps_tow - 3*3600);
//
		xp = xpe * cos(thetaGe) - ype * sin(thetaGe);
		yp = xpe * sin(thetaGe) + ype * cos(thetaGe);
		zp = zpe;
		xv = xve * cos(thetaGe) - yve * sin(thetaGe) - OMEGADOT_PZ90 * yae;
		yv = xve * sin(thetaGe) + yve * cos(thetaGe) + OMEGADOT_PZ90 * xae;
		zv = zve;
		xa = xae * cos(thetaGe) - yae * sin(thetaGe);
		ya = xae * sin(thetaGe) + yae * cos(thetaGe);
		za = zae;

//		xp = xpe; yp = ype; zp = zpe; xv = xve; yv = yve; zv = zve; xa = xae; ya = yae; za = zae;

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
		tau = timeTransit_[i];
		xp_temp =  xp_temp * cos(tau * OMEGADOT_PZ90) + yp_temp * sin(tau * OMEGADOT_PZ90);
		xv_temp =  xv_temp * cos(tau * OMEGADOT_PZ90) + yv_temp * sin(tau * OMEGADOT_PZ90);
		yp_temp = -xp_temp * sin(tau * OMEGADOT_PZ90) + yp_temp * cos(tau * OMEGADOT_PZ90);
		yv_temp = -xv_temp * sin(tau * OMEGADOT_PZ90) + yv_temp * cos(tau * OMEGADOT_PZ90);

		double xp_tempe, yp_tempe, zp_tempe;
		xp_tempe = xp_temp;
		yp_tempe = yp_temp;
		zp_tempe = zp_temp;
		xp_temp = xp_tempe * cos(thetaGe) + yp_tempe * sin(thetaGe);
		yp_temp = -xp_tempe * sin(thetaGe) + yp_tempe * cos(thetaGe);
		zp_temp = zp_tempe;

		//	Datum 변환 (PZ-90 to WGS84)
		svPosition_[i][0] = xp_temp - 0.36;
		svPosition_[i][1] = yp_temp + 0.08;
		svPosition_[i][2] = zp_temp + 0.18;

		svPosition_[i][0] = xp_temp;
		svPosition_[i][1] = yp_temp;
		svPosition_[i][2] = zp_temp;
		svVelocity_[i][0] = xv_temp;
		svVelocity_[i][1] = yv_temp;
		svVelocity_[i][2] = zv_temp;
		break;

	case FLAG_GAL:
		A 			= currentGalEphemerides_[i-INDEX_GAL_MIN].RootA * currentGalEphemerides_[i-INDEX_GAL_MIN].RootA;
		ecc			= currentGalEphemerides_[i-INDEX_GAL_MIN].Ecc;
		M0 			= currentGalEphemerides_[i-INDEX_GAL_MIN].M0;
		cic 		= currentGalEphemerides_[i-INDEX_GAL_MIN].Cic;
		cis			= currentGalEphemerides_[i-INDEX_GAL_MIN].Cis;
		crc 		= currentGalEphemerides_[i-INDEX_GAL_MIN].Crc;
		crs 		= currentGalEphemerides_[i-INDEX_GAL_MIN].Crs;
		cuc 		= currentGalEphemerides_[i-INDEX_GAL_MIN].Cuc;
		cus 		= currentGalEphemerides_[i-INDEX_GAL_MIN].Cus;
		deltaN 	= currentGalEphemerides_[i-INDEX_GAL_MIN].DeltaN;
		i0 			= currentGalEphemerides_[i-INDEX_GAL_MIN].I0;
		idot 		= currentGalEphemerides_[i-INDEX_GAL_MIN].Idot;
		omega0 	= currentGalEphemerides_[i-INDEX_GAL_MIN].Omega0;
		omegadot 	= currentGalEphemerides_[i-INDEX_GAL_MIN].OmegaDot;
		w 			= currentGalEphemerides_[i-INDEX_GAL_MIN].Omega;
		TOE			= currentGalEphemerides_[i-INDEX_GAL_MIN].T0e;

		tk = timeTx_[i] - TOE;

		n0 = sqrt(MU_WGS84 / (A * A * A));
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
		omegak = omega0 - (OMEGADOT_WGS84 * TOE) + (omegadot - OMEGADOT_WGS84) * tk;

		xp_temp = xo * cos(omegak) - yo * cos(ik) * sin(omegak);
		yp_temp = xo * sin(omegak) + yo * cos(ik) * cos(omegak);
		zp_temp = yo * sin(ik);

		Ekdot  =  nk/(1 - ecc * cos(Ek));
		nukdot =  Ekdot * sqrt(1 - ecc * ecc) / (1 - ecc * cos(Ek));
		ikdot  =  idot + 2 * nukdot * (cis * cos(2 * phik) - cic * sin(2 * phik));
		ukdot  =  nukdot + 2 * nukdot * (cus * cos(2 * phik) - cuc * sin(2 * phik));
		rkdot  =  ecc * A * Ekdot * sin(Ek) + 2 * nukdot * (crs * cos(2 * phik) - crc * sin(2 * phik));
		omegakdot = omegadot - OMEGADOT_WGS84;
		xodot  =  rkdot * cos(uk) - rk * ukdot * sin(uk);
		yodot  =  rkdot * sin(uk) + rk * ukdot * cos(uk);

		// Earth Rotation
		tau = timeTransit_[i];
		svPosition_[i][0] =  xp_temp * cos(tau * OMEGADOT_WGS84) + yp_temp * sin(tau * OMEGADOT_WGS84);
		svPosition_[i][1] = -xp_temp * sin(tau * OMEGADOT_WGS84) + yp_temp * cos(tau * OMEGADOT_WGS84);
		svPosition_[i][2] =  zp_temp;

		xv_temp =  -xo * omegakdot * sin(omegak) + xodot * cos(omegak) - yodot * sin(omegak) * cos(ik) - yo * (omegakdot * cos(omegak) * cos(ik) - (ikdot) * sin(omegak) * sin(ik));
		yv_temp =  xo * omegakdot * cos(omegak) + xodot * sin(omegak) + yodot * cos(omegak) * cos(ik) - yo * (omegakdot * sin(omegak) * cos(ik) + (ikdot) * cos(omegak) * sin(ik));
		zv_temp =  yo * ikdot * cos(ik) + yodot * sin(ik);
		svVelocity_[i][0] =  xv_temp * cos(tau * OMEGADOT_WGS84) + yv_temp * sin(tau * OMEGADOT_WGS84);
		svVelocity_[i][1] = -xv_temp * sin(tau * OMEGADOT_WGS84) + yv_temp * cos(tau * OMEGADOT_WGS84);
		svVelocity_[i][2] =  zv_temp;
		break;

	case FLAG_BDS:
		A 			= currentBdsEphemerides_[i-INDEX_BDS_MIN].RootA;
		ecc			= currentBdsEphemerides_[i-INDEX_BDS_MIN].ecc;
		M0 			= currentBdsEphemerides_[i-INDEX_BDS_MIN].M0;
		cic 		= currentBdsEphemerides_[i-INDEX_BDS_MIN].Cic;
		cis			= currentBdsEphemerides_[i-INDEX_BDS_MIN].Cis;
		crc 		= currentBdsEphemerides_[i-INDEX_BDS_MIN].Crc;
		crs 		= currentBdsEphemerides_[i-INDEX_BDS_MIN].Crs;
		cuc 		= currentBdsEphemerides_[i-INDEX_BDS_MIN].Cuc;
		cus 		= currentBdsEphemerides_[i-INDEX_BDS_MIN].Cus;
		deltaN 	= currentBdsEphemerides_[i-INDEX_BDS_MIN].deltaN;
		i0 			= currentBdsEphemerides_[i-INDEX_BDS_MIN].i0;
		idot 		= currentBdsEphemerides_[i-INDEX_BDS_MIN].idot;
		omega0 	= currentBdsEphemerides_[i-INDEX_BDS_MIN].omega0;
		omegadot 	= currentBdsEphemerides_[i-INDEX_BDS_MIN].omegadot;
		w 			= currentBdsEphemerides_[i-INDEX_BDS_MIN].omega;
		TOE			= currentBdsEphemerides_[i-INDEX_BDS_MIN].toe;

		tk = timeTx_[i] - TOE;

		n0 = sqrt(MU_WGS84 / (A * A * A));
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
		omegak = omega0 - (OMEGADOT_WGS84 * TOE) + (omegadot - OMEGADOT_WGS84) * tk;

		xp_temp = xo * cos(omegak) - yo * cos(ik) * sin(omegak);
		yp_temp = xo * sin(omegak) + yo * cos(ik) * cos(omegak);
		zp_temp = yo * sin(ik);

		Ekdot  =  nk/(1 - ecc * cos(Ek));
		nukdot =  Ekdot * sqrt(1 - ecc * ecc) / (1 - ecc * cos(Ek));
		ikdot  =  idot + 2 * nukdot * (cis * cos(2 * phik) - cic * sin(2 * phik));
		ukdot  =  nukdot + 2 * nukdot * (cus * cos(2 * phik) - cuc * sin(2 * phik));
		rkdot  =  ecc * A * Ekdot * sin(Ek) + 2 * nukdot * (crs * cos(2 * phik) - crc * sin(2 * phik));
		omegakdot = omegadot - OMEGADOT_WGS84;
		xodot  =  rkdot * cos(uk) - rk * ukdot * sin(uk);
		yodot  =  rkdot * sin(uk) + rk * ukdot * cos(uk);

		// Earth Rotation
		tau = timeTransit_[i];
		svPosition_[i][0] =  xp_temp * cos(tau * OMEGADOT_WGS84) + yp_temp * sin(tau * OMEGADOT_WGS84);
		svPosition_[i][1] = -xp_temp * sin(tau * OMEGADOT_WGS84) + yp_temp * cos(tau * OMEGADOT_WGS84);
		svPosition_[i][2] =  zp_temp;

		xv_temp =  -xo * omegakdot * sin(omegak) + xodot * cos(omegak) - yodot * sin(omegak) * cos(ik) - yo * (omegakdot * cos(omegak) * cos(ik) - (ikdot) * sin(omegak) * sin(ik));
		yv_temp =  xo * omegakdot * cos(omegak) + xodot * sin(omegak) + yodot * cos(omegak) * cos(ik) - yo * (omegakdot * sin(omegak) * cos(ik) + (ikdot) * cos(omegak) * sin(ik));
		zv_temp =  yo * ikdot * cos(ik) + yodot * sin(ik);
		svVelocity_[i][0] =  xv_temp * cos(tau * OMEGADOT_WGS84) + yv_temp * sin(tau * OMEGADOT_WGS84);
		svVelocity_[i][1] = -xv_temp * sin(tau * OMEGADOT_WGS84) + yv_temp * cos(tau * OMEGADOT_WGS84);
		svVelocity_[i][2] =  zv_temp;
		break;

	default:
		break;
	}
} // function: CalcSvOrbit

void GnssCore::SvMotionEq_GLO(double *xvd, double *yvd, double *zvd, double *xp, double *yp, double *zp,
					double *xv, double *yv, double *zv, double *xa, double *ya, double *za){
	double r, g, h, k;

	r = sqrt((*xp)*(*xp) + (*yp)*(*yp) + (*zp)*(*zp));
	g = -MU_PZ90 / (r*r*r);
	h = J2_PZ90 * 1.5 * (A_PZ90 * A_PZ90 / (r*r));
	k = 5 * (*zp)*(*zp) / (r*r);

	*xvd = g*(*xp)*(1-h*(k-1)) + (*xa) + OMEGADOT_PZ90*OMEGADOT_PZ90*(*xp) + 2*OMEGADOT_PZ90*(*yv);
	*yvd = g*(*yp)*(1-h*(k-1)) + (*ya) + OMEGADOT_PZ90*OMEGADOT_PZ90*(*yp) - 2*OMEGADOT_PZ90*(*xv);
	*zvd = g*(*zp)*(1-h*(k-3)) + (*za);
} // function: SvMotionEq_GLO

unsigned char GnssCore::GetConstFlag(unsigned char i){

	if (i >= INDEX_GPS_MIN && i <= INDEX_GPS_MAX)
		return FLAG_GPS;
	else if (i >= INDEX_GLO_MIN && i <= INDEX_GLO_MAX)
		return FLAG_GLO;
	else if (i >= INDEX_GAL_MIN && i <= INDEX_GAL_MAX)
		return FLAG_GAL;
	else if (i >= INDEX_BDS_MIN && i <= INDEX_BDS_MIN)
		return FLAG_BDS;
	else
		return 9;

} // function: GetConstFlag

void GnssCore::CalcElAz(unsigned char i){

	Vector3d svxyz;
	svxyz(0) = svPosition_[i][0];
	svxyz(1) = svPosition_[i][1];
	svxyz(2) = svPosition_[i][2];
	Vector3d enu = ConvertECEF2ENU(svxyz, usrPositionLLA_[0], usrPositionLLA_[1], usrPositionLLA_[2]);

	double r = sqrt(enu[0]*enu[0] + enu[1]*enu[1]);
	svElRad_[i] = atan2(enu[2], r);
	svAzRad_[i] = atan2(enu[0], enu[1]);
} // function: CalcElAz

void GnssCore::CalcIonoDelay(unsigned char i){
double R2S = 1/PI;
double S2R = PI;

double psi, lat_i, lon_i, lat_u, lon_u, lat_m, t, F;
double elRad = svElRad_[i];
double azRad = svAzRad_[i];
double AMP,PER,x; // sum
lat_u = usrPositionLLA_[0] * R2S;
lon_u = usrPositionLLA_[1] * R2S;

psi = 0.0137/(elRad * R2S + 0.11) - 0.022;

lat_i = lat_u + psi * cos(azRad);
//lat_i = lat_u + psi * cos(azRad * R2S);

if (lat_i > 0.416)
	lat_i = 0.416;
else if (lat_i < -0.416)
	lat_i = -0.416;

lon_i = lon_u + (psi * sin(azRad) / cos(lat_i * S2R));
lat_m = lat_i + 0.064 * cos((lon_i - 1.617)*S2R);
//lon_i = lon_u + (psi * sin(azRad * R2S) / cos(lat_i));
//lat_m = lat_i + 0.064 * cos((lon_i - 1.617));

t = (4.32 * 10000) * lon_i + timeCurrent_;
t = fmod(t, 86400);
if (t > 86400)
	t = t - 86400;
else if (t < 0)
	t = t + 86400;

F = 1 + 16 * pow((0.53 - elRad * R2S),3);

IonosphericModel ionParams = GNSS::getInstance()->GetIonutc();
AMP = ionParams.a0
		+ ionParams.a1 * lat_m
			+ ionParams.a2 * (lat_m * lat_m)
				+ ionParams.a3 * (lat_m * lat_m * lat_m);
if (AMP < 0)
	AMP = 0;

PER = ionParams.b0
		+ ionParams.b1 * lat_m
			+ ionParams.b2 * (lat_m * lat_m)
				+ ionParams.b3 * (lat_m * lat_m * lat_m);
if (PER < 72000)
	PER = 72000;

x = 2 * PI * (t - 50400)/PER;
if (fabs(x) > 1.57)
	svIonoErr_[i] = SPEED_OF_LIGHT * F * 5e-09;
else
	svIonoErr_[i] = SPEED_OF_LIGHT * F * (5e-09 + AMP * (1 - x*x/2 + x*x*x*x/24));

double alpha;
if (GetConstFlag(i) == FLAG_GLO){
	alpha = (FREQ_GPS_L1*FREQ_GPS_L1) / (mFrequency_[i] * mFrequency_[i]);
	svIonoErr_[i] = alpha * svIonoErr_[i];
}

} //function : CalcIonoDelay

void GnssCore::CalcTropoDelay(unsigned char i){
double elRad = svElRad_[i];

// SBAS Model
double P, T, e, beta, lambda;
double k1 = 77.604;
double k2 = 382000;
double Rd = 287.054;
double gm = 9.784;
double g = 9.80665;

double P0, dP;
double T0, dT;
double b0, db;
double e0, de;
double l0, dl;
double latdeg = usrBestposLLA_[0]*R2D;
P0 = 1017.25 + (1015.75 - 1017.25)*(latdeg - 30)/(45 - 30);
dP = -3.75 + (-2.25 + 3.75)*(latdeg - 30)/(45 - 30);
T0 = 294.15 + (283.15 - 294.15)*(latdeg - 30)/(45 - 30);
dT = 7 + (11 - 7) *(latdeg - 30)/(45 - 30);
b0 = 6.05e-3 + (5.58e-3 - 6.05e-3)*(latdeg - 30)/(45 - 30);
db = 0.25e-3 + (0.32e-3 - 0.25e-3)*(latdeg - 30)/(45 - 30);
e0 = 21.79 + (11.66 - 21.79)*(latdeg - 30)/(45 - 30);
de = 8.85 + (7.24 - 8.85)*(latdeg - 30)/(45 - 30);
l0 = 3.15 + (2.57-3.15)*(latdeg - 30)/(45 - 30);
dl = 0.33 + (0.46 - 0.33)*(latdeg - 30)/(45 - 30);

double D = 182.0; // day of year
double Dmin;
if (latdeg > 0)
	Dmin = 28.0;
else
	Dmin = 211.0;
P = P0 - dP*cos(2*PI*(D-Dmin)/365.25);
T = T0 - dT*cos(2*PI*(D-Dmin)/365.25);
e = e0 - de*cos(2*PI*(D-Dmin)/365.25);
beta = b0 - db*cos(2*PI*(D-Dmin)/365.25);
lambda = l0 - dl*cos(2*PI*(D-Dmin)/365.25);

double zhyd, zwet;
double dhyd, dwet;

zhyd = 1e-6*k1*Rd*P/gm;
zwet = 1e-6*k2*Rd*e/((gm*(lambda+1)-beta*Rd)*T);
Position bp = GNSS::getInstance()->GetBestpos();
dhyd = pow((1 - beta*bp.height/T),g/(Rd*beta))*zhyd;
dwet = pow((1 - beta*bp.height/T),(lambda+1)*g/(Rd*beta)-1)*zwet;

double mEl;
mEl = 1.001/sqrt(0.002001 + sin(elRad)*sin(elRad));

double val;
val = (dhyd + dwet)*mEl;

svTropoErr_[i] = val;
//printf("%2i %f %f\n", i, svTropoErr_[i], val);
//svTropoErr_[i] = 2.312/sin(sqrt(elRad * elRad  + 0.001904))
//						+ 0.084/sin(sqrt(elRad * elRad + 0.6854 * 0.001));

}

void GnssCore::CalcCarrierSmoothedCode(unsigned char i){
	double code, phase;
	uint8_t n1, n2;
	unsigned char SMOOTH_LENGTH1 = SMOOTH_LENGTH_1;
	unsigned char SMOOTH_LENGTH2 = SMOOTH_LENGTH_2;
	double temp;
	double csc1, csc2;
	double phase_prev, csc1_prev, csc2_prev;
	float lambda;

	code 			= mCurrentCode_[i];
	phase 			= mCurrentPhase_[i];
	phase_prev 	= mPreviousPhase_[i];
	csc1_prev 		= mPreviousCSC1_[i];
	csc2_prev 		= mPreviousCSC2_[i];
	lambda 		= mWaveLength_[i];

	n1 = mCountOnCSC1_[i];
	n2 = mCountOnCSC2_[i];

	// 100s Smoothing
	if (n1 == 1){
		csc1 = code;
		n1++;
	} else {
		n1 = min(n1, SMOOTH_LENGTH1);

		temp = (phase - phase_prev) * lambda;
		csc1 = code/n1 + (csc1_prev - temp) * (n1-1)/n1;
		if (n1 < SMOOTH_LENGTH_1)
			n1++;
	}
	mCountOnCSC1_[i] = n1;
	mCurrentCSC1_[i] = csc1;

	// 30s Smoothing
	if (n2 == 1){
		csc2 = code;
		n2++;
	} else {
		n2 = min(n2, SMOOTH_LENGTH2);
		temp = (phase - phase_prev) * lambda;
		csc2 = code/n2 + (csc2_prev - temp) * (n2-1)/n2;
		if (n2 < SMOOTH_LENGTH2)
			n2++;
	}
	mCountOnCSC2_[i] = n2;
	mCurrentCSC2_[i] = csc2;
} // function: EstimateCSC

void GnssCore::CalcLeastSquaredPosition() {

	double Norm_dX = 100;

	CheckConstellation();
//	if (~CheckConstellation) // Set constellation mode
//			return;

	MatrixXd W = SetErrCovariance();
	VectorXd drho(numSvForPos_, 1);
	VectorXd dx(3+numConstForPos_, 1);

//	// Calculate Position
	int iter = 0;
	while(Norm_dX > 0.001) {
		iter++;
		MatrixXd G = SetGeomMatrix();
		MatrixXd S = (G.transpose()*W*G).inverse() * G.transpose() * W;
		VectorXd drho = SetResidualVector();

		dx = S * drho;

		usrPositionECEF_[0] = usrPositionECEF_[0] + dx(0);
		usrPositionECEF_[1] = usrPositionECEF_[1] + dx(1);
		usrPositionECEF_[2] = usrPositionECEF_[2] + dx(2);
		switch (constMode_) {
		case GPS:
			usrGpsClockOffset_ = usrGpsClockOffset_ + dx(3);
			break;
		case GLO:
			usrGloClockOffset_ = usrGloClockOffset_ + dx(3);
			break;
		case GAL:
			usrGalClockOffset_ = usrGalClockOffset_ + dx(3);
			break;
		case BDS:
			usrBdsClockOffset_ = usrBdsClockOffset_ + dx(3);
			break;
		case GPS_GLO:
			usrGpsClockOffset_ = usrGpsClockOffset_ + dx(3);
			usrGloClockOffset_ = usrGloClockOffset_ + dx(4);
			break;
		case GPS_GAL:
			usrGpsClockOffset_ = usrGpsClockOffset_ + dx(3);
			usrGalClockOffset_ = usrGalClockOffset_ + dx(4);
			break;
		case GPS_BDS:
			usrGpsClockOffset_ = usrGpsClockOffset_ + dx(3);
			usrBdsClockOffset_ = usrBdsClockOffset_ + dx(4);
			break;
		case GLO_GAL:
			usrGloClockOffset_ = usrGloClockOffset_ + dx(3);
			usrGalClockOffset_ = usrGalClockOffset_ + dx(4);
			break;
		case GLO_BDS:
			usrGloClockOffset_ = usrGloClockOffset_ + dx(3);
			usrBdsClockOffset_ = usrBdsClockOffset_ + dx(4);
			break;
		case GAL_BDS:
			usrGalClockOffset_ = usrGalClockOffset_ + dx(3);
			usrBdsClockOffset_ = usrBdsClockOffset_ + dx(4);
			break;
		case GPS_GLO_GAL:
			usrGpsClockOffset_ = usrGpsClockOffset_ + dx(3);
			usrGloClockOffset_ = usrGloClockOffset_ + dx(4);
			usrGalClockOffset_ = usrGalClockOffset_ + dx(5);
			break;
		case GPS_GLO_BDS:
			usrGpsClockOffset_ = usrGpsClockOffset_ + dx(3);
			usrGloClockOffset_ = usrGloClockOffset_ + dx(4);
			usrBdsClockOffset_ = usrBdsClockOffset_ + dx(5);
			break;
		case GPS_GAL_BDS:
			usrGpsClockOffset_ = usrGpsClockOffset_ + dx(3);
			usrGalClockOffset_ = usrGalClockOffset_ + dx(4);
			usrBdsClockOffset_ = usrBdsClockOffset_ + dx(5);
			break;
		case GLO_GAL_BDS:
			usrGloClockOffset_ = usrGloClockOffset_ + dx(3);
			usrGalClockOffset_ = usrGalClockOffset_ + dx(4);
			usrBdsClockOffset_ = usrBdsClockOffset_ + dx(5);
			break;
		case GPS_GLO_GAL_BDS:
			usrGpsClockOffset_ = usrGpsClockOffset_ + dx(3);
			usrGloClockOffset_ = usrGloClockOffset_ + dx(4);
			usrGalClockOffset_ = usrGalClockOffset_ + dx(5);
			usrBdsClockOffset_ = usrBdsClockOffset_ + dx(6);
			break;
		default:
			break;
		}

		//
		Norm_dX = sqrt(dx(0)*dx(0) + dx(1)*dx(1) + dx(2)*dx(2));
	}
	double err[3];

	Vector3d xyz;
	xyz(0) = usrPositionECEF_[0];
	xyz(1) = usrPositionECEF_[1];
	xyz(2) = usrPositionECEF_[2];
	Vector3d enu = ConvertECEF2ENU(xyz, truelat * D2R, truelon * D2R, truealt);

	xyz(0) = usrBestposECEF_[0];
	xyz(1) = usrBestposECEF_[1];
	xyz(2) = usrBestposECEF_[2];
	Vector3d enu_bp = ConvertECEF2ENU(xyz, truelat * D2R, truelon * D2R, truealt);

	Position bp = GNSS::getInstance()->GetBestpos();
	err[0] = usrPositionECEF_[0] - usrBestposECEF_[0];
	err[1] = usrPositionECEF_[1] - usrBestposECEF_[1];
	err[2] = usrPositionECEF_[2] - usrBestposECEF_[2];

	printf("\n");
//	printf("# of Sv :: %2i   || # of Const. :: %2i \n", numSvForPos_, numConstForPos_);
	printf("ESTIMATE:: %10.2f %10.2f %10.2f  --> err %6.3f %6.3f %6.3f (%2i)\n",
			usrPositionECEF_[0], usrPositionECEF_[1], usrPositionECEF_[2],
			enu(0), enu(1), enu(2), numSvForPos_);
	printf("BESTPOS :: %10.2f %10.2f %10.2f  --> err %6.3f %6.3f %6.3f (%2i)\n",
			usrBestposECEF_[0], usrBestposECEF_[1], usrBestposECEF_[2],
			enu_bp(0), enu_bp(1), enu_bp(2), bp.number_of_satellites_in_solution);
//	printf(" Error  :: %10.2f %10.2f %10.2f\n",err[0],err[1],err[2]);

	FILE *save_file;
	char SaveFileName[50];
	sprintf(SaveFileName, "positionerr.txt");

	save_file = fopen(SaveFileName, "ab");
	fprintf(save_file, "%f,%f,%f,%f,%f,%f,%f\n",
			timeCurrent_,enu(0),enu(1),enu(2),enu_bp(0),enu_bp(1),enu_bp(2));
	fclose (save_file);

} // function: EstimateUsrPos

MatrixXd GnssCore::SetErrCovariance(){

	double xair = 0.2;
	double tauair = 100.0;
	double vair = 0;
	double dI = 0.004;
	double sigPr, Fpp, sigIono, sigAir, sigTropo, sigTotal;

	MatrixXd W = MatrixXd::Constant(numSvForPos_, numSvForPos_, 0);

	uint8_t idxSv = 0;
	for (int i = 0; i < NUMBER_OF_SATELLITES; i++){
		if (isGoodForPos_[i]) {
//			sigPr = 0.16 + 1.07 * exp(-svElRad_[i] / 15.5);
//
//			Fpp = 1 / sqrt(1 - ((EARTH_RADIUS/1000 * cos(svElRad_[i]) / (EARTH_RADIUS + IONO_LAYER_HEIGHT)/1000)*(EARTH_RADIUS/1000 * cos(svElRad_[i]) / (EARTH_RADIUS + IONO_LAYER_HEIGHT)/1000)));
//			sigIono = dI * (xair + 2 * tauair * vair) * Fpp;
//
//			sigAir = 0.16 + 1.07 * exp(-svElRad_[i] / 15.5);
//
//			sigTropo = 0.12 * 1.001/sqrt(0.002001 + sin(svElRad_[i])*sin(svElRad_[i]));
////			sigTropo = 0;
//			sigTotal= sigPr*sigPr + sigIono*sigIono + sigAir*sigAir + sigTropo*sigTropo;

			sigTotal = 1;
			W(idxSv, idxSv) = sigTotal;
			idxSv++;
		}
	}
	return W;
}

MatrixXd GnssCore::SetGeomMatrix(){

	MatrixXd G = MatrixXd::Constant(numSvForPos_,3+numConstForPos_,0);

	double dx, dy, dz, r;
	uint8_t idxSv = 0;
	uint8_t idxConst = 0;

	for (int i = 0; i < NUMBER_OF_SATELLITES; i++){
		if (isGoodForPos_[i]) {

			dx = svPosition_[i][0] - usrPositionECEF_[0];
			dy = svPosition_[i][1] - usrPositionECEF_[1];
			dz = svPosition_[i][2] - usrPositionECEF_[2];
			r = sqrt(dx*dx + dy*dy + dz*dz);

			G(idxSv, 0) = -dx/r;
			G(idxSv, 1) = -dy/r;
			G(idxSv, 2) = -dz/r;

			switch (GetConstFlag(i)) {
			case FLAG_GPS:
				idxConst = 1;
				break;
			case FLAG_GLO:
				idxConst = 2;
				if (constMode_ == GLO || constMode_ == GLO_GAL ||
						constMode_ == GLO_BDS || constMode_ == GLO_GAL_BDS)
					idxConst = 1;
				break;
			case FLAG_GAL:
				idxConst = 3;
				if (constMode_ == GAL || constMode_ == GAL_BDS)
					idxConst = 1;
				else if (constMode_ == GPS_GAL || constMode_ == GLO_GAL ||
						constMode_ == GPS_GAL_BDS || constMode_ == GLO_GAL_BDS)
					idxConst = 2;
				break;
			case FLAG_BDS:
				idxConst = 4;
				if (constMode_ == BDS)
					idxConst = 1;
				else if (constMode_ == GPS_BDS || constMode_ == GLO_BDS || constMode_ == GAL_BDS)
					idxConst = 2;
				else if (constMode_ == GPS_GLO_BDS || constMode_ == GPS_GAL_BDS ||	constMode_ == GLO_GAL_BDS)
					idxConst = 3;
				break;
			default:
				break;
			}
			G(idxSv, 2 + idxConst) = 1;
			idxSv++;
		}
	}
	return G;
} // function: SetGeomMatrix

VectorXd GnssCore::SetResidualVector(){

	double rangeMeasurement, usrClockOffset;
	double dx, dy, dz, r;
	VectorXd drho = VectorXd::Constant(numSvForPos_,1,0);

	uint8_t idxSv = 0;
	for (int i = 0; i < NUMBER_OF_SATELLITES; i++){
		if (isGoodForPos_[i]){

			dx = svPosition_[i][0] - usrPositionECEF_[0];
			dy = svPosition_[i][1] - usrPositionECEF_[1];
			dz = svPosition_[i][2] - usrPositionECEF_[2];
			r = sqrt(dx*dx + dy*dy + dz*dz);

			if (USE_SMOOTHING)
				rangeMeasurement = mCurrentCSC1_[i];
			else
				rangeMeasurement = mCurrentCode_[i];

//			printf("%i %f\n", i, rangeMeasurement);
			switch (GetConstFlag(i)){
			case FLAG_GPS:
				usrClockOffset = usrGpsClockOffset_;
				break;
			case FLAG_GLO:
				usrClockOffset = usrGloClockOffset_;
				break;
			case FLAG_GAL:
				usrClockOffset = usrGalClockOffset_;
				break;
			case FLAG_BDS:
				usrClockOffset = usrBdsClockOffset_;
				break;
			default:
				cout << "INVALID CONSTELLATION FLAG "<< endl;
				break;
			}
			drho(idxSv) = rangeMeasurement + SPEED_OF_LIGHT*svClockOffset_[i]
							- svTropoErr_[i] - svIonoErr_[i] - (r + usrClockOffset);
			idxSv++;

		}

	}
	return drho;
} // function: SetResidualVector

void GnssCore::ClearGnssFilter(){

	// Save Current Status
	timePrevious_ = timeCurrent_;
	for (uint8_t i = 0; i < NUMBER_OF_SATELLITES; i++){
		if (CheckSvForProcess(i)){
			mPreviousCode_[i] 		= mCurrentCode_[i];
			mPreviousPhase_[i] 		= mCurrentPhase_[i];
			mPreviousLockTime_[i] 	= mCurrentLockTime_[i];

			mPreviousCSC1_[i] 		= mCurrentCSC1_[i];
			mPreviousCSC2_[i] 		= mCurrentCSC2_[i];
		} else {
			mPreviousCode_[i]  		= 0;
			mPreviousPhase_[i] 		= 0;
			mPreviousLockTime_[i] 	= 0;

			mPreviousCSC1_[i]			= 0;
			mPreviousCSC2_[i] 		= 0;

			mCountOnCSC1_[i]			= 1;
			mCountOnCSC2_[i]			= 1;
//			mCurrentCSC1_[i] 			= 0;
		}
	}

	// Clear variables
	numSvForPos_ = 0;
	numConstForPos_ = 0;
	constMode_ = 0;
	gpsCapable_ = false;
	gloCapable_ = false;
	galCapable_ = false;
	bdsCapable_ = false;

	RangeData obsRemoval = {0,};
	for (uint8_t i = 0; i < NUMBER_OF_SATELLITES; i++){
		mCurrentCode_[i] = 0;
		mCurrentPhase_[i] = 0;
		mCurrentLockTime_[i] = 0;
		mCN0_[i] = 0;

		isMeasurementOn_[i] = false;
		isGoodForPos_[i] = false;

		switch (GetConstFlag(i)) {
		case FLAG_GPS:
			obsGpsL1_[i] = obsRemoval;
			obsGpsL2_[i] = obsRemoval;
			break;
		case FLAG_GLO:
			obsGloL1_[i-INDEX_GLO_MIN] = obsRemoval;
			obsGloL2_[i-INDEX_GLO_MIN] = obsRemoval;
			break;
		case FLAG_GAL:
			obsGalE5a_[i-INDEX_GAL_MIN] = obsRemoval;
			break;
		case FLAG_BDS:
			obsBdsB1I_[i-INDEX_BDS_MIN] = obsRemoval;
			break;
		default:
			break;
		}

		svAzRad_[i] = 0;
		svElRad_[i] = 0;

		timeTx_[i] = 0;
		timeTxRaw_[i] = 0;
		timeTransit_[i] = 0;

		svClockOffset_[i] = 0;
		svClockDrift_[i] = 0;
		for (uint8_t j = 0; j < 3; j++){
			svPosition_[i][j] = 0;
			svVelocity_[i][j] = 0;
		}

		svIonoErr_[i] = 0;
		svTropoErr_[i] = 0;
	}
}

bool GnssCore::CheckConstellation(){

	for (int i = 0; i<NUMBER_OF_SATELLITES; i++){
		switch (GetConstFlag(i)){
		case FLAG_GPS:
			if (isGoodForPos_[i])
				gpsCapable_ = true;
			break;
		case FLAG_GLO:
			if (isGoodForPos_[i])
				gloCapable_ = true;
			break;
		case FLAG_GAL:
			if (isGoodForPos_[i])
				galCapable_ = true;
			break;
		case FLAG_BDS:
			if (isGoodForPos_[i])
				bdsCapable_ = true;
			break;
		}
	}
	constMode_ = gpsCapable_ + 2*gloCapable_ + 4*galCapable_ + 8*bdsCapable_;

	numConstForPos_ = gpsCapable_ + gloCapable_ + galCapable_ + bdsCapable_;

	if (numConstForPos_ > 0)
		return true;
	else
		return false;
}

