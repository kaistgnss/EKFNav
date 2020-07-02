/*
 * config.h
 *
 *  Created on: 2017. 6. 22.
 *      Author: Dongwoo Kim
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define FALSE 0
#define TRUE  1

// Macro function
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

//Enable them (set as 1) for the SIGNA operation; 0 for disabling
// MQM //
#define kEnableSnrMonitor 			1
#define kEnableInnovationMonitor	1
#define kEnableDivergenceMonitor	1
#define kEnableAccRampStepMonitor	1
// DQM //
#define kEnableDqmMonitor			1
#define kEnableYeteMonitor			1
//
#define kEnableBprMonitor			1
#define kEnableBadrMonitor			1
#define kEnableSigmaMeanMonitor		1
#define kEnablePRcorrMonitor		1
#define kEnableRRcorrMonitor		1

// Constellation
#define MaskAngle	(5)
#define FlagGPS 0
#define FlagGLO 1
#define FlagGPSGLO (2)
#define ConstellationMode (2)
// 0 : GPS only
// 1 : GLO only
// 2 : GPS/GLO
#define StandAlone (0)
#define DGPS (1)

//============================================================================
// Constants setting
//============================================================================
#define PRN_GPS_min				(1)
#define PRN_GPS_max				(32)
#define PRN_GLO_min				(33)
#define PRN_GLO_max				(56)

#define kNumSatellite		(56)	//the number of satellites
#define kNumGPS 				(32)
#define kNumGLO				(24)

#define kTs						(0.5)					//[second]
#define kFs						(1.0/kTs)				//=2.0; Sampling frequency [Hz]
#define kSmoothLen				(200)					//200 points/epoches
#define kSmoothLen2				(60)
#define kSmoothFilterLenInSec	(kSmoothLen*kTs)	//=100 sec
#define kSmoothFilterLenInSec2	(kSmoothLen2*kTs)	//=30 sec
#define kTwoFilterLenth			(2*kSmoothLen)   //=400pnts = 200sec
#define kTwoFilterLenth2			(2*kSmoothLen2)   //=60pnts = 30sec

#endif /* CONFIG_H_ */
