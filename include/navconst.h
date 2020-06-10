/*
 * navconst.h
 *
 *  Created on: 2019. 6. 12.
 *      Author: gnss
 */

#ifndef INCLUDE_NAVCONST_H_
#define INCLUDE_NAVCONST_H_

/* about Earth - WGS84 */
#define Mu_WGS84				(3.986005e14)		/* GM */
#define OmegaDot_WGS84		(7.2921151467e-5)	/* earth angular rate */
#define A_WGS84				(6378137.0)		/* radius(meters) 	*/
#define Ecc_WGS84				(0.08181919)		/* eccentricity 	*/
#define Flat_WGS84			(1/298.257223563) // Flattening
#define E2_Earth 				((2-Flat_WGS84)*Flat_WGS84)

/* about Earth - PZ-90 */
#define A_PZ90 				(6378136)			// Radius of Earth
#define Mu_PZ90				(3.9860044e14)	// Gravitational constant * mass of Earth
#define OmegaDot_PZ90		(7.292115e-5)		// earth angular rate
#define Flat_PZ90				(1/298.257222101)	// Flattening
#define J2_PZ90				(1.0826257e-3)	// second zonal harmonic of the geopotential
//#define Ecc_PZ90				(sqrt(1 - (1-Flat_PZ90)))

/* GNSS */
#define Altitude_Ionosphere	(350000.0)			/* meters	*/
#define SPEED_OF_LIGHT		(2.99792458e8) 	// m/s
#define Freq_L1_GPS			(1.57542e9) 		// Hz
#define Wave_L1_GPS			(SPEED_OF_LIGHT/Freq_L1)	/*0.1903*/
#define F_relative			(-4.442807633e-10)

#define Freq_L1_GLO			(1.602e9)
#define FreqStepGLO			(0.5625e6)

/* Math */
#define WEEK_SECOND			(604800)
#define HALFWEEK_SECOND		(302400)
#define PI						(3.1415926535898)
#define r2d					(180.0/PI)
#define d2r					(PI/180.0)

#endif /* INCLUDE_NAVCONST_H_ */
