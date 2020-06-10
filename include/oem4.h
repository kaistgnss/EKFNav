/***********************************************************************
 *
 *       Filename:   NOem4Hdr.h (original name of the file,
 *                               now it is named oem4.h - Ludde)
 *       Company:    (c) NovAtel Inc.
 *                   1120 68th Avenue N.E.
 *                   Calgary, Alberta.
 *                   Canada. T2E 8S5
 *
 *----------------------------------------------------------------------
 *
 *       COPYRIGHT NovAtel Inc., all rights reserved.
 *
 *       No part of this software may be reproduced or modified in any
 *       form or by any means - electronic, mechanical, photocopying,
 *       recording, or otherwise - without the prior written consent of
 *       NovAtel Inc. and author(s).
 *
 *----------------------------------------------------------------------
 *
 *       Uses NOT Permitted ( You may NOT )
 *          A. Provide use of this software in a computer service 
 *             business with the purpose of making a profit by direct 
 *             use of this software.
 *          B. Grant sublicense, lease, or other rights in this software
 *             to others.
 *
 *       Use Permitted ( You may )
 *          A. Modify and use this software freely in the development 
 *             of your software in order to communicate to the NovAtel 
 *             GPSCard and process the data received either in realtime 
 *             or post-processing mode.
 *
 *       IN NO EVENT WILL NOVATEL BE LIABLE FOR ANY INDIRECT,INCIDENTAL,
 *       SPECIAL OR CONSEQUENTIAL DAMAGES WHETHER THROUGH TORT, CONTRACT
 *       OR OTHERWISE. THIS SOFTWARE IS SUPPLIED ON AN 'AS IS' BASIS.
 *
 **********************************************************************/
#ifndef _OEM4_H
#define _OEM4_H
#include <stdio.h> // for printf
#include <string.h> // for memset
//#include "imt_msg.h"

/* CRC c-code taken form OEM4 manual 2 */
#define CRC32_POLYNOMIAL   0xEDB88320L


typedef struct OEM4_BINARY_HEADER        // Standard binary header
{
  unsigned int               pc_sec1;     /* PC seconds, from gettimeofday(), not GPS time */
  unsigned int               pc_usec1;    /* PC microseconds, from gettimeofday() !GPS time*/
  unsigned int               pc_sec2;     /* PC seconds, from gettimeofday(), not GPS time */
  unsigned int               pc_usec2;    /* PC microseconds, from gettimeofday() !GPS time*/
  unsigned char          sop1;          // start of packet first byte
  unsigned char          sop2;          // start of packet second byte
  unsigned char          sop3;          // start of packet third  byte
  unsigned char          header_length; // Length of the header ( From start of packet )
  unsigned short         number;        // Message number
  unsigned char          type;          // Message type
  unsigned char          port_address;  // Address of the data port the log was received on
  unsigned short         length;        // Message length (Not including header or CRC)
  unsigned short         sequence;      // Sequence #
  unsigned char          idle;          // Idle time
  unsigned char          gps_stat;      // GPS Time Status 
  unsigned short         gps_week;      // GPS Week number
  unsigned int          millisecs;     // Milliseconds into week
  unsigned int          status;        // Receiver status word
  unsigned short         Reserved;      // 
  unsigned short         version;       // Receiver software version
//  unsigned int               pc_sec1;     /* PC seconds, from gettimeofday(), not GPS time */
//    unsigned int               pc_usec1;    /* PC microseconds, from gettimeofday() !GPS time*/
//    unsigned int               pc_sec2;     /* PC seconds, from gettimeofday(), not GPS time */
//    unsigned int               pc_usec2;    /* PC microseconds, from gettimeofday() !GPS time*/
//    unsigned char          sop1;          // start of packet first byte
//    unsigned char          sop2;          // start of packet second byte
//    unsigned char          sop3;          // start of packet third  byte
//    unsigned char          header_length; // Length of the header ( From start of packet )
//    unsigned short         number;        // Message number
//    unsigned char          type;          // Message type
//    unsigned char          port_address;  // Address of the data port the log was received on
//    unsigned short         length;        // Message length (Not including header or CRC)
//    unsigned short         sequence;      // Sequence #
//    unsigned char          idle;          // Idle time
//    unsigned char          gps_stat;      // GPS Time Status
//    unsigned short         gps_week;      // GPS Week number
//    unsigned int          millisecs;     // Milliseconds into week
//    unsigned int          status;        // Receiver status word
//    unsigned short         Reserved;      //
//    unsigned short         version;       // Receiver software version
} OEM4_BINARY_HEADER;

enum OEM4_TIME_STATUS /* modified from TIME_STATUS - Ludde */
{
   GPSTIME_UNKNOWN = 0,
   GPSTIME_USER,
   GPSTIME_USERADJUSTING,
   GPSTIME_COARSEADJUSTING = 80,
   GPSTIME_COARSE = 100,
   GPSTIME_COARSESTEERING = 120,
   GPSTIME_FINEADJUSTING = 140,
   GPSTIME_FINE = 160,
   GPSTIME_FINESTEERING = 180,
   GPSTIME_FREEWHEELING = 130,
   GPSTIME_SATTIME = 200
};

enum OEM4_PORT_NAME   /* modified from PORT_NAME - Ludde */
{
   NO_PORTS = 0,
   COM1_ALL = 1,
   COM2_ALL = 2,
   COM3_ALL = 3,
   USB_ALL = 4,
   THISPORT_ALL = 6,
   ALLPORTS = 8,
   COM1 = 32,
   COM1_1,
   COM1_2,
   COM1_3,
   COM1_4,
   COM1_5,
   COM1_6,
   COM1_7,
   COM1_8,
   COM1_9,
   COM1_10,
   COM1_11,
   COM1_12,
   COM1_13,
   COM1_14,
   COM1_15,
   COM1_16,
   COM1_17,
   COM1_18,
   COM1_19,
   COM1_20,
   COM1_21,
   COM1_22,
   COM1_23,
   COM1_24,
   COM1_25,
   COM1_26,
   COM1_27,
   COM1_28,
   COM1_29,
   COM1_30,
   COM1_31,
   COM2 = 64,
   COM2_1,
   COM2_2,
   COM2_3,
   COM2_4,
   COM2_5,
   COM2_6,
   COM2_7,
   COM2_8,
   COM2_9,
   COM2_10,
   COM2_11,
   COM2_12,
   COM2_13,
   COM2_14,
   COM2_15,
   COM2_16,
   COM2_17,
   COM2_18,
   COM2_19,
   COM2_20,
   COM2_21,
   COM2_22,
   COM2_23,
   COM2_24,
   COM2_25,
   COM2_26,
   COM2_27,
   COM2_28,
   COM2_29,
   COM2_30,
   COM2_31,
   COM3 = 96,
   COM3_1,
   COM3_2,
   COM3_3,
   COM3_4,
   COM3_5,
   COM3_6,
   COM3_7,
   COM3_8,
   COM3_9,
   COM3_10,
   COM3_11,
   COM3_12,
   COM3_13,
   COM3_14,
   COM3_15,
   COM3_16,
   COM3_17,
   COM3_18,
   COM3_19,
   COM3_20,
   COM3_21,
   COM3_22,
   COM3_23,
   COM3_24,
   COM3_25,
   COM3_26,
   COM3_27,
   COM3_28,
   COM3_29,
   COM3_30,
   COM3_31,
   USB = 128,
   USB_1,
   USB_2,
   USB_3,
   USB_4,
   USB_5,
   USB_6,
   USB_7,
   USB_8,
   USB_9,
   USB_10,
   USB_11,
   USB_12,
   USB_13,
   USB_14,
   USB_15,
   USB_16,
   USB_17,
   USB_18,
   USB_19,
   USB_20,
   USB_21,
   USB_22,
   USB_23,
   USB_24,
   USB_25,
   USB_26,
   USB_27,
   USB_28,
   USB_29,
   USB_30,
   USB_31,
   //THISPORT = 192,
   THISPORT_1, 
   THISPORT_2, 
   THISPORT_3, 
   THISPORT_4, 
   THISPORT_5, 
   THISPORT_6, 
   THISPORT_7, 
   THISPORT_8, 
   THISPORT_9, 
   THISPORT_10,
   THISPORT_11,
   THISPORT_12,
   THISPORT_13,
   THISPORT_14,
   THISPORT_15,
   THISPORT_16,
   THISPORT_17,
   THISPORT_18,
   THISPORT_19,
   THISPORT_20,
   THISPORT_21,
   THISPORT_22,
   THISPORT_23,
   THISPORT_24,
   THISPORT_25,
   THISPORT_26,
   THISPORT_27,
   THISPORT_28,
   THISPORT_29,
   THISPORT_30,
   THISPORT_31,
   MAX_PORT
};


/* Code added after this is NOT created by Novatel 
 * Created by: Per-Ludvig Normark <ludde@relgyro.stanford.edu>
 *             Stanford University, GPS Laboratory
 *             Jan 2001
*/

/* hex */
#define  OEM4_SOP1  0xaa     /* start of binary log first  byte     */
#define  OEM4_SOP2  0x44     /* start of binary log second byte     */
#define  OEM4_SOP3  0x12     /* OEM4 specific, OEM3 = ox11 
			      * start of binary log third  byte     */

#define OEM4_LARGEST_MSG_SIZE 3200 //sizeof(BINARY_BUFFER)
#define OEM4_HEADER_SIZE 28   //sizeof(OEM4_BINARY_HEADER) /* XX */

#define  OEM4_RX_CR '\r'      /* termintaing char for each cmd to send to the rx */
#define  OEM4_RX_COM_REPLY "[COM"
#define  OEM4_RX_COM1_REPLY "[COM1]"
#define  OEM4_RX_COM2_REPLY "[COM2]"
#define  OEM4_RX_COM3_REPLY "[COM3]"
#define  OEM4_RX_COM_ERROR  "<ERROR"
#define  OEM4_RX_REPLY_SIZE 7 /* the size of the reply */

//#define OEM4_MAX_CHANNELS  12 /* the nr of parallell channels in the OEM4 Rx */
#define OEM4_MAX_CHANNELS  50 /* the nr of parallell channels in the OEM4 Rx */
#define OEM4_MAX_SV        50 /* max nr of sv to retrive almanac from */

/* OEM4 general binary message type */
typedef struct {
  OEM4_BINARY_HEADER header;
  unsigned char data[3200]; /* big enough for worst case? ALMANAC 3172+4(CRC)bytes*/
} OEM4_BINARY_MSG;

/* == OEM4 binary RANGE message type ========================================== */
/* tracking status of each channel (1 byte) */
typedef struct {
  unsigned int tracking_state     : 5; /* L1/L2:Wide/Narrow Phase Lock etc.         */
  unsigned int sv_channel_nr      : 5; /* which channel on the gps card             */
  unsigned int phase_lock         : 1; /* 0 not locked, 1 locked                    */
  unsigned int parity_known       : 1; /* 0 not known,  1 known                     */
  unsigned int code_lock          : 1; /* 0 not locked, 1 locked                    */
  unsigned int correlator_spacing : 3; /* 0,3 Reserved, 1 standard, 2 narrow, 4 PAC */
  unsigned int satellite_system   : 3; /* 0 GPS 1-7 Reserved                        */
  unsigned int filler1            : 1; /* reserved bit                              */
  unsigned int grouping           : 1; /* 0 Not grouped, 1 grouped                  */
  unsigned int frequency          : 2; /* 0=L1, 1=L2, 2-3 reserved                  */
  unsigned int code_type          : 3; /* 0=C/A, 1=P, 2=P codeless, 3-7 reserved    */
  unsigned int filler2            : 1; /* reserved bit                              */
  unsigned int primary_L1_channel : 1; /* 0 Not primary, 1 Primary                  */
  unsigned int filler3            : 3; /* Reserved                                  */
  unsigned int channel_assignment : 1; /* 0 automatic, 1 forced                     */
} OEM4_RANGEB_CHANNEL_STATUS;

/* one channel measurements */
typedef struct {
	uint16_t   prn;      /* PRN nr of range measurements                     */
	uint16_t   filler;   /* reserved 2 bytes, not used                       */
	double     psr;      /* [m] Pseudo measurement [m]                       */
	float      psr_std;  /* [m] Pseudo measurement std deviation             */
	double     adr;      /* [cycles] Carrier phase (accumulated Doppler range)*/
	float      adr_std;  /* [cycles] Estimated carrier phase std deviation   */
	float      dopp;     /* [Hz] Doppler frequency                           */
	float      C_No;     /* [dB-Hz] Carrier to noise density ratio           */
	float      locktime; /* [s] Nr of continuos tracking (no cycle slip)     */
	OEM4_RANGEB_CHANNEL_STATUS status; /* Tracking status                          */
} OEM4_RANGEB_CHANNEL;

typedef struct {
  OEM4_BINARY_HEADER  header;/* OEM4 header */
  unsigned int       obs;   /* the nr of observables to follow (L1 and L2)     */
  OEM4_RANGEB_CHANNEL ch[2*OEM4_MAX_CHANNELS];  /* each ch measurement, L1/L2   */
  unsigned int       crc_filler;    /* the CRC 4 bytes (most likely not here)  */
} OEM4_RANGEB_MSG;


/* == OEM4 binary ALMANAC message type ======================================== */
/* one sv almanac data */
typedef struct {
  unsigned int prn;       /* PRN nr of range measurements                     */
  unsigned int week;      /* Almanac reference week nr                        */
  double        seconds;   /* [s] Almanac reference seconds into week          */
  double        ecc;       /* Eccentricity                                     */
  double        omega_dot; /* [rad] Rate of right ascension                    */
  double        omega_o;   /* [rad] Right ascension                            */
  double        w;         /* [rad] Argument of perigee                        */
  double        M0;        /* [rad] Mean anomaly of refernece time             */
  double        af0;       /* [sec] Clock aging parameter-constant term        */
  double        af1;       /* [sec/sec] Clock aging parameter-first order term */
  double        n;         /* [rad/sec] Corrected mean motion                  */
  double        a;         /* [m] Semi major axis                              */
  double        i0;        /* [rad] Angle of inclination relative to 0.3 Pi    */
  unsigned int sv_config;      /* satellite configuration                     */
  unsigned int health_prn;     /* health from subframe 4 or 5                 */
  unsigned int health_almanac; /* health from subframe almanac                */
  unsigned int antispoof;      /* Is antispoof on? 0 = False, 1 = True        */
} OEM4_ALMANACB_SV;

typedef struct {
  OEM4_BINARY_HEADER  header; /* OEM4 header                                    */
  unsigned int       msgs;   /* the nr of sv almanc messgaes to follow         */
  OEM4_ALMANACB_SV    sv[OEM4_MAX_SV]; /* each sv almanac measurement           */
  unsigned int       crc_filler;      /* the CRC 4 bytes (most likely not here)*/
} OEM4_ALMANACB_MSG;

/* == OEM4 binary EPHEMERIS message type ===================================== */
/* This message needs to be byte aligned (not Word aligned as GCC default does)
 * since the ephemeris is 30 bytes long (not divisible by 4). 
 * siezof( OEM4_EPHEMB_MSG) is 134 if __attribute__ ((packed)) is used, else 136 
 * (2 bytes padded) */
//typedef struct {
//                                     /* OEM4 header                            */
//  OEM4_BINARY_HEADER  header        __attribute__ ((packed)); /* (bytealigned) */
//                                     /* PRN number for satellite               */
//  unsigned int       prn           __attribute__ ((packed)); /* (bytealigned) */
//                                     /* Ephemeris reference week               */
//  unsigned int       ref_week      __attribute__ ((packed)); /* (bytealigned) */
//                                     /* Ephemeris reference seconds            */
//  unsigned int       ref_seconds   __attribute__ ((packed)); /* (bytealigned) */
//                                     /* Raw subframe 1 data (no parity)        */
//  char                subframe1[30] __attribute__ ((packed)); /* (bytealigned) */
//                                     /* Raw subframe 1 data (no parity)        */
//  char                subframe2[30] __attribute__ ((packed)); /* (bytealigned) */
//                                     /* Raw subframe 1 data (no parity)        */
//  char                subframe3[30] __attribute__ ((packed)); /* (bytealigned) */
//                                     /* the CRC 4 bytes (always placed here)   */
//  unsigned int       crc           __attribute__ ((packed)); /* (bytealigned) */
//} OEM4_EPHEMB_MSG;

typedef struct {
                                     /* OEM4 header                            */
  OEM4_BINARY_HEADER  header        __attribute__ ((packed)); /* (bytealigned) */
                                     /* PRN number for satellite               */
  unsigned int       prn           __attribute__ ((packed)); /* (bytealigned) */
                                     /* Ephemeris reference week               */
  unsigned int       ref_week      __attribute__ ((packed)); /* (bytealigned) */
                                     /* Ephemeris reference seconds            */
  unsigned int       ref_seconds   __attribute__ ((packed)); /* (bytealigned) */
                                     /* Raw subframe 1 data (no parity)        */
  char                subframe1[30] __attribute__ ((packed)); /* (bytealigned) */
                                     /* Raw subframe 1 data (no parity)        */
  char                subframe2[30] __attribute__ ((packed)); /* (bytealigned) */
                                     /* Raw subframe 1 data (no parity)        */
  char                subframe3[30] __attribute__ ((packed)); /* (bytealigned) */
                                     /* the CRC 4 bytes (always placed here)   */
  unsigned int       crc           __attribute__ ((packed)); /* (bytealigned) */
} OEM4_EPHEMB_MSG;

/* == OEM4 binary SQMDATA message type ======================================== */
/* see sqm.h in IMT directory for how these correlators are spaced for OEM4 firmware
 * version 1.200s48 - Ludde 26 Feb 2002 */
/* one channel measurements */
typedef struct {
  long            a1sum; /* [] Accumulation 1 for correlatorspacing                     */
  long            a2sum; /* [] Accumulation 2 for correlatorspacing                     */
  long            a3sum; /* [] Accumulation 3 for correlatorspacing                     */
  long            a4sum; /* [] Accumulation 4 for correlatorspacing                     */
  long            a5sum; /* [] Accumulation 5 for correlatorspacing                     */
  unsigned int   sync;  /* [] 1 correlators in sync with master channel, 0 not in sync */
} OEM4_SQMDATAB_CHANNEL;

typedef struct {
  OEM4_BINARY_HEADER  header; /* OEM4 header                                               */
  unsigned int       prn;    /* the PRN number beeing tracked                             */
  unsigned int       obs;    /* number of channels to follow                              */
  OEM4_SQMDATAB_CHANNEL ch[2];/* In-phase and/or Quadra-phase correlator spaced accumulations */
  long                crc_filler; /* the CRC 4 bytes (most likely not here)                */
} OEM4_SQMDATAB_MSG;


/* scan for synch bytes OBS! bytes_to_check = bytes_in_buf - HEADER_SIZE */
int OEM4_found_header(unsigned char *buf,int start_buf, 
		 int bytes_to_check,int *start_msg);

/* get the size of the data including the header. Must have a "full"
 * header (bytes_in_buf >= HEADER_SIZE */
unsigned int OEM4_get_msg_size(unsigned char *buf);
unsigned int OEM4_get_header_size(unsigned char *buf);

/* calculates the checksum and returns 1 if ok -1 if failed */
int OEM4_checksum_ok(unsigned char *msg_buf,unsigned int msg_size);

void OEM4_rx_status(unsigned int status);

/* re-packing functions added to change OEM4 format to IMT format */

//int OEM4_repack_RANGE_to_IMT_RANGE(OEM4_RANGEB_MSG *range, unsigned char *rgeb, int rx_nr);
//int OEM4_repack_RANGE_to_IMT_RANGE(OEM4_RANGEB_MSG *range, unsigned char *rgeb);
//
//int OEM4_repack_ALMANAC_to_IMT_ALMANAC(OEM4_ALMANACB_MSG *almanac,
//				       unsigned char *almb, unsigned int which_sv,int rx_nr);
//
//int OEM4_repack_EPHEMB_to_IMT_EPHM(OEM4_EPHEMB_MSG *ephem, unsigned char *repb,int rx_nr);
//
//int OEM4_repack_SQM_to_IMT_RANGE(OEM4_SQMDATAB_MSG *sqm, unsigned char *imt_sqm,int rx_nr);

//void IMT_MSG_update_length(IMT_MSG *msg);

#ifdef OEM4_CLKB_SATXYZ
/* (PROPAGATEDCLOCKMODEL and SATXYZ messages are not used any morein the IMT, 
 * but their datastructs are still defined here for any future use 
 * repacking fkn to imt message are not completed (repacks to OEM3 format) since
 * no IMT equivalent message type for CLOCK or SATXYZ is defined - Ludde 8-oct-2001 */

/* == OEM4 binary CLOCK message type ========================================= */
/* (PROPAGATEDCLOCKMODEL) */
typedef struct {
  OEM4_BINARY_HEADER  header;       /* OEM4 header                             */
  double              range_bias;   /* [m] range bias measurements             */
  double              drift;        /* [m/s] Rx clock drift, + faster clock, 
				     *  - slower local local receiver clock    */
  double              var;          /* [m2] Variance in range bias measurement */
  unsigned int       type;         /* Clock model type                        */
  unsigned int       crc;          /* CRC value (always here)                 */
} OEM4_CLOCKB_MSG;

/* == OEM4 binary SATXYZ message type ======================================== */
typedef struct {
  unsigned int       prn;          /* Satellite PRN number               */
  double              x;            /* [m] Satellite X coordinates (ECEF) */
  double              y;            /* [m] Satellite Y coordinates (ECEF) */
  double              z;            /* [m] Satellite Z coordinates (ECEF) */
  double              clk_corr;     /* [m] Satellite clock correction     */
  double              ion_corr;     /* [m] Ionospheric correction         */
  double              trop_corr;    /* [m] Troposheric correction         */
  double              filler1;      /* reserved                           */ 
  double              filler2;      /* reserved                           */ 
} OEM4_SATXYZB_SV;

typedef struct {
  OEM4_BINARY_HEADER  header;       /* OEM4 header                             */
  double              filler1;      /* reserved                                */
  unsigned int       obs;          /* Number of satellites with XYZ to follow */
  OEM4_SATXYZB_SV     sv[OEM4_MAX_SV];/* XYZ measurements                      */
  unsigned int       crc;          /* the CRC 4 bytes (most likely not here)  */
} OEM4_SATXYZB_MSG;

int OEM4_repack_CLOCK_to_IMT_CLOCK(OEM4_CLOCKB_MSG *clock, unsigned char *clkb);

int OEM4_repack_SATXYZ_to_IMT_SATXYZ(OEM4_SATXYZB_MSG *satxyz, unsigned char *svdb);
#endif








#endif





