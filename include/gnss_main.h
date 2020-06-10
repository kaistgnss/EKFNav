#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/thread.hpp>
#include <signal.h>
#include <time.h>
#include <sys/stat.h>
#include <stdio.h>

#include "novatel/novatel.h"
#include "mavlink/tmtc_ladgnss.h"

using namespace novatel;
using namespace std;

Novatel my_gps;

int mode;
char start_time[50];
char LogDirName[50];

static int start2 = 0;
int Save_Mode;
int Log_Mode;
int Fault_Mode;

SerialPort SerialPort_GPS;

void Reading_GNSS() {

	mode=1;
	// Save start time
	struct tm *t;
	time_t timer;
	timer = time(NULL);
	t = localtime(&timer);
	sprintf(start_time, "%02d%02d%02d", t->tm_year-100, t->tm_mon+1, t->tm_mday);

	// Serial port configuration
	strcpy(SerialPort_GPS.uartName, 	"/dev/ttyUSB2"); // novatel
	//strcpy(SerialPort_GPS.uartName, 	"/dev/udmabuf");
	//strcpy(SerialPort_Modem.uartName, 	"/dev/ttyModem"); // mavlink
	SerialPort_GPS.baudRate = 115200; // novatel

	Log_Mode = 1; // Log Mode on;

	bool result = my_gps.Connect(SerialPort_GPS.uartName, SerialPort_GPS.baudRate, 1);

	if (result) {
		cout << "port 1 Successfully connected." << endl;
	} else {
		cout << "port 1 Failed to connect." << endl;
		return;
	}

	if (result) {
		cout << "port 1 Successfully reset." << endl;
	} else {
		cout << "port 1 Failed to reset." << endl;
		return;
	}

	if (start2 == 0){

		if ((mode == 1)) {

//			my_gps.SendCommand("sbascontrol enable AUTO", true);
			my_gps.SendCommand("clockadjust enable", true);

			my_gps.SendCommand("dynamics air", true);
			my_gps.SendCommand("ecutoff 0.0", true);
			my_gps.SendCommand("csmooth 2", true);

			// required Logs for recording RINEX format
			my_gps.ConfigureLogs("versionb once");
			my_gps.ConfigureLogs("markposb once");
			//my_gps.ConfigureLogs("ionutcb once");
			// required Logs for Data process
			my_gps.ConfigureLogs("timeb ontime 0.5");
			my_gps.ConfigureLogs("bestposb ontime 0.5");
			//my_gps.ConfigureLogs("bestvelb ontime 0.5");
			my_gps.ConfigureLogs("rangeb ontime 0.5");
			my_gps.ConfigureLogs("gphdtdualantenna ontime 0.2");
			my_gps.ConfigureLogs("bestxyzb ontime 0.5");
			my_gps.ConfigureLogs("gpsephemb onchanged");
			my_gps.ConfigureLogs("gloephemerisb onchanged");
			my_gps.ConfigureLogs("ionutcb onchanged");
			start2 = 1;
		}

	}
}
