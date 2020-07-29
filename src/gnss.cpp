#include "../include/novatel/gnss.h"

#include <cmath>
#include <valarray>
#include <fstream>
#include <iostream>
#include <sstream>

//#include "oem4.h"
//#include "signa.h"

using namespace std;
using namespace gnss;

/////////////////////////////////////////////////////
// includes for default time callback
#define WIN32_LEAN_AND_MEAN
#include "boost/date_time/posix_time/posix_time.hpp"
////////////////////////////////////////////////////

//***********************************************************//
//------------- External Declaration of C Functions -------------//
//extern "C" void SIGNA_core(OEM4_RANGEB_MSG *msg, NavEphData *eph);
//extern void EstimateUsrPos(NavSystem *NavSys, NavParameter *NavParam);
//------------------------------------------------------------------------//

//int ready_surveyedpos = 0;
//
void RunGnssThread(){
	SerialPort SerialPort_GPS;

	// Serial port configuration
	strcpy(SerialPort_GPS.uartName, "/dev/ttyUSB2"); // novatel
	SerialPort_GPS.baudRate = 115200; // novatel

	bool result = GNSS::getInstance()->Connect(SerialPort_GPS.uartName, SerialPort_GPS.baudRate, 1);

	if (result) {
		cout << "port 1 Successfully connected." << endl;
	} else {
		cout << "port 1 Failed to connect." << endl;
		return;
	}

	GNSS::getInstance()->SendCommand("clockadjust enable", true);
	GNSS::getInstance()->SendCommand("dynamics auto", true);
	GNSS::getInstance()->SendCommand("pdpfilter disable", true);
	GNSS::getInstance()->SendCommand("setionotype Klobuchar", true);

//	GNSS::getInstance()->SendCommand("setionotype AUTO", true);
	GNSS::getInstance()->SetSvElevationAngleCutoff(5.0, FLAG_GPS);
	GNSS::getInstance()->SetSvElevationAngleCutoff(5.0, FLAG_GLO);
	//GNSS::getInstance()->SetSvElevationAngleCutoff(5.0, FLAG_GAL);
	//GNSS::getInstance()->SetSvElevationAngleCutoff(5.0, FLAG_BDS);
	//GNSS::getInstance()->SetSvElevationAngleCutoff(5.0, FLAG_QZS);

	GNSS::getInstance()->SetSvElevationAngleCutoff(85.0, FLAG_NIC);

	// required Logs for recording RINEX format
//	my_gps.ConfigureLogs("versionb once");
//	my_gps.ConfigureLogs("markposb once");

//	 required Logs for Data process
	GNSS::getInstance()->ConfigureLogs("timeb ontime 0.5");
	GNSS::getInstance()->ConfigureLogs("bestposb ontime 0.5");
	GNSS::getInstance()->ConfigureLogs("bestvelb ontime 0.5");
	GNSS::getInstance()->ConfigureLogs("bestxyzb ontime 0.5");
	GNSS::getInstance()->ConfigureLogs("rangeb ontime 0.5");
	//GNSS::getInstance()->ConfigureLogs("dualantennaheadingb ontime 0.5");
	if (USE_GPS)
		GNSS::getInstance()->ConfigureLogs("gpsephemb onchanged");
	if (USE_GLO)
		GNSS::getInstance()->ConfigureLogs("gloephemerisb onchanged");
	if (USE_GAL) {
		GNSS::getInstance()->ConfigureLogs("galinavephemerisb onchanged");
//		GNSS::getInstance()->ConfigureLogs("galionb onchanged");
	}
	if (USE_BDS)
		GNSS::getInstance()->ConfigureLogs("bdsephemerisb onchanged");
	if (USE_QZS)
		GNSS::getInstance()->ConfigureLogs("qzssephemerisb onchanged");
	GNSS::getInstance()->ConfigureLogs("ionutcb onchanged");
	//	my_gps.ConfigureLogs("gphdtdualantenna ontime 0.2");

};

/* Singleton Constructor */
GNSS* GNSS::inst = nullptr;
GNSS* GNSS::getInstance(){
	if (inst == nullptr)
		inst = new GNSS();
	return inst;
}

void GNSS::ParseNMEA(unsigned char *message, size_t length) {
	char msg[length];
	memcpy(&msg, message, length);

	char buf[3+length] = "$GP";
	strcat(buf, msg);

	char NMEA_ID[3];
	memcpy(&NMEA_ID, message, 3);

	if (strcmp(NMEA_ID, "HDT") != -1) {

		char *ptr = strtok(buf, ",");
		ptr = strtok(NULL, ",");
//		GnssCore::gnssHeadingMeasurement_ = atof(ptr);
	} else if (strcmp(NMEA_ID, "GSA") != -1) {
//		char *ptr = strtok(buf, ",");
//		ptr = strtok(NULL, ",");
//		ptr = strtok(NULL, ",");
//		dualheading = atof(ptr);

	}
} // function : ParseNMEA

void GNSS::ParseBinary(unsigned char *message, size_t length, BINARY_LOG_TYPE message_id) {

	uint16_t payload_length;
	uint16_t header_length;
	uint16_t prn;

	char message_id_str[50];
	switch (message_id) {
	case TIMEB_LOG_TYPE :
		sprintf(message_id_str, "TIME");
		break;
	case RANGEB_LOG_TYPE :
		sprintf(message_id_str, "RANGE");
		break;
	case GPSEPHEMB_LOG_TYPE :
		sprintf(message_id_str, "GPS EPHEM");
		break;
	case GLOEPHEMB_LOG_TYPE :
		sprintf(message_id_str, "GLO EPHEM");
		break;
	case BESTPOSB_LOG_TYPE :
		sprintf(message_id_str, "BESTPOSB");
		break;
	case IONUTCB_LOG_TYPE :
		sprintf(message_id_str, "IONUTC");
		break;
	case BESTVELB_LOG_TYPE :
		sprintf(message_id_str, "BESTVELB");
		break;
	case BESTXYZB_LOG_TYPE :
		sprintf(message_id_str, "BESTXYZB");
		break;
	case DUALANTENNAHEADING_LOG_TYPE :
		sprintf(message_id_str, "DUALANTENNAHEADINGB");
		break;
	default :
		break;
	}

	// obtain the received crc
	switch (message_id) {
	case TIMEB_LOG_TYPE:
		memcpy(&msgTimeb_, message, sizeof(msgTimeb_));

		break;
	case BESTPOSB_LOG_TYPE:
		memcpy(&msgBestposb_, message, sizeof(msgBestposb_));
		timeBestposb_ = read_timestamp_;
		isBestposReady_ = true;
//		printf("%x %x\n", msgBestposb_.gal_bds_used_mask, msgBestposb_.gps_glo_used_mask);

//		if (best_position_callback_)
//			best_position_callback_(msgBESTPOSB_, read_timestamp_);
		break;

	case RANGEB_LOG_TYPE:

		header_length = (uint16_t) *(message + 3);
		payload_length = (((uint16_t) *(message + 9)) << 8)	+ ((uint16_t) *(message + 8));

		// Copy header and #observations following
		memcpy(&msgRangeb_, message, header_length + 4);

		//Copy repeated fields
		memcpy(&msgRangeb_.range_data, message + header_length + 4, (44 * msgRangeb_.number_of_observations));

		//Copy CRC
		memcpy(&msgRangeb_.crc, message + header_length + payload_length, 4);

		/* GNSS Data arrange*/
		unsigned int obsNumber;
		memcpy(&obsNumber,message + header_length, 4);

		uint16_t index;
		ChannelStatus channelStatus;
		for (int i = 0; i < obsNumber; i++){
			channelStatus = msgRangeb_.range_data[i].channel_status;

			prn = msgRangeb_.range_data[i].satellite_prn;

			switch (channelStatus.satellite_sys){
			case FLAG_GPS:
				/******************* GPS *****************/
				index = prn - PRNOFFSET_GPS;
				switch (channelStatus.signal_type){
				case GPS_L1CA:
					GnssCore::getInstance()->obsGpsL1_[index] = msgRangeb_.range_data[i];
					GnssCore::getInstance()->isMeasurementOn_[index] = true;
					break;
				case GPS_L2P_SEMI_CODELESS:
					GnssCore::getInstance()->obsGpsL2_[index] = msgRangeb_.range_data[i];
					break;
				default:
					break;
				}
				break;
				/******************* GPS *****************/
				/***************** GLONASS ***************/
			case FLAG_GLO:
				index = prn - PRNOFFSET_GLO;
				switch (channelStatus.signal_type){
				case GLO_L1CA:
					GnssCore::getInstance()->obsGloL1_[index] = msgRangeb_.range_data[i];
					GnssCore::getInstance()->isMeasurementOn_[prn - 6] = true;
					break;
				case GLO_L2P:
					GnssCore::getInstance()->obsGloL2_[index] = msgRangeb_.range_data[i];
					break;
				default:
					break;
				}
				break;
				/***************** GLONASS ***************/
				/***************** Galileo ***************/
			case FLAG_GAL:
				index = prn - PRNOFFSET_GAL;
				switch (channelStatus.signal_type) {
				case GAL_E1:
					GnssCore::getInstance()->obsGalE1_[index] = msgRangeb_.range_data[i];
					GnssCore::getInstance()->isMeasurementOn_[prn + 55] = true;
					break;
					//
				case GAL_E5b:
					GnssCore::getInstance()->obsGalE5b_[index] = msgRangeb_.range_data[i];
//					GnssCore::getInstance()->isMeasurementOn_[prn - 1] = true;
					break;
				default:
					break;
				}
				break;
				/***************** Galileo ***************/
				/***************** Beidou ***************/
			case FLAG_BDS:
				index = prn - PRNOFFSET_BDS;
				switch (channelStatus.signal_type){
				case BDS_B1_D1:
					GnssCore::getInstance()->obsBdsB1I_[index] = msgRangeb_.range_data[i];
					GnssCore::getInstance()->isMeasurementOn_[prn + 91] = true;
				default:
					break;
				}
				break;
				/***************** Beidou ***************/
				/****************** QZSS ****************/
			case FLAG_QZS:
				index = prn - PRNOFFSET_QZS;
				switch (channelStatus.signal_type){
				case QZS_L1CA:
					GnssCore::getInstance()->obsQzsL1CA_[index] = msgRangeb_.range_data[i];
					GnssCore::getInstance()->isMeasurementOn_[prn - 38] = true;
				default:
					break;
				}
				break;
				/****************** QZSS ****************/

			default:
				break;
			}
		}

		double timeRangeHeader;
		timeRangeHeader = (double) msgRangeb_.header.gps_millisecs / 1000.0;

		if (isBestposReady_)
			GnssCore::getInstance()->ProcessRange(timeRangeHeader);
		break;

	case GPSEPHEMB_LOG_TYPE:
		GpsEphemeris msgGpsEphemerisFor1sv;
		memcpy(&msgGpsEphemerisFor1sv, message, sizeof(msgGpsEphemerisFor1sv));

		prn = msgGpsEphemerisFor1sv.prn; // 1~32
		index = prn - PRNOFFSET_GPS;
		GnssCore::getInstance()->currentGpsEphemerides_[index] = msgGpsEphemerisFor1sv;
		GnssCore::getInstance()->isCurrentEphemOn_[index] = true;

		if (msgGpsEphemerisFor1sv.health == 0)
			GnssCore::getInstance()->isEphemHealthGood_[index] = true;
		break;

	case GLOEPHEMB_LOG_TYPE:
		GloEphemeris msgGloEphemerisFor1sv;
		memcpy(&msgGloEphemerisFor1sv, message, sizeof(msgGloEphemerisFor1sv));

		prn = msgGloEphemerisFor1sv.prn; // 38~61
		msgGloEphemerisFor1sv.e_time = msgGloEphemerisFor1sv.e_time / 1000.0;

		index = prn - PRNOFFSET_GLO;
		GnssCore::getInstance()->currentGloEphemerides_[index] = msgGloEphemerisFor1sv;
		GnssCore::getInstance()->isCurrentEphemOn_[prn - 6] = true;

		if (msgGloEphemerisFor1sv.health < 4)
			GnssCore::getInstance()->isEphemHealthGood_[prn - 6] = true;
		break;

	case GALINAVEPHEMB_LOG_TYPE:
		GalInavEphemeris msgGalEphemerisFor1sv;
		memcpy(&msgGalEphemerisFor1sv, message, sizeof(msgGalEphemerisFor1sv));

		prn = msgGalEphemerisFor1sv.satid;
		index = prn - PRNOFFSET_GAL;

		GnssCore::getInstance()->currentGalEphemerides_[index] = msgGalEphemerisFor1sv;
		GnssCore::getInstance()->isCurrentEphemOn_[prn + 55] = true;

		printf("%i prn %i \n",index, msgGalEphemerisFor1sv.IODNav);
		if (msgGalEphemerisFor1sv.E1bDVS == 0 && msgGalEphemerisFor1sv.E1bHealth == 0)
			GnssCore::getInstance()->isEphemHealthGood_[prn + 55] = true;
		break;

	case BDSEPHEMB_LOG_TYPE:
		BdsEphemeris msgBdsEphemerisFor1sv;
		memcpy(&msgBdsEphemerisFor1sv, message, sizeof(msgBdsEphemerisFor1sv));

		prn = msgBdsEphemerisFor1sv.satid;
		index = prn - PRNOFFSET_BDS;

		GnssCore::getInstance()->currentBdsEphemerides_[index] = msgBdsEphemerisFor1sv;
		GnssCore::getInstance()->isCurrentEphemOn_[prn + 91] = true;

		if (msgBdsEphemerisFor1sv.health1 == 0 &&
				(msgBdsEphemerisFor1sv.AODC < 25 && msgBdsEphemerisFor1sv.AODE < 25))
			GnssCore::getInstance()->isEphemHealthGood_[prn + 91] = true;
		break;

	case QZSSEPHEMB_LOG_TYPE:
		QzssEphemeris msgQzssEphemerisFor1sv;
		memcpy(&msgQzssEphemerisFor1sv, message, sizeof(msgQzssEphemerisFor1sv));

		prn = msgQzssEphemerisFor1sv.satid;
		index = prn - PRNOFFSET_QZS;

		GnssCore::getInstance()->currentQzsEphemerides_[index] = msgQzssEphemerisFor1sv;
		GnssCore::getInstance()->isCurrentEphemOn_[prn - 38] = true;

		if (msgQzssEphemerisFor1sv.health == 0)
			GnssCore::getInstance()->isEphemHealthGood_[prn - 38] = true;
		break;
	case GALCLOCKB_LOG_TYPE:
		memcpy(&msgGalClockb_, message, sizeof(msgGalClockb_));
		break;
///*
	case IONUTCB_LOG_TYPE:
		memcpy(&msgIonutcb_, message, sizeof(msgIonutcb_));
		break;

	case GALIONOB_LOG_TYPE:
		memcpy(&msgGalIonob_, message, sizeof(msgGalIonob_));
		break;

	case BESTVELB_LOG_TYPE:
		memcpy(&msgBestvelb_, message, sizeof(msgBestvelb_));

		if (best_velocity_callback_)
			best_velocity_callback_(msgBestvelb_, read_timestamp_);
		break;

	case BESTXYZB_LOG_TYPE:
		memcpy(&msgBestxyzb_, message, sizeof(msgBestxyzb_));
		if (best_position_ecef_callback_)
			best_position_ecef_callback_(msgBestxyzb_, read_timestamp_);
		break;

	case DUALANTENNAHEADING_LOG_TYPE:
		memcpy(&msgHeading_, message, sizeof(msgHeading_));
		isDualheadingReady_ = true;
		break;
	}
}

/*!
 * Default callback method for timestamping data.  Used if a
 * user callback is not set.  Returns the current time from the
 * CPU clock as the number of seconds from Jan 1, 1970
 */
inline double DefaultGetTime() {
	boost::posix_time::ptime present_time(
			boost::posix_time::microsec_clock::universal_time());
	boost::posix_time::time_duration duration(present_time.time_of_day());
	return (double) (duration.total_milliseconds()) / 1000.0;
}

inline void SaveMessageToFile(unsigned char *message, size_t length,
		const char *filename) {
	ofstream outfile;
	outfile.open(filename, ios::out | ios::app); // "./test_data/filename.txt"
	if (outfile.is_open()) {
		for (int index = 0; index < length; index++) {
			outfile << message[index];
		}
	}
	outfile.close();
}

inline void printHex(unsigned char *data, int length) {
	for (int i = 0; i < length; ++i) {
		printf("0x%X ", (unsigned) (unsigned char) data[i]);
	}
	printf("\n");
}

// stolen from: http://oopweb.com/CPP/Documents/CPPHOWTO/Volume/C++Programming-HOWTO-7.html
void Tokenize(const std::string& str, std::vector<std::string>& tokens,
		const std::string& delimiters = " ") {
	// Skip delimiters at beginning.
	std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	std::string::size_type pos = str.find_first_of(delimiters, lastPos);

	while (std::string::npos != pos || std::string::npos != lastPos) {
		// Found a token, add it to the vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

inline void DefaultAcknowledgementHandler() {
	; //std::cout << "Acknowledgement received." << std::endl;
}

inline void DefaultDebugMsgCallback(const std::string &msg) {
	; //std::cout << "Novatel Debug: " << msg << std::endl;
}

inline void DefaultInfoMsgCallback(const std::string &msg) {
	std::cout << "Novatel Info: " << msg << std::endl;
}

inline void DefaultWarningMsgCallback(const std::string &msg) {
	std::cout << "Novatel Warning: " << msg << std::endl;
}

inline void DefaultErrorMsgCallback(const std::string &msg) {
	std::cout << "Novatel Error: " << msg << std::endl;
}

inline void DefaultBestPositionCallback(Position best_position,
		double time_stamp) {
	std::cout << "BESTPOS: \n  GPS Week: " << best_position.header.gps_week << std::endl
			<< "  GPS milliseconds: " << best_position.header.gps_millisecs
			<< std::endl << "  Latitude: " << best_position.latitude
			<< std::endl << "  Longitude: " << best_position.longitude
			<< std::endl << "  Height: " << best_position.height
			<< std::endl << "  Solution status: "
			<< best_position.solution_status << std::endl << "  position type: "
			<< best_position.position_type << std::endl
			<< "  number of svs tracked: "
			<< (double) best_position.number_of_satellites << std::endl
			<< "  number of svs used: "
			<< (double) best_position.number_of_satellites_in_solution
			<< std::endl;
}

inline void DefaultRawEphemCallback(RawEphemeris ephemeris, double time_stamp) {
	std::cout << "Got RAWEPHEM for PRN " << ephemeris.prn << std::endl;
}

GNSS::GNSS() {
	serial_port_ = NULL;
	reading_status_ = false;
	time_handler_ = DefaultGetTime;
	handle_acknowledgement_ = DefaultAcknowledgementHandler;
	best_position_callback_ = DefaultBestPositionCallback;
	raw_ephemeris_callback_ = DefaultRawEphemCallback;
	log_debug_ = DefaultDebugMsgCallback;
	log_info_ = DefaultInfoMsgCallback;
	log_warning_ = DefaultWarningMsgCallback;
	log_error_ = DefaultErrorMsgCallback;
	reading_acknowledgement_ = false;
	buffer_index_ = 0;
	read_timestamp_ = 0;
	parse_timestamp_ = 0;
	ack_received_ = false;
	waiting_for_reset_complete_ = false;
	is_connected_ = false;
	isBestposReady_ = false;
}

GNSS::~GNSS() {
	cout << "Destructing GNSS instance... " << endl;
	Disconnect();
}

bool GNSS::Connect(std::string port, int baudrate, bool search) {

	bool connected = Connect_(port, baudrate);

	if (!connected && search) {
		// search additional baud rates

//        int bauds_to_search[6]={9600,19200,38400,57600,115200,230400};
		int bauds_to_search[6] = { 9600, 19200, 38400, 57600, 115200, 230400 };
		bool found = false;
		for (int ii = 0; ii < 6; ii++) {
			std::stringstream search_msg;
			search_msg << "Searching for receiver with baudrate: "
					<< bauds_to_search[ii];
			log_info_(search_msg.str());
			if (Connect_(port, bauds_to_search[ii])) {
				found = true;
				break;
			}
		}

		// if the receiver was found on a different baud rate, 
		// change its setting to the selected baud rate and reconnect
		if (found) {
			// change baud rate to selected value
			std::stringstream cmd;
			cmd << "COM THISPORT " << baudrate << "\r\n";
			std::stringstream baud_msg;
			baud_msg << "Changing receiver baud rate to " << baudrate;
			log_info_(baud_msg.str());
			try {
				serial_port_->write(cmd.str());
			} catch (std::exception &e) {
				std::stringstream output;
				output << "Error changing baud rate: " << e.what();
				log_error_(output.str());
				return false;
			}

			Disconnect();
			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

			connected = Connect_(port, baudrate);
		}
	}

	if (connected) {
		// start reading
		StartReading();
		printf("baud connected to %i bps\r\n", baudrate);
		is_connected_ = true;
		return true;
	} else {
		log_error_("Failed to connect.");
		return false;
	}

}

bool GNSS::Connect_(std::string port, int baudrate = 115200) {
	try {

		//serial::Timeout my_timeout(50, 200, 0, 200, 0); // 115200 working settings
		//serial_port_ = new serial::Serial(port,baudrate,my_timeout);

		serial_port_ = new serial::Serial(port, baudrate,
				serial::Timeout::simpleTimeout(10));

		if (!serial_port_->isOpen()) {
			std::stringstream output;
			output << "Serial port: " << port << " failed to open."
					<< std::endl;
			log_error_(output.str());
			delete serial_port_;
			serial_port_ = NULL;
			return false;
		} else {
			std::stringstream output;
			output << "Serial port: " << port << " opened successfully."
					<< std::endl;
			log_info_(output.str());
		}

		// stop any incoming data and flush buffers
		serial_port_->write("UNLOGALL\r\n");
		// wait for data to stop cominig in
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

		// clear serial port buffers
		serial_port_->flush();

		// look for GPS by sending ping and waiting for response
		if (!Ping()) {
			std::stringstream output;
			output << "Novatel GPS not found on port: " << port
					<< " at baudrate " << baudrate << std::endl;
			log_error_(output.str());
			delete serial_port_;
			serial_port_ = NULL;
			is_connected_ = false;
			return false;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error connecting to gps on com port " << port << ": "
				<< e.what();
		log_error_(output.str());
		is_connected_ = false;
		return false;
	}

	return true;
}

void GNSS::Disconnect() {
	log_info_("Novatel disconnecting.");
	StopReading();
	// sleep longer than the timeout period
	boost::this_thread::sleep(boost::posix_time::milliseconds(150));


	try {
		if ((serial_port_ != NULL) && (serial_port_->isOpen())) {
			log_info_("Sending UNLOGALL and closing port.");
			serial_port_->write("UNLOGALL\r\n");
			serial_port_->close();
			delete serial_port_;
			serial_port_ = NULL;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error during disconnect: " << e.what();
		log_error_(output.str());
	}
}

bool GNSS::Ping(int num_attempts) {

	while ((num_attempts--) > 0) {
		std::stringstream output;
		output << "Searching for Novatel receiver..." << std::endl;
		log_info_(output.str());
		if (UpdateVersion()) {
			std::stringstream output;
			output << "Found Novatel receiver." << std::endl;
			output << "\tModel: " << model_ << std::endl;
			output << "\tSerial Number: " << serial_number_ << std::endl;
			output << "\tHardware version: " << hardware_version_ << std::endl;
			output << "\tSoftware version: " << software_version_ << std::endl
					<< std::endl;
			;
			output << "Receiver capabilities:" << std::endl;
			output << "\tL2: ";
			if (l2_capable_)
				output << "+" << std::endl;
			else
				output << "-" << std::endl;
			output << "\tRaw measurements: ";
			if (raw_capable_)
				output << "+" << std::endl;
			else
				output << "-" << std::endl;
			output << "\tRTK: ";
			if (rtk_capable_)
				output << "+" << std::endl;
			else
				output << "-" << std::endl;
			output << "\tSPAN: ";
			if (span_capable_)
				output << "+" << std::endl;
			else
				output << "-" << std::endl;
			output << "\tGLONASS: ";
			if (glonass_capable_)
				output << "+" << std::endl;
			else
				output << "-" << std::endl;
			log_info_(output.str());
			return true;
		}
	}

	// no response found
	return false;

}

void GNSS::SendRawEphemeridesToReceiver(RawEphemerides raw_ephemerides) {
	try {
		for (uint8_t index = 0; index < MAX_NUM_SAT; index++) {
			cout << "SIZEOF: " << sizeof(raw_ephemerides.ephemeris[index])
					<< endl;
			if (sizeof(raw_ephemerides.ephemeris[index]) == 106 + HEADER_SIZE) {
				uint8_t* msg_ptr =
						(unsigned char*) &raw_ephemerides.ephemeris[index];
				bool result = SendBinaryDataToReceiver(msg_ptr,
						sizeof(raw_ephemerides.ephemeris[index]));
				if (result)
					cout << "Sent RAWEPHEM for PRN "
							<< (double) raw_ephemerides.ephemeris[index].prn
							<< endl;
			}
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error in Novatel::SendRawEphemeridesToReceiver(): "
				<< e.what();
		log_error_(output.str());
	}
}

bool GNSS::SendBinaryDataToReceiver(uint8_t* msg_ptr, size_t length) {

	try {
		stringstream output1;
		std::cout << length << std::endl;
		std::cout << "Message Pointer" << endl;
		printHex((unsigned char*) msg_ptr, length);
		size_t bytes_written;

		if ((serial_port_ != NULL) && (serial_port_->isOpen())) {
			bytes_written = serial_port_->write(msg_ptr, length);
		} else {
			log_error_("Unable to send message. Serial port not open.");
			return false;
		}
		// check that full message was sent to serial port
		if (bytes_written == length) {
			return true;
		} else {
			log_error_("Full message was not sent over serial port.");
			output1 << "Attempted to send " << length << "bytes. "
					<< bytes_written << " bytes sent.";
			log_error_(output1.str());
			return false;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error in Novatel::SendBinaryDataToReceiver(): " << e.what();
		log_error_(output.str());
		return false;
	}
}

bool GNSS::SendCommand(std::string cmd_msg, bool wait_for_ack) {
	try {
		// sends command to GPS receiver
		serial_port_->write(cmd_msg + "\r\n");
		// wait for acknowledgement (or 2 seconds)
		if (wait_for_ack) {
			boost::mutex::scoped_lock lock(ack_mutex_);
			boost::system_time const timeout = boost::get_system_time()
					+ boost::posix_time::milliseconds(2000);
			if (ack_condition_.timed_wait(lock, timeout)) {
				log_info_("Command `" + cmd_msg + "` sent to GPS receiver.");
				return true;
			} else {
				log_error_("Command '" + cmd_msg + "' failed.");
				return false;
			}
		} else {
			log_info_("Command `" + cmd_msg + "` sent to GPS receiver.");
			return true;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error in Novatel::SendCommand(): " << e.what();
		log_error_(output.str());
		return false;
	}
}

bool GNSS::SetSvElevationAngleCutoff(float angle, CONSTELLATION_FLAG flag) {
	try {
		std::stringstream ang_cmd;
		switch (flag){
		case FLAG_GPS:
			ang_cmd << "ECUTOFF " << angle;
			break;
		case FLAG_GLO:
			ang_cmd << "GLOECUTOFF " << angle;
			break;
		case FLAG_GAL:
			ang_cmd << "GALECUTOFF " << angle;
			break;
		case FLAG_BDS:
			ang_cmd << "BDSECUTOFF " << angle;
			break;
		case FLAG_QZS:
			ang_cmd << "QZSSECUTOFF " << angle;
			break;
		case FLAG_NIC:
			ang_cmd << "NAVICECUTOFF " << angle;
			break;
		}
		return SendCommand(ang_cmd.str());
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error in Novatel::SetSvElevationCutoff(): " << e.what();
		log_error_(output.str());
		return false;
	}
}

bool GNSS::SetInitialPosition(double latitude, double longitude, double height) {
	std::stringstream pos_cmd;
	pos_cmd << "SETAPPROXPOS " << latitude << " " << longitude << " " << height;
	return SendCommand(pos_cmd.str());

}

bool GNSS::SetInitialTime(uint32_t gps_week, double gps_seconds) {
	std::stringstream time_cmd;
	time_cmd << "SETAPPROXTIME " << gps_week << " " << gps_seconds;
	return SendCommand(time_cmd.str());
}
/*
 uint8_t          sync1;          //!< start of packet first byte (0xAA)
 uint8_t          sync2;          //!< start of packet second byte (0x44)
 uint8_t          sync3;          //!< start of packet third  byte (0x12)
 uint8_t          header_length; 	//!< Length of the header in bytes ( From start of packet )
 uint16_t         message_id;    	//!< Message ID number
 uint8_t          message_type;  	//!< Message type - binary, ascii, nmea, etc...
 uint8_t          port_address;  	//!< Address of the data port the log was received on
 uint16_t         message_length;	//!< Message length (Not including header or CRC)
 uint16_t         sequence;      	//!< Counts down from N-1 to 0 for multiple related logs
 uint8_t          idle;          	//!< Time the processor was idle in last sec between logs with same ID
 uint8_t          time_status;    //!< Indicates the quality of the GPS time
 uint16_t         gps_week;      	//!< GPS Week number
 uint32_t         gps_millisecs; 	//!< Milliseconds into week
 uint32_t         status;        	//!< Receiver status word
 uint16_t         Reserved;      	//!< Reserved for internal use
 uint16_t         version;       	//!< Receiver software build number (0-65535)
 */

bool GNSS::InjectAlmanac(Almanac almanac) {
	try {
		MessageType type;
		type.format = BINARY;
		type.response = ORIGINAL_MESSAGE;

		almanac.header.sync1 = NOVATEL_SYNC_BYTE_1;
		almanac.header.sync2 = NOVATEL_SYNC_BYTE_2;
		almanac.header.sync3 = NOVATEL_SYNC_BYTE_3;
		almanac.header.header_length = HEADER_SIZE;
		almanac.header.message_id = ALMANACB_LOG_TYPE;
		almanac.header.message_type = type;
		almanac.header.port_address = THISPORT;
		almanac.header.message_length = 4 + almanac.number_of_prns * 112;
		almanac.header.sequence = 0;
		almanac.header.idle = 0; //!< ignored on input
		almanac.header.time_status = 0; //!< ignored on input
		almanac.header.gps_week = 0; //!< ignored on input
		almanac.header.gps_millisecs = 0; //!< ignored on input
		almanac.header.status = 0; //!< ignored on input
		almanac.header.Reserved = 0; //!< ignored on input
		almanac.header.version = 0; //!< ignored on input

		cout << "SIZEOF: " << sizeof(almanac) << endl;
		uint8_t* msg_ptr = (unsigned char*) &almanac;
		uint32_t crc = CalculateBlockCRC32(sizeof(almanac) - 4, msg_ptr);
		memcpy(almanac.crc, &crc, sizeof(crc)); // TODO: check byte ordering for crc
		bool result = SendBinaryDataToReceiver(msg_ptr, sizeof(almanac));
		if (result) {
			cout << "Sent ALMANAC." << endl;
			return true;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error in Novatel::InjectAlmanac(): " << e.what();
		log_error_(output.str());
		return false;
	}
}

bool GNSS::SetCarrierSmoothing(uint32_t l1_time_constant,
		uint32_t l2_time_constant) {
	try {
		std::stringstream smooth_cmd;
		if ((2 >= l1_time_constant) || (l1_time_constant >= 2000)) {
			log_error_(
					"Error in SetCarrierSmoothing: l1_time_constant set to improper value.");
			return false;
		} else if ((5 >= l2_time_constant) || (l2_time_constant >= 2000)) {
			log_error_(
					"Error in SetCarrierSmoothing: l2_time_constant set to improper value.");
			return false;
		} else {
			smooth_cmd << "CSMOOTH " << l1_time_constant << " "
					<< l2_time_constant;
		}
		return SendCommand(smooth_cmd.str());
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error in Novatel::SetCarrierSmoothing(): " << e.what();
		log_error_(output.str());
		return false;
	}
}

bool GNSS::HardwareReset() {
	// Resets receiver to cold start, does NOT clear non-volatile memory!
	try {
		std::stringstream rst_cmd;
		rst_cmd << "RESET";
		bool command_sent = SendCommand(rst_cmd.str(), false);
		if (command_sent) {
			boost::mutex::scoped_lock lock(reset_mutex_);
			waiting_for_reset_complete_ = true;
			boost::system_time const timeout = boost::get_system_time()
					+ boost::posix_time::milliseconds(5000);
			if (reset_condition_.timed_wait(lock, timeout)) {
				log_info_("Hardware Reset Complete.");
				return true;
			} else {
				log_error_("Hardware Reset never Completed.");
				waiting_for_reset_complete_ = false;
				return false;
			}
		} else {
			return false;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		waiting_for_reset_complete_ = false;
		output << "Error in Novatel::HardwareReset(): " << e.what();
		log_error_(output.str());
		return false;
	}
}

bool GNSS::HotStartReset() {
	try {
		std::stringstream rst_cmd;
		rst_cmd << "RESET";
		bool command_sent = SendCommand(rst_cmd.str(), false);
		if (command_sent) {
			boost::mutex::scoped_lock lock(reset_mutex_);
			waiting_for_reset_complete_ = true;
			boost::system_time const timeout = boost::get_system_time()
					+ boost::posix_time::milliseconds(10000);
			if (reset_condition_.timed_wait(lock, timeout)) {
				log_info_("HotStartReset Complete.");
				return true;
			} else {
				log_error_("HotStartReset never Completed.");
				waiting_for_reset_complete_ = false;
				return false;
			}
		} else {
			return false;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		waiting_for_reset_complete_ = false;
		output << "Error in Novatel::HotStartReset(): " << e.what();
		log_error_(output.str());
		return false;
	}
}

bool GNSS::WarmStartReset() {
	try {
		std::stringstream rst_pos_cmd;
		std::stringstream rst_time_cmd;
		rst_pos_cmd << "FRESET " << LAST_POSITION; //!< FRESET doesn't reply with an ACK
		bool pos_reset = SendCommand(rst_pos_cmd.str(), false);
		rst_time_cmd << "FRESET " << LBAND_TCXO_OFFSET; //!< FRESET doesn't reply with an ACK
		bool time_reset = SendCommand(rst_time_cmd.str(), false);
		if (pos_reset && time_reset) {
			boost::mutex::scoped_lock lock(reset_mutex_);
			waiting_for_reset_complete_ = true;
			boost::system_time const timeout = boost::get_system_time()
					+ boost::posix_time::milliseconds(100000);
			if (reset_condition_.timed_wait(lock, timeout)) {
				log_info_("WarmStartReset Complete.");
				return true;
			} else {
				log_error_("WarmStartReset never Completed.");
				waiting_for_reset_complete_ = false;
				return false;
			}
		} else {
			return false;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		waiting_for_reset_complete_ = false;
		output << "Error in Novatel::WarmStartReset(): " << e.what();
		log_error_(output.str());
		return false;
	}
}

bool GNSS::ColdStartReset() {
	try {
		std::stringstream rst_cmd;
		rst_cmd << "FRESET STANDARD";
		bool command_sent = SendCommand(rst_cmd.str(), false); //!< FRESET doesn't reply with an ACK
		if (command_sent) {
			boost::mutex::scoped_lock lock(reset_mutex_);
			waiting_for_reset_complete_ = true;
			boost::system_time const timeout = boost::get_system_time()
					+ boost::posix_time::milliseconds(10000);
			if (reset_condition_.timed_wait(lock, timeout)) {
				log_info_("ColdStartReset Complete.");
				return true;
			} else {
				log_error_("ColdStartReset never Completed.");
				waiting_for_reset_complete_ = false;
				return false;
			}
		} else {
			return false;
		}
	} catch (std::exception &e) {
		std::stringstream output;
		waiting_for_reset_complete_ = false;
		output << "Error in Novatel::ColdStartReset(): " << e.what();
		log_error_(output.str());
		return false;
	}
}

void GNSS::SaveConfiguration() {
	try {
		bool result = SendCommand("SAVECONFIG");
		if (result)
			log_info_("Receiver configuration has been saved.");
		else
			log_error_("Failed to save receiver configuration!");
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error in Novatel::SaveConfiguration(): " << e.what();
		log_error_(output.str());
	}
}

void GNSS::ConfigureLogs(std::string log_string) {
	// parse log_string on semicolons (;)
	std::vector<std::string> logs;

	Tokenize(log_string, logs, ";");

	// request each log from the receiver and wait for an ack
	for (std::vector<std::string>::iterator it = logs.begin(); it != logs.end();
			++it) {
		// try each command up to five times
		int ii = 0;
		while (ii < 5) {
			try {
				// send log command to gps (e.g. "LOG BESTUTMB ONTIME 1.0")
				serial_port_->write("LOG " + *it + "\r\n");
				std::stringstream cmd;
				cmd << "LOG " << *it;
				log_info_(cmd.str());
				// wait for acknowledgement (or 2 seconds)
				boost::mutex::scoped_lock lock(ack_mutex_);
				boost::system_time const timeout = boost::get_system_time()
						+ boost::posix_time::milliseconds(2000);
				if (ack_condition_.timed_wait(lock, timeout)) {
					log_info_(
							"Ack received for requested log: " + *it + "\r\n");
					break;
				} else {
					log_error_(
							"No acknowledgement received for log: " + *it
									+ "\r\n");
				}
				ii++;
			} catch (std::exception &e) {
				std::stringstream output;
				output << "Error configuring receiver logs: " << e.what();
				log_error_(output.str());
			}
		}

	}

}

void GNSS::Unlog(std::string log) {
	try {
		std::stringstream unlog_cmd;
		unlog_cmd << "UNLOG " << log;
		bool result = SendCommand(unlog_cmd.str());
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error in Novatel::Unlog(): " << e.what();
		log_error_(output.str());
	}
}

void GNSS::UnlogAll() {
	try {
		bool result = SendCommand("UNLOGALL THISPORT");
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error in Novatel::UnlogAll(): " << e.what();
		log_error_(output.str());
	}
}

void GNSS::ConfigureInterfaceMode(std::string com_port, std::string rx_mode,
		std::string tx_mode) {

	try {
		// send command to set interface mode on com port
		// ex: INTERFACEMODE COM2 RX_MODE TX_MODE
		serial_port_->write(
				"INTERFACEMODE " + com_port + " " + rx_mode + " " + tx_mode
						+ "\r\n");
		// wait for acknowledgement (or 2 seconds)
		boost::mutex::scoped_lock lock(ack_mutex_);
		boost::system_time const timeout = boost::get_system_time()
				+ boost::posix_time::milliseconds(2000);
		if (ack_condition_.timed_wait(lock, timeout)) {
			log_info_(
					"Ack received.  Interface mode for port " + com_port
							+ " set to: " + rx_mode + " " + tx_mode);
		} else {
			log_error_(
					"No acknowledgement received for interface mode command.");
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error configuring interface mode: " << e.what();
		log_error_(output.str());
	}
}

void GNSS::ConfigureBaudRate(std::string com_port, int baudrate) {
	try {
		// send command to set baud rate on GPS com port
		// ex: COM com1 9600 n 8 1 n off on
		std::stringstream cmd;
		cmd << "COM " << com_port << " " << baudrate << " n 8 1 n off on\r\n";
		serial_port_->write(cmd.str());
		// wait for acknowledgement (or 2 seconds)
		boost::mutex::scoped_lock lock(ack_mutex_);
		boost::system_time const timeout = boost::get_system_time()
				+ boost::posix_time::milliseconds(2000);
		if (ack_condition_.timed_wait(lock, timeout)) {
			std::stringstream log_out;
			log_out << "Ack received.  Baud rate on com port " << com_port
					<< " set to " << baudrate << std::endl;
			log_info_(log_out.str());
		} else {
			log_error_(
					"No acknowledgement received for com configure command.");
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error configuring baud rate: " << e.what();
		log_error_(output.str());
	}
}

bool GNSS::UpdateVersion() {
	// request the receiver version and wait for a response
	// example response:
	//#VERSIONA,COM1,0,71.5,FINESTEERING,1362,340308.478,00000008,3681,2291;
	//    1,GPSCARD,"L12RV","DZZ06040010","OEMV2G-2.00-2T","3.000A19","3.000A9",
	//    "2006/Feb/ 9","17:14:33"*5e8df6e0

	try {
		// clear port
		serial_port_->flush();
		// read out any data currently in the buffer
		std::string read_data = serial_port_->read(5000);
		while (read_data.length())
			read_data = serial_port_->read(5000);

		// send request for version
		serial_port_->write("log versiona once\r\n");
		// wait for response from the receiver
		boost::this_thread::sleep(boost::posix_time::milliseconds(500));

		// read from the serial port until a new line character is seen
		std::string gps_response = serial_port_->read(15000);

		std::vector<std::string> packets;

		Tokenize(gps_response, packets, "\n");

		// loop through all packets in file and check for version messages
		// stop when the first is found or all packets are read
		for (size_t ii = 0; ii < packets.size(); ii++) {
			if (ParseVersion(packets[ii])) {
				return true;
			}
		}
	} catch (std::exception &e) {
		std::stringstream output;
		output << "Error reading version info from receiver: " << e.what();
		log_error_(output.str());
		return false;
	}

	return false;

}

bool GNSS::ParseVersion(std::string packet) {
	// parse the results - message should start with "#VERSIONA"
	size_t found_version = packet.find("VERSIONA");
	if (found_version == string::npos)
		return false;

	// parse version information
	// remove header
	size_t pos = packet.find(";");
	if (pos == string::npos) {
		log_error_("Error parsing received version."
				" End of message was not found");
		log_debug_(packet);
		return false;
	}

	// remove header from message
	std::string message = packet.substr(pos + 1, packet.length() - pos - 2);
	// parse message body by tokening on ","
	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> sep(",");
	tokenizer tokens(message, sep);
	// set up iterator to go through token list
	tokenizer::iterator current_token = tokens.begin();
	string num_comps_string = *(current_token);
	int number_components = atoi(num_comps_string.c_str());
	// make sure the correct number of tokens were found
	int token_count = 0;
	for (current_token = tokens.begin(); current_token != tokens.end();
			++current_token) {
		//log_debug_(*current_token);
		token_count++;
	}

	// should be 9 tokens, if not something is wrong
	if (token_count != (8 * number_components + 1)) {
		log_error_("Error parsing received version. "
				"Incorrect number of tokens found.");
		std::stringstream err_out;
		err_out << "Found: " << token_count << "  Expected: "
				<< (8 * number_components + 1);
		log_error_(err_out.str());
		log_debug_(packet);
		return false;
	}

	current_token = tokens.begin();
	// device type is 2nd token
	string device_type = *(++current_token);
	// model is 3rd token
	model_ = *(++current_token);
	// serial number is 4th token
	serial_number_ = *(++current_token);
	// model is 5rd token
	hardware_version_ = *(++current_token);
	// model is 6rd token
	software_version_ = *(++current_token);

	// parse the version:
	if (hardware_version_.length() > 3)
		protocol_version_ = hardware_version_.substr(1, 4);
	else
		protocol_version_ = "UNKNOWN";

	// parse model number:
	// is the receiver capable of raw measurements?
	if (model_.find("L") != string::npos)
		raw_capable_ = true;
	else
		raw_capable_ = false;

	// can the receiver receive L2?
	if (model_.find("12") != string::npos)
		l2_capable_ = true;
	else
		l2_capable_ = false;

	// can the receiver receive GLONASS?
	if (model_.find("G") != string::npos)
		glonass_capable_ = true;
	else
		glonass_capable_ = false;

	// Is this a SPAN unit?
	if ((model_.find("I") != string::npos)
			|| (model_.find("J") != string::npos))
		span_capable_ = true;
	else
		span_capable_ = false;

	// Can the receiver process RTK?
	if (model_.find("R") != string::npos)
		rtk_capable_ = true;
	else
		rtk_capable_ = false;

	// fix for oem4 span receivers - do not use l12 notation
	// i think all oem4 spans are l1 l2 capable and raw capable
	if ((protocol_version_ == "OEM4") && (span_capable_)) {
		l2_capable_ = true;
		raw_capable_ = true;
	}

	return true;

}

void GNSS::StartReading() {
	if (reading_status_)
		return;
	// create thread to read from sensor
	reading_status_ = true;
	read_thread_ptr_ = boost::shared_ptr<boost::thread>(
			new boost::thread(boost::bind(&GNSS::ReadSerialPort, this)));
}

void GNSS::StopReading() {
	reading_status_ = false;
}

void GNSS::ReadSerialPort() {
	unsigned char buffer[MAX_NOUT_SIZE];
	size_t len;

	log_info_("Started read thread.");

	// continuously read data from serial port
	while (reading_status_) {
		try {
			// read data
			//if you have a delay, you should use other size
//			len = serial_port_->read(buffer, MAX_NOUT_SIZE);
//			len = serial_port_->read(buffer, 100);
			len = serial_port_->read(buffer, 8192/4);

		} catch (std::exception &e) {
			std::stringstream output;
			output << "Error reading from serial port: " << e.what();
			log_error_(output.str());
			//return;
		}
		// timestamp the read
		if (time_handler_)
			read_timestamp_ = time_handler_();
		else
			read_timestamp_ = 0;

		if (len > 1)
//			std::cout << reading_status_ << read_timestamp_ <<  "  bytes: " << len << std::endl;
		// add data to the buffer to be parsed
		BufferIncomingData(buffer, len);
//		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
}

void GNSS::BufferIncomingData(unsigned char *message, unsigned int length) {

	// add incoming data to buffer
	for (unsigned int ii = 0; ii < length; ii++) {

		// make sure bufIndex is not larger than buffer
		if (buffer_index_ >= MAX_NOUT_SIZE) {
			buffer_index_ = 0;
			log_warning_("Overflowed receive buffer. Buffer cleared.");
		}

		if (buffer_index_ == 0) {	// looking for beginning of message
			if (message[ii] == NOVATEL_SYNC_BYTE_1) {// beginning of msg found - add to buffer
				data_buffer_[buffer_index_++] = message[ii];
				bytes_remaining_ = 0;
			} else if (message[ii] == NOVATEL_ACK_BYTE_1) {
				// received beginning of acknowledgement
				reading_acknowledgement_ = true;
				buffer_index_ = 1;
			} else if ((message[ii] == NOVATEL_RESET_BYTE_1)
					&& waiting_for_reset_complete_) {
				// received {COM#} acknowledging receiver reset complete
				reading_reset_complete_ = true;
				buffer_index_ = 1;
			} else {
				//log_debug_("BufferIncomingData::Received unknown data.");
			}
		} else if (buffer_index_ == 1) {	// verify 2nd character of header
			if (message[ii] == NOVATEL_SYNC_BYTE_2) {// 2nd byte ok - add to buffer
				data_buffer_[buffer_index_++] = message[ii];
			} else if ((message[ii] == NOVATEL_ACK_BYTE_2)
					&& reading_acknowledgement_) {
				// 2nd byte of acknowledgement
				buffer_index_ = 2;
			} else if ((message[ii] == NOVATEL_RESET_BYTE_2)
					&& reading_reset_complete_) {
				// 2nd byte of receiver reset complete message
				buffer_index_ = 2;
			} else {
				// start looking for new message again
				buffer_index_ = 0;
				bytes_remaining_ = 0;
				reading_acknowledgement_ = false;
				reading_reset_complete_ = false;
			} // end if (msg[i]==0x44)
		} else if (buffer_index_ == 2) {	// verify 3rd character of header
			if (message[ii] == NOVATEL_SYNC_BYTE_3) {// 2nd byte ok - add to buffer
				data_buffer_[buffer_index_++] = message[ii];
			} else if ((message[ii] == NOVATEL_ACK_BYTE_3)
					&& (reading_acknowledgement_)) {
				log_info_("RECEIVED AN ACK.");

				// final byte of acknowledgement received
				buffer_index_ = 0;
				reading_acknowledgement_ = false;
				boost::lock_guard<boost::mutex> lock(ack_mutex_);
				ack_received_ = true;
				ack_condition_.notify_all();

				// ACK received
				handle_acknowledgement_();
			} else if ((message[ii] == NOVATEL_RESET_BYTE_3)
					&& reading_reset_complete_) {
				// 3rd byte of receiver reset complete message
				buffer_index_ = 3;
			} else {
				// start looking for new message again
				buffer_index_ = 0;
				bytes_remaining_ = 0;
				reading_acknowledgement_ = false;
				reading_reset_complete_ = false;
			} // end if (msg[i]==0x12)
		} else if (buffer_index_ == 3) { // number of bytes in header - not including sync
			if ((message[ii] == NOVATEL_RESET_BYTE_4)
					&& (message[ii + 2] == NOVATEL_RESET_BYTE_6)
					&& reading_reset_complete_ && waiting_for_reset_complete_) {
				// 4th byte of receiver reset complete message
//                log_info_("RECEIVER RESET COMPLETE RECEIVED.");
				buffer_index_ = 0;
				reading_reset_complete_ = false;
				boost::lock_guard<boost::mutex> lock(reset_mutex_);
				waiting_for_reset_complete_ = false;
				reset_condition_.notify_all();

			} else {
				reading_reset_complete_ = false;
				data_buffer_[buffer_index_++] = message[ii];
				// length of header is in byte 4
				header_length_ = message[ii];
			}
		} else if (buffer_index_ == 5) { // get message id
			data_buffer_[buffer_index_++] = message[ii];
			bytes_remaining_--;
			message_id_ = BINARY_LOG_TYPE(
					((data_buffer_[buffer_index_ - 1]) << 8)
							+ data_buffer_[buffer_index_ - 2]);
			// } else if (buffer_index_ == 8) {	// set number of bytes
			// 	data_buffer_[buffer_index_++] = message[ii];
			// 	// length of message is in byte 8
			// 	// bytes remaining = remainder of header  + 4 byte checksum + length of body
			// 	// TODO: added a -2 to make things work right, figure out why i need this
			// 	bytes_remaining_ = message[ii] + 4 + (header_length_-7) - 2;
		} else if (buffer_index_ == 9) {
			data_buffer_[buffer_index_++] = message[ii];
			bytes_remaining_ = (header_length_ - 10) + 4
					+ (data_buffer_[9] << 8) + data_buffer_[8];
//			printf("\n ========================================= \n");
//			printf("remaining bytes : %d  length : %d \n", bytes_remaining_, length);
		} else if (bytes_remaining_ == 1) {	// add last byte and parse
			data_buffer_[buffer_index_++] = message[ii];
			// BINARY_LOG_TYPE message_id = (BINARY_LOG_TYPE) (((data_buffer_[5]) << 8) + data_buffer_[4]);
			// log_info_("Sending to ParseBinary");

//				FILE *save_file;
//				char SaveFileName[50];
//				sprintf(SaveFileName, "Ground.gps");
//
//				save_file = fopen(SaveFileName, "ab");
//				fwrite(data_buffer_, sizeof(char), buffer_index_, save_file);
//				fclose (save_file);


			ParseBinary(data_buffer_, buffer_index_, message_id_);

			// reset counters
			buffer_index_ = 0;
			bytes_remaining_ = 0;
		} else {	// add data to buffer
			data_buffer_[buffer_index_++] = message[ii];
			bytes_remaining_--;
		}

		/* ---------------- ascii NMEA Parser -----------------*/

		if (buffer_index_ != 0) {
			nmea_flag = 0;
		}

		if (nmea_flag == 3){
			 if (message[ii] != 10){
				 data_buffer_nmea_[buffer_index_nmea] = message[ii];
				 buffer_index_nmea++;
			 } else {
				 data_buffer_nmea_[buffer_index_nmea] = message[ii];
				 buffer_index_nmea++;

				 ParseNMEA(data_buffer_nmea_, buffer_index_nmea);

				 nmea_flag = 0;
				 buffer_index_nmea = 0;
			 }
		}

		if (message[ii] == '$' && nmea_flag == 0){ // $
			nmea_flag++;
		}
		if (message[ii] == 'G' && nmea_flag == 1){ // G
			nmea_flag++;
		}
		if (message[ii] == 'P' && nmea_flag == 2){ // P
			nmea_flag++;
		}

	}	// end for
}


void GNSS::UnpackCompressedRangeData(const CompressedRangeData &cmp,
		RangeData &rng) {
	rng.satellite_prn = cmp.range_record.satellite_prn;

	rng.channel_status = cmp.channel_status;

	rng.pseudorange = double(cmp.range_record.pseudorange) / 128.0;

	rng.pseudorange_standard_deviation = UnpackCompressedPsrStd(
			cmp.range_record.pseudorange_standard_deviation);

	rng.accumulated_doppler = UnpackCompressedAccumulatedDoppler(cmp,
			rng.pseudorange);

	rng.accumulated_doppler_std_deviation =
			(cmp.range_record.accumulated_doppler_std_deviation + 1.0) / 512.0;

	rng.doppler = cmp.range_record.doppler / 256.0;

	rng.locktime = cmp.range_record.locktime / 32.0;

	rng.carrier_to_noise = (float) (cmp.range_record.carrier_to_noise + 20);
}

double GNSS::UnpackCompressedPsrStd(const uint16_t &val) const {
	switch (val) {
	case 0:
		return (0.050);
		break;
	case 1:
		return (0.075);
		break;
	case 2:
		return (0.113);
		break;
	case 3:
		return (0.169);
		break;
	case 4:
		return (0.253);
		break;
	case 5:
		return (0.380);
		break;
	case 6:
		return (0.570);
		break;
	case 7:
		return (0.854);
		break;
	case 8:
		return (1.281);
		break;
	case 9:
		return (2.375);
		break;
	case 10:
		return (4.750);
		break;
	case 11:
		return (9.500);
		break;
	case 12:
		return (19.000);
		break;
	case 13:
		return (38.000);
		break;
	case 14:
		return (76.000);
		break;
	case 15:
		return (152.000);
		break;
	default:
		return (0);
	}
}

double GNSS::UnpackCompressedAccumulatedDoppler(
		const CompressedRangeData &cmp, const double &uncmpPsr) const {

	double scaled_adr = (double) cmp.range_record.accumulated_doppler / 256.0;

	double adr_rolls = uncmpPsr;

	switch (cmp.channel_status.satellite_sys) {
	case 0: // GPS

		if (cmp.channel_status.signal_type == 0) // L1
				{
			adr_rolls /= CMP_GPS_WAVELENGTH_L1;
		} else if ((cmp.channel_status.signal_type == 5) ||  // L2 P
				(cmp.channel_status.signal_type == 9) ||  // L2 P codeless
				(cmp.channel_status.signal_type == 17))   // L2C
				{
			adr_rolls /= CMP_GPS_WAVELENGTH_L2;
		} else {
			/*      std::cout << "Unknown GPS Frequency type!" << std::endl;
			 std::cout << "PRN: "
			 << cmp.range_record.satellite_prn
			 << "\tSatellite System: "
			 << cmp.channel_status.satellite_sys
			 << "\tSignal Type: "
			 << cmp.channel_status.signal_type
			 << std::endl;*/
		}

		break;

	case 1: // GLO
		// TODO: Need to compute actual wavelengths here, this is incorrect
		if (cmp.channel_status.signal_type == 0) // L1
				{
			adr_rolls /= CMP_GPS_WAVELENGTH_L1;
		} else if (cmp.channel_status.signal_type == 5) // L2 P
				{
			adr_rolls /= CMP_GPS_WAVELENGTH_L2;
		} else {
			/*      std::cout << "Unknown GLO Frequency type!" << std::endl;
			 std::cout << "PRN: "
			 << cmp.range_record.satellite_prn
			 << "\tSatellite System: "
			 << cmp.channel_status.satellite_sys
			 << "\tSignal Type: "
			 << cmp.channel_status.signal_type
			 << std::endl;*/
		}
		break;

	case 2: // WAAS
		if (cmp.channel_status.signal_type == 1) // L1
				{
			adr_rolls /= CMP_GPS_WAVELENGTH_L1;
		} else {
			/*      std::cout << "Unknown WAAS Frequency type!" << std::endl;
			 std::cout << "PRN: "
			 << cmp.range_record.satellite_prn
			 << "\tSatellite System: "
			 << cmp.channel_status.satellite_sys
			 << "\tSignal Type: "
			 << cmp.channel_status.signal_type
			 << std::endl;*/
		}
		break;

	default:
		/*    std::cout << "Unknown Satellite System type!" << std::endl;
		 std::cout << "PRN: "
		 << cmp.range_record.satellite_prn
		 << "\tSatellite System: "
		 << cmp.channel_status.satellite_sys
		 << "\tSignal Type: "
		 << cmp.channel_status.signal_type
		 << std::endl;*/
		break;
	}

	adr_rolls = (adr_rolls + scaled_adr) / CMP_MAX_VALUE;

	if (adr_rolls <= 0) {
		adr_rolls -= 0.5;
	} else {
		adr_rolls += 0.5;
	}

	return (scaled_adr - (CMP_MAX_VALUE * (int) adr_rolls));
}

/* --------------------------------------------------------------------------
 Calculate a CRC value to be used by CRC calculation functions.
 -------------------------------------------------------------------------- */
unsigned int GNSS::CRC32Value(int i) {
	int j;
	unsigned int ulCRC;
	ulCRC = i;
	for (j = 8; j > 0; j--) {
		if (ulCRC & 1)
			ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}

/* --------------------------------------------------------------------------
 Calculates the CRC-32 of a block of data all at once
 -------------------------------------------------------------------------- */
unsigned int GNSS::CalculateBlockCRC32(unsigned int ulCount, /* Number of bytes in the data block */
unsigned char *ucBuffer) /* Data block */
{
	unsigned int ulTemp1;
	unsigned int ulTemp2;
	unsigned int ulCRC = 0;
	while (ulCount-- != 0) {
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value(((int) ulCRC ^ *ucBuffer++) & 0xff);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return (ulCRC);
}

// this functions matches the conversion done by the Novatel receivers

bool GNSS::ConvertLLaUTM(double Lat, double Long, double *northing,
		double *easting, int *zone, bool *north) {
	const double a = 6378137.0;
	const double ee = 0.00669437999;
	const double k0 = 0.9996;
	const double e2 = ee / (1 - ee);

	double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180; // -180.00 .. 179.9;
	double LatRad = GRAD_A_RAD(Lat);
	double LongRad = GRAD_A_RAD(LongTemp);
	double LongOriginRad;

	double N, T, C, A, M;

	//Make sure the longitude is between -180.00 .. 179.9
	*zone = int((LongTemp + 180) / 6.0) + 1;
	if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
		*zone = 32;

	// Special zones for Svalbard
	if (Lat >= 72.0 && Lat < 84.0) {
		if (LongTemp >= 0.0 && LongTemp < 9.0)
			*zone = 31;
		else if (LongTemp >= 9.0 && LongTemp < 21.0)
			*zone = 33;
		else if (LongTemp >= 21.0 && LongTemp < 33.0)
			*zone = 35;
		else if (LongTemp >= 33.0 && LongTemp < 42.0)
			*zone = 37;
	}
	LongOriginRad = GRAD_A_RAD((*zone - 1) * 6 - 180 + 3);

	N = a / sqrt(1 - ee * sin(LatRad) * sin(LatRad));
	T = tan(LatRad) * tan(LatRad);
	C = e2 * cos(LatRad) * cos(LatRad);
	A = cos(LatRad) * (LongRad - LongOriginRad);
	M = a
			* ((1 - ee / 4 - 3 * ee * ee / 64 - 5 * ee * ee * ee / 256) * LatRad
					- (3 * ee / 8 + 3 * ee * ee / 32 + 45 * ee * ee * ee / 1024)
							* sin(2 * LatRad)
					+ (15 * ee * ee / 256 + 45 * ee * ee * ee / 1024)
							* sin(4 * LatRad)
					- (35 * ee * ee * ee / 3072) * sin(6 * LatRad));

	*easting = (double) (k0 * N
			* (A + (1 - T + C) * A * A * A / 6
					+ (5 - 18 * T + T * T + 72 * C - 58 * e2) * A * A * A * A
							* A / 120) + 500000.0);
	*northing = (double) (k0
			* (M
					+ N * tan(LatRad)
							* (A * A / 2
									+ (5 - T + 9 * C + 4 * C * C) * A * A * A
											* A / 24
									+ (61 - 58 * T + T * T + 600 * C - 330 * e2)
											* A * A * A * A * A * A / 720)));

	if (Lat < 0) {
		*northing += 10000000; //10000000 meter offset for southern hemisphere
		*north = false;
	} else
		*north = true;

	return true;
}

//
//void GNSS::SetBaudRate(int baudrate,  com_port, SerialPort *SerialPort_GPS){
//
//	com_
//	strcpy(SerialPort_GPS->uartName, com_port); // novatel
//	SerialPort_GPS->baudRate = baudrate;
//}
