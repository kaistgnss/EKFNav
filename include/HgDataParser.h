#pragma once

#ifdef _WIN32
#ifdef HGDATAPARSER_EXPORTS
#define HGDATAPARSER_API __declspec(dllexport)
#else
#define HGDATAPARSER_API __declspec(dllimport)
#endif
#else // UNIX
#define HGDATAPARSER_API
#endif // _WIN32


/*--------------------------------------*/
/*-----------TYPE DEFINITIONS-----------*/
/*--------------------------------------*/
#ifndef HGTYPES
typedef signed char         INT8, *PINT8;
typedef signed short        INT16, *PINT16;
typedef signed int          INT32, *PINT32;
//typedef signed __int64      INT64, *PINT64;
typedef unsigned char       UINT8, *PUINT8;
typedef unsigned short      UINT16, *PUINT16;
typedef unsigned int        UINT32, *PUINT32;
//typedef unsigned __int64    UINT64, *PUINT64;

typedef signed int LONG32, *PLONG32;

typedef unsigned int ULONG32, *PULONG32;
typedef unsigned int DWORD32, *PDWORD32;
#define HGTYPES
#endif

#ifndef HG_UTILS
#define RAD_TO_DEG 180/PI
#define DEG_TO_RAD PI*180
#define HG_UTILS
#endif
namespace HgDataParser
{
	/*--------------------------------------*/
	/*---------------COMMON-----------------*/
	/*--------------------------------------*/

	int HgChecksum(UINT8 *buffer, int startOffset, int byteLength);


	/*--------------------------------------*/
	/*---------------HG1120-----------------*/
	/*--------------------------------------*/

	// Message definitions
	//Status word Container Structure for HG1120
	struct HG1120StatusWord
	{
		bool IMUOK; // 0 = OK | 1 = Failed
		bool SensorBoardInitializationSuccessful; // 0 = OK | 1 = Failed
		bool AccelerometerXValidity; // 0 = OK | 1 = Failed
		bool AccelerometerYValidity; // 0 = OK | 1 = Failed
		bool AccelerometerZValidity; // 0 = OK | 1 = Failed
		bool GyroXValidity; // 0 = OK | 1 = Failed
		bool GyroYValidity; // 0 = OK | 1 = Failed
		bool GyroZValidity; // 0 = OK | 1 = Failed
		bool MagnetometerValidity; // 0 = OK | 1 = Failed
		bool PowerUpBITStatus; // 0 = OK | 1 = Failed
		bool ContinuousBITStatus; // 0 = OK | 1 = Failed
		bool PowerUpTest; // 0 = OK | 1 = Failed
						  //Zero Method
		void ZeroMessage()
		{
			IMUOK = false;
			SensorBoardInitializationSuccessful = false;
			AccelerometerXValidity = false;
			AccelerometerYValidity = false;
			AccelerometerZValidity = false;
			GyroXValidity = false;
			GyroYValidity = false;
			GyroZValidity = false;
			MagnetometerValidity = false;
			PowerUpBITStatus = false;
			ContinuousBITStatus = false;
			PowerUpTest = false;
		}
	};
	//Control Message Container Structure for HG1120
	struct HG1120ControlMessage
	{
		UINT8 IMUAddress;
		UINT8 MessageID; //Last Message ID (Control or Intertial)
		float AngularRate[3]; // [rad/s] X, Y, Z
		float LinearAcceleration[3]; // [m/s2] X, Y, Z
		float MagField[3]; //[mili gauss] X, Y, Z
		UINT8 MultiplexedCounter; //MUX counter value
		struct HG1120StatusWord StatusWord;
		short SoftwareVersionNumber; //Software version number
		float AccelerometerGyroSensorTemperature; // [캜]
		float MagnetometerTemperature; // [캜]
		bool DIO1; //value of DIO1 switch
		bool DIO2; //value of DIO2 switch
		bool DIO3; //value of DIO3 switch
		bool DIO4; //value of DIO4 switch
		short Checksum;
		//Zero Method
		void ZeroMessage()
		{
			IMUAddress = 0;
			MessageID = 0;
			AngularRate[0] = AngularRate[1] = AngularRate[2] = 0;
			LinearAcceleration[0] = LinearAcceleration[1] = LinearAcceleration[2] = 0;
			MagField[0] = MagField[1] = MagField[2] = 0;
			MultiplexedCounter = 0;
			StatusWord.ZeroMessage();
			SoftwareVersionNumber = 0;
			AccelerometerGyroSensorTemperature = 0;
			MagnetometerTemperature = 0;
			DIO1 = 0;
			DIO2 = 0;
			DIO3 = 0;
			DIO4 = 0;
			Checksum = 0;
		}
	};
	//Inertial Message Container Structure for HG1120
	struct HG1120InertialMessage
	{
		struct HG1120ControlMessage ControlMessage;
		float DeltaAngle[3]; // [rad] X, Y, Z
		float DeltaVelocity[3]; // [m/s] X, Y, Z
								//Zero Method
		void ZeroMessage()
		{
			ControlMessage.ZeroMessage();
			DeltaAngle[0] = DeltaAngle[1] = DeltaAngle[2] = 0;
			DeltaVelocity[0] = DeltaVelocity[1] = DeltaVelocity[2] = 0;
		}
	};


	// Get Message functions - use this to get data

	HGDATAPARSER_API int GetHG1120X04ControlMessage(UINT8 *buffer, int startOffset, struct HG1120ControlMessage* Message);
	HGDATAPARSER_API int GetHG1120X0CControlMessage(UINT8 *buffer, int startOffset, struct HG1120ControlMessage* Message);
	HGDATAPARSER_API int GetHG1120X05InertialMessage(UINT8 *buffer, int startOffset, struct HG1120InertialMessage* Message);
	HGDATAPARSER_API int GetHG1120X0DInertialMessage(UINT8 *buffer, int startOffset, struct HG1120InertialMessage* Message);


	//AHRS Control Message Container Structure for HG1120
	struct HG1120X06AHRSMessage
	{
		UINT8 IMUAddress;
		UINT8 MessageID; //Last Message ID (Control or Intertial)
		float AngularRate[3]; // [rad/s] X, Y, Z
		float LinearAcceleration[3]; // [m/s2] X, Y, Z
		float MagField[3]; // [mili gauss] X, Y, Z
		float Heading[3]; // [rad] Roll, Pitch, True Heading
		struct HG1120StatusWord StatusWord;
		short SoftwareVersionNumber;
		float AccelerometerGyroSensorTemperature; // [캜]
		float MagnetometerTemperature; // [캜]
		UINT8 Counter;
		bool DIO1;
		bool DIO2;
		bool DIO3;
		bool DIO4;
		short Checksum;
		//Zero Method
		void ZeroMessage()
		{
			IMUAddress = 0;
			MessageID = 0;
			AngularRate[0] = AngularRate[1] = AngularRate[2] = 0;
			LinearAcceleration[0] = LinearAcceleration[1] = LinearAcceleration[2] = 0;
			MagField[0] = MagField[1] = MagField[2] = 0;
			Heading[0] = Heading[1] = Heading[2] = 0;
			StatusWord.ZeroMessage();
			SoftwareVersionNumber = 0;
			AccelerometerGyroSensorTemperature = 0;
			MagnetometerTemperature = 0;
			Counter = 0;
			DIO1 = 0;
			DIO2 = 0;
			DIO3 = 0;
			DIO4 = 0;
			Checksum = 0;
		}
	};

	// Get Message functions - use this to get data

	HGDATAPARSER_API int GetHG1120X06AHRSMessage(UINT8 *buffer, int startOffset, struct HG1120X06AHRSMessage * Message);






	/*--------------------------------------*/
	/*---------------HG4930-----------------*/
	/*--------------------------------------*/

	//Message Definitions
	//Status Container Structure for HG4930
	struct HG4930StatusWord
	{
		bool IMU; // 0 = OK | 1 = Failed
		bool Gyro; // 0 = OK | 1 = Failed
		bool Accelerometer; // 0 = OK | 1 = Failed
		bool GyroOK; // 0 = OK | 1 = Failed
		bool GyroXValidity; // 0 = OK | 1 = Failed [inverted from ICD]
		bool GyroYValidity; // 0 = OK | 1 = Failed [inverted from ICD]
		bool GyroZValidity; // 0 = OK | 1 = Failed [inverted from ICD]
		bool IMUOK; // 0 = OK | 1 = Failed
		bool GyroHealth; // 0 = OK | 1 = Failed
		bool StartDataFlag; // 0 = OK | 1 = Failed
		bool ProcessTest; // 0 = OK | 1 = Failed
		bool MemoryTest; // 0 = OK | 1 = Failed
		bool ElectronicsTest; // 0 = OK | 1 = Failed
		bool GyroHealth2; // 0 = OK | 1 = Failed
		bool AccelerometerHealth; // 0 = OK | 1 = Failed
								  //Zero Method
		void ZeroMessage()
		{
			IMU = 0;
			Gyro = 0;
			Accelerometer = 0;
			GyroOK = 0;
			GyroXValidity = 0;
			GyroYValidity = 0;
			GyroZValidity = 0;
			IMUOK = 0;
			GyroHealth = 0;
			StartDataFlag = 0;
			ProcessTest = 0;
			MemoryTest = 0;
			ElectronicsTest = 0;
			GyroHealth2 = 0;
			AccelerometerHealth = 0;
		}
	};
	//Control Message Container Structure for HG4930
	struct HG4930ControlMessage
	{
		UINT8 IMUAddress;
		UINT8 MessageID; //Last Message ID (Control or Intertial)
		float AngularRate[3]; // [rad/s] X, Y, Z
		float LinearAcceleration[3]; // [m/s2] X, Y, Z
		UINT8 MultiplexedCounter; // MUX counter value
		UINT8 ModeIndicator; // 0 = Power up BIT | 1 = Continuous BIT
		UINT8 StatusWord2A2BFlag; //Indicator which status word is being sent [0 = 2A | 1 = 2B]
		struct HG4930StatusWord StatusWord;
		short SoftwareVersionNumber; //Software version number
		INT8 Temperature; // [캜]
		short Checksum; // Stored last message checksum
						//Zero Method
		void ZeroMessage()
		{
			IMUAddress = 0;
			MessageID = 0;
			AngularRate[0] = AngularRate[1] = AngularRate[2] = 0;
			LinearAcceleration[0] = LinearAcceleration[1] = LinearAcceleration[2] = 0;
			MultiplexedCounter = 0;
			ModeIndicator = 0;
			StatusWord2A2BFlag = 0;
			StatusWord.ZeroMessage();
			SoftwareVersionNumber = 0;
			Temperature = 0;
			Checksum = 0;
		}
	};
	//Inertial Message Container Structure for HG4930
	struct HG4930InertialMessage
	{
		struct HG4930ControlMessage ControlMessage;
		float DeltaAngle[3]; // [rad] X, Y, Z
		float DeltaVelocity[3]; // [m/s] X, Y, Z
								//Zero Method
		void ZeroMessage()
		{
			ControlMessage.ZeroMessage();
			DeltaAngle[0] = DeltaAngle[1] = DeltaAngle[2] = 0;
			DeltaVelocity[0] = DeltaVelocity[1] = DeltaVelocity[2] = 0;
		}
	};

	// Get Message functions - use this to get data

	HGDATAPARSER_API int GetHG4930X01ControlMessage(UINT8 *buffer, int startOffset, struct HG4930ControlMessage *Message);
	HGDATAPARSER_API int GetHG4930X02InertialMessage(UINT8 *buffer, int startOffset, struct HG4930InertialMessage *Message);


}
