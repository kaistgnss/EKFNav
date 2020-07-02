#include "HgDataParser.h"

namespace HgDataParser
{
	/*-----------------------------------------------------------------------------------------------------------------------------*/
	//COMMON
	/*-----------------------------------------------------------------------------------------------------------------------------*/
	// Deserialize Data Overloaded functions definition
	int Deserialize(UINT8 *buffer, int startOffset, struct HG1120InertialMessage* Message, UINT8 MsgType);
	int Deserialize(UINT8 *buffer, int startOffset, struct HG1120ControlMessage* Message, UINT8 MsgType);
	int Deserialize(UINT8 *buffer, int startOffset, struct HG1120X06AHRSMessage *Message, UINT8 MsgType);
	int Deserialize(UINT8 *buffer, int startOffset, struct HG4930ControlMessage *Message, UINT8 MsgType);
	int Deserialize(UINT8 *buffer, int startOffset, struct HG4930InertialMessage *Message, UINT8 MsgType);

	/// <summary>Calculate HG IMU standartized Checksum</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="byteLength">MSG Byte length - 2 (excluding checksum from message)</param>
	/// <returns>0 - OK, 1 - Incorrect</returns>
	int HgChecksum(UINT8 *buffer, int startOffset, int byteLength)
	{

		UINT16 u16sum = 0;
		UINT8* pb = &buffer[startOffset];

		for (int i = 0; i < byteLength; i = i + 2)
		{
			u16sum += *(UINT16 *)(pb + i);
		}
		UINT16 Checksum = *(UINT16 *)(pb + byteLength);

		if (Checksum == u16sum)
			return 0;
		else
			return 1;
	}

	/*-----------------------------------------------------------------------------------------------------------------------------*/
	//HG1120
	/*-----------------------------------------------------------------------------------------------------------------------------*/

	/// <summary>Fill HG1120 Control Message structure from raw byte array containing 0x04 Control Message</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <returns>(inherited from deserialize function) 0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int GetHG1120X04ControlMessage(UINT8 *buffer, int startOffset, struct HG1120ControlMessage* Message)
	{
		return Deserialize(buffer, startOffset, Message, 0x04);
	}

	/// <summary>Fill HG1120 Control Message structure from raw byte array containing 0x0C Control Message</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <returns>(inherited from deserialize function) 0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int GetHG1120X0CControlMessage(UINT8 *buffer, int startOffset, struct HG1120ControlMessage* Message)
	{
		return Deserialize(buffer, startOffset, Message, 0x0C);
	}

	/*-----------------------------------------------------------------------------------------------------------------------------*/

	/// <summary>Fill HG1120 Inertial Message structure from raw byte array containing 0x05 Inertial Message</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <returns>(inherited from deserialize function) 0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int GetHG1120X05InertialMessage(UINT8 *buffer, int startOffset, struct HG1120InertialMessage* Message)
	{
		return Deserialize(buffer, startOffset, Message, 0x05);
	}

	/// <summary>Fill HG1120 Inertial Message structure from raw byte array containing 0x0D Inertial Message</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <returns>(inherited from deserialize function) 0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int GetHG1120X0DInertialMessage(UINT8 *buffer, int startOffset, struct HG1120InertialMessage* Message)
	{
		return Deserialize(buffer, startOffset, Message, 0x0D);
	}

	/*-----------------------------------------------------------------------------------------------------------------------------*/

	/// <summary>Fill HG1120 AHRS Message structure from raw byte array containing 0x06 AHRS Message</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <returns>(inherited from deserialize function) 0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int GetHG1120X06AHRSMessage(UINT8 *buffer, int startOffset, struct HG1120X06AHRSMessage * Message)
	{
		return Deserialize(buffer, startOffset, Message, 0x06);
	}
	/*-----------------------------------------------------------------------------------------------------------------------------*/

	/// <summary>HG1120 Control Message from raw byte array Deserialize overloaded function</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <param name="MsgType">0x definition of message type</param>
	/// <returns>0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int Deserialize(UINT8 *buffer, int startOffset, struct HG1120ControlMessage* Message, UINT8 MsgType)
	{
		if (startOffset < 0)
			return -1;

		UINT8* pb = &buffer[startOffset];


		Message->IMUAddress = pb[0];
		Message->MessageID = pb[1];

		if (MsgType == 0x04)
		{
			// rad/s
			Message->AngularRate[0] = *(short*)(pb + 2) * 0.001144409f;
			Message->AngularRate[1] = *(short*)(pb + 4) * 0.001144409f;
			Message->AngularRate[2] = *(short*)(pb + 6) * 0.001144409f;

			// m/s2
			Message->LinearAcceleration[0] = *(short*)(pb + 8) * 0.02232422f;
			Message->LinearAcceleration[1] = *(short*)(pb + 10) * 0.02232422f;
			Message->LinearAcceleration[2] = *(short*)(pb + 12) * 0.02232422f;
		}
		else if (MsgType == 0x0C)
		{
			// rad/s
			Message->AngularRate[0] = *(short*)(pb + 2) * 0.0005722046f;
			Message->AngularRate[1] = *(short*)(pb + 4) * 0.0005722046f;
			Message->AngularRate[2] = *(short*)(pb + 6) * 0.0005722046f;
			// m/s2
			Message->LinearAcceleration[0] = *(short*)(pb + 8) * 0.01116211f;
			Message->LinearAcceleration[1] = *(short*)(pb + 10) * 0.01116211f;
			Message->LinearAcceleration[2] = *(short*)(pb + 12) * 0.01116211f;
		}
		else
		{
			return -2;
		}

		Message->MagField[0] = *(short*)(pb + 14) * 0.438404f;
		Message->MagField[1] = *(short*)(pb + 16) * 0.438404f;
		Message->MagField[2] = *(short*)(pb + 18) * 0.438404f;

		Message->MultiplexedCounter = (UINT8)((pb[20] >> 0) & 0x0F);


		Message->StatusWord.IMUOK = (pb[20] & 0x10) != 0;
		Message->StatusWord.SensorBoardInitializationSuccessful = (pb[20] & 0x20) != 0;
		Message->StatusWord.AccelerometerXValidity = (pb[20] & 0x40) != 0;
		Message->StatusWord.AccelerometerYValidity = (pb[20] & 0x80) != 0;
		Message->StatusWord.AccelerometerZValidity = (pb[21] & 0x01) != 0;
		Message->StatusWord.GyroXValidity = (pb[21] & 0x02) != 0;
		Message->StatusWord.GyroYValidity = (pb[21] & 0x04) != 0;
		Message->StatusWord.GyroZValidity = (pb[21] & 0x08) != 0;
		Message->StatusWord.MagnetometerValidity = (pb[21] & 0x10) != 0;
		Message->StatusWord.PowerUpBITStatus = (pb[21] & 0x20) != 0;
		Message->StatusWord.ContinuousBITStatus = (pb[21] & 0x40) != 0;
		Message->StatusWord.PowerUpTest = (pb[21] & 0x80) != 0;


		/* Template for Multiplexed Status word*/
		/* a lot of device reporting not used - create a message variable if needed*/
		switch (Message->MultiplexedCounter)
		{
		case 0:
			Message->SoftwareVersionNumber = (*(short*)(pb + 22));  // SoftwareVersionNumber - "Software Version Number"
			break;
		case 1:
			//SensorElectronicsStatus = (pb[22] & 0x01) != 0; // SensorElectronicsStatus - "Sensor Electronics status"
			//SensorDataReadyStatus = (pb[22] & 0x02) != 0; // SensorDataReadyStatus - "Sensor Data Ready status"
			//TemperatureStatus = (pb[22] & 0x04) != 0; // TemperatureStatus - "Temperature status"
			//AccelerometerXHealth = (pb[22] & 0x08) != 0; // AccelerometerXHealth - "Accelerometer X Health"
			//AccelerometerYHealth = (pb[22] & 0x10) != 0; // AccelerometerYHealth - "Accelerometer Y Health"
			//AccelerometerZHealth = (pb[22] & 0x20) != 0; // AccelerometerZHealth - "Accelerometer Z Health"
			//GyroXHealth = (pb[22] & 0x40) != 0; // GyroXHealth - "Gyro X Health"
			//GyroYHealth = (pb[22] & 0x80) != 0; // GyroYHealth - "Gyro Y Health"
			//GyroZHealth = (pb[23] & 0x01) != 0; // GyroZHealth - "Gyro Z Health"
			break;
		case 2:
			//AccelerometerXHealthLatched = (pb[22] & 0x08) != 0; // AccelerometerXHealthLatched - "Accelerometer X Health latched"
			//AccelerometerYHealthLatched = (pb[22] & 0x10) != 0; // AccelerometerYHealthLatched - "Accelerometer Y Health latched"
			//AccelerometerZHealthLatched = (pb[22] & 0x20) != 0; // AccelerometerZHealthLatched - "Accelerometer Z Health latched"
			//GyroXHealthLatched = (pb[22] & 0x40) != 0; // GyroXHealthLatched - "Gyro X Health latched"
			//GyroYHealthLatched = (pb[22] & 0x80) != 0; // GyroYHealthLatched - "Gyro Y Health latched"
			//GyroZHealthLatched = (pb[23] & 0x01) != 0; // GyroZHealthLatched - "Gyro Z Health latched"
			break;
		case 3:
			//MagnetometerXHealth = (pb[22] & 0x01) != 0; // MagnetometerXHealth - "Magnetometer X Health"
			//MagnetometerYHealth = (pb[22] & 0x02) != 0; // MagnetometerYHealth - "Magnetometer Y Health"
			//MagnetometerZHealth = (pb[22] & 0x04) != 0; // MagnetometerZHealth - "Magnetometer Z Health"
			break;
		case 5:
			//LoopCompletionTest = (pb[22] & 0x01) != 0;// LoopCompletionTest - "Loop Completion Test"
			//RAMTest = (pb[22] & 0x02) != 0;// RAMTest - "RAM Test"
			//CoefficientTableCRCTest = (pb[22] & 0x04) != 0;// CoefficientTableCRCTest - "Coefficient Table CRC Test"
			//ConfigurationTableCRCTest = (pb[22] & 0x08) != 0;// ConfigurationTableCRCTest - "Configuration Table CRC Test"
			//NormalModeSWCRCTest = (pb[22] & 0x10) != 0;// NormalModeSWCRCTest - "Normal Mode SW CRC Test"
			//StackOverflowTest = (pb[22] & 0x40) != 0;// StackOverflowTest - "Stack Overflow Test"
			//WatchdogTimerTest = (pb[22] & 0x80) != 0; // WatchdogTimerTest - "Watchdog Timer Test"
			//ProcessorTest = (pb[23] & 0x01) != 0;// ProcessorTest - "Processor Test"
			break;
		case 6:
			//LoopCompletionTestLatched = (pb[22] & 0x01) != 0; // LoopCompletionTestLatched - "Loop Completion Test latched"
			//RAMTestLatched = (pb[22] & 0x02) != 0;// RAMTestLatched - "RAM Test latched"
			//CoefficientTableCRCTestLatched = (pb[22] & 0x04) != 0; // CoefficientTableCRCTestLatched - "Coefficient Table CRC Test latched"
			//ConfigurationTableCRCTestLatched = (pb[22] & 0x08) != 0; // ConfigurationTableCRCTestLatched - "Configuration Table CRC Test latched"
			//NormalModeSWCRCTestLatched = (pb[22] & 0x10) != 0; // NormalModeSWCRCTestLatched - "Normal Mode SW CRC Test latched"
			//StackOverflowTestLatched = (pb[22] & 0x40) != 0; // StackOverflowTestLatched - "Stack Overflow Test latched"
			//WatchdogTimerTestLatched = (pb[22] & 0x80) != 0; // WatchdogTimerTestLatched - "Watchdog Timer Test latched"
			//ProcessorTestLatched = (pb[23] & 0x01) != 0; // ProcessorTestLatched - "Processor Test latched"
			break;
		case 7:
			Message->AccelerometerGyroSensorTemperature = (*(short*)(pb + 22)* 0.0039f);  // AccelerometerGyroSensorTemperature - "Accelerometer/Gyro Sensor Temperature"
			break;
		case 8:
			Message->MagnetometerTemperature = (*(short*)(pb + 22)* 0.0039f);  // MagnetometerTemperature - "Magnetometer Temperature"
			break;
		case 9:
			Message->DIO1 = (pb[22] & 0x01) != 0;// DIO1 - "DIO 1"
			Message->DIO2 = (pb[22] & 0x02) != 0;// DIO2 - "DIO 2"
			Message->DIO3 = (pb[22] & 0x04) != 0;// DIO3 - "DIO 3"
			Message->DIO4 = (pb[22] & 0x08) != 0;// DIO4 - "DIO 4"
			break;
		}


		Message->Checksum = (*(short*)(pb + 24));



		return HgChecksum(buffer, startOffset, 24);

	}

	/// <summary>HG1120 Inertial Message from raw byte array Deserialize overloaded function</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <param name="MsgType">0x definition of message type</param>
	/// <returns>0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int Deserialize(UINT8 *buffer, int startOffset, struct HG1120InertialMessage* Message, UINT8 MsgType)
	{
		int status = 0;
		if (startOffset < 0)
			return -1;

		UINT8* pb = &buffer[startOffset];
		if (MsgType == 0x05)
		{
			status = GetHG1120X04ControlMessage(buffer, startOffset, &Message->ControlMessage);
			// rad
			Message->DeltaAngle[0] = *(INT32*)(pb + 24) * 5.820766E-11f;
			Message->DeltaAngle[1] = *(INT32*)(pb + 28) * 5.820766E-11f;
			Message->DeltaAngle[2] = *(INT32*)(pb + 32) * 5.820766E-11f;
			// m/s
			Message->DeltaVelocity[0] = *(INT32*)(pb + 36) * 1.135468E-09f;
			Message->DeltaVelocity[1] = *(INT32*)(pb + 40) * 1.135468E-09f;
			Message->DeltaVelocity[2] = *(INT32*)(pb + 44) * 1.135468E-09f;
		}
		else if (MsgType == 0x0D)
		{
			status = GetHG1120X0CControlMessage(buffer, startOffset, &Message->ControlMessage);
			// rad
			Message->DeltaAngle[0] = *(INT32*)(pb + 24) * 1.164153E-10f;
			Message->DeltaAngle[1] = *(INT32*)(pb + 28) * 1.164153E-10f;
			Message->DeltaAngle[2] = *(INT32*)(pb + 32) * 1.164153E-10f;
			// m/s
			Message->DeltaVelocity[0] = *(INT32*)(pb + 36) * 2.270937E-09f;
			Message->DeltaVelocity[1] = *(INT32*)(pb + 40) * 2.270937E-09f;
			Message->DeltaVelocity[2] = *(INT32*)(pb + 44) * 2.270937E-09f;
		}
		else
		{
			return -2;
		}

		Message->ControlMessage.Checksum = (*(short*)(pb + 48));

		if (status >= 0)
			return HgChecksum(buffer, startOffset, 48);
		else
			return status;
	}

	/// <summary>HG1120 AHRS Message from raw byte array Deserialize overloaded function</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <param name="MsgType">0x definition of message type</param>
	/// <returns>0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int Deserialize(UINT8 *buffer, int startOffset, struct HG1120X06AHRSMessage *Message, UINT8 MsgType)
	{
		if (startOffset < 0)
			return -1;

		UINT8* pb = &buffer[startOffset];

		Message->IMUAddress = pb[0];
		Message->MessageID = pb[1];
		Message->AngularRate[0] = *(float*)(pb + 2);
		Message->AngularRate[1] = *(float*)(pb + 6);
		Message->AngularRate[2] = *(float*)(pb + 10);

		Message->LinearAcceleration[0] = *(float*)(pb + 14);
		Message->LinearAcceleration[1] = *(float*)(pb + 18);
		Message->LinearAcceleration[2] = *(float*)(pb + 22);

		Message->MagField[0] = *(float*)(pb + 26);
		Message->MagField[1] = *(float*)(pb + 30);
		Message->MagField[2] = *(float*)(pb + 34);

		Message->Heading[0] = *(INT32*)(pb + 38)*1.0f;
		Message->Heading[1] = *(INT32*)(pb + 42)*1.0f;
		Message->Heading[2] = *(INT32*)(pb + 46)*1.0f;

		Message->SoftwareVersionNumber = (*(short*)(pb + 50));
		Message->AccelerometerGyroSensorTemperature = (*(float*)(pb + 52));
		Message->MagnetometerTemperature = (*(float*)(pb + 56));
		Message->Counter = (UINT8)((pb[60] >> 0) & 0x0F);
		Message->DIO1 = (pb[60] & 0x10) != 0;
		Message->DIO2 = (pb[60] & 0x20) != 0;
		Message->DIO3 = (pb[60] & 0x40) != 0;
		Message->DIO4 = (pb[60] & 0x80) != 0;

		Message->StatusWord.IMUOK = (pb[62] & 0x01) != 0;
		Message->StatusWord.SensorBoardInitializationSuccessful = (pb[62] & 0x02) != 0;
		Message->StatusWord.AccelerometerXValidity = (pb[62] & 0x04) != 0;
		Message->StatusWord.AccelerometerYValidity = (pb[62] & 0x08) != 0;
		Message->StatusWord.AccelerometerZValidity = (pb[62] & 0x10) != 0;
		Message->StatusWord.GyroXValidity = (pb[62] & 0x20) != 0;
		Message->StatusWord.GyroYValidity = (pb[62] & 0x40) != 0;
		Message->StatusWord.GyroZValidity = (pb[62] & 0x80) != 0;
		Message->StatusWord.MagnetometerValidity = (pb[63] & 0x01) != 0;
		Message->StatusWord.PowerUpBITStatus = (pb[63] & 0x02) != 0;
		Message->StatusWord.ContinuousBITStatus = (pb[63] & 0x04) != 0;
		Message->StatusWord.PowerUpTest = (pb[63] & 0x08) != 0;

		/* Template for Additional Status words
		"Sensor Electronics status" = (pb[63] & 0x10) != 0);
		"Sensor Data Ready status" = (pb[63] & 0x20) != 0);
		"Temperature status" = (pb[63] & 0x40) != 0);
		"Accelerometer X Health" = (pb[63] & 0x80) != 0);
		"Accelerometer Y Health" = (pb[64] & 0x01) != 0);
		"Accelerometer Z Health" = (pb[64] & 0x02) != 0);
		"Gyro X Health" = (pb[64] & 0x04) != 0);
		"Gyro Y Health" = (pb[64] & 0x08) != 0);
		"Gyro Z Health" = (pb[64] & 0x10) != 0);
		"Accelerometer X Health latched" = (pb[64] & 0x20) != 0);
		"Accelerometer Y Health latched" = (pb[64] & 0x40) != 0);
		"Accelerometer Z Health latched" = (pb[64] & 0x80) != 0);
		"Gyro X Health latched" = (pb[65] & 0x01) != 0);
		"Gyro Y Health latched" = (pb[65] & 0x02) != 0);
		"Gyro Z Health latched" = (pb[65] & 0x04) != 0);
		"Magnetometer X Health" = (pb[65] & 0x08) != 0);
		"Magnetometer Y Health" = (pb[65] & 0x10) != 0);
		"Magnetometer Z Health" = (pb[65] & 0x20) != 0);
		"Loop Completion Test" = (pb[65] & 0x40) != 0);
		"RAM Test" = (pb[65] & 0x80) != 0);
		"Coefficient Table CRC Test" = (pb[66] & 0x01) != 0);
		"Configuration Table CRC Test" = (pb[66] & 0x02) != 0);
		"Normal Mode SW CRC Test" = (pb[66] & 0x04) != 0);
		"Stack Overflow Test" = (pb[66] & 0x08) != 0);
		"Watchdog Timer Test" = (pb[66] & 0x10) != 0);
		"Processor Test" = (pb[66] & 0x20) != 0);
		"Loop Completion Test latched" = (pb[66] & 0x40) != 0);
		"RAM Test latched" = (pb[66] & 0x80) != 0);
		"Coefficient Table CRC Test latched" = (pb[67] & 0x01) != 0);
		"Configuration Table CRC Test latched" = (pb[67] & 0x02) != 0);
		"Normal Mode SW CRC Test latched" = (pb[67] & 0x04) != 0);
		"Stack Overflow Test latched" = (pb[67] & 0x08) != 0);
		"Watchdog Timer Test latched" = (pb[67] & 0x10) != 0);
		"Processor Test latched" = (pb[67] & 0x20) != 0);*/
		Message->Checksum = (*(short*)(pb + 68));

		return HgChecksum(buffer, startOffset, 68);
	}




	/*-----------------------------------------------------------------------------------------------------------------------------*/
	//HG4930
	/*-----------------------------------------------------------------------------------------------------------------------------*/

	/// <summary>Fill HG4930 Control Message structure from raw byte array containing 0x01 Control Message</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <returns>(inherited from deserialize function) 0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int GetHG4930X01ControlMessage(UINT8 *buffer, int startOffset, struct HG4930ControlMessage *Message)
	{
		return Deserialize(buffer, startOffset, Message, 0x01);
	}
	/*-----------------------------------------------------------------------------------------------------------------------------*/

	/// <summary>Fill HG4930 Inertial Message structure from raw byte array containing 0x02 Inertial Message</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <returns>(inherited from deserialize function) 0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int GetHG4930X02InertialMessage(UINT8 *buffer, int startOffset, struct HG4930InertialMessage *Message)
	{
		return Deserialize(buffer, startOffset, Message, 0x02);
	}

	/*-----------------------------------------------------------------------------------------------------------------------------*/

	/// <summary>HG4930 Control Message from raw byte array Deserialize overloaded function</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <param name="MsgType">0x definition of message type</param>
	/// <returns>0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int Deserialize(UINT8 *buffer, int startOffset, struct HG4930ControlMessage *Message, UINT8 MsgType)
	{
		if (startOffset < 0)
			return -1;

		UINT8 *pb = &buffer[startOffset];

		Message->IMUAddress = pb[0];
		Message->MessageID = pb[1];

		if (MsgType == 0x01)
		{
			// rad/s
			Message->AngularRate[0] = *(short*)(pb + 2) * 0.0005722046f;
			Message->AngularRate[1] = *(short*)(pb + 4) * 0.0005722046f;
			Message->AngularRate[2] = *(short*)(pb + 6) * 0.0005722046f;
			// m/s2
			Message->LinearAcceleration[0] = *(short*)(pb + 8) * 0.01116211f;
			Message->LinearAcceleration[1] = *(short*)(pb + 10) * 0.01116211f;
			Message->LinearAcceleration[2] = *(short*)(pb + 12) * 0.01116211f;
		}
		else
		{
			return -2;
		}
		Message->MultiplexedCounter = (UINT8)((pb[14] >> 0) & 0x03);
		Message->ModeIndicator = (UINT8)((pb[14] >> 2) & 0x03);
		Message->StatusWord.IMU = (pb[14] & 0x10) != 0;
		Message->StatusWord.Gyro = (pb[14] & 0x20) != 0;
		Message->StatusWord.Accelerometer = (pb[14] & 0x40) != 0;
		Message->StatusWord.GyroOK = (pb[14] & 0x80) != 0;
		Message->StatusWord.GyroXValidity = (pb[15] & 0x01) == 0 /*false fail*/;
		Message->StatusWord.GyroYValidity = (pb[15] & 0x02) == 0 /*false fail*/;
		Message->StatusWord.GyroZValidity = (pb[15] & 0x04) == 0 /*false fail*/;
		Message->StatusWord.IMUOK = (pb[15] & 0x80) != 0;
		Message->StatusWord2A2BFlag = ((pb[17] & 0x80) != 0) ? (UINT8)1 : (UINT8)0;
		if (Message->StatusWord2A2BFlag == 0)
			Message->SoftwareVersionNumber = (*(short*)(pb + 16));

		if (Message->StatusWord2A2BFlag == 1)
			Message->Temperature = ((INT8)pb[16]);

		Message->StatusWord.GyroHealth = (pb[17] & 0x01) != 0;
		Message->StatusWord.StartDataFlag = (pb[17] & 0x02) != 0;
		Message->StatusWord.ProcessTest = (pb[17] & 0x04) != 0;
		Message->StatusWord.MemoryTest = (pb[17] & 0x08) != 0;
		Message->StatusWord.ElectronicsTest = (pb[17] & 0x10) != 0;
		Message->StatusWord.GyroHealth2 = (pb[17] & 0x20) != 0;
		Message->StatusWord.AccelerometerHealth = (pb[17] & 0x40) != 0;
		Message->Checksum = (*(short*)(pb + 18));

		return HgChecksum(buffer, startOffset, 18);
	}

	/// <summary>HG4930 Inertial Message from raw byte array Deserialize overloaded function</summary>
	/// <param name="buffer">Buffer containing binary/hex data</param>
	/// <param name="startOffset">Byte containing the IMU Address (0 based)</param>
	/// <param name="Message">container structure</param>
	/// <param name="MsgType">0x definition of message type</param>
	/// <returns>0 - OK | -1 - incorrect start offset | -2 - incorrect message type | 1 - Incorrect Checksum</returns>
	int Deserialize(UINT8 *buffer, int startOffset, struct HG4930InertialMessage *Message, UINT8 MsgType)
	{
		if (startOffset < 0)
			return -1;

		int status = 0;
		UINT8 *pb = &buffer[startOffset];
		if (MsgType == 0x02)
		{
			status = GetHG4930X01ControlMessage(buffer, startOffset, &Message->ControlMessage);
			// rad
			Message->DeltaAngle[0] = *(INT32*)(pb + 18) * 1.164153E-10f;
			Message->DeltaAngle[1] = *(INT32*)(pb + 22) * 1.164153E-10f;
			Message->DeltaAngle[2] = *(INT32*)(pb + 26) * 1.164153E-10f;
			// m/s
			Message->DeltaVelocity[0] = *(INT32*)(pb + 30) * 2.270937E-09f;
			Message->DeltaVelocity[1] = *(INT32*)(pb + 34) * 2.270937E-09f;
			Message->DeltaVelocity[2] = *(INT32*)(pb + 38) * 2.270937E-09f;
		}
		else
		{
			return -2;
		}
		Message->ControlMessage.Checksum = (*(short*)(pb + 42));

		if (status >= 0)
			return HgChecksum(buffer, startOffset, 42);
		else
			return status;

	}

}
