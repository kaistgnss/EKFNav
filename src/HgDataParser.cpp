#include "HgDataParser.h"
#include "Serial.h"

void HgDataParser::ReadPort()
{
	status = serial_init(&SerialHandle, comPort);
	if (status!=0)
	{
		printf("failed to open Comport:%s\n",comPort);
	}
	status = serial_configure(SerialHandle, baudrate, SER_PARITY_NO, 1, 8);
	if (status!=0)
	{
		printf("failed to configure Com port:%s\tstatus:%d\n",comPort,status);
	}
	printf("Input COM:\t%s\t%d\n",comPort,baudrate);
}

void HgDataParser::ReadIMU()
{
	HgDataParser::HG4930InertialMessage Message;
	Message.ZeroMessage();
	status = serial_init(&SerialHandle, comPort);
	//Checksum errors counter
	int noChecksumErrors = 0;

	memcpy(ReadBuffer, ReadBufferAct, (ReadBufferEnd - ReadBufferAct));
	ReadBufferEnd = ReadBuffer + (ReadBufferEnd - ReadBufferAct);
	ReadBufferAct = ReadBuffer;

	if (!serial_read(SerialHandle, (char*)ReadBufferEnd, BUFFER_SIZE, &BytesRead))
	{
		printf("\nFailed to read Data!\n");
	}
	else
	{
		ReadBufferEnd += (int)BytesRead;

		while (ReadBufferAct <= ReadBufferEnd - INERTIAL_MESSAGE_LEN)
		{
			if (*(UINT8*)ReadBufferAct == 0x0E)
			{
				switch (*(UINT8*)(ReadBufferAct + 1))
				{
				#ifdef HG4930
				case 0x01:
				{status = HgDataParser::GetHG4930X01ControlMessage(ReadBufferAct, 0, &Message.ControlMessage); ReadBufferAct += 19; break; }
				case 0x02:
				{status = HgDataParser::GetHG4930X02InertialMessage(ReadBufferAct, 0, &Message); ReadBufferAct += 43; break; }
				#endif
				default:
				{ status = 2; }
				}
				gyro[0]=-Message.ControlMessage.AngularRate[1];
				gyro[1]=Message.ControlMessage.AngularRate[2];
				gyro[2]=-Message.ControlMessage.AngularRate[0];
				acc[0]=-Message.ControlMessage.LinearAcceleration[1];
				acc[1]=Message.ControlMessage.LinearAcceleration[2];
				acc[2]=-Message.ControlMessage.LinearAcceleration[0];
				num++;
				boost::posix_time::ptime present_time(
					boost::posix_time::microsec_clock::universal_time());
				boost::posix_time::time_duration duration(present_time.time_of_day());
				time = (duration.total_microseconds()) / 1000000.0;
			}
			ReadBufferAct++;
		}
	}
	serial_close(SerialHandle);
}

int HgDataParser::HgChecksum(UINT8 *buffer, int startOffset, int byteLength)
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


int HgDataParser::GetHG4930X01ControlMessage(UINT8 *buffer, int startOffset, struct HG4930ControlMessage *Message)
{
	return HgDataParser::Deserialize_Control(buffer, startOffset, Message, 0x01);
}

int HgDataParser::GetHG4930X02InertialMessage(UINT8 *buffer, int startOffset, struct HG4930InertialMessage *Message)
{
	return HgDataParser::Deserialize_Inertial(buffer, startOffset, Message, 0x02);
}

int HgDataParser::Deserialize_Control(UINT8 *buffer, int startOffset, struct HG4930ControlMessage *Message, UINT8 MsgType)
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

	return HgDataParser::HgChecksum(buffer, startOffset, 18);
}

int HgDataParser::Deserialize_Inertial(UINT8 *buffer, int startOffset, struct HG4930InertialMessage *Message, UINT8 MsgType)
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


