#include "Serial.h"

int serial_init(int * SerialHandle, char* comPort)
{
#ifdef _WIN32
	WCHAR TempWchar[64];
	WCHAR comPortWchar[64];
	size_t numConverted = 0;
	mbstowcs_s(&numConverted, TempWchar, comPort, 32);

	swprintf_s(comPortWchar, sizeof(comPortWchar) / sizeof(WCHAR), L"\\\\.\\%ls", TempWchar);
	*SerialHandle = (int)CreateFile(comPortWchar, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

	if (*SerialHandle == -1)
	{
		return -1;
	}

	return 0;
#else // UNIX
	*SerialHandle = open(comPort, O_RDWR | O_NOCTTY | O_NDELAY);
	if (*SerialHandle == -1)
		return -1;
	return 0;

#endif
}

int serial_configure(int SerialHandle, int baudrate, int parity, int stopBits, int byteSize)
{
#ifdef _WIN32
	BYTE byParity = parity;
	BYTE byStopBits = stopBits;
	BYTE byByteSize = byteSize;
	if (!SetCommMask((HANDLE)SerialHandle, EV_RXCHAR | EV_TXEMPTY))
	{
		return -2;
	}

	//Structure for setting COM parameters
	DCB dcb = { 0 };
	dcb.DCBlength = sizeof(DCB);

	//Get COM Port State handle
	if (!GetCommState((HANDLE)SerialHandle, &dcb))
	{
		return -3;
	}

	dcb.BaudRate = baudrate;
	dcb.ByteSize = byByteSize;
	dcb.Parity = byParity;
	if (byStopBits == 1)
		dcb.StopBits = ONESTOPBIT;
	else if (byStopBits == 2)
		dcb.StopBits = TWOSTOPBITS;
	else
		dcb.StopBits = ONE5STOPBITS;

	dcb.fDsrSensitivity = 0;
	dcb.fDtrControl = DTR_CONTROL_ENABLE;
	dcb.fOutxDsrFlow = 0;

	// Set COM port settings
	if (!SetCommState((HANDLE)SerialHandle, &dcb))
	{
		return -4;
	}


	//Set Serial Timeouts
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutMultiplier = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 50;
	timeouts.WriteTotalTimeoutConstant = 10;

	if (!SetCommTimeouts((HANDLE)SerialHandle, &timeouts))
	{
		return -5;
	}
	return 0;
#else // UNIX
	struct termios serial;

	//Serial Configuration
	serial.c_iflag = 0;
	serial.c_oflag = 0;
	serial.c_lflag = 0;
	serial.c_cflag = 0;

	serial.c_cc[VMIN] = 0;
	serial.c_cc[VTIME] = 0;

	switch (baudrate)
	{
	case 50: serial.c_cflag |= B50; break;
	case 75: serial.c_cflag |= B75; break;
	case 110: serial.c_cflag |= B110; break;
	case 134: serial.c_cflag |= B134; break;
	case 150: serial.c_cflag |= B150; break;
	case 200: serial.c_cflag |= B200; break;
	case 300: serial.c_cflag |= B300; break;
	case 600: serial.c_cflag |= B600; break;
	case 1200: serial.c_cflag |= B1200; break;
	case 1800: serial.c_cflag |= B1800; break;
	case 2400: serial.c_cflag |= B2400; break;
	case 4800: serial.c_cflag |= B4800; break;
	case 9600: serial.c_cflag |= B9600; break;
	case 19200: serial.c_cflag |= B19200; break;
	case 38400: serial.c_cflag |= B38400; break;
	case 57600: serial.c_cflag |= B57600; break;
	case 115200: serial.c_cflag |= B115200; break;
	case 230400: serial.c_cflag |= B230400; break;
	case 460800: serial.c_cflag |= B460800; break;
	case 500000: serial.c_cflag |= B500000; break;
	case 576000: serial.c_cflag |= B576000; break;
	case 921600: serial.c_cflag |= B921600; break;
	case 1000000: serial.c_cflag |= B1000000; break;
	case 1152000: serial.c_cflag |= B1152000; break;
	case 1500000: serial.c_cflag |= B1500000; break;
	case 2000000: serial.c_cflag |= B2000000; break;
	case 2500000: serial.c_cflag |= B2500000; break;
	case 3000000: serial.c_cflag |= B3000000; break;
	case 3500000: serial.c_cflag |= B3500000; break;
	case 4000000: serial.c_cflag |= B4000000; break;
	default:return -1; //invalid Baudrate
	}

	switch (byteSize)
	{
	case 5: serial.c_cflag |= CS5; break;
	case 6: serial.c_cflag |= CS6; break;
	case 7: serial.c_cflag |= CS7; break;
	case 8: serial.c_cflag |= CS8; break;
	default: return -2;
	}

	switch (stopBits)
	{
	case 1:  break; // 1 stop bit default
	case 2: serial.c_cflag |= CSTOPB; break;
	default: return -3;
	}

	switch (parity)
	{
	case 0: break; // no parity
	case 1: serial.c_cflag |= PARODD; break; // even parity
	case 2: serial.c_cflag |= PARENB | PARODD; break; //odd parity
	default: return -4;
	}

	if (tcsetattr(SerialHandle, TCSANOW, &serial)<0)
	{
		return -5;
	}
	return 0;
#endif
}

int serial_close(int SerialHandle)
{
#ifdef _WIN32
	return CloseHandle((HANDLE)SerialHandle);
#else // UNIX
	return close(SerialHandle);
#endif
}

int serial_read(int SerialHandle, char * buffer, int bytesToRead, int * bytesRead)
{
#ifdef _WIN32
	LPOVERLAPPED OvRead = 0;
	int status = 0;
	DWORD bytesReadDword = 0;
	status = ReadFile((HANDLE)SerialHandle, buffer, bytesToRead, &bytesReadDword, OvRead);

	*bytesRead = bytesReadDword;
	return status;
#else // UNIX
	*bytesRead = read(SerialHandle, buffer, bytesToRead);
	return 1;
#endif
}

int serial_write(int SerialHandle, char * buffer, int bytesToWrite, int * bytesSent)
{
#ifdef _WIN32
	LPOVERLAPPED OvRead = 0;
	int status = 0;
	DWORD bytesSentDword = 0;
	status = WriteFile((HANDLE)SerialHandle, buffer, bytesToWrite, &bytesSentDword, OvRead);

	*bytesSent = bytesSentDword;
	return status;
#else // UNIX
	*bytesSent = write(SerialHandle, buffer, bytesToWrite);
	return 1;
#endif
}

#ifndef _WIN32 //utility functions for unix systems
int _kbhit(void)
{
	return 0;
}

void Sleep(int ms)
{
	sleep(ms / 1000.0f);
}

int fopen_s(FILE **f, const char*name, const char *mode)
{
	*f = fopen(name, mode);
	if (!*f)
		return-1;
	return 0;
}

size_t fread_s(void * ptr, size_t inBufferSize, size_t size, size_t count, FILE* stream)
{
	return fread(ptr, size, count, stream);
}

#endif

