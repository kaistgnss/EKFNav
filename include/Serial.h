#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#include <conio.h>


#else //UNIX
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

int _kbhit(void);
void Sleep(int ms);
int fopen_s(FILE **f, const char*name, const char *mode);
size_t fread_s(void * ptr, size_t inBufferSize, size_t size, size_t count, FILE* stream);
#endif

#define SER_PARITY_NO 0
#define SER_PARITY_EVEN 1
#define SER_PARITY_ODD 2

int serial_init(int * SerialHandle, char* comPort);
int serial_close(int SerialHandle);
int serial_configure(int SerialHandle, int baudrate, int parity, int stopBits, int byteSize);
int serial_read(int SerialHandle, char * buffer, int bytesToRead, int * bytesRead);
int serial_write(int SerialHandle, char * buffer, int bytesToWrite, int * bytesSent);
