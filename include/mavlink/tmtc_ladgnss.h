#ifndef TMTC_LADGNSS_H
#define TMTC_LADGNSS_H

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS 1
#endif

/*- Includes -----------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>





#ifdef __GNUC__
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
/* Codes for GCC */
#elif _MSC_VER
/* Codes for Visual C++ */
#include <Windows.h>
#endif

#include "mavlink/mavlink_types.h"
#include "mavlink/mavlink.h"

/* Macro & Const -------------------------------------------------------------*/
#define UART_BUF		512
#define N_BLOCK		20

/* Structure -----------------------------------------------------------------*/

#ifdef __GNUC__

#define COM9  "/dev/ttyUSB0"
#define COM6  "/dev/ttyUSB1"
#define TRUE 1
#define FALSE 0
/* Codes for GCC */
typedef struct
{
  // TODO: kblee, Modify
  int fd;
  char uartName[32];
//  int8_t uartName[32];
  uint32_t baudRate;
  int16_t nBytesRecv;
  uint16_t nBytesSend;
  uint8_t rxBuf[UART_BUF];
  uint8_t txBuf[UART_BUF];
}SerialPort;

#elif _MSC_VER
/* Codes for Visual C++ */
typedef struct
{
  HANDLE hFile;
  int8_t uartName[32];
  uint32_t baudRate;
  uint16_t nBytesRecv;
  uint16_t nBytesSend;
  uint8_t rxBuf[UART_BUF];
  uint8_t txBuf[UART_BUF];
}SerialPort;
#endif

/* Declartion ----------------------------------------------------------------*/
//bool InitUart(SerialPort* pSerialPort);
//bool InitUart(SerialPort* pSerialPort);
bool RecvBytes(SerialPort* pSerialPort);
bool SendBytes(SerialPort* pSerialPort);
bool SendMavlink(SerialPort* pSerialPort, mavlink_message_t* pMsg);

#endif
