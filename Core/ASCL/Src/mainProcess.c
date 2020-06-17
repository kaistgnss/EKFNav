/*
 * mainProcess.c
 *
 *  Created on: Jun 15, 2020
 *      Author: KWJANG
 */



#include "mainProcess.h"

static int mainLoopCount 	= 	0;
static float a= 1.0;
void MainInitialize()
{
	// Initialize Main functions

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	//HAL_UART_Receive_DMA(&UART_VN200, (unsigned char *)VN200_rxBuffer, LENGTH_VN200_RXBUF);
}

void Tim6Process()
{
	// 1000Hz loop

	mainLoopCount++;

	if (mainLoopCount%TOGGLE_COUNT==0)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}

	// dmaCounter = __HAL_DMA_GET_COUNTER(&DMA_VN200);

}

void Tim7Process()
{
	// 1Hz loop

	printf("Welcome to ASCL Code\r\n");
	printf("%f \n",a);
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
}
