/*
 * linux_kbhit.h
 *
 *  Created on: Sep 15, 2017
 *      Author: gnss
 */

#ifndef INCLUDE_LINUX_KBHIT_H_
#define INCLUDE_LINUX_KBHIT_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>

int linux_kbhit(){
	struct termios oldt, newt;
	int ch;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;

	newt.c_lflag &= ~(ICANON|ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}



#endif /* INCLUDE_LINUX_KBHIT_H_ */
