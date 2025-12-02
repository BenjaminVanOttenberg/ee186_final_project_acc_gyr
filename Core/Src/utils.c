/*
 * utils.c
 *
 *  Created on: Nov 19, 2025
 *      Author: benotter
 */



#include "utils.h"
#include "usart.h"
#include <stdint.h>


int __io_putchar(int ch) {
	HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 10);
	return ch;
}



