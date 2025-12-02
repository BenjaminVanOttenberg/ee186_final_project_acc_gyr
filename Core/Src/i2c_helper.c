/*
 * i2c_helper.c
 *
 *  Created on: Nov 20, 2025
 *      Author: benotter
 */

#include "i2c_helper.h"

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include "i2c.h"



uint8_t i2c_read_one_val(I2C_HandleTypeDef* i2c_ch, uint8_t device_addr, uint8_t reg_addr) {
	uint8_t data_received = 0;
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(i2c_ch, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &data_received, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		  printf("I2C_2: Memory Read Error %d\r\n", ret);
		  printf("HAL_I2C ErrorCode: 0x%08lX\r\n", i2c_ch->ErrorCode);
		  printf("Device Address: 0x%02x\r\n", device_addr);
		  printf("Register Address: 0x%02x\r\n", reg_addr);
	  }
	return data_received;
}

void i2c_write_one_val(I2C_HandleTypeDef* i2c_ch, uint8_t device_addr, uint8_t reg_addr, uint8_t data_write) {
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(i2c_ch, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &data_write, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		  printf("I2C_2: Memory Read Error %d\r\n", ret);
		  printf("HAL_I2C ErrorCode: 0x%08lX\r\n", i2c_ch->ErrorCode);
		  printf("Device Address: 0x%02x\r\n", device_addr);
		  printf("Register Address: 0x%02x\r\n", reg_addr);
	  }
	return;
}


void i2c_read_x_vals(I2C_HandleTypeDef* i2c_ch, uint8_t device_addr, uint8_t reg_addr, uint8_t* buffer, int datapoints) {
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(i2c_ch, device_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, buffer, datapoints, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		  printf("I2C_2: Memory Read Error %d\r\n", ret);
		  printf("HAL_I2C ErrorCode: 0x%08lX\r\n", i2c_ch->ErrorCode);
		  printf("Device Address: 0x%02x\r\n", device_addr);
		  printf("Base Register Address: 0x%02x\r\n", reg_addr);
	  }
	return;
}



