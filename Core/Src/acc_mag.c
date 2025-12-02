/*
 * acc_mag.c
 *
 *  Created on: Nov 20, 2025
 *      Author: benotter
 */


#include "acc_mag.h"


#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"
#include "i2c_helper.h"



#define ACC_ADDR 	(0x19 << 1) // bit shifted << 1
#define MAG_ADDR 	(0x1E << 1) // bit shifted << 1

#define ACC_X_L  0x28
#define ACC_X_H  0x29
#define ACC_Y_L  0x2A
#define ACC_Y_H  0x2B
#define ACC_Z_L  0x2C
#define ACC_Z_H  0x2D


#define WHO_AM_I_A_Addr 0x0F
#define WHO_AM_I_M_Addr 0x4F

#define CTRL_REG1_A 0x20
#define CTRL_REG1_A_Settings 0b01010111
	// 0101 selects 100 Hz
	// 0 selects "normal mode" for power
	// 111 enables xyz

#define G_Force_Conversion 16384.0f




void squwak_acc_mag(I2C_HandleTypeDef* i2c_ch, char* i2c_channel_name) {
	uint8_t data_received = 0;
	printf("\nSquawking IMU on %s\r\n", i2c_channel_name);
	data_received = i2c_read_one_val(i2c_ch, ACC_ADDR, WHO_AM_I_A_Addr);
	printf("WHO_AM_I_A = 0x%02X\r\n", data_received);
	data_received = i2c_read_one_val(i2c_ch, MAG_ADDR, WHO_AM_I_M_Addr);
	printf("WHO_AM_I_M = 0x%02X\r\n", data_received);
}


void acc12_init(void) {
	printf("Check both lines are HIGH: \r\nSDA = %d, SCL = %d\r\n",
	         HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0),   // replace with SDA pin
	         HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1));  // replace with SCL pin

	squwak_acc_mag(&hi2c1, "I2C ch 1");
	squwak_acc_mag(&hi2c2, "I2C ch 2");

	i2c_write_one_val(&hi2c1, ACC_ADDR, CTRL_REG1_A, CTRL_REG1_A_Settings);
	printf("\nConfigured Accelerometer 1 at 100Hz, in Normal Power Mode, and XYZ axes enabled\r\n");
	i2c_write_one_val(&hi2c2, ACC_ADDR, CTRL_REG1_A, CTRL_REG1_A_Settings);
	printf("\nConfigured Accelerometer 2 at 100Hz, in Normal Power Mode, and XYZ axes enabled\r\n");

	HAL_Delay(500);
}


// wrapper for default off
void sample_raw_acc_data(I2C_HandleTypeDef* i2c_ch, Acc_Datapoint_Raw* data_out) {
	uint8_t buffer[6];

	uint8_t increment_enabled_ACC_X_L = (1 << 7) | ACC_X_L;
	i2c_read_x_vals(i2c_ch, ACC_ADDR, increment_enabled_ACC_X_L, buffer, 6);

	data_out->x = (int16_t)((buffer[1] << 8) | buffer[0]);
	data_out->y = (int16_t)((buffer[3] << 8) | buffer[2]);
	data_out->z = (int16_t)((buffer[5] << 8) | buffer[4]);

	// printf("Raw bytes: %02X %02X %02X %02X %02X %02X\r\n",
	//       buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
}


void convert_raw_acc_to_force(Acc_Datapoint_Raw* data_in, Acc_Datapoint_Float* data_out) {
	data_out->x = (float)data_in->x / G_Force_Conversion;
	data_out->y = (float)data_in->y / G_Force_Conversion;
	data_out->z = (float)data_in->z / G_Force_Conversion;
}

void sample_converted_acc_data(I2C_HandleTypeDef* i2c_ch, Acc_Datapoint_Float* data_out) {
	static Acc_Datapoint_Raw raw_data;
	sample_raw_acc_data(i2c_ch, &raw_data);
	convert_raw_acc_to_force(&raw_data, data_out);
}

