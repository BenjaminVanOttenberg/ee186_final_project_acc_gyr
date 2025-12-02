/*
 * i2c_helper.h
 *
 *  Created on: Nov 20, 2025
 *      Author: benotter
 */

#ifndef INC_I2C_HELPER_H_
#define INC_I2C_HELPER_H_


#include "main.h"
#include "i2c.h"



uint8_t i2c_read_one_val(I2C_HandleTypeDef* i2c_ch, uint8_t device_addr, uint8_t reg_addr);

void i2c_write_one_val(I2C_HandleTypeDef* i2c_ch, uint8_t device_addr, uint8_t reg_addr, uint8_t data_write);

void i2c_read_x_vals(I2C_HandleTypeDef* i2c_ch, uint8_t device_addr, uint8_t reg_addr, uint8_t* buffer, int datapoints);



#endif /* INC_I2C_HELPER_H_ */
