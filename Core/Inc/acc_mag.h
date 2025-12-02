/*
 * acc_mag.h
 *
 *  Created on: Nov 20, 2025
 *      Author: benotter
 */

#ifndef INC_ACC_MAG_H_
#define INC_ACC_MAG_H_

#include "main.h"


typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} Acc_Datapoint_Raw;

typedef struct {
    float x;
    float y;
    float z;
} Acc_Datapoint_Float;


void acc12_init(void);

void sample_raw_acc_data(I2C_HandleTypeDef* i2c_ch, Acc_Datapoint_Raw* data_out);

void convert_raw_acc_to_force(Acc_Datapoint_Raw* data_in, Acc_Datapoint_Float* data_out);

// wrapper for sample_raw_acc_data() and convert_raw_acc_to_force()
void sample_converted_acc_data(I2C_HandleTypeDef* i2c_ch, Acc_Datapoint_Float* data_out);


#endif /* INC_ACC_MAG_H_ */
