/*
 * acc_controller.h
 *
 *  Created on: Nov 20, 2025
 *      Author: benotter
 */

#ifndef INC_ACC_CONTROLLER_H_
#define INC_ACC_CONTROLLER_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "acc_mag.h"



typedef struct {
    float a1, a2;
    float b0, b1, b2;
    float z1, z2;
} BiquadLP;



void acc_controller_init(void);

// void acc_controller_calibration(void);

// float compute_DC_offset(int num_samples);

bool update_gyr_data(Acc_Datapoint_Float* gyr_data_out, bool lp, float* volume);

// wrapper for just getting the enable line
bool update_gyr_data_just_enable();

#endif /* INC_ACC_CONTROLLER_H_ */
