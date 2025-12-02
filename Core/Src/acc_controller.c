/*
 * acc_controller.c
 *
 *  Created on: Nov 20, 2025
 *      Author: benotter
 */


#include "acc_controller.h"
#include "acc_mag.h"

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"
#include "i2c_helper.h"


#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define ACC_FS 100.0f


Acc_Datapoint_Float acc1_data = {0,0,0};
Acc_Datapoint_Float acc2_data = {0,0,0};
Acc_Datapoint_Float acc_diff_data = {0,0,0};


Acc_Datapoint_Float gyr_lpf = {0,0,0};


static BiquadLP gy_lp;
void biquad_init_lowpass(BiquadLP *b, float cutoff_hz, float fs_hz) {
    float w0 = 2.0f * M_PI * cutoff_hz / fs_hz;
    float cosw = cosf(w0);
    float sinw = sinf(w0);
    float Q = 0.707106f;
    float alpha = sinw / (2.0f * Q);

    float b0 = (1.0f - cosw) / 2.0f;
    float b1 = (1.0f - cosw);
    float b2 = (1.0f - cosw) / 2.0f;
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cosw;
    float a2 =  1.0f - alpha;

    b->b0 = b0 / a0;
    b->b1 = b1 / a0;
    b->b2 = b2 / a0;
    b->a1 = a1 / a0;
    b->a2 = a2 / a0;

    b->z1 = 0.0f;
    b->z2 = 0.0f;
}

float biquad_process(BiquadLP *b, float x) {
    float y = b->b0 * x + b->z1;
    b->z1 = b->b1 * x - b->a1 * y + b->z2;
    b->z2 = b->b2 * x - b->a2 * y;
    return y;
}


void lpf_init(float fs) {
    biquad_init_lowpass(&gy_lp, 1.0f, fs);  // 0.5 Hz cutoff
}

float compute_DC_offset(int num_samples) {
    float sum = 0.0f;
    for(int i=0; i<num_samples; i++){
        sample_converted_acc_data(&hi2c1, &acc1_data);
        sample_converted_acc_data(&hi2c2, &acc2_data);
        float diff = acc1_data.y - acc2_data.y;
        sum += diff;
        HAL_Delay(5); // small delay for sensor
    }
    return sum / num_samples;
}

static float DC_offset = 0;
void acc_controller_init(void) {
	acc12_init();
	lpf_init(ACC_FS);
	HAL_Delay(1000);
	DC_offset = compute_DC_offset(100);
}

//static float acc_diff_DC_error = 0;
//
//void acc_controller_calibration(void) {
//	Acc_Datapoint_Float cal_data = {0, 0, 0};
//	int settling_iterations = 500;
//	for (int i = 0; i < settling_iterations; i++) {
//		update_gyr_data(&cal_data, 1);
//		printf("x: %f, y: %f, z: %f\r\n", 0.0f, cal_data.y, 0.0f);
//	}
//	int num_samples = 100;
//	// float samples[num_samples];
//	// while(cal_data.y == 0) update_gyr_data(&cal_data);
//	float avg = 0;
//	for (int i = 0; i < num_samples; i++) {
//		update_gyr_data(&cal_data, 1);
//		//samples[i] = cal_data.y;
//		avg += cal_data.y;
//	}
//	acc_diff_DC_error = avg / num_samples;
//}
//


#define AVG_WINDOW 15
float acc1_y_moving_average_buf[AVG_WINDOW];
float acc2_y_moving_average_buf[AVG_WINDOW];
int acc1_y_moving_average_i = 0;
int acc2_y_moving_average_i = 0;

float moving_average(float* buf, int* i, float data_point) {
	buf[*i] = data_point;
	*i = (*i + 1) % AVG_WINDOW;
	float result = 0;
	for (int j = 0; j < AVG_WINDOW; j++) {
		result += buf[j];
	}
	result = result / AVG_WINDOW;
	return result;
}

#define TURNING_BUF_WINDOW 5
float turning_buf[TURNING_BUF_WINDOW];
int turning_i = 0;

#define TURNING_THRESHOLD 0.001

bool turning_logic(float val) {
	static float turning_avg;
	static bool turning;
	turning_avg = moving_average(turning_buf, &turning_i, val);
	turning = (fabs(turning_avg) > TURNING_THRESHOLD) ? 1 : 0;

	static bool crosses_zero;

	int above = 0; int below = 0;
	for (int j = 0; j < TURNING_BUF_WINDOW; j++) {
		if (turning_buf[j] > 0) above++;
		if (turning_buf[j] < 0) below++;
	}
	crosses_zero = (above == TURNING_BUF_WINDOW || below == TURNING_BUF_WINDOW) ? 0 : 1;
	return turning & !crosses_zero;
	return turning;
}



// int calibrate_after = 1000;
bool update_gyr_data(Acc_Datapoint_Float* gyr_data_out, bool lp) {
	sample_converted_acc_data(&hi2c1, &acc1_data);
	// acc1_data.y = moving_average(acc1_y_moving_average_buf, &acc1_y_moving_average_i, acc1_data.y);
	sample_converted_acc_data(&hi2c2, &acc2_data);
	// acc2_data.y = moving_average(acc2_y_moving_average_buf, &acc2_y_moving_average_i, acc2_data.y);

	// acc_diff_data.x = acc1_data.x - acc2_data.x;
	acc_diff_data.y = acc1_data.y - acc2_data.y - DC_offset;
	// acc_diff_data.z = acc1_data.z - acc2_data.z;

	// low pass filter first order
//	if (!gyr_lpf_initialized) {
//		    gyr_lpf = acc_diff_data;
//		    gyr_lpf_initialized = true;
//	}
	// gyr_lpf.y = acc_diff_data.y + LPF_ALPHA * (gyr_lpf.y - acc_diff_data.y);

	gyr_lpf.y = lp ? biquad_process(&gy_lp, acc_diff_data.y) : acc_diff_data.y;
	static bool turning;
	turning = turning_logic(gyr_lpf.y);



	// gyr_lpf.y = acc_diff_data.y;

	// set to difference acc for now
	// *gyr_data_out = acc_diff_data;
	*gyr_data_out = gyr_lpf;
	return turning;
}

