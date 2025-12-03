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
    float Q = 0.707106f; // allegedly critically damped
    // Q += 0.01f; // underdamped to purposely produce oscillations
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

#define LEAK 0.005f
#define F_LEAK_THESHOLD 0.01f
#define F_LEAK 0.1f
float biquad_process(BiquadLP *b, float x) {
	// run a low pass filter
    float y = b->b0 * x + b->z1;
    b->z1 = b->b1 * x - b->a1 * y + b->z2;
    b->z2 = b->b2 * x - b->a2 * y;

    // leaky integrator to account for sensor drift
    b->z1 *= (1.0f - LEAK);
    b->z2 *= (1.0f - LEAK);

    // add a fast leak mode for when there's a spike so it doesn't trigger a turn signal
    if (fabs(y) >= F_LEAK_THESHOLD) {
    	b->z1 *= (1.0f - F_LEAK);
    	b->z2 *= (1.0f - F_LEAK);
    }

    if (fabs(y) >= 0.02) {
        	b->z1 *= 0.5;
        	b->z2 *= 0.5;
        }

//    if (fabs(y) >= F_LEAK_THESHOLD) {
//        	b->z1 *= F_LEAK_THESHOLD / fabs(y);
//        	b->z2 *= F_LEAK_THESHOLD / fabs(y);
//    }

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


#define AVG_WINDOW 50
float volume_buf[AVG_WINDOW];
int volume_i = 0;

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



#define TURNING_WINDOW 35

float moving_average_turning(float* buf, int* i, float data_point) {
	buf[*i] = data_point;
	*i = (*i + 1) % TURNING_WINDOW;
	float result = 0;
	for (int j = 0; j < TURNING_WINDOW; j++) {
		result += buf[j];
	}
	result = result / TURNING_WINDOW;
	return result;
}

float turning_buf[TURNING_WINDOW];
int turning_i = 0;

#define TURNING_THRESHOLD 0.004
#define TURNING_WINDOW_MARGIN 5
#define TURN_HOLD_CYCLES 20

bool turning_logic(float val,  float* volume) {
	 static float turning_avg;
	 static bool turning;
	 turning_avg = moving_average_turning(turning_buf, &turning_i, val);
	 turning = (fabs(turning_avg) > TURNING_THRESHOLD) ? 1 : 0;

	static bool crosses_threshold;
	int above = 0; int below = 0;
	for (int j = 0; j < TURNING_WINDOW; j++) {
		if (turning_buf[j] > TURNING_THRESHOLD) above++;
		if (turning_buf[j] < -TURNING_THRESHOLD) below++;
	}
	crosses_threshold = (above >= TURNING_WINDOW-TURNING_WINDOW_MARGIN || below >= TURNING_WINDOW-TURNING_WINDOW_MARGIN) ? 0 : 1;

	static bool crosses_zero;
	above = 0; below = 0;
	for (int j = 0; j < TURNING_WINDOW; j++) {
		if (turning_buf[j] > 0) above++;
		if (turning_buf[j] < 0) below++;
	}
	crosses_zero = (above == TURNING_WINDOW || below == TURNING_WINDOW) ? 0 : 1;

	static bool turn_trigger;
	turn_trigger = !crosses_threshold * !crosses_zero;


	*volume = moving_average(volume_buf, &volume_i, turn_trigger ? 1.0f : 0);
	turn_trigger = (*volume > 0.3 ? 1 : 0);

	static int hold_counter = 0;
		if (turn_trigger) {
			hold_counter = TURN_HOLD_CYCLES;   // reset hold when turning detected
		} else {
			if (hold_counter > 0) hold_counter--;
		}
		turn_trigger = (hold_counter > 0);
	return turn_trigger;
	//return turn_trigger;
}



// int calibrate_after = 1000;
bool update_gyr_data(Acc_Datapoint_Float* gyr_data_out, bool lp, float* volume) {
	sample_converted_acc_data(&hi2c1, &acc1_data);
	sample_converted_acc_data(&hi2c2, &acc2_data);
	acc_diff_data.y = acc1_data.y - acc2_data.y - DC_offset;

	// acc_diff_data.y = moving_average(gyr_window_buf, &gyr_window_i, acc_diff_data.y);


	gyr_lpf.y = lp ? biquad_process(&gy_lp, acc_diff_data.y) : acc_diff_data.y;
	static bool turning;
	turning = turning_logic(gyr_lpf.y, volume);

	// set to difference acc for now
	// *gyr_data_out = acc_diff_data;
	*gyr_data_out = gyr_lpf;
	return turning;
}

