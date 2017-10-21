/*
 * pid.h
 *
 *  Created on: Oct 15, 2017
 *      Author: matejko
 */

#ifndef PID_H_
#define PID_H_
typedef struct {
	float p_gain, i_gain, d_gain, period;
	int min, max;
	float *feedback, *output;
} PidConfig;

void pidUpdate(float goal, PidConfig *cnofig);
#endif /* PID_H_ */
