/*
 * pid.h
 *
 *  Created on: Oct 15, 2017
 *      Author: matejko
 */

#ifndef PID_H_
#define PID_H_
typedef struct {
	float p_gain, i_gain, d_gain, goal,u, integral,e;
	int min, max;
	volatile float *y;
} PidConfig;

void pid(volatile PidConfig *cnofig);
#endif /* PID_H_ */
