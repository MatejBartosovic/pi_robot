/*
 * constants.h
 *
 *  Created on: Oct 15, 2017
 *      Author: matejko
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#define TRANSMISSION_RATIO 51.45 			//https://www.pololu.com/product/3073
#define IMPULSE_PER_MOTOR_TURN 1336 		//4X334 = 1336
#define PULSE2RAD(x) (x*0.004702983014356) 	//2pi/1336
#define RAD2PULSE(x) (x/0.004702983014356)	//2pi/1336
#define ENCODER_RESET_VALUE 0x8000

#endif /* CONSTANTS_H_ */
