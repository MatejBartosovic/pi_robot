/*
 * communication.h
 *
 *  Created on: Oct 15, 2017
 *      Author: matejko
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "stm32f3xx_hal.h"
#include "stdbool.h"

#define HEADER				  0xAAAA
#define MSG_TYPE_OK           0b00000001
#define MSG_TYPE_ERROR        0b00000010
#define MSG_TYPE_SETPOINT     0b00000100
#define MSG_TYPE_SETP         0b00001000
#define MSG_TYPE_SETI         0b00010000
#define MSG_TYPE_SETD         0b00100000
#define MSG_TYPE_ODOMETRY     0b01000000
#define MSG_TYPE_RESET        0b10000000
#define PERIOD 				  0.002

typedef struct __attribute__((__packed__)){
            uint16_t header;
            uint8_t msgType;
            float leftVel;
            float rightVel;
            float leftPos;
            float righrPos;
            uint8_t crc;
        } OutMsg;

typedef struct __attribute__((__packed__)){
	uint16_t header;
	uint8_t msgType;
	float data1;
	float data2;
	uint8_t crc;
} InMsg;

uint8_t computeCRC(uint8_t *buffer, unsigned int num);
void sendOdom(float* data, uint8_t MSG_TYPE,UART_HandleTypeDef* uart );
bool checkMsg(InMsg* msg);

#endif /* COMMUNICATION_H_ */
