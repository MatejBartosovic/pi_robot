/*
 * communication.c
 *
 *  Created on: Oct 15, 2017
 *      Author: matejko
 */
#include "communication.h"
const uint8_t har_tableCRC[]={
            0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
            157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
            35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
            190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
            70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
            219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
            101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
            248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
            140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
            17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
            175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
            50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
            202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
            87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
            233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
            116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
    };
uint8_t computeCRC(const uint8_t *buffer, unsigned int num){
    unsigned int ind = 0;
    uint8_t msg_CRC = 0x00;

    while((ind<num)){
    	msg_CRC = har_tableCRC[msg_CRC ^ (*buffer++)];
        ind++;
    }

    return msg_CRC;
}

void sendOdom(volatile float* data, uint8_t MSG_TYPE,UART_HandleTypeDef* uart){
	uint8_t msg[25]; //this array has to have size greater than 24 because of some memory alignment problems which cause corrupted data send by dma
	msg[0] = 0xAA;
	msg[1] = 0xAA;
	msg[2] = MSG_TYPE;
	memcpy((msg+3),data,4*sizeof(float));
	msg[19] = computeCRC(msg,19);
	/*msg[0] = 0.00;
	msg[1] = 0x01;
	msg[2] = 0x02;

	msg[3] = 0x03;
	msg[4] = 0x04;
	msg[5] = 0x05;
	msg[6] = 0x06;

	msg[7] = 0x07;
	msg[8] = 0x08;
	msg[9] = 0x09;
	msg[10] =0x0a ;

	msg[11] = 0x0b;
	msg[12] = 0x0c;
	msg[13] = 0x0d;
	msg[14] = 0x0e;
	msg[15] = 0x0f;
	msg[16] = 0x10;
	msg[17] = 0x11;
	msg[18] = 0x12;
	msg[19] = 0x13;*/
	HAL_UART_Transmit_DMA(uart,msg, 20);

}

bool checkMsg(InMsg* msg){
	if((msg->header == HEADER) && (computeCRC((uint8_t*)msg,sizeof(InMsg)-1)==msg->crc)){
		return true;
	}
	return false;
}
