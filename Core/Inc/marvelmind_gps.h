/*
 * marvelmind_gps.h
 *
 *  Created on: Jul 5, 2025
 *      Author: felipe
 */

#ifndef INC_MARVELMIND_GPS_H_
#define INC_MARVELMIND_GPS_H_


#include <stdint.h>
#include <stdbool.h>

#define MARVELMIND_BUFFER_SIZE 		250
#define GENERIC_PAYLOAD_PACKET_ID 	0x0280
#define PACKET_TYPE_WRITE_TO_DEVICE 0x4A

typedef union {
    uint8_t b[2];
    uint16_t w;
} uni_8x2_16;

typedef union {
    uint8_t b[8];
    int64_t vi64;
} uni_8x8_64;

typedef struct {
    uint8_t data[256];
    uint8_t size;
    int64_t timestamp;
    bool updated;
} MarvelmindUserPayload;

void Marvelmind_Init(void);
void Marvelmind_ProcessByte(uint8_t byte);
void Marvelmind_CRC16(uint8_t *buf, uint8_t size);
MarvelmindUserPayload* Marvelmind_GetPayload(void);


#endif /* INC_MARVELMIND_GPS_H_ */
