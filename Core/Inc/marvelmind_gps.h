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

#define MARVELMIND_BUFFER_SIZE 		    250
#define PACKET_START_BYTE               0xFF
#define PACKET_TYPE_STREAM_FROM_HEDGE   0x47
#define HEDGEHOG_POS_PACKET_ID          0x0001
#define POS_PAYLOAD_SIZE                0x10

typedef union {
    uint8_t b[2];
    uint16_t w;
} uni_8x2_16;

typedef union {
    uint8_t b[8];
    int64_t vi64;
} uni_8x8_64;

typedef struct {
	int16_t  x, y, z;    // cm;
    uint8_t size;
    uint8_t  id;
    bool updated;
} MarvelmindUserPayload;

void Marvelmind_Init(void);
void Marvelmind_ProcessByte(uint8_t byte);
void Marvelmind_CRC16(uint8_t *buf, uint8_t size);
MarvelmindUserPayload* Marvelmind_GetPayload(void);


#endif /* INC_MARVELMIND_GPS_H_ */
