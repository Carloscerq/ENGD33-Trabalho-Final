/*
 * marvelmind_gps.c
 *
 *  Created on: Jul 5, 2025
 *      Author: felipe
 */


#include "marvelmind_gps.h"

static uint8_t  mm_buffer[MARVELMIND_BUFFER_SIZE];
static uint8_t  mm_buf_ofs = 0;
static uint8_t  mm_packet_size = 0;
static uint8_t  mm_packet_type = 0;
static uint16_t mm_packet_id = 0;

static MarvelmindUserPayload payload;

void Marvelmind_Init(void) {
    mm_buf_ofs = 0;
    mm_packet_id = 0;
    payload.updated = false;
}

static void restart_packet(void) {
    mm_buf_ofs = 0;
    mm_packet_id = 0;
}

static void process_write_packet(void) {
    if (mm_packet_id == HEDGEHOG_POS_PACKET_ID) {
        if (mm_buffer[4] < 8) return;

        payload.size = mm_buffer[4] - 8;

        payload.id        = mm_packet_id;
        payload.x         = (int16_t)( mm_buffer[13] | (mm_buffer[14] << 8) );
        payload.y         = (int16_t)( mm_buffer[15] | (mm_buffer[16] << 8) );
        payload.z         = (int16_t)( mm_buffer[17] | (mm_buffer[18] << 8) );
        payload.updated   = true;
    }
}

void Marvelmind_ProcessByte(uint8_t byte) {
    if (mm_buf_ofs >= MARVELMIND_BUFFER_SIZE) {
        restart_packet();
        return;
    }

    if (mm_buf_ofs == 0) {
            if (byte != PACKET_START_BYTE) {
                return;
            }
            mm_buffer[mm_buf_ofs++] = byte;
            return;
        }

    if (mm_buf_ofs == 1) {
            if (byte != PACKET_TYPE_STREAM_FROM_HEDGE) {
                restart_packet();
                return;
            }
            mm_packet_type       = byte;
            mm_buffer[mm_buf_ofs++] = byte;
            return;
        }

    if (mm_buf_ofs == 2) {
            mm_buffer[mm_buf_ofs++] = byte;
            return;
	}
	if (mm_buf_ofs == 3) {
		mm_buffer[mm_buf_ofs++] = byte;
		mm_packet_id = mm_buffer[2] | (mm_buffer[3] << 8);
		if (mm_packet_id != HEDGEHOG_POS_PACKET_ID) {
			restart_packet();
			return;
		}
		return;
	}

	if (mm_buf_ofs == 4) {
		mm_buffer[mm_buf_ofs++] = byte;
		// packet size = byte (payload size) + header(5) + CRC(2)
		mm_packet_size = byte + 7;
		if (byte < POS_PAYLOAD_SIZE) {
			restart_packet();
			return;
		}
		return;
	}

    mm_buffer[mm_buf_ofs++] = byte;

    if (mm_buf_ofs > 5 && mm_buf_ofs == mm_packet_size) {
        Marvelmind_CRC16(mm_buffer, mm_packet_size);
        if (mm_buffer[mm_packet_size] == 0 && mm_buffer[mm_packet_size + 1] == 0) {
            process_write_packet();
        }
        restart_packet();
    }
}

void Marvelmind_CRC16(uint8_t *buf, uint8_t size) {
    uni_8x2_16 sum;
    sum.w = 0xFFFF;

    for (int byte_cnt = size; byte_cnt > 0; byte_cnt--) {
        sum.w = (uint16_t)(((sum.w >> 8) << 8) + ((sum.w & 0xFF) ^ buf[size - byte_cnt]));
        for (int shift_cnt = 0; shift_cnt < 8; shift_cnt++) {
            if (sum.w & 0x01) sum.w = (sum.w >> 1) ^ 0xA001;
            else sum.w >>= 1;
        }
    }

    buf[size] = sum.b[0];
    buf[size + 1] = sum.b[1];
}

MarvelmindUserPayload* Marvelmind_GetPayload(void) {
    return &payload;
}
