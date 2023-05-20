
#include "frame.h"
#include "adc.h"
#include "scientisst.h"
#include <stdint.h>

void IRAM_ATTR frameFill(uint8_t *frame, uint16_t *adc_internal_res, uint32_t *adc_external_res, uint8_t io_state) {
    int i;
    uint8_t crc = 0;
    uint8_t frame_next_wr = 0;
    uint8_t wr_mid_byte_flag = 0;

    if (num_extern_active_chs) {
        // Get raw values from AX1 & AX2 (A6 and A7)
        for (i = 0; i < num_extern_active_chs; i++) {
            *(uint32_t *)(frame + frame_next_wr) |= adc_external_res[i];
            frame_next_wr += 3;
        }
    }

    sin_i++; // Increment sin iterator, doesn't matter if it's in sim or adc mode tbh, an if would cost more instructions

    // Store values of internal channels into frame
    for (i = 0; i < num_intern_active_chs; i++) {
        if (!wr_mid_byte_flag) {
            *(uint16_t *)(frame + frame_next_wr) |= adc_internal_res[i];
            frame_next_wr++;
            wr_mid_byte_flag = 1;
        } else {
            *(uint16_t *)(frame + frame_next_wr) |= adc_internal_res[i] << 4;
            frame_next_wr += 2;
            wr_mid_byte_flag = 0;
        }
    }

    // fill io state
    frame[packet_size - 2] = io_state;

    // Calculate CRC & SEQ Number---------------------------------------------------------

    // Store seq number
    *(uint16_t *)(frame + packet_size - 2) = crc_seq << 4;

    // calculate CRC (except last byte (seq+CRC) )
    for (i = 0; i < packet_size - 2; i++) {
        // calculate CRC nibble by nibble
        CALC_BYTE_CRC(crc, frame[i], crc_table);
    }

    // calculate CRC for seq
    crc = crc_table[crc] ^ (frame[packet_size - 2] >> 4);  // Calculate CRC for first 4 bits of seq
    CALC_BYTE_CRC(crc, frame[packet_size - 1], crc_table); // Calcultate CRC for last byte of seq

    crc = crc_table[crc];

    frame[packet_size - 2] |= crc;

    crc_seq++;
}