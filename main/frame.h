#ifndef SCIENTISST_FRAME_H
#define SCIENTISST_FRAME_H

#include "esp_attr.h"
#include "stdint.h"

// clang-format off
#define CALC_BYTE_CRC(_crc, _byte, _crc_table)({    \
    (_crc) = _crc_table[(_crc)] ^ ((_byte) >> 4);   \
    (_crc) = _crc_table[(_crc)] ^ ((_byte) & 0x0F); \
})
// clang-format on

void IRAM_ATTR frameFill(uint8_t *frame, uint16_t *adc_internal_res, uint32_t *adc_external_res, uint8_t io_state);

#endif