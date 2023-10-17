//
// Created by kiko on 10/10/23.
//

#pragma once

#include <stdint.h>
#include <sys/cdefs.h>

#include "sci_scientisst.h"

extern const uint8_t crc_table[16];
extern uint16_t crc_seq;

_Noreturn void taskAcquisition(void);
