/**
 * \file sci_task_aquisition.h
 * \brief Header file for sci_task_aquisition.c
 */

#pragma once

#include <stdint.h>
#include <sys/cdefs.h>

#include "sci_scientisst.h"

extern const uint8_t crc_table[16];
extern uint16_t crc_seq;

_Noreturn void taskAcquisition(void);
