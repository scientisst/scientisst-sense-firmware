//
// Created by kiko on 10/10/23.
//

#ifndef SCIENTISST_SENSE_SCI_TASK_AQUISITION_H
#define SCIENTISST_SENSE_SCI_TASK_AQUISITION_H

#include <stdint.h>
#include <sys/cdefs.h>

extern const uint8_t crc_table[16];
extern uint16_t crc_seq;

_Noreturn void task_acquisition(void *not_used);

#endif // SCIENTISST_SENSE_SCI_TASK_AQUISITION_H
