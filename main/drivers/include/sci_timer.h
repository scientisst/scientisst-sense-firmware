/**
 * \file sci_timer.h
 * \brief Header file for sci_timer.c
 *
 * This file also contains relevant macros like timer divider and timer scale, timer groups and timer indices.
 */

#pragma once

#include <stdbool.h>
#include <stdio.h>

#include "driver/timer.h"

#include "sci_scientisst.h"

#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds

#define TIMER_GROUP_MAIN 0
#define TIMER_IDX_MAIN 1

#define TIMER_GRP_BATTERY 1
#define TIMER_IDX_BATTERY 1

bool timerGrp0Isr(void);
bool timerGrp1Isr(void);
void timerGrpInit(int timer_group, int timer_idx, bool (*timer_isr)());
void timerStart(int timer_group, int timer_idx, uint32_t frequency);
void timerPause(int timer_group, int timer_idx);
