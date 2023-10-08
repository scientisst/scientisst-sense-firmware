/** \file timer.h
    \brief Timer functions for the Scientisst firmware.

    This file contains the functions for the timer used in the Scientisst
   firmware.
*/

#ifndef _TIMER_H
#define _TIMER_H

#include <stdio.h>

#define ABAT_CHECK_FREQUENCY 1 // 1 Hz

#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds

#define TIMER_GROUP_USED 0
#define TIMER_IDX_USED 1

#define TIMER_GRP_ABAT 1
#define TIMER_IDX_ABAT 1

bool timerGrp0Isr(void);
bool timerGrp1Isr(void);
void timerGrpInit(int timer_group, int timer_idx, bool (*timer_isr)());
void timerStart(int timer_group, int timer_idx, uint32_t frequency);
void timerPause(int timer_group, int timer_idx);

#endif
