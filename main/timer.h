#ifndef _TIMER_H
#define _TIMER_H

#include <stdio.h>

#define ABAT_CHECK_FREQUENCY 1       //1 Hz

#define TIMER_DIVIDER       16  //  Hardware timer clock divider
#define TIMER_SCALE         (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

#define TIMER_GROUP_USED 0
#define TIMER_IDX_USED 1
#define TIMER_GRP_ACQ_I2C 1
#define TIMER_IDX_ACQ_I2C 1
#define TIMER_GRP_ABAT 1
#define TIMER_IDX_ABAT 1


bool IRAM_ATTR timerGrp0Isr();
bool IRAM_ATTR timerGrp1Isr();
void timerGrpInit(int timer_group, int timer_idx, bool(*timer_isr)());
void timerStart(int timer_group, int timer_idx, uint32_t frequency);
void timerPause(int timer_group, int timer_idx);

#endif