#ifndef MAIN_TIMER_H_
#define MAIN_TIMER_H_

void gpioInterruptStop();
void gpioInterruptStart();
void gpioInterruptConfig();
void gpioInterruptTask(void *parameter);

#endif /* MAIN_TIMER_H_ */
