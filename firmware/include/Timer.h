#ifndef TIMER_H_
#define TIMER_H_

#include "cmsis_device.h"

// ----------------------------------------------------------------------------

class Timer
{
public:
  static void start(void);
  static void wait(uint32_t ticks);
  static inline uint32_t millis() { return TIM2->CNT; }
};

// ----------------------------------------------------------------------------

#endif // TIMER_H_
