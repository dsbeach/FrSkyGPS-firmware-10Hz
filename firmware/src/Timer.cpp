#include "Timer.h"
#include "cortexm/ExceptionHandlers.h"

#define FREQUENCY_HZ 1000u

void Timer::start(void) {
  TIM_TimeBaseInitTypeDef timerInitStructure;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Prescaler = SystemCoreClock / FREQUENCY_HZ;
  timerInitStructure.TIM_Period = 0xFFFFFFFF;
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);
  TIM_Cmd(TIM2, ENABLE);
}

void Timer::wait(uint32_t ticks) {
  uint32_t start = TIM2->CNT;
  while (TIM2->CNT - start < ticks);
}
