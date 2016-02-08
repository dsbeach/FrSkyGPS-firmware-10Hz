/*
    FrSkyGPS 10Hz firmware
    Copyright (C) 2016  David S. Beach
    mailto:david.s.beach@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

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
