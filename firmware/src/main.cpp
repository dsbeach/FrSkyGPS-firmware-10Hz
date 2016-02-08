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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "TinyGPS++.h"
#include "GPSController.h"
#include "SPortController.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

TinyGPSPlus gpsDevice;

int
main (int argc, char* argv[])
{

  // configure peripheral clocks
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);   // Enable GPIO A bank clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE); // Enable USART1 clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); // Enable USART2 clock
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  Timer::start(); // Timer uses free running TIM2 only

  // the gpsDevice will be fed a NEMA data stream from UART1
  // all parsing is done in the interrupt handler as the bytes arrive
  GPSController::setGPSDevice(&gpsDevice);

  // SPort protocol is feed thru uart2 in rs485 mode
  SPortController::setGPSDevice(&gpsDevice);

  // Infinite loop
  while (1)
    {
    }
  // Infinite loop, never return.
  return 0;
}

#pragma GCC diagnostic pop
