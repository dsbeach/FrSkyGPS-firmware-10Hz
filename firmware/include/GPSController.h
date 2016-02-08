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

#ifndef GPSCONTROLLER_H_
#define GPSCONTROLLER_H_

#include "cmsis_device.h"
#include "TinyGPS++.h"
#include <string.h>
#include "Timer.h"

extern "C" {
  void USART1_IRQHandler(void);
  void DMA1_Channel2_3_IRQHandler(void);
}

class GPSController
{
public:
  static void setGPSDevice(TinyGPSPlus* gpsDevice);
  static TinyGPSPlus* gpsDevice;
private:
  static void initUART1 (void);
  static void sendUART1(const char *data);
  static void sendUART1(const uint8_t[], size_t length);
  static void changeUART1BaudRate(uint16_t baudRate);
  static USART_InitTypeDef USART1_InitStructure;

};

#endif /* GPSCONTROLLER_H_ */
