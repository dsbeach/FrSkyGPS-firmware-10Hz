/*
 * GPSController.h
 *
 *  Created on: Feb 4, 2016
 *      Author: dbeach
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
