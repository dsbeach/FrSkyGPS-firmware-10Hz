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

#ifndef SPORTCONTROLLER_H_
#define SPORTCONTROLLER_H_

#include "TinyGPS++.h"
#include "FrSkySportSensorGps.h"

#define FRSKY_TELEMETRY_START_FRAME 0x7E
#define FRSKY_SENSOR_DATA_FRAME 0x10
#define FRSKY_STUFFING 0x7D
#define SENSOR_NO_DATA_ID 0x00


 /* Enable counter */

extern "C" {
    void USART2_IRQHandler(void);
    void DMA1_Channel4_5_IRQHandler(void);
}

class SPortController
{
public:
public:
  enum State { START_FRAME = 0, SENSOR_ID = 1, DATA_FRAME = 2, APP_ID_BYTE_1 = 3, APP_ID_BYTE_2 = 4, DATA_BYTE_1 = 5, DATA_BYTE_2 = 6, DATA_BYTE_3 = 7, DATA_BYTE_4 = 8, CRC_BYTE = 9 };
  static void setGPSDevice(TinyGPSPlus* gpsDevice);
  static void sendUART2(const uint8_t[], size_t length);
  static State state;
  static void decodeByte(uint8_t byte);
private:
  static TinyGPSPlus* gpsDevice;
  static FrSkySportSensorGps gpsSensor;
  static void initUART2 (void);
  static bool  hasStuffing;
  static uint16_t crc;
  static FrSkySportSensor::SportPacket packet;
  static void sendPacket(FrSkySportSensor::SportPacket* packet);
  static uint8_t sendBuffer[32];
  static uint16_t sendIndex;
  static uint16_t sendCRC;
  static void addByteToBuffer(uint8_t);
};

#endif /* SPORTCONTROLLER_H_ */
