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

    This file is derived from openXsensor (see https://github.com/openXsensor/openXsensor.git)
    as noted below.

*/

/*
  FrSky sensor base class for Teensy 3.x and 328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20150725
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_H_
#define _FRSKY_SPORT_SENSOR_H_

#include <stdint.h>

class FrSkySportSensor
{
  public:
    struct SportPacket
    {
      uint8_t id;
      uint16_t appId;
      uint32_t data;
    };

    // IDs of sensors (including the CRC)
    enum SensorId { ID1 = 0x00,  ID2 = 0xA1,  ID3 = 0x22,  ID4 = 0x83,  ID5 = 0xE4,  ID6 = 0x45,  ID7 = 0xC6,
                    ID8 = 0x67,  ID9 = 0x48,  ID10 = 0xE9, ID11 = 0x6A, ID12 = 0xCB, ID13 = 0xAC, ID14 = 0x0D,
                    ID15 = 0x8E, ID16 = 0x2F, ID17 = 0xD0, ID18 = 0x71, ID19 = 0xF2, ID20 = 0x53, ID21 = 0x34,
                    ID22 = 0x95, ID23 = 0x16, ID24 = 0xB7, ID25 = 0x98, ID26 = 0x39, ID27 = 0xBA, ID28 = 0x1b };
    // virtual function for sending a byte of data 
    virtual void decodeData(SportPacket* packet);
    virtual void encodeData(SportPacket* packet);
    SensorId sensorId;
    bool decodeOnly;

  protected:
    uint8_t sensorDataIdx;
    FrSkySportSensor(SensorId id, bool decodeOnly);
  private:
    FrSkySportSensor();
};

#endif // _FRSKY_SPORT_SENSOR_H_
