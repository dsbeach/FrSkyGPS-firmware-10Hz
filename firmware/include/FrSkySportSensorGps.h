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
  FrSky GPS sensor class for Teensy 3.x and 328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20150725
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_GPS_H_
#define _FRSKY_SPORT_SENSOR_GPS_H_

#include "FrSkySportSensor.h"
#include "TinyGPS++.h"
#include "Timer.h"

#define GPS_DEFAULT_ID ID4
#define GPS_DATA_COUNT 7
#define GPS_LAT_LON_DATA_ID   0x0800
#define GPS_ALT_DATA_ID       0x0820
#define GPS_SPEED_DATA_ID     0x0830
#define GPS_COG_DATA_ID       0x0840
#define GPS_DATE_TIME_DATA_ID 0x0850

#define GPS_LAT_LON_DATA_PERIOD   50  // alternate lat, lon
#define GPS_ALT_DATA_PERIOD       500
#define GPS_SPEED_DATA_PERIOD     500
#define GPS_COG_DATA_PERIOD       1000
#define GPS_DATE_TIME_DATA_PERIOD 500 // alternate date, time each 1/2 second


class FrSkySportSensorGps : public FrSkySportSensor
{
  public:
    FrSkySportSensorGps(SensorId id = ID1, bool decodeOnly = true);
    void addGPSDevice(TinyGPSPlus* gpsDevice);
    void setData(float lat, float lon, float alt, float speed, float cog, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
    virtual void encodeData(FrSkySportSensor::SportPacket* packet);
    virtual void decodeData(FrSkySportSensor::SportPacket* packet);

  private:
    static uint32_t setLatLon(float latLon, bool isLat);
    static uint32_t setDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate);
    TinyGPSPlus* gpsDevice;
    uint32_t lat;
    uint32_t lon;
    int32_t alt;
    uint32_t speed;
    uint32_t cog;
    uint32_t date;
    uint32_t time;
    uint32_t latTime;
    uint32_t lonTime;
    uint32_t altTime;
    uint32_t speedTime;
    uint32_t cogTime;
    uint32_t dateTime;
    uint32_t timeTime;
};

#endif // _FRSKY_SPORT_SENSOR_GPS_H_
