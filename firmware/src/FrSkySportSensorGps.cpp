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

#include "FrSkySportSensorGps.h" 

FrSkySportSensorGps::FrSkySportSensorGps(SensorId id, bool decodeOnly) : FrSkySportSensor(id, decodeOnly) 
{
  gpsDevice = NULL;
  lat = 0;
  lon = 0;
  cog = 0;
  speed = 0;
  alt = 0;
  date = 0;
  time = 0;
}

void FrSkySportSensorGps::addGPSDevice(TinyGPSPlus* gpsDevice)
{
  this->gpsDevice = gpsDevice;
}

void FrSkySportSensorGps::setData(float lat, float lon, float alt, float speed, float cog, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
  FrSkySportSensorGps::lat = setLatLon(lat, true);
  FrSkySportSensorGps::lon = setLatLon(lon, false);
  FrSkySportSensorGps::cog = cog * 100;
  FrSkySportSensorGps::speed = speed * 1944; // Convert m/s to knots
  FrSkySportSensorGps::alt = alt * 100;
  FrSkySportSensorGps::date = setDateTime(year, month, day, true);
  FrSkySportSensorGps::time = setDateTime(hour, minute, second, false);
}

uint32_t FrSkySportSensorGps::setLatLon(float latLon, bool isLat)
{
  uint32_t data = (uint32_t)((latLon < 0 ? -latLon : latLon) * 60 * 10000) & 0x3FFFFFFF;
  if(isLat == false) data |= 0x80000000;
  if(latLon < 0) data |= 0x40000000;
  return data;
}

uint32_t FrSkySportSensorGps::setDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate)
{
  uint32_t data = yearOrHour;
  data <<= 8;
  data |= monthOrMinute;
  data <<= 8;
  data |= dayOrSecond;
  data <<= 8;
  if(isDate == true) data |= 0xFF;
  return data;
}

void FrSkySportSensorGps::decodeData(FrSkySportSensor::SportPacket* packet)
{
  FrSkySportSensor::decodeData(packet);
}

void FrSkySportSensorGps::encodeData(FrSkySportSensor::SportPacket* packet)
{
  uint32_t now = Timer::millis();
  switch(sensorDataIdx)
  {
    case 0:
      if(now > latTime)
      {
        latTime = now + GPS_LAT_LON_DATA_PERIOD;
        packet->appId = GPS_LAT_LON_DATA_ID;
        packet->data = setLatLon(gpsDevice->location.lat(), true);;
      }
      break;
    case 1:
      if(now > lonTime)
      {
        lonTime = now + GPS_LAT_LON_DATA_PERIOD;
        packet->appId = GPS_LAT_LON_DATA_ID;
        packet->data = setLatLon(gpsDevice->location.lng(), false);
      }
      break;
    case 2:
      if(now > altTime)
      {
        altTime = now + GPS_ALT_DATA_PERIOD;
        packet->appId = GPS_ALT_DATA_ID;
        packet->data = (uint32_t)(gpsDevice->altitude.meters() * 100);
      }
      break;
    case 3:
      if(now > speedTime)
      {
        speedTime = now + GPS_SPEED_DATA_PERIOD;
        packet->appId = GPS_SPEED_DATA_ID;
        packet->data = (uint32_t)(gpsDevice->speed.knots() * 1000);
      }
      break;
    case 4:
      if(now > cogTime)
      {
        uint32_t cog = (uint32_t)(gpsDevice->course.deg() * 100);
        // TinyGPS has an apparent bug with UBlox7 null course reporting
        // It appears that after reporting a non-zero course TinyGPS will return the date
        // when course is not present in the NEMA message
        if (cog > 36000) { // safer to correct it here for now...
            cog = 0;
        }
        cogTime = now + GPS_COG_DATA_PERIOD;
        packet->appId = GPS_COG_DATA_ID;
        packet->data = cog;
      }
      break;
    case 5:
      if(now > dateTime)
      {
        dateTime = now + GPS_DATE_TIME_DATA_PERIOD;
        packet->appId = GPS_DATE_TIME_DATA_ID;
        packet->data = setDateTime(gpsDevice->date.year() % 100, gpsDevice->date.month(), gpsDevice->date.day(), true);
      }
      break;
    case 6:
      if(now > timeTime)
      {
        timeTime = now + GPS_DATE_TIME_DATA_PERIOD;
        packet->appId = GPS_DATE_TIME_DATA_ID;
        packet->data = setDateTime(gpsDevice->time.hour(), gpsDevice->time.minute(), gpsDevice->time.second(), false);
      }
      break;
  }
  sensorDataIdx++;
  if(sensorDataIdx >= GPS_DATA_COUNT) sensorDataIdx = 0;
}
