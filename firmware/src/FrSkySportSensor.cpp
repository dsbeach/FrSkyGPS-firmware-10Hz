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

#include "FrSkySportSensor.h"

FrSkySportSensor::FrSkySportSensor(SensorId id, bool decodeOnly)
  : sensorId(id),decodeOnly(decodeOnly), sensorDataIdx(0) {
}

FrSkySportSensor::FrSkySportSensor(void)
  : FrSkySportSensor(FrSkySportSensor::ID1, true) {
}

void FrSkySportSensor::encodeData(FrSkySportSensor::SportPacket* packet) {
  packet->appId = 0;
  packet->data = 0;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

void FrSkySportSensor::decodeData(FrSkySportSensor::SportPacket* packet) {
  return;
}

#pragma GCC diagnostic pop
