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
