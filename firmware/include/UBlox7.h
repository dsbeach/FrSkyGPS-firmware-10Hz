#ifndef __UBlox7_h
#define __UBlox7_h

#include <string.h>
#include <stdio.h>
#include <stdint.h>

class UBlox7 {
public:
  static const char* SET_BAUD_57600;
  static const uint8_t UBX_CFG_RATE_10HZ[14];
};
#endif
