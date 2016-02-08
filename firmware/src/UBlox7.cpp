#include "UBlox7.h"


// based on specifications posted at
// https://www.u-blox.com/sites/default/files/products/documents/u-blox7-V14_ReceiverDescrProtSpec_(GPS.G7-SW-12001)_Public.pdf

// checksum calculations from
// http://siliconsparrow.com/demos/nmeachecksum.php

const char* UBlox7::SET_BAUD_57600 = "$PUBX,41,1,0007,0003,57600,0*2B\r\n\0";
const uint8_t UBlox7::UBX_CFG_RATE_10HZ[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
