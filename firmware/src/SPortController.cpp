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

#include <SPortController.h>
#include "cmsis_device.h"

TinyGPSPlus* SPortController::gpsDevice = 0;
FrSkySportSensorGps SPortController::gpsSensor = FrSkySportSensorGps(FrSkySportSensor::ID1);
SPortController::State SPortController::state = SPortController::START_FRAME;
bool SPortController::hasStuffing = false;
uint16_t SPortController::crc = 0;
FrSkySportSensor::SportPacket SPortController::packet;
uint8_t SPortController::sendBuffer[32];
uint16_t SPortController::sendIndex;
uint16_t SPortController::sendCRC;

void SPortController::addByteToBuffer(uint8_t byte) {
  if(byte == FRSKY_TELEMETRY_START_FRAME) // must be stuffed
  {
    sendBuffer[sendIndex++] = FRSKY_STUFFING;
    sendBuffer[sendIndex++] = 0x5E; // 0x7E xor 0x20
  }
  else if(byte == FRSKY_STUFFING) // stuff the stuffing as well
  {
      sendBuffer[sendIndex++] = FRSKY_STUFFING;
      sendBuffer[sendIndex++] = 0x5D; // 0x7D xor 0x20
  }
  else
  {
      sendBuffer[sendIndex++] = byte;
  }
  sendCRC += byte;
  sendCRC += sendCRC >> 8; sendCRC &= 0x00ff;
  sendCRC += sendCRC >> 8; sendCRC &= 0x00ff;
}

void SPortController::sendPacket(FrSkySportSensor::SportPacket* packet) {
  for (sendIndex = 0; sendIndex < 32; sendIndex++) {
      sendBuffer[sendIndex] = 0;
  }
  sendIndex = 0;
  addByteToBuffer(FRSKY_SENSOR_DATA_FRAME);

  uint8_t *bytes = (uint8_t*)&packet->appId;
  addByteToBuffer(bytes[0]);
  addByteToBuffer(bytes[1]);
  bytes = (uint8_t*)&packet->data;
  addByteToBuffer(bytes[0]);
  addByteToBuffer(bytes[1]);
  addByteToBuffer(bytes[2]);
  addByteToBuffer(bytes[3]);
  addByteToBuffer(0xFF - sendCRC);
  sendUART2(sendBuffer, sendIndex);
}

void SPortController::decodeByte(uint8_t byte)
{
  if (byte == FRSKY_TELEMETRY_START_FRAME) {
    state = SENSOR_ID;
  }          // Regardless of the state restart state machine when start frame found
  else
  {
    if (hasStuffing == true) { byte ^= 0x20; hasStuffing = false; }                             // Xor next byte with 0x20 to remove stuffing
    if ((byte == FRSKY_STUFFING) && (state > DATA_FRAME) && (state <= CRC_BYTE)) hasStuffing = true; // Skip stuffing byte in data and mark to xor next byte with 0x20
    else if (state == SENSOR_ID) {
      packet.id = byte;
      packet.appId = 0;
      packet.data = 0;
      state = DATA_FRAME;
      if (packet.id == gpsSensor.sensorId) {
          gpsDevice->pause(true);
          gpsSensor.encodeData(&packet);
          // if sensor has no data an empty packet will be sent according to spec
          gpsDevice->pause(false);
          Timer::wait(4);
          sendPacket(&packet);
        }
    }
    else if ((state == DATA_FRAME) && (byte == FRSKY_SENSOR_DATA_FRAME))
    {
      crc = byte;                                                                              // Data frame found, initialize the CRC and start collecting APP ID
      state = APP_ID_BYTE_1;
    }
    else if (state == APP_ID_BYTE_1) {
      ((uint8_t*)&packet.appId)[0] = byte;
      state = APP_ID_BYTE_2; // APP ID first byte collected, look for second byte
    }
    else if (state == APP_ID_BYTE_2) {
      ((uint8_t*)&packet.appId)[1] = byte;
      state = DATA_BYTE_1; // APP ID second byte collected, store APP ID and start looking for DATA
    }
    else if (state == DATA_BYTE_1) {
      ((uint8_t*)&packet.data)[0] = byte;
      state = DATA_BYTE_2; // DATA first byte collected, look for second byte
    }
    else if (state == DATA_BYTE_2) {
      ((uint8_t*)&packet.data)[1] = byte;
      state = DATA_BYTE_3; // DATA second byte collected, look for third byte
    }
    else if (state == DATA_BYTE_3) {
      ((uint8_t*)&packet.data)[2] = byte;
      state = DATA_BYTE_4; // DATA third byte collected, look for fourth byte
    }
    else if (state == DATA_BYTE_4) {
      ((uint8_t*)&packet.data)[3] = byte;
      state = CRC_BYTE; // DATA fourth byte collected, store DATA and look for CRC
    }
    else if (state == CRC_BYTE) {
      if (byte == (0xFF - crc)) {
        hasStuffing = false;
        state = START_FRAME;
      }
      else {
        // log CRC failure
      }
    }
    else {
      // packet overrun - ignore the bytes until we pickup a new frame
      if ((byte != 0x00) && (byte != 0xff)) {
        // complain - something is probably not good!
      }
      hasStuffing = false;
      state = START_FRAME;
      packet.id = 0;
      packet.appId = 0;
      packet.data = 0;
    }

    // Update CRC value
    if ((state > APP_ID_BYTE_1) && (state <= CRC_BYTE) && (hasStuffing == false)) { crc += byte; crc += crc >> 8; crc &= 0x00ff; }
  }
}

void SPortController::setGPSDevice(TinyGPSPlus* theDevice)
{
  SPortController::gpsDevice = theDevice;
  SPortController::gpsSensor.addGPSDevice(theDevice);
  SPortController::initUART2();
}

void
SPortController::initUART2 (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  USART_InitTypeDef USART2_InitStructure;

  GPIO_PinAFConfig (GPIOA, GPIO_PinSource1, GPIO_AF_1);
  GPIO_PinAFConfig (GPIOA, GPIO_PinSource2, GPIO_AF_1);
  GPIO_PinAFConfig (GPIOA, GPIO_PinSource3, GPIO_AF_1);

  /* Configure pin1 for Data Enable -------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART2 pins:  Rx and Tx ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init (GPIOA, &GPIO_InitStructure);

  /* Enable USART2 IRQ */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStructure);

  /* Enable the DMA Channel 4 Interrupt for transmit */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStructure);

  USART2_InitStructure.USART_BaudRate = 57600;
  USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART2_InitStructure.USART_StopBits = USART_StopBits_1;
  USART2_InitStructure.USART_Parity = USART_Parity_No;
  USART2_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART2_InitStructure.USART_Mode = USART_Mode_Rx;

  USART_OverrunDetectionConfig(USART2, USART_OVRDetection_Disable);

  USART_Init (USART2, &USART2_InitStructure);

  USART_DECmd(USART2, ENABLE); // rs485 mode
  USART_DEPolarityConfig(USART2, USART_DEPolarity_Low);
  USART_SetDEAssertionTime(USART2, 2);
  USART_SetDEDeassertionTime(USART2, 2);

  USART_Cmd (USART2, ENABLE);

  USART_ITConfig (USART2, USART_IT_RXNE, ENABLE); // receive via int
}

void DMA1_Channel4_5_IRQHandler(void){
  //__asm__ __volatile__ ("bkpt #0");
  // dma transfer is done, but perhaps not sent
  DMA_ITConfig( DMA1_Channel4, DMA_IT_TC, DISABLE); // done for now
  USART_ITConfig (USART2, USART_IT_TC, ENABLE);
}

void USART2_IRQHandler(void)
{
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
      uint8_t byte = (uint8_t)USART_ReceiveData(USART2);
      SPortController::decodeByte(byte);
  }
  else if (USART_GetITStatus(USART2, USART_IT_TC) != RESET)
  {
      USART_ITConfig (USART2, USART_IT_TC, DISABLE);
      USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE; // enabled, recv enabled, RXNE int enabled
      GPIO_ResetBits(GPIOA, GPIO_Pin_1); // receive mode via data enable pin
  }
  else
    {
      //__asm__ __volatile__ ("bkpt #0");
      // clear all the error flags
      if (USART_GetITStatus(USART2, USART_IT_TC) != RESET)
      {
          USART_ClearITPendingBit(USART2, USART_IT_TC);
      }
      if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
      {
          USART_RequestCmd(USART2, USART_Request_TXFRQ, ENABLE);
      }
      if (USART_GetITStatus(USART2, USART_IT_ORE) != RESET)
      {
      USART_ClearITPendingBit(USART2, USART_IT_ORE);
      }
      if (USART_GetITStatus(USART2, USART_IT_CTS) != RESET)
      {
      USART_ClearITPendingBit(USART2, USART_IT_CTS);
      }
      if (USART_GetITStatus(USART2, USART_IT_LBD) != RESET)
      {
      USART_ClearITPendingBit(USART2, USART_IT_LBD);
      }
      if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
      {
      USART_ClearITPendingBit(USART2, USART_IT_IDLE);
      }
      if (USART_GetITStatus(USART2, USART_IT_NE) != RESET)
      {
      USART_ClearITPendingBit(USART2, USART_IT_NE);
      }
      if (USART_GetITStatus(USART2, USART_IT_PE) != RESET)
      {
      USART_ClearITPendingBit(USART2, USART_IT_PE);
      }
    }
  USART_ClearFlag(USART2, USART_FLAG_TC);
  USART_ClearFlag(USART2, USART_FLAG_WU);
  USART_ClearFlag(USART2, USART_FLAG_CM);
  USART_ClearFlag(USART2, USART_FLAG_EOB);
  USART_ClearFlag(USART2, USART_FLAG_RTO);
  USART_ClearFlag(USART2, USART_FLAG_CTS);
  USART_ClearFlag(USART2, USART_FLAG_LBD);
  USART_ClearFlag(USART2, USART_FLAG_IDLE);
  USART_ClearFlag(USART2, USART_FLAG_ORE);
  USART_ClearFlag(USART2, USART_FLAG_NE);
  USART_ClearFlag(USART2, USART_FLAG_FE);
  USART_ClearFlag(USART2, USART_FLAG_PE);
}

void SPortController::sendUART2(const uint8_t data[], size_t length) {
  DMA_InitTypeDef USART2_DMA_InitStructure;

  GPIO_SetBits(GPIOA, GPIO_Pin_1); // send mode via data enable pin

  DMA_DeInit(DMA1_Channel4);
  DMA_Cmd(DMA1_Channel4, DISABLE);

  /*------------------------------- DMA---------------------------------------*/
  /* Common DMA configuration */
  USART2_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->TDR;
  USART2_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  USART2_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data;
  USART2_DMA_InitStructure.DMA_BufferSize = length;
  USART2_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  USART2_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  USART2_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  USART2_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  USART2_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  USART2_DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  USART2_DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &USART2_DMA_InitStructure);


  // put the usart in transmit mode
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE; // enabled, xmit enabled, TC int enabled
  DMA_ITConfig( DMA1_Channel4, DMA_IT_TC, ENABLE);
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
  DMA_Cmd(DMA1_Channel4, ENABLE);

}

