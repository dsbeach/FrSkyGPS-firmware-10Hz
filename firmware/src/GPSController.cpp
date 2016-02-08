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


#include "GPSController.h"
#include "UBlox7.h"

TinyGPSPlus* GPSController::gpsDevice = 0;
USART_InitTypeDef GPSController::USART1_InitStructure;

void GPSController::setGPSDevice(TinyGPSPlus* theDevice) {
  GPSController::gpsDevice = theDevice;
  initUART1();
  Timer::wait(5000); // give the gps time to wake up

  // set the GPS chip to 57600 baud
  sendUART1(UBlox7::SET_BAUD_57600);
  Timer::wait(2000); // it can take a while to shift up and reset
  // set uart1 to 57600
  changeUART1BaudRate(57600);
  // set the rate to 10HZ
  sendUART1(UBlox7::UBX_CFG_RATE_10HZ, 14); // off to the races!

}

void
GPSController::initUART1 (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  GPIO_PinAFConfig (GPIOA, GPIO_PinSource9, GPIO_AF_1);
  GPIO_PinAFConfig (GPIOA, GPIO_PinSource10, GPIO_AF_1);

  // sometimes when debugging we need to start 'clean'
  USART_ITConfig (USART1, USART_IT_RXNE, DISABLE);
  USART_DeInit(USART1);

  /* Configure USART1 pins:  Rx and Tx ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init (GPIOA, &GPIO_InitStructure);

  /* Enable USART1 IRQ */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3; // usart2 always first
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStructure);

  /* Enable the DMA Channel 4 Interrupt for transmit */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStructure);

  USART1_InitStructure.USART_BaudRate = 9600;
  USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART1_InitStructure.USART_StopBits = USART_StopBits_1;
  USART1_InitStructure.USART_Parity = USART_Parity_No;
  USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_OverrunDetectionConfig(USART1, USART_OVRDetection_Disable);

  USART_Init (USART1, &USART1_InitStructure);

  // all we want in interrupts is RXNE
  USART_Cmd (USART1, ENABLE);
  USART_ITConfig (USART1, USART_IT_RXNE, ENABLE);
}


void GPSController::changeUART1BaudRate(uint16_t baudRate) {
  USART_Cmd (USART1, DISABLE);
  USART_DeInit(USART1);
  USART1_InitStructure.USART_BaudRate = baudRate;
  USART_Init (USART1, &USART1_InitStructure);
  USART_Cmd (USART1, ENABLE);
  USART_ITConfig (USART1, USART_IT_RXNE, ENABLE);
}

void GPSController::sendUART1(const char *data) {
  GPSController::sendUART1((const uint8_t*)data, strlen(data));
}

void GPSController::sendUART1(const uint8_t data[], size_t length) {
  DMA_InitTypeDef USART1_DMA_InitStructure;

  DMA_DeInit(DMA1_Channel2);
  DMA_Cmd(DMA1_Channel2, DISABLE);

  /*------------------------------- DMA---------------------------------------*/
  /* Common DMA configuration */
  USART1_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->TDR;
  USART1_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  USART1_DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data;
  USART1_DMA_InitStructure.DMA_BufferSize = length;
  USART1_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  USART1_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  USART1_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  USART1_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  USART1_DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  USART1_DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  USART1_DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &USART1_DMA_InitStructure);

  DMA_ITConfig( DMA1_Channel2, DMA_IT_TC, ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  DMA_Cmd(DMA1_Channel2, ENABLE);
}

void DMA1_Channel2_3_IRQHandler(void){
  //__asm__ __volatile__ ("bkpt #0");
  // dma transfer is done, but perhaps not sent
  DMA_ITConfig( DMA1_Channel2, DMA_IT_TC, DISABLE); // done for now
  USART_ITConfig (USART1, USART_IT_TC, ENABLE);
}

void USART1_IRQHandler(void)
{
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    GPSController::gpsDevice->encode((char)USART_ReceiveData(USART1));
  }
  else if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
  {
    //__asm__ __volatile__ ("bkpt #0");
    USART_ITConfig (USART1, USART_IT_TC, DISABLE);
  }
  else
    {
      // clear any error flags
      if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
      {
          USART_ClearITPendingBit(USART1, USART_IT_TC);
      }
      if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
      {
          USART_RequestCmd(USART1, USART_Request_TXFRQ, ENABLE);
      }
      if (USART_GetITStatus(USART1, USART_IT_ORE) != RESET)
      {
      USART_ClearITPendingBit(USART1, USART_IT_ORE);
      }
      if (USART_GetITStatus(USART1, USART_IT_CTS) != RESET)
      {
      USART_ClearITPendingBit(USART1, USART_IT_CTS);
      }
      if (USART_GetITStatus(USART1, USART_IT_LBD) != RESET)
      {
      USART_ClearITPendingBit(USART1, USART_IT_LBD);
      }
      if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
      {
      USART_ClearITPendingBit(USART1, USART_IT_IDLE);
      }
      if (USART_GetITStatus(USART1, USART_IT_NE) != RESET)
      {
      USART_ClearITPendingBit(USART1, USART_IT_NE);
      }
      if (USART_GetITStatus(USART1, USART_IT_PE) != RESET)
      {
      USART_ClearITPendingBit(USART1, USART_IT_PE);
      }
    }
  USART_ClearFlag(USART1, USART_FLAG_TC);
  USART_ClearFlag(USART1, USART_FLAG_WU);
  USART_ClearFlag(USART1, USART_FLAG_CM);
  USART_ClearFlag(USART1, USART_FLAG_EOB);
  USART_ClearFlag(USART1, USART_FLAG_RTO);
  USART_ClearFlag(USART1, USART_FLAG_CTS);
  USART_ClearFlag(USART1, USART_FLAG_LBD);
  USART_ClearFlag(USART1, USART_FLAG_IDLE);
  USART_ClearFlag(USART1, USART_FLAG_ORE);
  USART_ClearFlag(USART1, USART_FLAG_NE);
  USART_ClearFlag(USART1, USART_FLAG_FE);
  USART_ClearFlag(USART1, USART_FLAG_PE);
}
