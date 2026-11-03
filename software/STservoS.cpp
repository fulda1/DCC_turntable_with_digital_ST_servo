/*
  Small library for Waveshare ST3215 servo   
  
  Created by Jindra Fucik / https://www.fucik.name
  
*/

#include "STservoS.h"                                                                                 // Header


STservoS* pointerToSTservo;                                                         // Pointer to the servo Class

/**********************************************************************************************************************/
/* Constructor and Destructor */

STservoS::STservoS() {                                                                                    // Class constructor
}                                                                                                       // empty for now


STservoS::~STservoS(void) {                                                                                   // Class Destructor
}

/**********************************************************************************************************************/
/* Initializing Library */

void STservoS::init(void) {
  pointerToSTservo = this;                                                                             // I assign the pointer the address of the following class
  uartDmaInit();              // init serial port
}

/**********************************************************************************************************************/
/* init uart for communication */
void STservoS::uartDmaInit()
{
    GPIO_InitTypeDef gpio = {};
    USART_InitTypeDef uart = {};

    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOD |
        RCC_APB2Periph_USART1, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // USART1 default pins: PD5 = TX, PD6 = RX.
    // Verify the pin mapping for your package/board.
    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio);

    uart.USART_BaudRate = BAUD_RATE;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);
}

/**********************************************************************************************************************/
/* enable DMA for uart Rx */
void STservoS::startRxDma(uint16_t length)
{
    DMA_InitTypeDef dma = {};

    DMA_DeInit(DMA1_Channel5);

    dma.DMA_PeripheralBaseAddr =
        (uint32_t)&USART1->DATAR;
    dma.DMA_MemoryBaseAddr = (uint32_t)rxBuffer;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = length; //BUFFER_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel5, &dma);
    DMA_Cmd(DMA1_Channel5, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}

/**********************************************************************************************************************/
/* enable DMA for uart Tx */
void STservoS::startTxDma(uint16_t length)
{
    DMA_InitTypeDef dma = {};

    DMA_DeInit(DMA1_Channel4);

    dma.DMA_PeripheralBaseAddr =
        (uint32_t)&USART1->DATAR;
    dma.DMA_MemoryBaseAddr = (uint32_t)txBuffer;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = length;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_M2M = DMA_M2M_Disable;

    DMA_Init(DMA1_Channel4, &dma);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel4, ENABLE);
}

/**********************************************************************************************************************/
/* stop unused Rx DMA */
void STservoS::stopRxDma()
{
    USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);
    DMA_Cmd(DMA1_Channel5, DISABLE);
}

/**********************************************************************************************************************/
/* find servo by ping */

void STservoS::Find(void) {
    ServoAddress = SendPing();
}

/**********************************************************************************************************************/
/* is servo address valid? */

bool STservoS::IsValid(void) {
    return ServoAddress < 254;
}

uint8_t STservoS::LastErr(void) {
    return ServoErr;
}

/**********************************************************************************************************************/
/* Send command to uart and receive response */
void STservoS::SendReceiveData(uint8_t RespLen) {
    //for (uint16_t i = 0; i < BUFFER_SIZE; i++)
    //  rxBuffer[i] = 0;

    // Arm reception BEFORE transmitting.
    startRxDma(RespLen);

    // Transmit the request using DMA.
    startTxDma(txLength);

    while (DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET)
    {
        // Wait for DMA transmission to complete.
    }

    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA_Cmd(DMA1_Channel4, DISABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);

    // DMA completion means the last byte reached the USART
    // data register, not necessarily the TX pin.
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
    }

    // Wait for the first response byte.
    uint32_t start = millis();
    uint32_t lastByteTime = start;
    bool receivedAny = false;

    while ((uint32_t)(millis() - start) < RESPONSE_TIMEOUT_MS)
    {
        uint16_t remaining =
            DMA_GetCurrDataCounter(DMA1_Channel5);

        uint16_t received = RespLen - remaining;

        if (received > 0)
        {
            if (!receivedAny || received != rxLength)
            {
                receivedAny = true;
                rxLength = received;
                lastByteTime = millis();
            }

            // End the response after a quiet interval.
            if ((uint32_t)(millis() - lastByteTime)
                >= INTERBYTE_TIMEOUT_MS)
                break;
        }

        if (remaining == 0)
            break;
    }

    rxLength = RespLen -
        DMA_GetCurrDataCounter(DMA1_Channel5);

    stopRxDma();

    // rxBuffer[0 .. rxLength-1] contains the response.
    // Process the received bytes here.
    // Example: set a breakpoint and inspect rxBuffer.
}

/**********************************************************************************************************************/
/* Send PING command and return address from received packet */
uint8_t STservoS::SendPing() {
  uint8_t ReTryCnt = 0;
  do {
    // 0xFF 0xFF 0xFE 0x02 0x01 0xFE
    txLength = 6;
    txBuffer[0] = 0xFF;
    txBuffer[1] = 0xFF;
    txBuffer[2] = 0xFE; // 0xFE = broadcast address
    txBuffer[3] = 0x02;
    txBuffer[4] = 0x01;
    txBuffer[5] = 0xFE;

    SendReceiveData(6); // response should be 6 bytes

    if (rxLength != 6) {
      ServoErr = 1;
      rxBuffer[2] = 0xFF;
    } else {
      ServoErr=0; // No error
      uint8_t Check = 0;
      for (int i = 2; i < 6; i++) {Check += rxBuffer[i];}
      if ((Check != 0xFF) || (rxBuffer[0] != 0xFF) || (rxBuffer[1] != 0xFF)) {ServoErr =2;rxBuffer[2] = 0xFF;} 
      if (rxBuffer[4] != 0) {ServoErr =3;rxBuffer[2] = 0xFF;}    // some error??
    }
    ReTryCnt++;
  } while ((ReTryCnt < 3) && (ServoErr > 0));

  return rxBuffer[2];   // servo address
}


/**********************************************************************************************************************/
/* Send RESET command to servo - this will reset RAM variables to defaults or read them from sensors (useful when rotate over 0 */
uint8_t STservoS::SendReset(void) {
  // 0xFF 0xFF ID 0x02 0x0A CHK
  txLength = 6;
  txBuffer[0] = 0xFF;
  txBuffer[1] = 0xFF;
  txBuffer[2] = ServoAddress;
  txBuffer[3] = 0x02;
  txBuffer[4] = 0x0A;
  txBuffer[5] = ~(ServoAddress + 12);

  SendReceiveData(6); // response should be 6 bytes

  if (rxLength != 6) {ServoErr = 1; return 0xFF;}
  uint8_t Check = 0;
  for (int i = 2; i < 6; i++) {Check += rxBuffer[i];}
  if ((Check != 0xFF) || (rxBuffer[0] != 0xFF) || (rxBuffer[1] != 0xFF)) {ServoErr =2; return 0xFF;} 
  if (rxBuffer[4] != 0) {ServoErr =3;}    // some error??

  return rxBuffer[2];   // servo address
}


/**********************************************************************************************************************/
/* Send READ command for one byte (8 bit) register, return value of register */
uint8_t STservoS::ReadByte(uint8_t ServoReg) {
  int ReTryCount = 0;
  do {
    // 0xFF 0xFF ID 0x04 0x02 REG 0x01 CHK
    txLength = 8;
    txBuffer[0] = 0xFF;
    txBuffer[1] = 0xFF;
    txBuffer[2] = ServoAddress;
    txBuffer[3] = 0x04;
    txBuffer[4] = 0x02;
    txBuffer[5] = ServoReg;
    txBuffer[6] = 0x01;
    txBuffer[7] = ~(ServoAddress + ServoReg + 7);

    SendReceiveData(7); // response should be 7 bytes

    if (rxLength != 7) {
      ServoErr = 1;
      rxBuffer[5] = 0xFF;
    } else {
      ServoErr=0; // No error
      uint8_t Check = 0;
      for (int i = 2; i < 7; i++) {Check += rxBuffer[i];}
      if ((Check != 0xFF) || (rxBuffer[0] != 0xFF) || (rxBuffer[1] != 0xFF)) {ServoErr =2; rxBuffer[5]=0xFF;} 
      if (rxBuffer[4] != 0) {ServoErr =3;rxBuffer[5]=0xFF;}    // some error??
    }
  } while ((ReTryCount < 3) && (ServoErr > 0));

  return rxBuffer[5];   // Returned value
}


/**********************************************************************************************************************/
/* Send READ command for two bytes (16 bit) register, return value of register */
uint16_t STservoS::ReadWord(uint8_t ServoReg) {
  int ReTryCount = 0;
  do {
    // 0xFF 0xFF ID 0x04 0x02 REG 0x02 CHK
    txLength = 8;
    txBuffer[0] = 0xFF;
    txBuffer[1] = 0xFF;
    txBuffer[2] = ServoAddress;
    txBuffer[3] = 0x04;
    txBuffer[4] = 0x02;
    txBuffer[5] = ServoReg;
    txBuffer[6] = 0x02;
    txBuffer[7] = ~(ServoAddress + ServoReg + 8);

    SendReceiveData(8); // response should be 8 bytes

    if (rxLength != 8) {
      ServoErr = 1;
      rxBuffer[5] = 0xFF;
      rxBuffer[6] = 0xFF;
    } else {
      ServoErr=0; // No error
      uint8_t Check = 0;
      for (int i = 2; i < 8; i++) {Check += rxBuffer[i];}
      if ((Check != 0xFF) || (rxBuffer[0] != 0xFF) || (rxBuffer[1] != 0xFF)) {ServoErr =2; rxBuffer[5] = 0xFF; rxBuffer[6] = 0xFF;} 
      if (rxBuffer[4] != 0) {ServoErr =3; rxBuffer[5] = 0xFF; rxBuffer[6] = 0xFF;}    // some error??
    }
  } while ((ReTryCount < 3) && (ServoErr > 0));

  return (256 * (uint16_t)rxBuffer[6]) + (uint16_t)rxBuffer[5];   // Returned value
}


/**********************************************************************************************************************/
/* Send WRITE command for one byte (8 bit) register, return status (0=no error) */
uint8_t STservoS::WriteByte(uint8_t ServoReg, uint8_t ServoData) {
  int ReTryCount = 0;
  do {
    // 0xFF 0xFF ID 0x04 0x03 REG DATA CHK
    txLength = 8;
    txBuffer[0] = 0xFF;
    txBuffer[1] = 0xFF;
    txBuffer[2] = ServoAddress;
    txBuffer[3] = 0x04;
    txBuffer[4] = 0x03;
    txBuffer[5] = ServoReg;
    txBuffer[6] = ServoData;
    txBuffer[7] = ~(ServoAddress + ServoReg + ServoData + 7);

    SendReceiveData(6); // response should be 6 bytes

    if (rxLength != 6) {
      ServoErr = 1;
    } else {
      ServoErr=0; // No error
      uint8_t Check = 0;
      for (int i = 2; i < 6; i++) {Check += rxBuffer[i];}
      if ((Check != 0xFF) || (rxBuffer[0] != 0xFF) || (rxBuffer[1] != 0xFF)) {ServoErr =2;} 
      if (rxBuffer[4] != 0) {ServoErr =3;}    // some error??
    }
  } while ((ReTryCount < 3) && (ServoErr > 0));

  return ServoErr;   // no error
}

/**********************************************************************************************************************/
/* Send WRITE command for two bytes (16 bit) register, return status (0=no error) */
uint8_t STservoS::WriteWord(uint8_t ServoReg, uint16_t ServoData) {
  int ReTryCount = 0;
  do {
    // 0xFF 0xFF ID 0x05 0x03 REG DAT_L DAT_H CHK
    txLength = 9;
    txBuffer[0] = 0xFF;
    txBuffer[1] = 0xFF;
    txBuffer[2] = ServoAddress;
    txBuffer[3] = 0x05;
    txBuffer[4] = 0x03;
    txBuffer[5] = ServoReg;
    txBuffer[6] = (uint8_t)(ServoData & 0x00FF);
    txBuffer[7] = (uint8_t)(ServoData >> 8);
    txBuffer[8] = ~(ServoAddress + ServoReg + (uint8_t)(ServoData & 0x00FF) + (uint8_t)(ServoData >> 8) + 8);

    SendReceiveData(6); // response should be 6 bytes

    if (rxLength != 6)  {
      ServoErr = 1;
    } else {
      ServoErr=0; // No error
      uint8_t Check = 0;
      for (int i = 2; i < 6; i++) {Check += rxBuffer[i];}
      if ((Check != 0xFF) || (rxBuffer[0] != 0xFF) || (rxBuffer[1] != 0xFF)) {ServoErr =2;} 
      if (rxBuffer[4] != 0) {ServoErr =3;}    // some error??
    }
  } while ((ReTryCount < 3) && (ServoErr > 0));

  return ServoErr;   // no error
}

/**********************************************************************************************************************/
/* End */
