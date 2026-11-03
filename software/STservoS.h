/*
  Small library for Waveshare ST3215 servo   
  
  Created by Jindra Fucik / https://www.fucik.name
  

*/

#ifndef STSERVOS_h
#define STSERVOS_h

#include <Arduino.h>                                                                                                        // Library for typical Arduino IDE functions
// #include <stdint.h>                                                                                                         // Type library for 'uintX_t' types


class STservoS {
    private:
		uint8_t ServoAddress = 255;               // attached servo address
		uint8_t ServoErr = 0;                     // last command error code

    static constexpr uint32_t BAUD_RATE = 1000000;
    static constexpr uint16_t BUFFER_SIZE = 10;
    static constexpr uint32_t RESPONSE_TIMEOUT_MS = 50;   // timeout value for servo response read
    static constexpr uint32_t INTERBYTE_TIMEOUT_MS = 2;   // timeout between 2 characters

    uint8_t txBuffer[BUFFER_SIZE];
    uint8_t rxBuffer[BUFFER_SIZE];

    uint16_t txLength = 0;
    uint16_t rxLength = 0;

    private:
        /*
        *   uartDmaInit() Initialze uart (speen, etc.)
        *   Input:
        *       - none
        *   Returns:
        *       - none
        */
        void uartDmaInit();
        /*
        *   startRxDma(length) Initialze DMA for Rx, length is number of bytes to be read
        *   Input:
        *       - length - nimber of bytes to be read
        *   Returns:
        *       - none
        */
        void startRxDma(uint16_t length);
        /*
        *   startTxDma(length) Initialze DMA for Tx, length is number of bytes to be send
        *   Input:
        *       - length - nimber of bytes to be read
        *   Returns:
        *       - none
        */
        void startTxDma(uint16_t length);
        /*
        *   stopRxDma() Close DMA for Rx. For cases, not all bytes are received
        *   Input:
        *       - none
        *   Returns:
        *       - none
        */
        void stopRxDma();
        /*
        *   SendReceiveData() Send data from TxBuffer and receive response to RxBuffer
        *   Input:
        *       - RespLen - expected response length
        *   Returns:
        *       - none
        */
        void SendReceiveData(uint8_t RespLen);

    public:
        /*
        *   STservoS() Class Constructor
        *   Input:
        *       - none
        *   Returns:
        *       - None
        */
        STservoS();
        /*
        *   ~STservoS() Class Destructor
        *   Input:
        *       - None
        *   Returns:
        *       - None
        */
        ~STservoS(void);
        /*
        *   init() Init library (serial port)
        *   Input:
        *       - None
        *   Returns:
        *       - None
        */
        void init(void);
        /*
        *   Find() Find the servo using servo ping command
        *   Input:
        *       - None
        *   Returns:
        *       - None
        */
        void Find(void);
        /*
        *   IsValid() return information about servo address validity
        *   Input:
        *       - None
        *   Returns:
        *       - True if address is valid
        */
        bool IsValid(void);
        /*
        *   LastErr() Error status for last operation
        *   Input:
        *       - None
        *   Returns:
        *       - last error (0=no error)
        */
        uint8_t LastErr(void);
        /*
        *   SendPing() send broadcast ping command and return address of servo from reply
        *   Input:
        *       - None
        *   Returns:
        *       - address of servo from servo response
        */
        uint8_t SendPing(void);
        /*
        *   SendReset(ServoID) send reset command to one servo and return address of servo from reply
        *   Input:
        *       - ServoID
        *   Returns:
        *       - address of servo from servo response
        */
        uint8_t SendReset(void);
        /*
        *   ReadByte(ServoID, ServoReg) Send READ command for one byte (8 bit) register, return value of register
        *   Input:
        *       - ServoID, ServoRegister
        *   Returns:
        *       - value of register
        */
        uint8_t ReadByte(uint8_t ServoReg);
        /*
        *   ReadWord(ServoReg) Send READ command for two bytes (16 bit) register, return value of register
        *   Input:
        *       - ServoRegister
        *   Returns:
        *       - value of register
        */
        uint16_t ReadWord(uint8_t ServoReg);
        /*
        *   WriteByte(ServoReg, ServoData) Send WRITE command for one byte (8 bit) register, return status (0=no error)
        *   Input:
        *       - ServoRegister, value to write
        *   Returns:
        *       - write status
        */
        uint8_t WriteByte(uint8_t ServoReg, uint8_t ServoData);
        /*
        *   WriteWord(ServoReg, ServoData) Send WRITE command for one byte (8 bit) register, return status (0=no error)
        *   Input:
        *       - ServoRegister, value to write
        *   Returns:
        *       - write status
        */
        uint8_t WriteWord(uint8_t ServoReg, uint16_t ServoData);

};

#endif
