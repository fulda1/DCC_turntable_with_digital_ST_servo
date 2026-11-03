/*
Basic DCC stack for experiments
*/

#include <STservoS.h>

#define LED_BUILTIN PC0
#define DCC_IN PC5        // DCC input is also T1ETR channel = External Timer Reset = can be used to start timer if needed.
#define DCC_ACK PC4        // DCC output for ACK

STservoS STservo;         // assign servo

// Constant CVs (read only)
#define CV7 10            // CV7 - version 1.0
#define CV8 13            // CV8 - manufactured (DIY)
#define CV29 128          // CV29 - accessory decoder

volatile uint32_t LastRising = 0;    // time stamp of last rising pulse
volatile uint8_t PreambleBits = 0;        // counter of preamble bits
volatile uint8_t DCCSTATE = 0;            // receiver state machine
volatile uint8_t SHIFT0;                  // receiver shift register
volatile uint8_t DCCBYTE;                 // received byte counter
volatile uint8_t DCCDATA[4];              // buffer for received bytes
volatile bool NEW_PACKET = false;         // new packet received
volatile bool DCC4BYTE;                   // received 3 or 4 byte packet?
bool PROG_2X = false;
bool RESET_FLG = false;
bool CLEAR_FLG = true;
uint8_t PageReg = 0;                      // page register for paged operations
uint16_t StartAddress;                    // first accepted accessory address
uint16_t EndAddress;                      // last accepted accessory address (and CCW address)
//uint16_t ManualCW;                        // address for manual CV rotation
uint8_t CV3 = 0;                          // virtual CV for actual position
uint8_t CV4 = 0;                          // virtual CV for actual position
bool NEW_ORDER = false;                   // new servo position is requested
uint16_t requested_pos = 0;               // requested servo position
uint16_t actual_pos = 0;                  // actual servo position
bool GO_OVER = false;                     // servo go over zero = need reset after
unsigned long LastResetTimestamp = 0;     // delay for servo reset
unsigned long StepDivider = 0;            // step packet divider - maximum 4 steps per sec


int ServoStat = 2;  // state machine for servo commands (start with ram init)
  // 0 - wait for order
  // 1 - wait until motor is running
  // 2 - check if reset is requested, if yes, perform reset, if no, go to 0
  // 3 - wait 2 ms after reset
  // 4 - init ram values -> go to 0


// hmm "volatile const" sounds chaotic, but that is because it can be changed by flash write.
const PROGMEM uint8_t CVlist[64] __attribute__((aligned(64))) = {5,0,10,3, // address 5 (lo 5, hi 0), speed 10, 3 output pairs
                                                                0x00,0x00,0x00,0x08,  // 1 position 0, 2048
                                                                0xAA,0x02,0xAA,0x0A,  // 2 position 682, 2730
                                                                0x54,0x05,0x54,0x0D,  // 3 position 1364, 3412
                                                                0x00,0x00,0x00,0x00,  // 4 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 5 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 6 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 7 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 8 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 9 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 10 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 11 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 12 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 13 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 14 position 0, 0 (empty)
                                                                0x00,0x00,0x00,0x00,  // 15 position 0, 0 (empty)
                                                                };
/*This is progmem array used as list of CVs. this MCU have limited EEPROM and progmem is feasible. CVs are mapped to save some space
Pos|CV|Note (Pos mean position in array)
---+--+-----
0  | 1|    Primary address lo
-- | 3| RO Actual possition lo -- after write CV8=0
-- | 4| RO Actual possition hi /
-- | 7| RO Version
-- | 8| RO Manufacturer ID
1  | 9|    Primary address hi
-- |29| RO Configuration 128 = accessory decoder
2  |33|    Rotation speed
3  |34|    Number of positions (each position = 4CV) 1..15 (+128 for restore position)
---+--+-----
4  |35|    Position 1 straight lo -- set by write CV8=1   \
5  |36|    Position 1 straight hi /                        -- one position
6  |37|    Position 1 turn lo -- set by write CV8=2       /
7  |38|    Position 1 turn hi /
---+--+-----
8  |39|    Position 2 straight lo -- set by write CV8=3   \
9  |40|    Position 2 straight hi /                        -- one position
10 |41|    Position 2 turn lo -- set by write CV8=4       /
11 |42|    Position 2 turn hi /
... repeat up to 15 positions
---+--+-----
60 |91|    Position 15 straight lo -- set by write CV8=3   \
61 |92|    Position 15 straight hi /                        -- one position
62 |93|    Position 15 turn lo -- set by write CV8=4       /
63 |94|    Position 15 turn hi /

*/
uint8_t *pCVlist = CVlist;    // pointer to CVlist - just to force compiler to execute

void DCC_Change(void) {
  if (digitalRead(DCC_IN) == HIGH) {
    // HIGH -> just start measuring
    //LastRising = micros(); // save actual timestamp
    LastRising = SysTick->CNT;
  } else {
    // LOW get bit and evaluate ...
    //uint32_t PulseLength = micros();
    uint32_t PulseLength = SysTick->CNT;
    //if (PulseLength < LastRising) {Serial.print(LastRising);Serial.print('/');Serial.println(PulseLength);}
    //if (PulseLength < LastRising) {PulseLength += 1000;}
    if (PulseLength < LastRising) {PulseLength += F_CPU / 1000;}
    PulseLength -= LastRising;   // measure bit time
    //Serial.println(micros()); 
    //if (PulseLength > 4200000000) {PulseLength = 30;} // not understand
    uint8_t NextBit = (PulseLength < (76 * (F_CPU / 1000000)))?1:0;           // time < 76 micros mean "1", longer mean "0"

    // Preamble checker -> 12 bits with value "1" means preamble at any time
    if (NextBit) {
      if (++PreambleBits == 12) {DCCSTATE = 1;}           // seen 12 bits with value 1 -> seen preamble
    } else {
      PreambleBits = 0;                                 // any bit with value 0 reset counter
    }

    // Receiver state machine
    switch (DCCSTATE) {
      case 0:   // Preamble
        // just wait, previous part of code done the rest
        break;
      case 1:   // WaitLow
        // wait for bit with value "0" - meaning start of effective byte
        if (!NextBit) {
          if (NEW_PACKET) {   // Error, previous packet is not decoded yet :(
            DCCSTATE = 0;     // reset receiver and skip this packet
          } else {
            DCCSTATE++;       // next state
            DCCBYTE = 0;      // clear byte count register
            DCCDATA[3] = 0;   // for sure for 3 byte packets
          }
        }
        break;
      case 2 ... 8: // ReadBit
        // read single bit and add it to working byte
        SHIFT0 = (SHIFT0 << 1) + NextBit;     // add next bit to right side of shift register
        DCCSTATE++;       // next state
        break;
      case 9:     // ReadLastBit
        // read last bit in byte and then evaluate all byte
        SHIFT0 = (SHIFT0 << 1) + NextBit;     // add last bit to right side of shift register
        DCCBYTE++;            // next byte is complete
        DCCSTATE += DCCBYTE;  // next state as per received byte
        break;
      case 10:    // EndByte1
        DCCSTATE = NextBit ? 0 : 2;   // we received 1st byte, we do not understand 1 byte packet, then next one must follow
        DCCDATA[0] = SHIFT0;          // store byte to queue
        break;
      case 11:    // EndByte2
        DCCSTATE = NextBit ? 0 : 2;   // we received 2nd byte, we do not understand 2 bytes packet, then next one must follow
        DCCDATA[1] = SHIFT0;          // store byte to queue
        break;
      case 12:    // EndByte3
        DCCDATA[2] = SHIFT0;          // store byte to queue
        if (NextBit) {                // is it last bit (1) or some next follow (0)
          DCCSTATE = 0;               // last bit, then reset state
          NEW_PACKET = true;          // set flag for main
          DCC4BYTE = false;           // this is 3 byte packet (not 4 byte)
        } else {
          DCCSTATE = 2;               // next one bit will follow
          DCC4BYTE = true;            // this is 4 byte packet (or more)
        }
        break;
      case 13:    // EndByte4
        DCCDATA[3] = SHIFT0;          // store byte to queue
        DCCSTATE = 0;                 // last bit, reset state
        if (NextBit) {                // is it last bit (1) or some next follow (0)
          NEW_PACKET = true;          // set flag for main
        }
        break;
    }

  }
}

void LoadCVs() {
  pCVlist = CVlist;    // pointer to CVlist - just to force compiler to execute
  if (pCVlist < FLASH_BASE) (pCVlist += FLASH_BASE);

  StartAddress = ((uint16_t)pCVlist[1] * 256) + pCVlist[0] -1;  // primary address hi (CV#9) and address lo (CV#1)
  EndAddress = StartAddress + pCVlist[3];                       // end address is start address + number of outputs + 1 for manual controll
  //ManualCW = StartAddress + pCVlist[3];

  /* Serial.print("CV1 ");  Serial.println(pCVlist[0]);
  Serial.print("CV9 ");  Serial.println(pCVlist[1]);
  Serial.print("CV33 ");  Serial.println(pCVlist[2]);
  Serial.print("CV34 ");  Serial.println(pCVlist[3]);
  Serial.print("CV35 ");  Serial.println(pCVlist[4]);
  Serial.print("CV36 ");  Serial.println(pCVlist[5]); */

}

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  // initialize digital pin for ACK as an output.
  pinMode(DCC_ACK, OUTPUT);
  digitalWrite(DCC_ACK, LOW);
  //Serial.begin(921600);
  attachInterrupt(DCC_IN, GPIO_Mode_IN_FLOATING, DCC_Change, EXTI_Mode_Interrupt, EXTI_Trigger_Rising_Falling);
  //Serial.println("OK");
  LoadCVs();
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  ServoInit();  

}

void ServoInit() {
  do {
    STservo.init();
    STservo.Find();
    if (!STservo.IsValid()) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(125);
      digitalWrite(LED_BUILTIN, LOW);
      delay(125);
      }
  } while (!STservo.IsValid());

  // Data in eeprom 
  if (STservo.ReadByte(33) != 0) {
    STservo.WriteByte(33,0);   // Using mode 0
  }
  if (STservo.ReadWord(9) != 0) {
    STservo.WriteWord(9,0);     // minimum angle = 0 for multi turn
  }
  if (STservo.ReadWord(11) != 0) {
    STservo.WriteWord(11,0);   // maximum angle = 0 for multi turn
  }
  actual_pos = STservo.ReadWord(56);
      //digitalWrite(LED_BUILTIN, HIGH);
}


void Decode() {
  //Serial.println(DCCDATA[0],HEX);
  uint8_t MyXOR =0;
  for (uint i = 0; i < 4; i++) {MyXOR ^= DCCDATA[i];}
  if (!MyXOR) {
    CLEAR_FLG = true;
    if (DCCDATA[0] == 0) {                      // address 0 = broadcast
      Broadcast();
    } else if ((DCCDATA[0] & 0xF0) == 0x70) {     // 0111xxxx may be service mode
      CheckSM();
    } else if (DCCDATA[0] == 0xFF) {            // idle packet
      CLEAR_FLG = false;                        // do nothing
    } else if ((DCCDATA[0] & 0xC0) == 0x80) {     // 10xxxxxx accessory operation
      Accessory();
      /*Serial.print(DCCDATA[0],HEX);
      Serial.print(' ');
      Serial.print(DCCDATA[1],HEX);
      Serial.print(' ');
      Serial.println(DCCDATA[2],HEX);*/
    }
    if (CLEAR_FLG) {
      RESET_FLG = false;
    }
  }
}

void Accessory() {
  uint16_t output = 0;
  // 10AA AAAA | 1AAA DAAR
  //   76 5432    A98  10   - address bits A=10; bits A98 are inverted
  output = (uint16_t)((DCCDATA[1] & 0x70) ^ 0x70 ) << 4;    // bits 10,9,8 isolated, inverted and shifted
  output |= (DCCDATA[0] & 0x3F) << 2;                       // bits 7,6,5,4,3,2 isolated, shifted
  output |= (DCCDATA[1] & 0x06) >> 1;                       // bits 1,0 isolated, shifted
  if ((output >= StartAddress) && (output <= EndAddress)) {
    // address for me
    if (output == EndAddress) {   // last address is for manual controll
      if (DCCDATA[1] & 0x01) {
        StartRotationCW();     // -- straight
      } else {
        StartRotationCCW();   // _/ turn
      }
    } else {
      uint outpos = (output - StartAddress) * 4;
      if (DCCDATA[1] & 0x01) {
        RotateTo((pCVlist[outpos + 5] * 256) + pCVlist[outpos + 4]);
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        RotateTo((pCVlist[outpos + 7] * 256) + pCVlist[outpos + 6]);
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
  }
  
  /*if (DCCDATA[1] & 0x01) {
    Serial.print('V');
    Serial.print(output+1);
    Serial.println('-');
  } else {
    Serial.print('V');
    Serial.print(output+1);
    Serial.println('/');
  }*/
}

void StartRotationCW() {
  //Serial.println("Rotate CW");
  if (millis() - StepDivider > 250) {
    requested_pos = (actual_pos + 8) & 4095;
    NEW_ORDER = true;
    StepDivider = millis();    // block for next 250 ms
  }
}

void StartRotationCCW() {
  //Serial.println("Rotate CCW");
  if (millis() - StepDivider > 250) {
    requested_pos = (actual_pos - 8) & 4095;
    NEW_ORDER = true;
    StepDivider = millis();    // block for next 250 ms
  }
}

void RotateTo(uint16_t TargetPosition) {
  //Serial.print("Rotate to:");
  //Serial.println(TargetPosition);
        //digitalWrite(LED_BUILTIN, LOW);
        requested_pos = TargetPosition;
        NEW_ORDER = true;
}

void CheckSM() {
  if (RESET_FLG) {
    if (PROG_2X) {
      if (DCC4BYTE) {
        ChkProg4();     // check for direct CV (4 byte) programming
      } else {
        // 0111CRRR DDDDDDDD EEEEEEEE
        PROG_2X = false;
        //uint8_t NewCVval = DCCDATA[1];    // save data
        switch (DCCDATA[0] & 7) {
          case 7:                         // Reg 8 (CV 8 read + reset)
            if (!(DCCDATA[0] & 8) && (CV8 == DCCDATA[1])) {SendACK();}    // verify CV 8
            if (DCCDATA[0] & 8) {SetPos(DCCDATA[1]);}
            break;
          case 6:                         // Reg 7 (CV 7 read only)
            if (!(DCCDATA[0] & 8) && (CV7 == DCCDATA[1])) {SendACK();}
            break;
          case 5:                         // Reg 6 (page register)
            if (DCCDATA[0] & 8) {
              // write page register
              PageReg = DCCDATA[1] << 2;  // loosing high bits, that is OK for accessory CV 1 = CV 513 (unfortunately also CV 257 and CV 769)
              // SendACK()
            } else {
              // verify page register
              if ((DCCDATA[1] << 2) == PageReg) {SendACK();}
            }
            break;
          case 4:                         // Reg 5 (CV 29 read only)
            if (!(DCCDATA[0] & 8) && (CV29 == DCCDATA[1])) {SendACK();}
            break;
          default:                        // Page register + last 2 bits = CV
            uint16_t CVaddr = FindCV(PageReg + (DCCDATA[0] & 3));
            if (CVaddr != 512) {
              // CV exist
              if (DCCDATA[0] & 8) {
                // write
                if (CVaddr < 64) {WriteCV(CVaddr,DCCDATA[1]); LoadCVs(); SendACK();}   // for real CVs only -> write it
              } else {
                // verify
                if (CVaddr & 256) {
                  // virtual / read only CV
                  if ((CVaddr & 0xFF) == DCCDATA[1]) {SendACK();}   // value is equal to requested
                } else {
                  // real CV
                  if (pCVlist[CVaddr] == DCCDATA[1]) {SendACK();}   // value is equal to requested
                }
              }
            }
            break;
        }
      }
    PROG_2X = false;
    } else {
      PROG_2X = true;
      //RESET_FLG = false;
      CLEAR_FLG = false;
    }
  } else {
    PROG_2X = false;
  }
}

void ChkProg4() {
  // 0111CCAA AAAAAAAA DDDDDDDD EEEEEEEE
  PROG_2X = false;
  if (!(DCCDATA[0] & 0x01)) {   // CVs 256 to 512 (768 to 1023) are not supported
    uint16_t CVaddr = FindCV(DCCDATA[1]);
    if ((CVaddr != 512) && (DCCDATA[0] & 0x0C)) {   // CV found and command is not 00 (undefined)
      // CC = 10 - bit manipulation, CC = 01 - verify, CC = 11 - write
      if (((DCCDATA[0] & 0x0C) == 0x0C) && (CVaddr < 64)) {WriteCV(CVaddr,DCCDATA[2]); LoadCVs(); SendACK();}    // for real CVs only -> write it
      if (((DCCDATA[0] & 0x0C) == 0x0C) && (DCCDATA[1] == 7)) {SetPos(DCCDATA[2]);}                   // write to CV8 -> set position
      if ((DCCDATA[0] & 0x0C) == 0x04) {  // verify
        if (CVaddr & 256) {
          // virtual / read only CV
          if ((CVaddr & 0xFF) == DCCDATA[2]) {SendACK();}   // value is equal to requested
        } else {
          // real CV
          if (pCVlist[CVaddr] == DCCDATA[2]) {SendACK();}   // value is equal to requested
        }
      }
      if ((DCCDATA[0] & 0x0C) == 0x08) {  // bit manipulation
        // 0111CCAA AAAAAAAA 111KDBBB EEEEEEEE
        uint8_t BitMask = 1 << (DCCDATA[2] & 7);  // bit mask for operation
        if (DCCDATA[2] & 0x10) {          // K=0 - verify bit; K=1 - set bit
          // Set bit
          if (CVaddr < 64) {
            if (DCCDATA[2] & 0x08) {      // bit value
              WriteCV(CVaddr,pCVlist[CVaddr] | BitMask);   // set bit
              LoadCVs();
              SendACK();
            } else {
              WriteCV(CVaddr,pCVlist[CVaddr] & ~BitMask);   // clear bit
              LoadCVs();
              SendACK();
            }
          }
        } else {
          // Verify bit
          uint8_t CVval = (CVaddr & 256) ? (CVaddr & 0xFF) : pCVlist[CVaddr];    // if virtual CV or real one
          if ((DCCDATA[2] & 0x08) && (CVval & BitMask)) {SendACK();}            // requested 1, received 1
          if (!(DCCDATA[2] & 0x08) && !(CVval & BitMask)) {SendACK();}          // requested 0, received 0
        }
      }
    }
  }
}



void WriteCV(uint8_t CVaddr, uint8_t CVdata) {
  /*Serial.print("WriteCV ");
  Serial.print(CVaddr);
  Serial.print(" = ");
  Serial.println(CVdata);*/

  uint8_t Tmplist[64];        // temporary storage
  for (int i = 0; i < 64; i++) {Tmplist[i] = pCVlist[i];}
  Tmplist[CVaddr] = CVdata;
  WriteData((uint32_t *)Tmplist);
  LoadCVs();
}

void Write2CV(uint8_t CVaddr, uint8_t CV1data, uint8_t CV2data) {
  /*Serial.print("WriteCV ");
  Serial.print(CVaddr);
  Serial.print(" = ");
  Serial.println(CVdata);*/

  uint8_t Tmplist[64];        // temporary storage
  for (int i = 0; i < 64; i++) {Tmplist[i] = pCVlist[i];}
  Tmplist[CVaddr] = CV1data;
  Tmplist[CVaddr+1] = CV2data;
  WriteData((uint32_t *)Tmplist);
  LoadCVs();
}

void WriteData(uint32_t pbuf[]) {

  uint32_t FAddress = CVlist;     // pointer to array in flash
  FAddress |= FLASH_BASE;         // real flash buegin at 0x0800 0000

  if (!(FAddress & 0x3F)) {   // array must be properly aligned

    // Erase block
    FLASH_Unlock_Fast();              // unlock first
    FLASH_ErasePage_Fast(FAddress);   // and then erase

    // Reset write buffer before write
    FLASH_BufReset();

    // upload data to buffer
    for (int i = 0; i<16; i++) {
      FLASH_BufLoad(FAddress,pbuf[i]);
      FAddress += 4;
    }

    FAddress = CVlist;   // restore pointer
    FAddress |= FLASH_BASE;

    // write data from buffer
    FLASH_ProgramPage_Fast(FAddress);

    // lock flash
    FLASH_Lock_Fast();

  }
}

uint16_t FindCV(uint8_t LookingCV) {
  // 0 to 255 = pointer to memory array
  // 256 to 511 = read only (low byte = value)
  // 512 = not found
  if (LookingCV < 29) {   // CVs 1 to 29 are complex, solved individually
    switch (LookingCV) {
      case 0:
        return 0;             // CV 1 in array position 0
        break;
      case 2:
        return 256 + CV3;     // CV 3 - read only
        break;
      case 3:
        return 256 + CV4;     // CV 4 - read only
        break;
      case 6:
        return 256 + CV7;     // CV 7 - read only
        break;
      case 7:
        return 256 + CV8;     // CV 8 - read only
        break;
      case 8:
        return 1;             // CV 9 in array position 1
        break;
      case 28:
        return 256 + CV29;    // CV 29 - read only
        break;
      default:                // rest are unknown / not supported
        return 512;
        break;
    }
  } else if ((LookingCV >= 32) && (LookingCV <= 93)) {   // CVs 33 to 94 are simple, just offset -31
    return LookingCV - 30;
  } else {
    return 512;               // others are unknown
  }
}

void SetPos(uint8_t PosToSet) {
  // TBD
  // 0 = actual position to CV 3,4
  // 1..30 = actual position to CV 35,36; 37,38; ...
  // 33 = reset CVs to default
  // 34 = reset position CVs as per CV34
  if (PosToSet == 0) {
    actual_pos = STservo.ReadWord(56);
    CV3 = (actual_pos & 0xFF);
    CV4 = ((actual_pos >> 8) & 0xFF);
  }
  if ((PosToSet >= 1) && (PosToSet < 30)) {
    actual_pos = STservo.ReadWord(56);
    Write2CV((PosToSet * 2) + 2,actual_pos & 0xFF,(actual_pos >> 8) & 0xFF);
  }
  if (PosToSet == 33) {
    // reset all CVs to default
    uint8_t Tmplist[64];        // temporary storage
    for (int i = 0; i < 64; i++) {Tmplist[i] = 0;}

    Tmplist[0] = 5;   // Address Lo
    Tmplist[1] = 0;   // Address Hi
    Tmplist[2] = 10;  // Speed
    Tmplist[3] = 3;   // 3 output pairs

    Tmplist[4] = 0x00;   // -- 1st position 0
    Tmplist[5] = 0x00;   // /
    Tmplist[6] = 0x00;   // -- 2nd position 2048
    Tmplist[7] = 0x08;   // /

    Tmplist[8] = 0xAA;   // -- 3rd position 682
    Tmplist[9] = 0x02;   // /
    Tmplist[10] = 0xAA;   // -- 4th position 2730
    Tmplist[11] = 0x0A;   // /

    Tmplist[12] = 0x54;   // -- 5th position 1364
    Tmplist[13] = 0x05;   // /
    Tmplist[14] = 0x54;   // -- 6th position 3412
    Tmplist[15] = 0x0D;   // /

    WriteData((uint32_t *)Tmplist);
    LoadCVs();
  }
  if (PosToSet == 34) {
    // lineary split as per CV34
    uint16_t Increment = 2048 / pCVlist[3];
    uint16_t Position = 0;
    uint8_t Tmplist[64];        // temporary storage
    for (int i = 0; i < 64; i++) {Tmplist[i] = 0;}            // erase all
    for (int i = 0; i < 4; i++) {Tmplist[i] = pCVlist[i];}    // copy basic one
    for (int i = 0; i < pCVlist[3]; i++) {
      Tmplist[(i*4)+4] = Position & 0xFF;
      Tmplist[(i*4)+5] = Position >> 8;
      Tmplist[(i*4)+6] = Position & 0xFF;
      Tmplist[(i*4)+7] = (Position >> 8) | 0x08;
      Position += Increment;
    }
    WriteData((uint32_t *)Tmplist);
    LoadCVs();
  }
}

void SendACK() {
  //Serial.println("SendACK");
  digitalWrite(DCC_ACK, HIGH);
  delay(6);
  digitalWrite(DCC_ACK, LOW);
}

void Broadcast() {
  if (DCCDATA[1] == 0) {      // data 0 = reset packet is only understand broadcast
    PROG_2X = false;
    RESET_FLG = true;
    CLEAR_FLG = false;
  }
}


// the loop function runs over and over again forever
void loop() {
  if (NEW_PACKET) {   // is new packet received?
    Decode();
    NEW_PACKET = false;
  }
  switch (ServoStat) {
    // 0 - wait for order
    case 0: 
      if (NEW_ORDER) {
        if (requested_pos != actual_pos) {
          if ((actual_pos - requested_pos) > 2048) {
            STservo.WriteWord(42,requested_pos + 4096);
            GO_OVER = true;
          } else if ((requested_pos - actual_pos) > 2048) {
            STservo.WriteWord(42,abs(requested_pos - 4096) | 0x8000);
            GO_OVER = true;
          } else {
            STservo.WriteWord(42,requested_pos);
            GO_OVER = false;
          }
          actual_pos = requested_pos;
        }
        NEW_ORDER = false;
        ServoStat++ ;       // wait until servo is moving
      }
      break;
    // 1 - wait until motor is running
    case 1: 
      {
        uint8_t b = STservo.ReadByte(66);
        if (b == 0) {
          digitalWrite(LED_BUILTIN, LOW);
        } else {
          digitalWrite(LED_BUILTIN, HIGH);
        }
        if ((GO_OVER) && (b==0)) {
          int MyPos = STservo.ReadWord(56);
          if ((abs(MyPos - actual_pos) > 20) && (abs(MyPos - actual_pos) < 4076)) {
            b=1;
          }
        }         
        if (b == 0) {   // register 66 "moving sign" = 1 when servo is moving; = 0 when motor is stopped
          if (GO_OVER) {
            ServoStat++ ;     // must reset
            GO_OVER = false;
          } else {
            ServoStat = 0 ;   // can leave
          }
          actual_pos = STservo.ReadWord(56);    // read actual servo position, not requested.
        }
      }
      break;
    // 2 - send reset packet
    case 2: 
      STservo.SendReset();
      LastResetTimestamp = millis();
      ServoStat++ ;         // wait for end
      break;
    // 3 - wait 2 ms after reset + init ram values -> go to 0
    case 3: 
      if (millis() - LastResetTimestamp > 2) {   // register 66 "moving sign" = 1 when servo is moving; = 0 when motor is stopped
        STservo.WriteWord(46,(uint16_t)pCVlist[2] * 10 );  // reset speed factor
        STservo.WriteByte(41,pCVlist[2]);  // reset acceleration factor
        ServoStat = 0 ;   // done
      }
  }

  //  Serial.println(TIM1->INTFR); // & TIM_IT_Update);
  //digitalWrite(LED_BUILTIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
  //delay(10);                      // wait for a second
}
