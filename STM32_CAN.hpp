/*
 * STM32_CAN.hpp

 use with STM32 HAL for your MCU!


 Copyright (c) 2022 Minos Eigenheer


Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inherited NMEA2000 object for STM32 MCU's with internal CAN
based setup. See also NMEA2000 library.
*/


#ifndef STM32_CAN_H_
#define STM32_CAN_H_

#include "stm32f1xx_hal.h"
#include "main.h"

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string>

#include "RingBuffer.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;


class tSTM32_CAN
{

  public:
    enum CANbaudRatePrescaler {
    	CAN1000kbit = 1,
    	CAN500kbit = 2,
		CAN250kbit = 4,
		CAN200kbit = 5,
		CAN125kbit = 8,
		CAN100kbit = 10,
		CAN50kbit = 20
    };

  	tSTM32_CAN(CAN_HandleTypeDef *_canBus, CANbaudRatePrescaler _CANbaudRate);

	CAN_HandleTypeDef *canBus;

    struct CAN_message_t {
      uint32_t id = 0;          // can identifier
  //    uint16_t timestamp = 0;   // time when message arrived
  //    uint8_t idhit = 0; // filter that id came from
      struct {
        bool extended = 0; // identifier is extended (29-bit)
        bool remote = 0;  // remote transmission request packet type
        bool overrun = 0; // message overrun
        bool reserved = 0;
      } flags;
      uint8_t len = 8;      // length of data
      uint8_t buf[8] = { 0 };       // data
  //    uint8_t mb = 0;       // used to identify mailbox reception
  //    uint8_t bus = 0;      // used to identify where the message came from when events() is used.
  //    bool seq = 0;         // sequential frames
    };

	std::string CANname;

  protected:
    uint16_t MaxCANReceiveFrames;
    uint16_t MaxCANSendFrames;

	uint8_t prioBits;
	uint32_t maxPrio;

	CAN_TxHeaderTypeDef CANTxHeader;
	CAN_RxHeaderTypeDef CANRxHeader;

	uint8_t CANTxdata[8];
	uint8_t CANRxdata[8];

	uint32_t CANTxMailbox;

	const CANbaudRatePrescaler CANbaudRate;
	uint32_t CANRxFIFO;

	bool bufferFull;


  protected:
    tPriorityRingBuffer<CAN_message_t>  *rxRing;
    tPriorityRingBuffer<CAN_message_t>  *txRing;

    bool CANWriteTxMailbox(unsigned long id, unsigned char len, const unsigned char *buf, bool extended);


  public:
    void InitCANFrameBuffers();
    HAL_StatusTypeDef CANInit();
    HAL_StatusTypeDef SetCANFilter(bool ExtendedId, uint32_t FilterNum, uint32_t Mask, uint32_t Id);
    bool CANOpen();

    bool CANSendFrame(unsigned long id, unsigned char len, const unsigned char* buf, bool wait_sent = false, bool extended_id = true);
    bool CANSendFrame(CAN_message_t* message);
    bool CANGetFrame(unsigned long& id, unsigned char& len, unsigned char* buf);
    bool CANGetFrameStruct(CAN_message_t* message);

    // triggered by interrupt
    void CANReadRxMailbox(CAN_HandleTypeDef *hcan, uint32_t CANRxFIFO);
    bool SendFromTxRing();


};


tSTM32_CAN* getInstance(CAN_HandleTypeDef *hcan); // returns instance with certain CAN_HandleTypeDef struct




//-----------------------------------------------------------------------------

extern "C" int _write(int file, char *ptr, int len);
uint32_t pow(uint32_t base, uint32_t exp);


#endif /* NMEA2000_STM32_H_ */

