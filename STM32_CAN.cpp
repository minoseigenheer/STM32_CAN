/*
 NMEA2000_STM32.cpp

 Use with STM32 HAL for your MCU!
 Inherited NMEA2000 object for the STM32F105 internal CAN
 See also NMEA2000 library.
 And can be used as universal CAN library.


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

 */

#include "STM32_CAN.hpp"


#ifdef DEBUG // STM32CubeIDE is automatically defining DEBUG for the debug build
//#define STM32_CAN_DEBUG
#define STM32_CAN_DEBUG_ERRORS
#endif

#if defined(STM32_CAN_DEBUG)
# define DbgPrintf(fmt, args...)     printf(fmt, ## args); printf("\n")
#else
# define DbgPrintf(fmt, args...)
#endif

#if defined(STM32_CAN_DEBUG_ERRORS)
# define ErrDbgPrintf(fmt, args...)     printf(fmt, ## args); printf("\n")
#else
# define ErrDbgPrintf(fmt, args...)
#endif


size_t nextInstanceID = 0;
tSTM32_CAN *STM32CANInstances[4];

//*****************************************************************************
tSTM32_CAN::tSTM32_CAN(CAN_HandleTypeDef *_canBus, CANbaudRatePrescaler _CANbaudRate) :
		canBus(_canBus),
		CANbaudRate(_CANbaudRate) {

	//NMEA2000_STM32_instance = this;
	STM32CANInstances[nextInstanceID] = this;
	nextInstanceID++;

	//STM32CANInstances.push_back(this); // add this instance to STM32CANInstances
	DbgPrintf("created tSTM32_CAN instance %p", this);

	// TODO for now we only use RX fifo 0
	CANRxFIFO = CAN_RX_FIFO0;

	rxRing = 0;
	txRing = 0;

	MaxCANReceiveFrames = 32;
	MaxCANSendFrames = 16;

	// TODO make prioBits configurable for each instance
	prioBits = 3; // For NMEA2000 or SAE J1939 we use only the 3 highest bits of the ID for the priority
	maxPrio = pow(2, prioBits) - 1; // max unsigned value is 2^prioBits -1   (for 3 bits priority 0...7)

	bufferFull = false;

}

//*****************************************************************************
bool tSTM32_CAN::CANOpen() {

	bool ret = true;

	// CAN initialisation instead of using the by the STM32cubeIDE configuration tool generated init function
	if (CANInit() != HAL_OK) {
		ret = false;
	}

	// activate CAN callback interrupts
	if (HAL_CAN_ActivateNotification(canBus,
			CAN_IT_RX_FIFO0_MSG_PENDING
			| CAN_IT_TX_MAILBOX_EMPTY
			| CAN_IT_ERROR_WARNING
			| CAN_IT_ERROR_PASSIVE
			| CAN_IT_BUSOFF
			| CAN_IT_LAST_ERROR_CODE
			| CAN_IT_ERROR
		) != HAL_OK) {
		ret = false;
	}

	// Enable CAN
	if (HAL_CAN_Start(canBus) == HAL_OK) {
		DbgPrintf("%s started", CANname.c_str());
	}
	else {
		ret = false;
	}

	return ret;
}

//*****************************************************************************
bool tSTM32_CAN::CANSendFrame(unsigned long id, unsigned char len, const unsigned char* buf, bool wait_sent, bool extended_id) {
	//TODO wait_sent

	bool ret = false;

	uint8_t prio;

	if (extended_id) {
		prio = (uint8_t)(id >> (29-prioBits) & maxPrio);
	}
	else {
		prio = (uint8_t)(id >> (11-prioBits) & maxPrio);
	}

	HAL_CAN_DeactivateNotification(canBus, CAN_IT_TX_MAILBOX_EMPTY);

	bool TxMailboxesFull = HAL_CAN_GetTxMailboxesFreeLevel(canBus) == 0;
	bool SendFromBuffer = false;

	// If TX buffer has already some frames waiting with higher prio or mailbox is full, buffer frame
	if ( !txRing->isEmpty(prio) || TxMailboxesFull ) {
		CAN_message_t *msg = txRing->getAddRef(prio);
		if ( msg!=0 ) {
			msg->id = id;
			msg->flags.extended = extended_id;
			if ( len > 8 ) len = 8;
			msg->flags.remote = 0;
			msg->len = len;
			memcpy(msg->buf, buf, len);
			ret = true;
			//frame buffered
			DbgPrintf("%s frame 0x%lx buffered", CANname.c_str(), id);
		}
		SendFromBuffer = true;
	}

	if ( !TxMailboxesFull ) {
		if ( SendFromBuffer ) {
			ret = SendFromTxRing();
		} else {
			ret = CANWriteTxMailbox(id, len, buf, extended_id);
		}
		/* transmit entry accepted */
	}
	else {
		DbgPrintf("%s All TX mailboxes full", CANname.c_str());
	}

	HAL_CAN_ActivateNotification(canBus, CAN_IT_TX_MAILBOX_EMPTY);

	return ret;
}

//*****************************************************************************
bool tSTM32_CAN::CANSendFrame(tSTM32_CAN::CAN_message_t* message) {

	bool ret = false;

		uint8_t prio;

		if (message->flags.extended) {
			prio = (uint8_t)(message->id >> (29-prioBits) & maxPrio);
		}
		else {
			prio = (uint8_t)(message->id >> (11-prioBits) & maxPrio);
		}

		HAL_CAN_DeactivateNotification(canBus, CAN_IT_TX_MAILBOX_EMPTY);

		bool TxMailboxesFull = HAL_CAN_GetTxMailboxesFreeLevel(canBus) == 0;
		if ( TxMailboxesFull ) {
			DbgPrintf("%s all TX mailboxes are full", CANname.c_str());
		}
		bool SendFromBuffer = false;

		// If TX buffer has already some frames waiting with higher prio or mailbox is full, buffer frame
		if ( !txRing->isEmpty(prio) || TxMailboxesFull ) {
			CAN_message_t *msg = txRing->getAddRef(prio);
			if ( msg!=0 ) {
				memcpy(msg, message, sizeof(*msg));
				ret = true;
				//frame buffered
				bufferFull = false;
				DbgPrintf("%s frame 0x%lx buffered", CANname.c_str(), message->id);
			}
			else if (!bufferFull){
				bufferFull = true;
				ErrDbgPrintf("%s TX ringbuffer is full", CANname.c_str());
			}
			SendFromBuffer = true;
		}

		if ( !TxMailboxesFull ) {
			if ( SendFromBuffer ) {
				ret = SendFromTxRing();
			} else {
				ret = CANWriteTxMailbox(message->id, message->len, message->buf, message->flags.extended);
			}
			/* transmit entry accepted */
		}

		HAL_CAN_ActivateNotification(canBus, CAN_IT_TX_MAILBOX_EMPTY);

		return ret;
}
//*****************************************************************************
bool tSTM32_CAN::CANGetFrame(unsigned long& id, unsigned char& len, unsigned char* buf) {

	bool ret = false;

	HAL_CAN_DeactivateNotification(canBus, CAN_IT_RX_FIFO0_MSG_PENDING);

	const CAN_message_t *msg = rxRing->getReadRef();
	if ( msg!=0 ) {
	    id = msg->id;
	    len = msg->len;
		if ( len > 8 ) len = 8;
	    memcpy(buf, msg->buf, len);
	    ret = true;
	}

	HAL_CAN_ActivateNotification(canBus, CAN_IT_RX_FIFO0_MSG_PENDING);

	return ret;

}
//*****************************************************************************
bool tSTM32_CAN::CANGetFrameStruct(tSTM32_CAN::CAN_message_t* message) {

	bool ret = false;

	HAL_CAN_DeactivateNotification(canBus, CAN_IT_RX_FIFO0_MSG_PENDING);

	const CAN_message_t *msg = rxRing->getReadRef();
	if ( msg!=0 ) {
	    //message = msg;
	    memcpy(message, msg, sizeof(*msg));
	    ret = true;
	}

	HAL_CAN_ActivateNotification(canBus, CAN_IT_RX_FIFO0_MSG_PENDING);

	return ret;

}

// *****************************************************************************
void tSTM32_CAN::InitCANFrameBuffers() {

  if ( MaxCANReceiveFrames == 0 ) MaxCANReceiveFrames = 32; // Use default, if not set
  if ( MaxCANReceiveFrames < 10 ) MaxCANReceiveFrames = 10; // Do not allow less than 10 - should have enough memory.
  if ( MaxCANSendFrames == 0 ) MaxCANSendFrames = 50;  // Use big enough default buffer
  if ( MaxCANSendFrames < 30 ) MaxCANSendFrames = 30; // Do not allow less than 30 - should have enough memory.

  //TODO deleting ring buffer results in hard fault!
  // if buffer is initialized with a different size delete it
  if ( rxRing != 0 && rxRing->getSize() != MaxCANReceiveFrames ) {
    delete rxRing;
    rxRing = 0;
  }
  if ( txRing != 0 && txRing->getSize() != MaxCANSendFrames ) {
    delete txRing;
    txRing = 0;
  }

  if ( rxRing == 0 ) {
	  rxRing=new tPriorityRingBuffer<CAN_message_t>(MaxCANReceiveFrames, maxPrio);
	  DbgPrintf("%s RX ring buffer initialized: MaxCANReceiveFrames: %u | maxPrio: %lu", CANname.c_str(), MaxCANReceiveFrames, maxPrio);
  }
  if ( txRing == 0 ) {
	  txRing=new tPriorityRingBuffer<CAN_message_t>(MaxCANSendFrames, maxPrio);
	  DbgPrintf("%s TX ring buffer initialized: MaxCANSendFrames: %u | maxPrio: %lu", CANname.c_str(), MaxCANReceiveFrames, maxPrio);
  }


}

// *****************************************************************************
bool tSTM32_CAN::CANWriteTxMailbox(unsigned long id, unsigned char len, const unsigned char *buf, bool extended) {

	if (extended) {
		CANTxHeader.IDE = CAN_ID_EXT;
		CANTxHeader.ExtId = id & 0x1FFFFFFF;
	} else {
		CANTxHeader.IDE = CAN_ID_STD;
		CANTxHeader.StdId = id & 0x7FF;
	}
	CANTxHeader.RTR = CAN_RTR_DATA;
	CANTxHeader.DLC = len;
	CANTxHeader.TransmitGlobalTime = DISABLE;

    if ( len > 8 ) len = 8;
    memcpy(CANTxdata, buf, len);
	//for (int i = 0; i < len; i++) {
	//	CANTxdata[i] = buf[i];
	//}

	// send message
	if (HAL_CAN_AddTxMessage(canBus, &CANTxHeader, CANTxdata, &CANTxMailbox) == HAL_OK) {
		DbgPrintf("%s Added frame 0x%lx to TX mailbox %lu", CANname.c_str(), id, CANTxMailbox);
		return true;
	} else {
		ErrDbgPrintf("%s Failed to write TX mailbox %lu (0x%lx)", CANname.c_str(), CANTxMailbox, id);
		return false;
	}

}

// *****************************************************************************
bool tSTM32_CAN::SendFromTxRing() {
	const CAN_message_t *txMsg;
	txMsg = txRing->getReadRef(); // always get highest prio message from the buffer
	if ( txMsg != 0 ) {
		return CANWriteTxMailbox(txMsg->id, txMsg->len, txMsg->buf, txMsg->flags.extended);
	} else {
		return false;
	}

}

// *****************************************************************************
void tSTM32_CAN::CANReadRxMailbox(CAN_HandleTypeDef *hcan, uint32_t CANRxFIFOn) {
	CAN_message_t *rxMsg;
	uint32_t id;
	uint8_t prio;
	if (hcan == canBus) {
		if (HAL_CAN_GetRxMessage(hcan, CANRxFIFOn, &CANRxHeader, CANRxdata) == HAL_OK) {
			if (CANRxHeader.IDE == CAN_ID_EXT) {
				id = CANRxHeader.ExtId;
				prio = (uint8_t)(id >> (29-prioBits) & maxPrio);
			}
			else {
				id = CANRxHeader.StdId;
				prio = (uint8_t)(id >> (11-prioBits) & maxPrio);
			}
			rxMsg = rxRing->getAddRef(prio);
			if ( rxMsg!=0 ) {
				rxMsg->len = CANRxHeader.DLC;
				if ( rxMsg->len > 8 ) rxMsg->len = 8;
				rxMsg->flags.remote = CANRxHeader.RTR == CAN_RTR_REMOTE;
				rxMsg->flags.extended = CANRxHeader.IDE == CAN_ID_EXT;
				rxMsg->id = id;
				memcpy(rxMsg->buf, CANRxdata, rxMsg->len);
			}
			DbgPrintf("%s Received CAN message 0x%lx", CANname.c_str(), id);
		}
	}

	// I think we don't have to check the fifo fill level if we use interrupts?
	// HAL_CAN_GetRxFifoFillLevel(*canBus, CAN_RX_FIFO1);

}


/**
  * @brief CAN Initialization Function
  * @retval bool success or not
  */
HAL_StatusTypeDef tSTM32_CAN::CANInit()
{
	// CAN1000kbitPrescaler, TimeSeg1 and TimeSeg2 are configured for 1000 kbit/s @ defined clock speed
	// Baud rate has to be dividable by 1000 (500, 250, 200, 125, 100...)

	CAN_TypeDef *CANinstance;
#ifdef CAN1
	if (canBus == &hcan1) {
		CANinstance = CAN1;
		CANname = "CAN1";
	}
#ifdef CAN2
	else if (canBus == &hcan2) {
		CANinstance = CAN2;
		CANname = "CAN2";
	}
#endif
#ifdef CAN3
	else if (canBus == &hcan3) {
		CANinstance = CAN3;
		CANname = "CAN3";
	}
#endif
	else {
		// CAN_HandleTypeDef *hcan is unknown
		return HAL_ERROR;
	}
#endif

	DbgPrintf("%s initialization", CANname.c_str());

	uint32_t CAN1000kbitPrescaler;
	uint32_t CANtimeSeg1;
	uint32_t CANtimeSeg2;


	// usually the APB1 clock is running at the following speed if max clock frequencies are used:
	// STM32F103/105/107   36'000'000 Hz
	// STM32F405/407       42 000'000 Hz
	uint32_t APB1clockSpeed = HAL_RCC_GetPCLK1Freq();

	if (APB1clockSpeed == 24000000) {
		CAN1000kbitPrescaler = 2;
		CANtimeSeg1 = CAN_BS1_10TQ;
		CANtimeSeg2 = CAN_BS2_1TQ;
	}
	else if (APB1clockSpeed == 36000000) {
		CAN1000kbitPrescaler = 2;
		CANtimeSeg1 = CAN_BS1_15TQ;
		CANtimeSeg2 = CAN_BS2_2TQ;
	}
	else if (APB1clockSpeed == 42000000) {
		CAN1000kbitPrescaler = 3;
		CANtimeSeg1 = CAN_BS1_11TQ;
		CANtimeSeg2 = CAN_BS2_2TQ;
	}
	else if (APB1clockSpeed == 48000000) {
		CAN1000kbitPrescaler = 3;
		CANtimeSeg1 = CAN_BS1_13TQ;
		CANtimeSeg2 = CAN_BS2_2TQ;
	}
	else {
		// There are no settings four your ABT1 clock speed yet!
		// On the following website you can find a matching prescaler, TS1 and TS2
		// Add the values for your clock speed and 1000kbit/s
		// http://www.bittiming.can-wiki.info/?CLK=36&ctype=bxCAN&SamplePoint=87.5
		ErrDbgPrintf("%s initialization error (No CAN settings for this clock speed.)", CANname.c_str());
		return HAL_ERROR;
	}

	canBus->Instance = CANinstance;
	canBus->Init.Prescaler = (CAN1000kbitPrescaler * CANbaudRate);
	canBus->Init.Mode = CAN_MODE_NORMAL;
	canBus->Init.SyncJumpWidth = CAN_SJW_1TQ;
	canBus->Init.TimeSeg1 = CANtimeSeg1;
	canBus->Init.TimeSeg2 = CANtimeSeg2;
	canBus->Init.TimeTriggeredMode = DISABLE;
	canBus->Init.AutoBusOff = DISABLE;
	canBus->Init.AutoWakeUp = DISABLE;
	canBus->Init.AutoRetransmission = ENABLE;
	canBus->Init.ReceiveFifoLocked = DISABLE;
	canBus->Init.TransmitFifoPriority = ENABLE;

	return HAL_CAN_Init(canBus);
}

/*******************************************************************************
  * @brief  Set STM32 HAL CAN filter
  * @param  ExtendedIdentifier: 0 = normal CAN identifier; 1 = extended CAN identifier
  * @param  FilterNum CAN bus filter number for this CAN bus 0...13 / 0...27
  * @param  Mask uint32_t bit mask
  * @param  Filter uint32_t CAN identifier
  * @retval success or not
  */
HAL_StatusTypeDef tSTM32_CAN::SetCANFilter(bool ExtendedId, uint32_t FilterNum, uint32_t Mask, uint32_t Id)
{
	HAL_StatusTypeDef ret = HAL_ERROR;

	// For dual CAN MCU's we have 28 filter banks to share between the CAN busses
	// For single CAN SlaveStartFilterBank does nothing and we have 14 filter banks.
	// If not defined different we use filter 0 .. 13 for primary CAN bus and 14 ... 27 for secondary CAN bus
	#if !defined(SlaveStartFilterBank)
	const uint32_t SlaveStartFilterBank = 14;
	#endif

	#if defined(CAN2) // we have two CAN busses
		const int32_t TotalFilterBanks = 27;
    #elif defined(CAN1) // we have only one CAN bus
		const int32_t TotalFilterBanks = 13;
	#else // we have no CAN defined
		const int32_t TotalFilterBanks = -1;
	#endif

	int32_t FilterBank = -1;
	if (canBus->Instance == CAN1
			&& FilterNum <= TotalFilterBanks
			&& FilterNum < SlaveStartFilterBank ) {
		FilterBank = FilterNum;
	}
	else if (canBus->Instance == CAN2
			&& FilterNum <= TotalFilterBanks - SlaveStartFilterBank) {
		FilterBank = FilterNum + SlaveStartFilterBank;
	}

	if ( FilterBank >= 0
			&& IS_CAN_ALL_INSTANCE(canBus->Instance) )
	{
		CAN_FilterTypeDef sFilterConfig;

		sFilterConfig.FilterBank = FilterBank;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

		if (ExtendedId == false)
		{
			sFilterConfig.FilterMaskIdHigh = Mask << 5 & 0xFFFF;
			sFilterConfig.FilterMaskIdLow = 1 << 2; // Mask the IDE bit 2 (standard ID) // allows both remote request and data frames
			sFilterConfig.FilterIdHigh = Id << 5 & 0xFFFF; // STDID[10:0]
			sFilterConfig.FilterIdLow =  0x0000; // IDE bit 2 needs to be 0 for standard ID
		}
		else
		{ // ExtendedIdentifier == true
			sFilterConfig.FilterMaskIdHigh = Mask >> 13 & 0xFFFF;
			sFilterConfig.FilterMaskIdLow = (Mask << 3 & 0xFFF8) | (1 << 2);
			sFilterConfig.FilterIdHigh = Id >> 13 & 0xFFFF; // EXTID[28:13]
			sFilterConfig.FilterIdLow = (Id << 3 & 0xFFF8) | (1 << 2); // EXTID[12:0] + IDE
		}

		sFilterConfig.FilterFIFOAssignment = CANRxFIFO;
		sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
		sFilterConfig.SlaveStartFilterBank = SlaveStartFilterBank; // CAN 0: 0...13 // CAN 1: 14...27 (28 filter banks in total)

		ret = HAL_CAN_ConfigFilter(canBus, &sFilterConfig);
		DbgPrintf("%s filter bank %li mask: %08lx, filter: %08lx", CANname.c_str(), FilterBank, Mask, Filter);
	}
	return ret;
}



/*******************************************************************************
  * @brief  returns pointer to tSTM32_CAN instance from certain CAN_HandleTypeDef struct
  * @param  hcan CAN_HandleTypeDef pointer
  * @retval tSTM32_CAN  pointer
  */
tSTM32_CAN* getInstance(CAN_HandleTypeDef *hcan)
{
	tSTM32_CAN* ret = 0;
	//DbgPrintf("searching for instance with CAN_HandleTypeDef = %p", hcan);
	for(size_t i = 0; i < nextInstanceID; ++i) {

		if (STM32CANInstances[i]->canBus == hcan) {
			ret = STM32CANInstances[i];
			//DbgPrintf("found matching instance %p", ret->canBus);
		}
	}
	return ret;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	getInstance(hcan)->CANReadRxMailbox(hcan, CAN_RX_FIFO0);
	//DbgPrintf("%s HAL_CAN_RxFifo0MsgPendingCallback", getInstance(hcan)->CANname.c_str());
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	getInstance(hcan)->SendFromTxRing(); // send message with highest priority on ring buffer
	//DbgPrintf("%s HAL_CAN_TxMailbox0CompleteCallback", getInstance(hcan)->CANname.c_str());
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	getInstance(hcan)->SendFromTxRing(); // send message with highest priority on ring buffer
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	getInstance(hcan)->SendFromTxRing(); // send message with highest priority on ring buffer
}


void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
   uint32_t errorCode = hcan->ErrorCode;

#if defined(STM32_CAN_DEBUG_ERRORS)
   if(errorCode & HAL_CAN_ERROR_NONE)            { ErrDbgPrintf("%s: No error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_EWG)             { ErrDbgPrintf("%s: Protocol Error Warning", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_EPV)             { ErrDbgPrintf("%s: Error Passive", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_BOF)             { ErrDbgPrintf("%s: Bus-off error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_STF)             { ErrDbgPrintf("%s: Stuff error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_FOR)             { ErrDbgPrintf("%s: Form error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_ACK)             { ErrDbgPrintf("%s: Acknowledgment error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_BR)              { ErrDbgPrintf("%s: Bit recessive error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_BD)              { ErrDbgPrintf("%s: Bit dominant error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_CRC)             { ErrDbgPrintf("%s: CRC error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_RX_FOV0)         { ErrDbgPrintf("%s: Rx FIFO 0 overrun error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_RX_FOV1)         { ErrDbgPrintf("%s: Rx FIFO 1 overrun error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_TX_ALST0)        { ErrDbgPrintf("%s: TxMailbox 0 transmit failure due to arbitration lost", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_TX_TERR0)        { ErrDbgPrintf("%s: TxMailbox 0 transmit failure due to transmit error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_TX_ALST1)        { ErrDbgPrintf("%s: TxMailbox 1 transmit failure due to arbitration lost", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_TX_TERR1)        { ErrDbgPrintf("%s: TxMailbox 1 transmit failure due to transmit error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_TX_ALST2)        { ErrDbgPrintf("%s: TxMailbox 2 transmit failure due to arbitration lost", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_TX_TERR2)        { ErrDbgPrintf("%s: TxMailbox 2 transmit failure due to transmit error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_TIMEOUT)         { ErrDbgPrintf("%s: Timeout error", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_NOT_INITIALIZED) { ErrDbgPrintf("%s: Peripheral not initialized", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_NOT_READY)       { ErrDbgPrintf("%s: Peripheral not ready", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_NOT_STARTED)     { ErrDbgPrintf("%s: Peripheral not started", getInstance(hcan)->CANname.c_str()); }
   if(errorCode & HAL_CAN_ERROR_PARAM)           { ErrDbgPrintf("%s: Parameter error", getInstance(hcan)->CANname.c_str()); }
#endif
}


// *****************************************************************************
//	Other 'Bridge' functions

//*****************************************************************************
// switch printf() to the debug interface SWO (STM32 ARM debug)
extern "C" int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

uint32_t pow(uint32_t base, uint32_t exp) {
	uint32_t result = 1;
	while (exp != 0)
	{
		result *= base;
		-- exp;
	}
	return result;
}
