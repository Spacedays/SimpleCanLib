#include <Arduino.h>
#include "ThreadSafeQueue.h"
#include "SimpleCAN.h"

extern "C" {
#include <can2040.h>
}

// #define _PICO_	// FIXME - Recomment

#if defined(_PICO_)

#ifdef Error_Handler
#undef Error_Handler

void Error_Handler(int Code=-1)
{
	Serial.printf("\nError SimpleCan_Pico %d\n", Code);
}
#endif

// Temp message storage for rxhandler; needed since can2040 does not use hardware to store messages
//		LATER - Changes needed for two CAN instances?
static struct can2040_msg _msg;	// TESTME later - Does emptying this matter at all?

class RxHandlerPico : public RxHandlerBase
{
	public:
		// Note: The constructor here is a must on order to initialize the base class. 
		RxHandlerPico(uint16_t dataLength) : RxHandlerBase(dataLength) {};
		bool CANReadFrame(SimpleCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen);		
 		void ReleaseRcvBuffer();		

	private:
		// can2040 callback function, called whenever messages send, are received, or error
		// void can2040_cb(struct can2040 *cd, uint32_t notify_case, struct can2040_msg *msg);	// TODO: remove
};

// Copy max _rxDataLength bytes from received frame to _rxData. 
// ISR, absolutely no printing to serial!!
bool RxHandlerPico::CANReadFrame(SimpleCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen)
{
	// struct can2040msg _msg;	// Variable can2040 msg being processed is stored in

	 //check if we have a queue. If not, operation is aborted.
	 // TESTME - Does not check for empty message. Should it? ---> 
		// #define UINT32_MAX  (0xffffffff)
	 	// || (RxHandlerPico::_msg.id == UINT32_MAX )

	if (ProfileCallback == NULL)
	{
		return false;
	}

	SCHeader->RxTimestamp = -1;			// Unusupported...
	SCHeader->FilterIndex = -1;
	SCHeader->IsFilterMatchingFrame = -1;
	SCHeader->RxFrameType = (_msg.id & CAN2040_ID_RTR) ? SCFrameType::CAN_REMOTE_FRAME : SCFrameType::CAN_DATA_FRAME;	// TESTME RTR interpretation
	SCHeader->DataLength = _msg.dlc;
	SCHeader->Format = SCCanType::CAN_CLASSIC;
	SCHeader->Identifier = _msg.id & 0x3FFFFFFF;	// TESTME - presumably the 2 MSB are RTR & EFF bits. Is this necessary?

	 // Check if this is a standard or extended CAN frame
	if(_msg.id & CAN2040_ID_EFF)	// TESTME EFF Interpretation
		SCHeader->IdType = SCIdType::CAN_EXTID;	// Extended frame
	else
		SCHeader->IdType = SCIdType::CAN_STDID;	// Standard frame

	//Deep copy data bytes
	if (SCHeader->RxFrameType==SCFrameType::CAN_DATA_FRAME)
		for(int i=0; i<SCHeader->DataLength && i<MaxDataLen; i++)
			pData[i]=_msg.data[i];

	// if (SCHeader->RxFrameType!=SCFrameType::CAN_DATA_FRAME)
	//	Serial.print(" RTR ");	// DEBUG

	// Serial.println("CRF end"); delay(50);
	return true;
}

// Let the hardware know the frame has been read.
void RxHandlerPico::ReleaseRcvBuffer()
{
	// Nothing to do for Pico;	
}

static RxHandlerPico Can1RxHandler(8);			// Preferably this should be allocated by the HAL, just paramtereized here!

class SimpleCan_Pico : public SimpleCan
{
	public:
		SimpleCan_Pico();
	
		//*******************************************************
		//*** Implementation of pure virtual methods ************
		
		// Initialize the CAN controller
		SCCanStatus Init(SCCanSpeed speed, CanIDFilter IDFilterFunc=0);
		
		// Register/deregister a callback function for message received events.
		// The notification handler is platform specific, that is why it's needed here.
		// These functions may be overloaded if required.
		SCCanStatus ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData);
		SCCanStatus DeactivateNotification();

		// Set bus termination on/off (may not be available on all platforms).
		// Default is on.
		SCCanStatus SetBusTermination(bool On);

		// Start and stop all activities. Changing acceptance filters requires stop()
		// before doing so on some platforms.
		SCCanStatus Start();
		SCCanStatus Stop();

		// Modify the global filter to reject everything which is not matching the other filters and to accept all remote frames. 
		SCCanStatus ConfigGlobalFilter();

		// Modify the acceptance filter. This may be forbidden while the controller is active.
		SCCanStatus ConfigFilter(FilterDefinition *filterDef);

		// Start sending messages from the queue to the CAN bus, until the TX queue is emty.
		bool TriggerSending() { return SendNextMessageFromQueue();};

		//*******************************************************
		//*** Other methods ************
		
		
		static void PIOx_IRQHandler();

		static bool SendNextMessageFromQueue();

		// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
		// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
		// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
		// bool RequestMessage(int NumBytes, int CanID, bool UseEFF=false);
		// static bool SendRequestMessage(int NumBytes, int CanID, bool UseEFF);	//TODO: seperate RTR msg?

		// SCCanStatus ConfigGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt);
		// SCCanStatus AddMessageToTxFifoQ(FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData);
		bool Loop();

		uint8_t TxData[8];

		static RxHandlerPico *RxHandlerP;		

	private:
		static CanIDFilter SendIDFilterFunc;
};

static struct can2040 cbus;	// LATER - Changes needed for two CAN instances?
RxHandlerPico* SimpleCan_Pico::RxHandlerP=nullptr;	// Presumably this must be static because of IRQs????
CanIDFilter SimpleCan_Pico::SendIDFilterFunc;

SimpleCan_Pico::SimpleCan_Pico()
{
	SendIDFilterFunc = 0;
}

// RxHandlerPico:: // TESTME: moving can2040_cb to global scope?
static void can2040_cb(struct can2040 *cd, uint32_t notify_case, struct can2040_msg *msg)
{
	if (SimpleCan_Pico::RxHandlerP == NULL)
		{
			return;
		}
	
	_msg.id = msg->id;
	_msg.dlc = msg->dlc;
	for (int i=0; i<8; i++){
		_msg.data[i] = (msg->data)[i];
  }

	switch(notify_case){
		case CAN2040_NOTIFY_ERROR:
		// errbuff.push(_msg);	// TODO later: handle error (RX buffer overflowed)
		break;
		/*
		case CAN2040_NOTIFY_TX:
		// Do nothing; TX scheduling is handled by can2040
		// sendbuff.push(temp);
		// SimpleCan_Pico::SendNextMessageFromQueue();
		*/
		break;
		case CAN2040_NOTIFY_RX:
		
			SimpleCan_Pico::RxHandlerP->Notify();	// alert that data is in _msg for reading
		break;
  }
}

SCCanStatus SimpleCan_Pico::Init(SCCanSpeed speed, CanIDFilter IDFilterFunc /*=0*/)
{
	// set speed

	//TODO set filter..	

	// Setup canbus
	uint32_t pio_num = 0;
	can2040_setup(&cbus, pio_num);
	
	// can2040_callback_config(&cbus, SimpleCan_Pico::RxHandlerP->can2040_cb);	// TODO - remove
	// Connect can2040 callback - copies message -> handles errors, -> calls Notify()
	can2040_callback_config(&cbus, can2040_cb);

	// Enable irqs
	irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);	// TODO - irq wat do
	NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
	NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

	return CAN_OK;	// TODO: error handling on init
}

void SimpleCan_Pico::PIOx_IRQHandler(void)
{
  can2040_pio_irq_handler(&cbus);
}

SCCanStatus SimpleCan_Pico::ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData)
{
	if (RxHandlerP != NULL)
	{
		return CAN_ERROR;
	}

	RxHandlerP = &Can1RxHandler;		// This is the static object inside SimpleCan_Pico
	RxHandlerP->SetProfileCallback(dataLength, callback, userData);
	return CAN_OK;
}

SCCanStatus SimpleCan_Pico::DeactivateNotification()
{
	RxHandlerP = NULL;
	return CAN_OK;
}

SCCanStatus SimpleCan_Pico::SetBusTermination(bool On)
{
	return CAN_UNSUPPORTED;
}

SCCanStatus SimpleCan_Pico::Start(void)
{
	// Start canbus
	uint32_t sys_clock = SYS_CLK_KHZ*1000; //125000000;	// TODO later: get from PIO config/elsewhere?
	uint32_t bitrate = 500000;			//TODO: relate to SCCanSpeed
	uint32_t gpio_rx = 16, gpio_tx = 17;	//TODO later: pass in variables
	
	can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
	Serial.println("CAN (Pico): Start");
	return CAN_OK;
}

SCCanStatus SimpleCan_Pico::Stop(void)
{
	// Stop message processing; Does not clear transmit message queue
	can2040_stop(&cbus);
	return CAN_OK;
}

SCCanStatus SimpleCan_Pico::ConfigFilter(FilterDefinition *SCFilter)
{
	//TODO - Write Filter implementation? (or copy from esp/stm)

	// For ESP32: Mask bit==1 ignores the bit. Acceptance code spans ID,RTR and first two data bytes (std frame).
	// For SimpleCAN: Mask bit==1 requires the bit to match.  Acceptance code spans ID only.

	if (SCFilter->FilterIndex>0) return CAN_UNSUPPORTED;

	if (SCFilter->IdType==CAN_STDID && SCFilter->FilterType==CAN_FILTER_MASK)
	{
		// Serial.println("CAN: setting filter single ID & mask, Std Frame");
		Serial.println("CAN: ~ std id mask filter not yet implemented");
		// uint32_t Code = SCFilter->FilterID1; 
		// uint32_t Mask = ~SCFilter->FilterID2;		// Lowest 4 bits unused, 5th would be RTR 
		// MODULE_CAN->MBX_CTRL.ACC.CODE[0] = Code >> 3; 
		// MODULE_CAN->MBX_CTRL.ACC.CODE[1] = (Code<<5) & 0xff; 

		// MODULE_CAN->MBX_CTRL.ACC.MASK[0] = Mask >> 3; 
		// MODULE_CAN->MBX_CTRL.ACC.MASK[1] = Mask<<5 | 0x1f; 
		// MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0xff; 
		// MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0xff; 
		// MODULE_CAN->MOD.B.AFM = 1;
	}
	else if (SCFilter->IdType==CAN_STDID && SCFilter->FilterType==CAN_FILTER_DUAL)
	{
		// Serial.println("CAN: Unsupported filter mode");
		Serial.println("CAN: ~ Dual filter not yet implemented");
		// Same as single, but with CODE/MASK[2/3] in addition.
		// ACR3 / AMR3 is shared between the two filters.
		 return CAN_UNSUPPORTED;
	}
	else if (SCFilter->IdType==CAN_EXTID && SCFilter->FilterType==CAN_FILTER_MASK)
	{
		// Serial.println("CAN: setting filter single ID & mask, Ext Frame");
		Serial.println("CAN: ~ ext id mask filter not yet implemented");
		// uint32_t Code = SCFilter->FilterID1; 
		// uint32_t Mask = ~SCFilter->FilterID2;		
		// MODULE_CAN->MBX_CTRL.ACC.CODE[0] = Code >> 21; 
		// MODULE_CAN->MBX_CTRL.ACC.CODE[1] = (Code>>13) & 0xff; 
		// MODULE_CAN->MBX_CTRL.ACC.CODE[2] = (Code>>5)  & 0xff; 
		// MODULE_CAN->MBX_CTRL.ACC.CODE[3] = (Code<<3) & 0xff;

		// MODULE_CAN->MBX_CTRL.ACC.MASK[0] = Mask >> 21; 
		// MODULE_CAN->MBX_CTRL.ACC.MASK[1] = (Mask>>13) & 0xff; 
		// MODULE_CAN->MBX_CTRL.ACC.MASK[2] = (Mask>>5)  & 0xff; 
		// MODULE_CAN->MBX_CTRL.ACC.MASK[3] = (Mask<<3) | 0x07;
		// MODULE_CAN->MOD.B.AFM = 1;
	}
	else
	{
		Serial.println("CAN: Unsupported filter mode");
		return CAN_UNSUPPORTED;
	}

	return CAN_OK;	
}

SCCanStatus SimpleCan_Pico::ConfigGlobalFilter()
{
	//TODO later - Write Filter implementation? (or copy from esp/stm)
	// ---
	return CAN_OK;
}

bool SimpleCan_Pico::SendNextMessageFromQueue()
{
	// Serial.println("CAN: SendNextMessageFromQueue)");
	if(TxQueue.NumElements)
	{
		CANTxMessage Msg;
		if (!TxQueue.Dequeue(&Msg))
			// Nothing to send...
			return true;

		// Serial.printf("CAN (Pico): Sending %d bytes with ID 0x%x as %s frame\n", Msg.Size, Msg.CanID, Msg.EFF?"EFF":"std");

		// Skip command if sender ID is disabled.
		if ( SendIDFilterFunc && !SendIDFilterFunc(Msg.CanID) ) 
			return true; 

		if (Msg.Size > 8 || Msg.Size < 0)
		{
			Serial.printf("CAN: Invalid message length: %d\n", Msg.Size);
			Msg.Size = 0;
		}

		can2040_msg CMsg;
		can2040_msg *outgoing_ptr = &CMsg;	// TESTME - valid? Does it need to be reset?

		// TESTME: added back EFF bit to iD 
		CMsg.id = Msg.CanID;
		// This should not be shifted; 2 MSB unused for 29 bit// CMsg.id << 2;	// TESTME - should this be shifted? Is it actually 29 bit?
		if (Msg.RTR)
			CMsg.id |= CAN2040_ID_RTR;
		if (Msg.EFF)
			CMsg.id |= CAN2040_ID_EFF;

		// schedule for transmission & copy to internal storage; returns 0 if success, (-) number if queue full
		if (can2040_transmit(&cbus, outgoing_ptr) < 0)
		{
			Serial.print("CAN: Message Tx failed - queue full!\n");
		} 
		// Serial.printf("Tx %d", TxQueue.NumElements);
	}

	return true;
}

bool SimpleCan_Pico::Loop()
{
	return RxHandlerP->Loop();
}

SimpleCan* CreateCanLib()
{
	return (SimpleCan*) new SimpleCan_Pico;
}


#endif