#ifdef SENSORLAB
/**
\brief 		Serial library on top of the UART interface
\author		Quentin Lampin <quentin.lampin@orange.com>
\date		Jan. 7th, 2015
**/

#include "openserial.h"
#include "opendefs.h"
#include "openhdlc.h"
#include "uart.h"
#include "leds.h"


#define INCREMENT(x,inc,mod) { x = (x+inc)&(mod-1);}
#define IN_BUFFER_COUNT(w_i,r_i,size) ((w_i - r_i + size) % size)

static openserial_vars_t openserial_vars;

void openserial_init(void)
{
	// initialize output
	memset(openserial_vars.buffer_output, 0, SERIAL_OUTPUT_BUFFER_LENGTH * sizeof(uint8_t));
	openserial_vars.buffer_output_write_index = 0;
	openserial_vars.buffer_output_read_index = 0;
	openserial_vars.buffer_output_overflow_errors = 0;
	openserial_vars.sending = FALSE;
	// initialize input
	memset(openserial_vars.buffer_input, 0, SERIAL_INPUT_BUFFER_LENGTH * sizeof(uint8_t));
	openserial_vars.buffer_input_write_index = 0;
	openserial_vars.buffer_input_read_index = 0;
	openserial_vars.buffer_input_overflow_errors = 0;
	openserial_vars.frame_received = FALSE;

	// discover if a DMA controller is available
	openserial_vars.uart_using_DMA = uart_DMA_available();

	// define UART interruption TX and RX callbacks
	uart_setCallbacks(on_uart_ready_to_send_interrupt, on_uart_rx_interrupt);
}


void openserial_write(uint8_t* bytes, uint16_t size)
{
	uint16_t crc; 						// HDLC frame crc
	uint8_t crc_byte1;					// CRC byte 1
	uint8_t crc_byte2;					// CRC byte 2
	bool is_escaped; 					// byte is escaped
	uint16_t pessimistic_frame_size;	// real size of the bytes (including escaped bytes)
	uint16_t in_buffer_count;			//	actual number of bytes in buffer
	uint16_t dma_chunk_size;			// size of the chunk of buffer passed to the DMA
	uint16_t i;							// iterator over the bytes array


	// initialize local variables
	crc = HDLC_CRCINIT;
	is_escaped = FALSE;
	i = 0;

	// check if there is enough space in the buffer
	// HDLC frame: 4 to 6bytes, let's take the worse case scenario (crc being escaped)
	pessimistic_frame_size = 6;
	// if byte[i] is escaped, count 2 bytes
	// else count 1 byte
	for (i = 0; i < size; i++){
		if (bytes[i] == HDLC_FLAG || bytes[i] == HDLC_ESCAPE){
			pessimistic_frame_size += 2;
		} else {
			pessimistic_frame_size += 1;
		}
	}
	// protect this section against interruptions
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();
	// if asked for more bytes than available:
	//	- increment the error counter
	//	- enable interruptions
	in_buffer_count = IN_BUFFER_COUNT(	openserial_vars.buffer_output_write_index,
										openserial_vars.buffer_output_read_index,
										SERIAL_OUTPUT_BUFFER_LENGTH);
	if ( pessimistic_frame_size > SERIAL_OUTPUT_BUFFER_LENGTH - in_buffer_count ) {
		openserial_vars.buffer_output_overflow_errors ++;
		ENABLE_INTERRUPTS();
	} else {
		// if there is enough room in the buffer
		// write in the buffer
		// 	- HDLC frame opening flag
		openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = HDLC_FLAG;
		INCREMENT(openserial_vars.buffer_output_write_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);
		//	- HDLC frame data
		i = 0;
		while (i < size) {
			// if byte is escaped
			//	- mask the byte
			//	- put escaped byte to buffer
			if (is_escaped == TRUE){
				openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = bytes[i] ^ HDLC_ESCAPE_MASK;
				is_escaped = FALSE;
				i += 1;
			} else {
				// update frame CRC
				crc = crcIteration(crc,bytes[i]);
				// if current byte value is of reserved value
				//	- put the escape flag in the buffer
				//	- set is_escaped flag to TRUE
				if (bytes[i] == HDLC_FLAG || bytes[i] == HDLC_ESCAPE){
					openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = HDLC_ESCAPE;
					is_escaped = TRUE;
				} else {
					openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = bytes[i];
					i += 1;
				}
			}
			INCREMENT(openserial_vars.buffer_output_write_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);
		}
		//	- HDLC frame CRC
		crc = ~crc;
		crc_byte1 = crc >> 0 & 0xff;
		crc_byte2 = crc >> 8 & 0xff;

		if (crc_byte1 == HDLC_FLAG || crc_byte1 == HDLC_ESCAPE){
			openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = HDLC_ESCAPE;
			INCREMENT(openserial_vars.buffer_output_write_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);
			openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = crc_byte1 ^ HDLC_ESCAPE_MASK;
			INCREMENT(openserial_vars.buffer_output_write_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);
		} else {
			openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = crc_byte1;
			INCREMENT(openserial_vars.buffer_output_write_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);
		}
		if (crc_byte2 == HDLC_FLAG || crc_byte2 == HDLC_ESCAPE){
			openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = HDLC_ESCAPE;
			INCREMENT(openserial_vars.buffer_output_write_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);
			openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = crc_byte2 ^ HDLC_ESCAPE_MASK;
			INCREMENT(openserial_vars.buffer_output_write_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);
		} else {
			openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = crc_byte2;
			INCREMENT(openserial_vars.buffer_output_write_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);
		}

		//	- HDLC frame closing flag
		openserial_vars.buffer_output[openserial_vars.buffer_output_write_index] = HDLC_FLAG;
		INCREMENT(openserial_vars.buffer_output_write_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);

		// open a new serial write session when the last write routine completes
		if (openserial_vars.sending == FALSE){
			openserial_vars.sending = TRUE;
			if(openserial_vars.uart_using_DMA){
				dma_chunk_size = (openserial_vars.buffer_output_write_index > openserial_vars.buffer_output_read_index) ?
									(openserial_vars.buffer_output_write_index - openserial_vars.buffer_output_read_index)
								: 	(SERIAL_OUTPUT_BUFFER_LENGTH - openserial_vars.buffer_output_read_index);
				uart_DMA_write(&openserial_vars.buffer_output[openserial_vars.buffer_output_read_index], dma_chunk_size, on_uart_dma_write_complete);
			}else{
				uart_writeByte(openserial_vars.buffer_output[openserial_vars.buffer_output_read_index]);
				INCREMENT(openserial_vars.buffer_output_read_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);
			}
		}
		ENABLE_INTERRUPTS();
	}
}

void on_uart_ready_to_send_interrupt(void){
	if(openserial_vars.buffer_output_read_index != openserial_vars.buffer_output_write_index){
		uart_writeByte(openserial_vars.buffer_output[openserial_vars.buffer_output_read_index]);
		INCREMENT(openserial_vars.buffer_output_read_index, 1, SERIAL_OUTPUT_BUFFER_LENGTH);
	} else {
		openserial_vars.sending = FALSE;
	}
}

uint16_t openserial_read(uint8_t *buffer_output, uint16_t buffer_output_length)
{
	uint16_t cursor;			// output buffer cursor
	uint16_t frame_end; 		// position of the closing HDLC frame flag
	uint16_t frame_length; 		// frame length (excluding escaped bytes)
	uint8_t byte; 				// current byte value
	bool byte_is_escaped;		// escape flag
	uint16_t crc;				// frame CRC
	uint16_t bytes_read;		// number of bytes in the frame (excluding flags, escapes and CRC)

	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	// discard bytes until a HDLC frame is found
	while(openserial_vars.buffer_input_read_index != openserial_vars.buffer_input_write_index){
		byte = openserial_vars.buffer_input[openserial_vars.buffer_input_read_index];
		INCREMENT(openserial_vars.buffer_input_read_index, 1, SERIAL_INPUT_BUFFER_LENGTH);
		if (byte == HDLC_FLAG) {
			break;
		}
	}
	// if we reach the write index then something is very (very) wrong
	if (openserial_vars.buffer_input_read_index == openserial_vars.buffer_input_write_index){
		leds_error_blink();
		return 0;
	}
	// locate the frame end and compute the frame length
	frame_length = 0;
	frame_end = openserial_vars.buffer_input_read_index;
	while (frame_end != openserial_vars.buffer_input_write_index){
		byte = openserial_vars.buffer_input[frame_end];
		if (byte == HDLC_FLAG){
			break;
		}
		if (byte != HDLC_ESCAPE){
			frame_length += 1;
		}
		INCREMENT(frame_end, 1, SERIAL_INPUT_BUFFER_LENGTH);
	}
	// we've counted the 2 CRC bytes, let's subtract these
	frame_length -= 2;
	bytes_read = frame_length;
	// if the frame length exceeds the output buffer, return nothing
	if (frame_length > buffer_output_length){
		return 0;
	}

	byte_is_escaped = FALSE;
	crc = HDLC_CRCINIT;
	cursor = 0;

	while (openserial_vars.buffer_input_read_index != frame_end){
		byte = openserial_vars.buffer_input[openserial_vars.buffer_input_read_index];
		if (byte_is_escaped){
			if (byte == HDLC_FLAG_ESCAPED){
				if (frame_length){
					buffer_output[cursor++] = HDLC_FLAG;
					frame_length -= 1;
				}
				crcIteration(crc, HDLC_FLAG);
			} else if (byte == HDLC_ESCAPE_ESCAPED){
				if (frame_length){
					buffer_output[cursor++] = HDLC_ESCAPE;
					frame_length -= 1;
				}
				crcIteration(crc, HDLC_ESCAPE);
			} else {
				// should never happen, unless 2 == 1
				leds_error_blink();
				return 0;
			}
			byte_is_escaped = FALSE;
		} else { // byte is not escaped
			if (byte == HDLC_ESCAPE){
				byte_is_escaped = TRUE;
			} else {
				 if (frame_length){
				 	buffer_output[cursor++] = byte;
				 	frame_length -= 1;
				 }
				 crcIteration(crc, byte);
			}
		}
		INCREMENT(openserial_vars.buffer_input_read_index, 1, SERIAL_INPUT_BUFFER_LENGTH);
	}

	// discard the HDLC closing FLAG
	INCREMENT(openserial_vars.buffer_input_read_index, 1, SERIAL_INPUT_BUFFER_LENGTH);
	if (openserial_vars.buffer_input_read_index == openserial_vars.buffer_input_write_index){
		openserial_vars.frame_received = FALSE;
	}
	ENABLE_INTERRUPTS();
	// check CRC and return accordingly
	if (crc == HDLC_CRCGOOD) {
		return bytes_read;
	} else {
		return 0;
	}
}

void on_uart_rx_interrupt(void)
{
	uint8_t byte;

	// protect this section against interruptions
	INTERRUPT_DECLARATION();
	DISABLE_INTERRUPTS();

	// read byte
	byte = uart_readByte();
	// if this is the first unread byte
	if(openserial_vars.buffer_input_write_index == openserial_vars.buffer_input_read_index && byte == HDLC_FLAG){
		openserial_vars.buffer_input[openserial_vars.buffer_input_write_index] = byte;
		INCREMENT(openserial_vars.buffer_input_write_index, 1, SERIAL_INPUT_BUFFER_LENGTH);
	}else{
		openserial_vars.buffer_input[openserial_vars.buffer_input_write_index] = byte;
		INCREMENT(openserial_vars.buffer_input_write_index, 1, SERIAL_INPUT_BUFFER_LENGTH);
	}
	// if last byte is HDLC_FLAG and there is at least 4 bytes to be read, set the frame received_flag
	if(openserial_vars.buffer_input_write_index - openserial_vars.buffer_input_read_index > 3 && byte == HDLC_FLAG){
		openserial_vars.frame_received = TRUE;
	}
	ENABLE_INTERRUPTS();
}

void on_uart_dma_write_complete(uint16_t bytes_written)
{
	uint16_t dma_chunk_size;

	INTERRUPT_DECLARATION();
	// protect this section against interruptions
	DISABLE_INTERRUPTS();
	// increment the read index, update the available space
	INCREMENT(openserial_vars.buffer_output_read_index, bytes_written, SERIAL_OUTPUT_BUFFER_LENGTH);
	// schedule another DMA write if need be
	if(openserial_vars.buffer_output_writers == 0 && openserial_vars.buffer_output_read_index != openserial_vars.buffer_output_write_index){
		dma_chunk_size = (openserial_vars.buffer_output_write_index > openserial_vars.buffer_output_read_index) ?
									(openserial_vars.buffer_output_write_index - openserial_vars.buffer_output_read_index)
								: 	(SERIAL_OUTPUT_BUFFER_LENGTH - openserial_vars.buffer_output_read_index);
		uart_DMA_write(&openserial_vars.buffer_output[openserial_vars.buffer_output_read_index], dma_chunk_size, on_uart_dma_write_complete);
	}else{
		openserial_vars.sending = FALSE;
	}
	ENABLE_INTERRUPTS();
}

/* legacy declarations */
void    openopenserial_init(void){}
owerror_t openserial_printStatus(uint8_t statusElement, uint8_t* buffer, uint8_t length){return E_SUCCESS;}
owerror_t openserial_printInfo(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2){return E_SUCCESS;}
owerror_t openserial_printError(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2){return E_SUCCESS;}
owerror_t openserial_printCritical(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2){return E_SUCCESS;}
owerror_t openserial_printData(uint8_t* buffer, uint8_t length){return E_SUCCESS;}
owerror_t openserial_printObservability(uint8_t* buffer, uint8_t length){return E_SUCCESS;}
uint8_t openserial_getNumDataBytes(void){return 0;}
uint8_t openserial_getInputBuffer(uint8_t* bufferToWrite, uint8_t maxNumBytes){return 0;}
void    openserial_startInput(void){}
void    openserial_startOutput(void){}
void    openserial_stop(void){}
bool    debugPrint_outBufferIndexes(void){return FALSE;}
void    openserial_echo(uint8_t* but, uint8_t bufLen){}

// interrupt handlers
void    isr_openserial_rx(void){}
void    isr_openserial_tx(void){}

#else
/**
\brief Definition of the "openserial" driver.

\author Fabien Chraim <chraim@eecs.berkeley.edu>, March 2012.
*/

#include "opendefs.h"
#include "openserial.h"
#include "IEEE802154E.h"
#include "neighbors.h"
#include "sixtop.h"
#include "icmpv6echo.h"
#include "idmanager.h"
#include "openqueue.h"
#include "openbridge.h"
#include "leds.h"
#include "schedule.h"
#include "uart.h"
#include "opentimers.h"
#include "openhdlc.h"
#include "schedule.h"
#include "icmpv6rpl.h"

//=========================== variables =======================================

openserial_vars_t openserial_vars;

//=========================== prototypes ======================================

#ifdef GOLDEN_IMAGE_SNIFFER
extern void sniffer_setListeningChannel(uint8_t channel);
#endif

owerror_t openserial_printInfoErrorCritical(
   char             severity,
   uint8_t          calling_component,
   uint8_t          error_code,
   errorparameter_t arg1,
   errorparameter_t arg2
);

void openserial_board_reset_cb(
   opentimer_id_t id
);

void openserial_goldenImageCommands(void);

// HDLC output
void outputHdlcOpen(void);
void outputHdlcWrite(uint8_t b);
void outputHdlcClose(void);
// HDLC input
void inputHdlcOpen(void);
void inputHdlcWrite(uint8_t b);
void inputHdlcClose(void);

//=========================== public ==========================================

void openserial_init() {
   uint16_t crc;
   
   // reset variable
   memset(&openserial_vars,0,sizeof(openserial_vars_t));
   
   // admin
   openserial_vars.mode                = MODE_OFF;
   openserial_vars.debugPrintCounter   = 0;
   
   // input
   openserial_vars.reqFrame[0]         = HDLC_FLAG;
   openserial_vars.reqFrame[1]         = SERFRAME_MOTE2PC_REQUEST;
   crc = HDLC_CRCINIT;
   crc = crcIteration(crc,openserial_vars.reqFrame[1]);
   crc = ~crc;
   openserial_vars.reqFrame[2]         = (crc>>0)&0xff;
   openserial_vars.reqFrame[3]         = (crc>>8)&0xff;
   openserial_vars.reqFrame[4]         = HDLC_FLAG;
   openserial_vars.reqFrameIdx         = 0;
   openserial_vars.lastRxByte          = HDLC_FLAG;
   openserial_vars.busyReceiving       = FALSE;
   openserial_vars.inputEscaping       = FALSE;
   openserial_vars.inputBufFill        = 0;
   
   // ouput
   openserial_vars.outputBufFilled     = FALSE;
   openserial_vars.outputBufIdxR       = 0;
   openserial_vars.outputBufIdxW       = 0;
   
   // set callbacks
   uart_setCallbacks(isr_openserial_tx,
                     isr_openserial_rx);
}

owerror_t openserial_printStatus(uint8_t statusElement,uint8_t* buffer, uint8_t length) {
   uint8_t i;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   openserial_vars.outputBufFilled  = TRUE;
   outputHdlcOpen();
   outputHdlcWrite(SERFRAME_MOTE2PC_STATUS);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   outputHdlcWrite(statusElement);
   for (i=0;i<length;i++){
      outputHdlcWrite(buffer[i]);
   }
   outputHdlcClose();
   ENABLE_INTERRUPTS();
   
   return E_SUCCESS;
}

owerror_t openserial_printInfoErrorCritical(
      char             severity,
      uint8_t          calling_component,
      uint8_t          error_code,
      errorparameter_t arg1,
      errorparameter_t arg2
   ) {
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   openserial_vars.outputBufFilled  = TRUE;
   outputHdlcOpen();
   outputHdlcWrite(severity);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   outputHdlcWrite(calling_component);
   outputHdlcWrite(error_code);
   outputHdlcWrite((uint8_t)((arg1 & 0xff00)>>8));
   outputHdlcWrite((uint8_t) (arg1 & 0x00ff));
   outputHdlcWrite((uint8_t)((arg2 & 0xff00)>>8));
   outputHdlcWrite((uint8_t) (arg2 & 0x00ff));
   outputHdlcClose();
   ENABLE_INTERRUPTS();
   
   return E_SUCCESS;
}

owerror_t openserial_printData(uint8_t* buffer, uint8_t length) {
   uint8_t  i;
   uint8_t  asn[5];
   INTERRUPT_DECLARATION();
   
   // retrieve ASN
   ieee154e_getAsn(asn);// byte01,byte23,byte4
   
   DISABLE_INTERRUPTS();
   openserial_vars.outputBufFilled  = TRUE;
   outputHdlcOpen();
   outputHdlcWrite(SERFRAME_MOTE2PC_DATA);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   outputHdlcWrite(asn[0]);
   outputHdlcWrite(asn[1]);
   outputHdlcWrite(asn[2]);
   outputHdlcWrite(asn[3]);
   outputHdlcWrite(asn[4]);
   for (i=0;i<length;i++){
      outputHdlcWrite(buffer[i]);
   }
   outputHdlcClose();
   ENABLE_INTERRUPTS();
   
   return E_SUCCESS;
}

owerror_t openserial_printPacket(uint8_t* buffer, uint8_t length, uint8_t channel) {
   uint8_t  i;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   openserial_vars.outputBufFilled  = TRUE;
   outputHdlcOpen();
   outputHdlcWrite(SERFRAME_MOTE2PC_SNIFFED_PACKET);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[1]);
   outputHdlcWrite(idmanager_getMyID(ADDR_16B)->addr_16b[0]);
   for (i=0;i<length;i++){
      outputHdlcWrite(buffer[i]);
   }
   outputHdlcWrite(channel);
   outputHdlcClose();
   
   ENABLE_INTERRUPTS();
   
   return E_SUCCESS;
}

owerror_t openserial_printInfo(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2) {
   return openserial_printInfoErrorCritical(
      SERFRAME_MOTE2PC_INFO,
      calling_component,
      error_code,
      arg1,
      arg2
   );
}

owerror_t openserial_printError(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2) {
   // blink error LED, this is serious
   leds_error_toggle();
   
   return openserial_printInfoErrorCritical(
      SERFRAME_MOTE2PC_ERROR,
      calling_component,
      error_code,
      arg1,
      arg2
   );
}

owerror_t openserial_printCritical(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2) {
   // blink error LED, this is serious
   leds_error_blink();
   
   // schedule for the mote to reboot in 10s
   opentimers_start(10000,
                    TIMER_ONESHOT,TIME_MS,
                    openserial_board_reset_cb);
   
   return openserial_printInfoErrorCritical(
      SERFRAME_MOTE2PC_CRITICAL,
      calling_component,
      error_code,
      arg1,
      arg2
   );
}

void openserial_board_reset_cb(opentimer_id_t id) {
   board_reset();
}

uint8_t openserial_getNumDataBytes() {
   uint8_t inputBufFill;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   inputBufFill = openserial_vars.inputBufFill;
   ENABLE_INTERRUPTS();

   return inputBufFill-1; // removing the command byte
}

uint8_t openserial_getInputBuffer(uint8_t* bufferToWrite, uint8_t maxNumBytes) {
   uint8_t numBytesWritten;
   uint8_t inputBufFill;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   inputBufFill = openserial_vars.inputBufFill;
   ENABLE_INTERRUPTS();
   
   if (maxNumBytes<inputBufFill-1) {
      openserial_printError(COMPONENT_OPENSERIAL,ERR_GETDATA_ASKS_TOO_FEW_BYTES,
                            (errorparameter_t)maxNumBytes,
                            (errorparameter_t)inputBufFill-1);
      numBytesWritten = 0;
   } else {
      numBytesWritten = inputBufFill-1;
      memcpy(bufferToWrite,&(openserial_vars.inputBuf[1]),numBytesWritten);
   }
   
   return numBytesWritten;
}

void openserial_startInput() {
   INTERRUPT_DECLARATION();
   
   if (openserial_vars.inputBufFill>0) {
      openserial_printError(COMPONENT_OPENSERIAL,ERR_INPUTBUFFER_LENGTH,
                            (errorparameter_t)openserial_vars.inputBufFill,
                            (errorparameter_t)0);
      DISABLE_INTERRUPTS();
      openserial_vars.inputBufFill=0;
      ENABLE_INTERRUPTS();
   }
   
   uart_clearTxInterrupts();
   uart_clearRxInterrupts();      // clear possible pending interrupts
   uart_enableInterrupts();       // Enable USCI_A1 TX & RX interrupt
   
   DISABLE_INTERRUPTS();
   openserial_vars.busyReceiving  = FALSE;
   openserial_vars.mode           = MODE_INPUT;
   openserial_vars.reqFrameIdx    = 0;
#ifdef FASTSIM
   uart_writeBufferByLen_FASTSIM(
      openserial_vars.reqFrame,
      sizeof(openserial_vars.reqFrame)
   );
   openserial_vars.reqFrameIdx = sizeof(openserial_vars.reqFrame);
#else
   uart_writeByte(openserial_vars.reqFrame[openserial_vars.reqFrameIdx]);
#endif
   ENABLE_INTERRUPTS();
}

void openserial_startOutput() {
   //schedule a task to get new status in the output buffer
   uint8_t debugPrintCounter;
   
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   openserial_vars.debugPrintCounter = (openserial_vars.debugPrintCounter+1)%STATUS_MAX;
   debugPrintCounter = openserial_vars.debugPrintCounter;
   ENABLE_INTERRUPTS();
   
   // print debug information
   switch (debugPrintCounter) {
      case STATUS_ISSYNC:
         if (debugPrint_isSync()==TRUE) {
            break;
         }
      case STATUS_ID:
         if (debugPrint_id()==TRUE) {
            break;
         }
      case STATUS_DAGRANK:
         if (debugPrint_myDAGrank()==TRUE) {
            break;
         }
      case STATUS_OUTBUFFERINDEXES:
         if (debugPrint_outBufferIndexes()==TRUE) {
            break;
         }
      case STATUS_ASN:
         if (debugPrint_asn()==TRUE) {
            break;
         }
      case STATUS_MACSTATS:
         if (debugPrint_macStats()==TRUE) {
            break;
         }
      case STATUS_SCHEDULE:
         if(debugPrint_schedule()==TRUE) {
            break;
         }
      case STATUS_BACKOFF:
         if(debugPrint_backoff()==TRUE) {
            break;
         }
      case STATUS_QUEUE:
         if(debugPrint_queue()==TRUE) {
            break;
         }
      case STATUS_NEIGHBORS:
         if (debugPrint_neighbors()==TRUE) {
            break;
         }
      case STATUS_KAPERIOD:
         if (debugPrint_kaPeriod()==TRUE) {
            break;
         }
      default:
         DISABLE_INTERRUPTS();
         openserial_vars.debugPrintCounter=0;
         ENABLE_INTERRUPTS();
   }
   
   // flush buffer
   uart_clearTxInterrupts();
   uart_clearRxInterrupts();          // clear possible pending interrupts
   uart_enableInterrupts();           // Enable USCI_A1 TX & RX interrupt
   DISABLE_INTERRUPTS();
   openserial_vars.mode=MODE_OUTPUT;
   if (openserial_vars.outputBufFilled) {
#ifdef FASTSIM
      uart_writeCircularBuffer_FASTSIM(
         openserial_vars.outputBuf,
         &openserial_vars.outputBufIdxR,
         &openserial_vars.outputBufIdxW
      );
#else
      uart_writeByte(openserial_vars.outputBuf[openserial_vars.outputBufIdxR++]);
#endif
   } else {
      openserial_stop();
   }
   ENABLE_INTERRUPTS();
}

void openserial_stop() {
   uint8_t inputBufFill;
   uint8_t cmdByte;
   bool busyReceiving;
   INTERRUPT_DECLARATION();
   
   DISABLE_INTERRUPTS();
   busyReceiving = openserial_vars.busyReceiving;
   inputBufFill = openserial_vars.inputBufFill;
   ENABLE_INTERRUPTS();
   
   // disable USCI_A1 TX & RX interrupt
   uart_disableInterrupts();
   
   DISABLE_INTERRUPTS();
   openserial_vars.mode=MODE_OFF;
   ENABLE_INTERRUPTS();
   //the inputBuffer has to be reset if it is not reset where the data is read.
   //or the function openserial_getInputBuffer is called (which resets the buffer)
   if (busyReceiving==TRUE){
      openserial_printError(COMPONENT_OPENSERIAL,ERR_BUSY_RECEIVING,
                                  (errorparameter_t)0,
                                  (errorparameter_t)inputBufFill);
   }
   
   if (busyReceiving == FALSE && inputBufFill>0) {
      DISABLE_INTERRUPTS();
      cmdByte = openserial_vars.inputBuf[0];
      ENABLE_INTERRUPTS();
      switch (cmdByte) {
         case SERFRAME_PC2MOTE_SETROOT:
            idmanager_triggerAboutRoot();
            break;
         case SERFRAME_PC2MOTE_DATA:
            openbridge_triggerData();
            break;
         case SERFRAME_PC2MOTE_TRIGGERSERIALECHO:
            //echo function must reset input buffer after reading the data.
            openserial_echo(&openserial_vars.inputBuf[1],inputBufFill-1);
            break;   
         case SERFRAME_PC2MOTE_COMMAND_GD: 
             // golden image command
            openserial_goldenImageCommands();
            break;
         default:
            openserial_printError(COMPONENT_OPENSERIAL,ERR_UNSUPPORTED_COMMAND,
                                  (errorparameter_t)cmdByte,
                                  (errorparameter_t)0);
            //reset here as it is not being reset in any other callback
            DISABLE_INTERRUPTS();
            openserial_vars.inputBufFill = 0;
            ENABLE_INTERRUPTS();
            break;
      }
   }
   
   DISABLE_INTERRUPTS();
   openserial_vars.inputBufFill  = 0;
   openserial_vars.busyReceiving = FALSE;
   ENABLE_INTERRUPTS();
}

void openserial_goldenImageCommands(void){
   uint8_t  input_buffer[7];
   uint8_t  numDataBytes;
   uint8_t  version;
#ifndef GOLDEN_IMAGE_NONE
   uint8_t  type;
#endif
   uint8_t  commandId;
   uint8_t  commandLen;
   uint8_t  comandParam_8;
   uint16_t comandParam_16;
   
   numDataBytes = openserial_getNumDataBytes();
   //copying the buffer
   openserial_getInputBuffer(&(input_buffer[0]),numDataBytes);
   version = openserial_vars.inputBuf[1];
#ifndef GOLDEN_IMAGE_NONE
   type    = openserial_vars.inputBuf[2];
#endif
   if (version != GOLDEN_IMAGE_VERSION) {
      // the version of command is wrong
      // log this info and return
      return;
   }
   
#ifdef GOLDEN_IMAGE_ROOT 
   if ( type != GD_TYPE_ROOT ){
       // image type is wrong
       return;
   }
#endif
#ifdef GOLDEN_IMAGE_SNIFFER
   if (type != GD_TYPE_SNIFFER) {
       // image type is wrong
       return;
   }
#endif
   commandId  = openserial_vars.inputBuf[3];
   commandLen = openserial_vars.inputBuf[4];
   
   if (commandLen>2 || commandLen == 0) {
       // the max command Len is 2, except ping commands
       return;
   } else {
       if (commandLen == 1) {
           comandParam_8 = openserial_vars.inputBuf[5];
       } else {
           // commandLen == 2
           comandParam_16 = (openserial_vars.inputBuf[5]      & 0x00ff) | \
                            ((openserial_vars.inputBuf[6]<<8) & 0xff00); 
       }
   }
   
   switch(commandId) {
       case COMMAND_SET_EBPERIOD:
           sixtop_setEBPeriod(comandParam_8); // one byte, in seconds
           break;
       case COMMAND_SET_CHANNEL:
#ifdef GOLDEN_IMAGE_ROOT
               //  this is dagroot image
               ieee154e_setSingleChannel(comandParam_8); // one byte
#endif
#ifdef GOLDEN_IMAGE_SNIFFER
               // this is sniffer image
               sniffer_setListeningChannel(comandParam_8); // one byte
#endif
           break;
       case COMMAND_SET_KAPERIOD: // two bytes, in slots
           sixtop_setKaPeriod(comandParam_16);
           break;
       case COMMAND_SET_DIOPERIOD: // two bytes, in mili-seconds
           icmpv6rpl_setDIOPeriod(comandParam_16);
           break;
       case COMMAND_SET_DAOPERIOD: // two bytes, in mili-seconds
           icmpv6rpl_setDAOPeriod(comandParam_16);
           break;
       case COMMAND_PING_MOTE:
           // this should not happen
           break;
       case COMMAND_SET_DAGRANK: // two bytes
           neighbors_setMyDAGrank(comandParam_16);
           break;
       case COMMAND_SET_SECURITY_STATUS: // one byte
           if (comandParam_8 ==1) {
               ieee154e_setIsSecurityEnabled(TRUE);
           } else {
               if (comandParam_8 == 0) {
                  ieee154e_setIsSecurityEnabled(FALSE);
               } else {
                   // security only can be 1 or 0 
                   break;
               }
           }
           break;
       case COMMAND_SET_FRAMELENGTH: // two bytes
           schedule_setFrameLength(comandParam_16);
           break;
       case COMMAND_SET_ACK_STATUS:
           if (comandParam_8 == 1) {
               ieee154e_setIsAckEnabled(TRUE);
           } else {
               if (comandParam_8 == 0) {
                   ieee154e_setIsAckEnabled(FALSE);
               } else {
                   // ack reply
                   break;
               }
           }
           break;
       default:
           // wrong command ID
           break;
   }
}

/**
\brief Trigger this module to print status information, over serial.

debugPrint_* functions are used by the openserial module to continuously print
status information about several modules in the OpenWSN stack.

\returns TRUE if this function printed something, FALSE otherwise.
*/
bool debugPrint_outBufferIndexes() {
   uint16_t temp_buffer[2];
   INTERRUPT_DECLARATION();
   DISABLE_INTERRUPTS();
   temp_buffer[0] = openserial_vars.outputBufIdxW;
   temp_buffer[1] = openserial_vars.outputBufIdxR;
   ENABLE_INTERRUPTS();
   openserial_printStatus(STATUS_OUTBUFFERINDEXES,(uint8_t*)temp_buffer,sizeof(temp_buffer));
   return TRUE;
}

//=========================== private =========================================

//===== hdlc (output)

/**
\brief Start an HDLC frame in the output buffer.
*/
port_INLINE void outputHdlcOpen() {
   // initialize the value of the CRC
   openserial_vars.outputCrc                          = HDLC_CRCINIT;
   
   // write the opening HDLC flag
   openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]     = HDLC_FLAG;
}
/**
\brief Add a byte to the outgoing HDLC frame being built.
*/
port_INLINE void outputHdlcWrite(uint8_t b) {
   
   // iterate through CRC calculator
   openserial_vars.outputCrc = crcIteration(openserial_vars.outputCrc,b);
   
   // add byte to buffer
   if (b==HDLC_FLAG || b==HDLC_ESCAPE) {
      openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]  = HDLC_ESCAPE;
      b                                               = b^HDLC_ESCAPE_MASK;
   }
   openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]     = b;
   
}
/**
\brief Finalize the outgoing HDLC frame.
*/
port_INLINE void outputHdlcClose() {
   uint16_t   finalCrc;
    
   // finalize the calculation of the CRC
   finalCrc   = ~openserial_vars.outputCrc;
   
   // write the CRC value
   outputHdlcWrite((finalCrc>>0)&0xff);
   outputHdlcWrite((finalCrc>>8)&0xff);
   
   // write the closing HDLC flag
   openserial_vars.outputBuf[openserial_vars.outputBufIdxW++]   = HDLC_FLAG;
}

//===== hdlc (input)

/**
\brief Start an HDLC frame in the input buffer.
*/
port_INLINE void inputHdlcOpen() {
   // reset the input buffer index
   openserial_vars.inputBufFill                       = 0;
   
   // initialize the value of the CRC
   openserial_vars.inputCrc                           = HDLC_CRCINIT;
}
/**
\brief Add a byte to the incoming HDLC frame.
*/
port_INLINE void inputHdlcWrite(uint8_t b) {
   if (b==HDLC_ESCAPE) {
      openserial_vars.inputEscaping = TRUE;
   } else {
      if (openserial_vars.inputEscaping==TRUE) {
         b                             = b^HDLC_ESCAPE_MASK;
         openserial_vars.inputEscaping = FALSE;
      }
      
      // add byte to input buffer
      openserial_vars.inputBuf[openserial_vars.inputBufFill] = b;
      openserial_vars.inputBufFill++;
      
      // iterate through CRC calculator
      openserial_vars.inputCrc = crcIteration(openserial_vars.inputCrc,b);
   }
}
/**
\brief Finalize the incoming HDLC frame.
*/
port_INLINE void inputHdlcClose() {
   
   // verify the validity of the frame
   if (openserial_vars.inputCrc==HDLC_CRCGOOD) {
      // the CRC is correct
      
      // remove the CRC from the input buffer
      openserial_vars.inputBufFill    -= 2;
   } else {
      // the CRC is incorrect
      
      // drop the incoming fram
      openserial_vars.inputBufFill     = 0;
   }
}

//=========================== interrupt handlers ==============================

//executed in ISR, called from scheduler.c
void isr_openserial_tx() {
   switch (openserial_vars.mode) {
      case MODE_INPUT:
         openserial_vars.reqFrameIdx++;
         if (openserial_vars.reqFrameIdx<sizeof(openserial_vars.reqFrame)) {
            uart_writeByte(openserial_vars.reqFrame[openserial_vars.reqFrameIdx]);
         }
         break;
      case MODE_OUTPUT:
         if (openserial_vars.outputBufIdxW==openserial_vars.outputBufIdxR) {
            openserial_vars.outputBufFilled = FALSE;
         }
         if (openserial_vars.outputBufFilled) {
            uart_writeByte(openserial_vars.outputBuf[openserial_vars.outputBufIdxR++]);
         }
         break;
      case MODE_OFF:
      default:
         break;
   }
}

// executed in ISR, called from scheduler.c
void isr_openserial_rx() {
   uint8_t rxbyte;
   uint8_t inputBufFill;
   
   // stop if I'm not in input mode
   if (openserial_vars.mode!=MODE_INPUT) {
      return;
   }
   
   // read byte just received
   rxbyte = uart_readByte();
   //keep lenght
   inputBufFill=openserial_vars.inputBufFill;
   
   if        (
                openserial_vars.busyReceiving==FALSE  &&
                openserial_vars.lastRxByte==HDLC_FLAG &&
                rxbyte!=HDLC_FLAG
              ) {
      // start of frame
      
      // I'm now receiving
      openserial_vars.busyReceiving         = TRUE;
      
      // create the HDLC frame
      inputHdlcOpen();
      
      // add the byte just received
      inputHdlcWrite(rxbyte);
   } else if (
                openserial_vars.busyReceiving==TRUE   &&
                rxbyte!=HDLC_FLAG
             ) {
      // middle of frame
      
      // add the byte just received
      inputHdlcWrite(rxbyte);
      if (openserial_vars.inputBufFill+1>SERIAL_INPUT_BUFFER_SIZE){
         // input buffer overflow
         openserial_printError(COMPONENT_OPENSERIAL,ERR_INPUT_BUFFER_OVERFLOW,
                               (errorparameter_t)0,
                               (errorparameter_t)0);
         openserial_vars.inputBufFill       = 0;
         openserial_vars.busyReceiving      = FALSE;
         openserial_stop();
      }
   } else if (
                openserial_vars.busyReceiving==TRUE   &&
                rxbyte==HDLC_FLAG
              ) {
         // end of frame
         
         // finalize the HDLC frame
         inputHdlcClose();
         
         if (openserial_vars.inputBufFill==0){
            // invalid HDLC frame
            openserial_printError(COMPONENT_OPENSERIAL,ERR_WRONG_CRC_INPUT,
                                  (errorparameter_t)inputBufFill,
                                  (errorparameter_t)0);
         
         }
         
         openserial_vars.busyReceiving      = FALSE;
         openserial_stop();
   }
   
   openserial_vars.lastRxByte = rxbyte;
}

//======== SERIAL ECHO =============

void openserial_echo(uint8_t* buf, uint8_t bufLen){
   INTERRUPT_DECLARATION();
   // echo back what you received
   openserial_printData(
      buf,
      bufLen
   );
   
    DISABLE_INTERRUPTS();
    openserial_vars.inputBufFill = 0;
    ENABLE_INTERRUPTS();
}
#endif

