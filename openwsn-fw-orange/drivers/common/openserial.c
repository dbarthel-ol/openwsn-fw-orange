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
