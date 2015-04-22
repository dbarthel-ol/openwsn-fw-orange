/**
\brief Serial driver, HDLC formatted frames

\author Quentin Lampin <quentin.lampin@orange.com>, February 2015.


HDLC frames format is as follow:

		| `flag` | `command` | `data`[...]  | `frame check sequence` | `flag` |
		|:------:|:---------:|:------------:|:----------------------:|:------:|
		|  0x7E  |   0xXX    |     0x00     |         16bit          |  0x7E  |

*/

#include "stdint.h"
#include "opendefs.h"


#define SERIAL_OUTPUT_BUFFER_LENGTH 1024 					// !!! Needs to be a power of 2, e.g. 256(2^8), 512(2^9), 1024(2^10), etc.
#define SERIAL_INPUT_BUFFER_LENGTH 512 						// !!! Needs to be a power of 2, e.g. 256(2^8), 512(2^9), 1024(2^10), etc.


typedef struct openserial_vars
{
	uint8_t buffer_output[SERIAL_OUTPUT_BUFFER_LENGTH];			// stores the mote -> platform data
	uint16_t buffer_output_write_index;							// index of the first byte available for writing in the buffer
	uint16_t buffer_output_read_index;							// index of the first byte available for reading in the buffer
	uint8_t buffer_output_overflow_errors;							// count of write errors due to a full buffer
	uint8_t buffer_output_writers;								// count of writer routines currently accessing the buffer

	uint8_t buffer_input[SERIAL_INPUT_BUFFER_LENGTH];			// stores the platform -> mote data
	uint16_t buffer_input_write_index;							// index of the first byte available for writing in the buffer
	uint16_t buffer_input_read_index;							// index of the first byte available for reading in the buffer
	uint16_t buffer_input_available;							// number of bytes available in the buffer
	uint8_t buffer_input_overflow_errors;							// count of write errors due to a full buffer


	bool sending;												// sending on the UART
	bool frame_received;										// frame received on the UART

	bool uart_using_DMA;
} openserial_vars_t;

void openserial_init(void);
void openserial_write(uint8_t* bytes, uint16_t size);

void on_uart_ready_to_send_interrupt(void);
void on_uart_rx_interrupt(void);
void on_uart_dma_write_complete(uint16_t bytes_written);




/* legacy declarations */
void    openopenserial_init(void);
owerror_t openserial_printStatus(uint8_t statusElement, uint8_t* buffer, uint8_t length);
owerror_t openserial_printInfo(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2);
owerror_t openserial_printError(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2);
owerror_t openserial_printCritical(uint8_t calling_component, uint8_t error_code,
                              errorparameter_t arg1,
                              errorparameter_t arg2);
owerror_t openserial_printData(uint8_t* buffer, uint8_t length);
owerror_t openserial_printObservability(uint8_t* buffer, uint8_t length);
uint8_t openserial_getNumDataBytes(void);
uint8_t openserial_getInputBuffer(uint8_t* bufferToWrite, uint8_t maxNumBytes);
void    openserial_startInput(void);
void    openserial_startOutput(void);
void    openserial_stop(void);
bool    debugPrint_outBufferIndexes(void);
void    openserial_echo(uint8_t* but, uint8_t bufLen);

// interrupt handlers
void    isr_openserial_rx(void);
void    isr_openserial_tx(void);