/*
 * serial.h
 *
 * Created: 2/21/2018 11:05:59 AM
 *  Author: Teodor
 */


#ifndef SERIAL_H_
#define SERIAL_H_

#include <avr/io.h>

#define _BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,bscale) (                \
((bscale) < 0) ?                                                      \
(int)((((float)(f_cpu)/(16*(float)(baud)))-1)*(1/1<<-(bscale)))        \
: (int)((float)(f_cpu)/((1<<(bscale))*16*(float)(baud)))-1 )

#define _BSCALE(f_cpu,baud) (                                         \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,-7) < 4096) ? -7 :              \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,-6) < 4096) ? -6 :              \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,-5) < 4096) ? -5 :              \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,-4) < 4096) ? -4 :              \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,-3) < 4096) ? -3 :              \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,-2) < 4096) ? -2 :              \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,-1) < 4096) ? -1 :              \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,0) < 4096) ? 0 :                \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,1) < 4096) ? 1 :                \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,2) < 4096) ? 2 :                \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,3) < 4096) ? 3 :                \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,4) < 4096) ? 4 :                \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,5) < 4096) ? 5 :                \
(_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,6) < 4096) ? 6 :                \
7 )

#define BSEL(f_cpu,baud)                                              \
_BAUD_BSEL_FROM_BAUDSCALE(f_cpu,baud,_BSCALE(f_cpu,baud))

#define BSCALE(f_cpu,baud) ((_BSCALE(f_cpu,baud)<0) ? (16+_BSCALE(f_cpu,baud)) : _BSCALE(f_cpu,baud))

#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256

typedef enum SERIAL_RET_enum {
	ERROR,
	OK,
} SERIAL_RET_t;

typedef struct SERIAL_struct {
	USART_t * _usart;
	PORT_t * _port;
	uint32_t _baud;
	uint8_t _rxc_int_lvl;
	uint8_t _txc_int_lvl;
	uint8_t _rx_en;
	uint8_t _tx_en;
	uint16_t _rx_buf_size;
	uint16_t _tx_buf_size;
	uint8_t * _rx_buf;
	uint8_t * _tx_buf;
	uint8_t _rx_buf_ovf;
	#if RX_BUFFER_SIZE <= 256
	volatile uint8_t _rx_wr_index;
	volatile uint8_t _rx_rd_index;
	volatile uint8_t _tx_wr_index;
	volatile uint8_t _tx_rd_index;
	#else
	volatile uint16_t _rx_wr_index;
	volatile uint16_t _rx_rd_index;
	volatile uint16_t _tx_wr_index;
	volatile uint16_t _tx_rd_index;
	#endif
	#if RX_BUFFER_SIZE < 256
	volatile uint8_t _rx_counter;
	volatile uint8_t _tx_counter;
	#else
	volatile uint16_t _rx_counter;
	volatile uint16_t _tx_counter;
	#endif
} SERIAL_t;

void serial_rx_isr_handler (SERIAL_t * serial);
void serial_tx_isr_handler(SERIAL_t * serial);
SERIAL_RET_t serial_set_baud (SERIAL_t * serial, uint32_t * f_cpu, uint32_t * baud);
SERIAL_RET_t serial_set_tx (SERIAL_t * serial, uint8_t tx_state);
SERIAL_RET_t serial_set_rx (SERIAL_t * serial, uint8_t rx_state);
SERIAL_RET_t serial_listen (SERIAL_t * serial);
SERIAL_RET_t serial_available (SERIAL_t * serial);
SERIAL_RET_t serial_putchar (SERIAL_t * serial, uint8_t c);
uint8_t serial_getchar (SERIAL_t * serial, SERIAL_t * debug);
SERIAL_RET_t serial_puts (SERIAL_t * serial, char * str);
SERIAL_RET_t serial_gets (SERIAL_t * serial, uint8_t * buf, SERIAL_t * debug);
SERIAL_RET_t serial_init (SERIAL_t * serial, USART_t * usart, PORT_t * port, uint32_t baud, uint32_t f_cpu);

#endif /* SERIAL_H_ */
