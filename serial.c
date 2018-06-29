/*
 * serial.c
 *
 * Created: 2/21/2018 11:05:51 AM
 *  Author: Teodor
 */

#include <avr/io.h>
#include <stdlib.h>
#include <math.h>
#include "serial.h"

void serial_rx_isr_handler(SERIAL_t * serial) {
	uint8_t status;
	uint8_t data;

	status = serial->_usart->STATUS;
	data = serial->_usart->DATA;
	if ((status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm)) == 0)
	{
		serial->_rx_buf[serial->_rx_wr_index++] = data;
		#if RX_BUFFER_SIZE == 256
			// special case for receiver buffer size=256
			if (++serial->_rx_counter == 0) serial->_rx_buf_ovf = 1;
		#else
			if (serial->_rx_wr_index == serial->_rx_buf_size) serial->_rx_wr_index = 0;
			if (++serial->_rx_counter == serial->_rx_buf_size)
			{
				serial->_rx_counter = 0;
				serial->_rx_buf_ovf = 1;
			}
		#endif
	}
}

void serial_tx_isr_handler(SERIAL_t * serial) {
	if (serial->_tx_counter)
	{
		--(serial->_tx_counter);
		serial->_usart->DATA = serial->_tx_buf[serial->_tx_rd_index++];
		#if TX_BUFFER_SIZE != 256
			if (serial->_tx_rd_index == serial->_tx_buf_size) serial->_tx_rd_index = 0;
		#endif
	}
}

SERIAL_RET_t serial_set_baud (SERIAL_t * serial, uint32_t * f_cpu, uint32_t * baud) {	
	serial->_baud = *baud;
	
	serial->_usart->BAUDCTRLA = (BSEL(*f_cpu, *baud) & 0xff);
	serial->_usart->BAUDCTRLB = (((BSCALE(*f_cpu, *baud) << USART_BSCALE_gp) & USART_BSCALE_gm) | (BSEL(*f_cpu, *baud) >> 8));
	
	return OK;
}

SERIAL_RET_t serial_set_tx (SERIAL_t * serial, uint8_t tx_state) {
	serial->_tx_en = tx_state;
	
	serial->_usart->CTRLB |= (uint8_t) tx_state;
	
	return OK;
}

SERIAL_RET_t serial_set_rx (SERIAL_t * serial, uint8_t rx_state) {
	serial->_rx_en = rx_state;
	
	serial->_usart->CTRLB |= rx_state;
	
	return OK;
}

SERIAL_RET_t serial_listen (SERIAL_t * serial) {	
	serial_set_rx(serial, (uint8_t) USART_RXEN_bm);
	serial->_usart->CTRLA |= USART_RXCINTLVL_LO_gc;

	return OK;
}

SERIAL_RET_t serial_available (SERIAL_t * serial) {
	if (serial->_rx_counter) {
		return OK;
	}
	
	return ERROR;
}

SERIAL_RET_t serial_putchar (SERIAL_t * serial, uint8_t c) {
	while (serial->_tx_counter == serial->_tx_buf_size);
	
	asm("cli");
	
	if (serial->_tx_counter || ((serial->_usart->STATUS & USART_DREIF_bm) == 0))
	{
		serial->_tx_buf[serial->_tx_wr_index++] = c;
		#if TX_BUFFER_SIZE != 256
		if (serial->_tx_wr_index == serial->_tx_buf_size) serial->_tx_wr_index = 0;
		#endif
		++(serial->_tx_counter);
	} else {
		serial->_usart->DATA = c;
	}
	
	asm("sei");
	
	return OK;
}

uint8_t serial_getchar (SERIAL_t * serial, SERIAL_t * debug) {
	uint8_t data;

	while (serial->_rx_counter == 0);
	
	data = serial->_rx_buf[serial->_rx_rd_index++];

	#if RX_BUFFER_SIZE != 256
	if (serial->_rx_rd_index == serial->_rx_buf_size) serial->_rx_rd_index = 0;
	#endif
	asm("cli");
	--(serial->_rx_counter);
	asm("sei");
	
	#ifdef DEBUG
		serial_putchar(debug, data);
	#endif
	
	return data;
}

SERIAL_RET_t serial_puts (SERIAL_t * serial, char * str) {
	SERIAL_RET_t ret = OK;
	
	while (*str) {
		ret = serial_putchar(serial, *str);
		str++;
	}
	
	return ret;
}

SERIAL_RET_t serial_gets (SERIAL_t * serial, uint8_t * buf, SERIAL_t * debug) {
	SERIAL_RET_t ret = OK;
	uint8_t c;
	uint8_t i = 0;
	
	while ((c = serial_getchar(serial, debug))) {
		buf[i++] = c;
	}
	
	return ret;
}

SERIAL_RET_t serial_init (SERIAL_t * serial, USART_t * usart, PORT_t * port, uint32_t baud, uint32_t f_cpu) {
	SERIAL_RET_t ret = OK;
	
	serial->_usart = usart;
	serial->_port = port;
	
	serial->_port->DIR |= PIN3_bm;
	serial->_port->OUT |= PIN3_bm;
	
	serial->_port->OUTSET |= USART_TXEN_bm;

	serial->_usart->CTRLC |= USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	
	serial->_usart->CTRLA &= ~(USART_RXCINTLVL_gm | USART_TXCINTLVL_gm | USART_DREINTLVL_gm);
	serial->_usart->CTRLA |= USART_DREINTLVL_OFF_gc | USART_TXCINTLVL_LO_gc;
	
	ret = serial_set_baud(serial, &f_cpu, &baud);
	
	serial->_usart->CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm);
	
	ret = serial_set_tx(serial, (uint8_t) USART_TXEN_bm);
	
	serial->_rx_buf_size = RX_BUFFER_SIZE;
	serial->_tx_buf_size = TX_BUFFER_SIZE;
	serial->_rx_rd_index = 0;
	serial->_rx_wr_index = 0;
	serial->_tx_rd_index = 0;
	serial->_tx_wr_index = 0;
	serial->_rx_counter = 0;
	serial->_tx_counter = 0;
	serial->_rx_buf_ovf = 0;
	
	serial->_rx_buf = (uint8_t *) malloc(RX_BUFFER_SIZE * sizeof(uint8_t));
	serial->_tx_buf = (uint8_t *) malloc(TX_BUFFER_SIZE * sizeof(uint8_t));
	
	return ret;
}