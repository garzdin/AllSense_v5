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

void serial_rx_isr_handler (SERIAL_t * serial) {
	uint8_t status;
	uint8_t data;
	uint8_t tmphead;

	status = serial->_usart->STATUS;

	if ((status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm)) == 0)
	{
		data = serial->_usart->DATA;
		tmphead = (serial->_rx_head + 1) & RX_BUFFER_MASK;
		serial->_rx_head = tmphead;

		if (tmphead == serial->_rx_tail) {
			serial->_rx_buf_ovf = 1;
		} else {
			if (serial->_rx_buf_ovf == 1) serial->_rx_buf_ovf = 0;
			serial->_rx_buf[tmphead] = data;
		}
	} else {
		serial->_rx_buf_ovf = 1;
	}
}

void serial_tx_isr_handler (SERIAL_t * serial) {
	uint8_t tmptail;

	if (serial->_tx_head != serial->_tx_tail) {
		tmptail = (serial->_tx_tail + 1) & TX_BUFFER_MASK;
		serial->_tx_tail = tmptail;
		serial->_usart->DATA = serial->_tx_buf[tmptail];
	} else {
		serial->_usart->CTRLB &= ~(USART_TXEN_bm);
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
	if (serial->_rx_head != serial->_rx_tail) {
		return OK;
	}

	return ERROR;
}

SERIAL_RET_t serial_putchar (SERIAL_t * serial, uint8_t c) {
	uint8_t tmphead;

	tmphead = (serial->_tx_head + 1) & TX_BUFFER_MASK;

	while (tmphead == serial->_tx_tail);

	serial->_tx_buf[tmphead] = c;
	serial->_tx_head = tmphead;

	serial->_usart->CTRLA |= USART_TXEN_bm;

	return OK;
}

uint8_t serial_getchar (SERIAL_t * serial) {
	uint8_t tmptail;

	while (serial->_rx_head == serial->_rx_tail);

	tmptail = (serial->_rx_tail + 1) & RX_BUFFER_MASK;
	serial->_rx_tail = tmptail;

	return (uint8_t) serial->_rx_buf[tmptail];
}

SERIAL_RET_t serial_puts (SERIAL_t * serial, char * str) {
	SERIAL_RET_t ret = OK;

	while (*str) {
		ret = serial_putchar(serial, *str);
		str++;
	}

	return ret;
}

SERIAL_RET_t serial_gets (SERIAL_t * serial, uint8_t * buf) {
	SERIAL_RET_t ret = OK;
	uint8_t c;
	uint8_t i = 0;

	while ((c = serial_getchar(serial))) {
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

	// ret = serial_set_tx(serial, (uint8_t) USART_TXEN_bm);

	serial->_rx_buf_size = RX_BUFFER_SIZE;
	serial->_tx_buf_size = TX_BUFFER_SIZE;
	serial->_rx_head = 0;
	serial->_rx_tail = 0;
	serial->_tx_head = 0;
	serial->_tx_tail = 0;
	serial->_rx_buf_ovf = 0;

	serial->_rx_buf = (uint8_t *) malloc(RX_BUFFER_SIZE * sizeof(uint8_t));
	serial->_tx_buf = (uint8_t *) malloc(TX_BUFFER_SIZE * sizeof(uint8_t));

	return ret;
}
