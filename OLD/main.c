#define F_CPU 7378200UL
#define BAUD_L 115200UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "serial.h"

// System Clocks initialization
void system_clocks_init(void)
{
	unsigned char n,s;

	// Save interrupts enabled/disabled state
	s=SREG;
	// Disable interrupts
	asm("cli");

	// External 7372.800 kHz oscillator initialization
	// Crystal oscillator increased drive current: Off
	// External Clock Source - Startup Time: 0.4-16 MHz Quartz Crystal - 16k CLK
	OSC.XOSCCTRL=OSC_FRQRANGE_2TO9_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;
	// Enable the external oscillator/clock source
	OSC.CTRL|=OSC_XOSCEN_bm;

	// System Clock prescaler A division factor: 1
	// System Clock prescalers B & C division factors: B:1, C:1
	// ClkPer4: 7372.800 kHz
	// ClkPer2: 7372.800 kHz
	// ClkPer:  7372.800 kHz
	// ClkCPU:  7372.800 kHz
	n=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
	CCP=CCP_IOREG_gc;
	CLK.PSCTRL=n;

	// Wait for the external oscillator to stabilize
	while ((OSC.STATUS & OSC_XOSCRDY_bm)==0);

	// Select the system clock source: External Oscillator or Clock
	n=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_XOSC_gc;
	CCP=CCP_IOREG_gc;
	CLK.CTRL=n;

	// Disable the unused oscillators: 2 MHz, 32 MHz, internal 32 kHz, PLL
	OSC.CTRL&= ~(OSC_RC2MEN_bm | OSC_RC32MEN_bm | OSC_RC32KEN_bm | OSC_PLLEN_bm);

	// ClkPer output disabled
	PORTCFG.CLKEVOUT&= ~(PORTCFG_CLKOUTSEL_gm | PORTCFG_CLKOUT_gm);
	// Restore interrupts enabled/disabled state
	SREG=s;
}

void gsm_init() {
	PORTD.DIR = 0x03;
	PORTD.OUT = 0x03;
}

SERIAL_t debug;
SERIAL_t gsm;
uint8_t * resp_buf;

int main(void)
{
	// Declare your local variables here
	unsigned char n;

	// Check the reset source
	n=RST.STATUS;
	if (n & RST_PORF_bm)
	{
		// Power on reset

	}
	else if (n & RST_EXTRF_bm)
	{
		// External reset

	}
	else if (n & RST_BORF_bm)
	{
		// Brown out reset

	}
	else if (n & RST_WDRF_bm)
	{
		// Watchdog reset

	}
	else if (n & RST_PDIRF_bm)
	{
		// Program and debug interface reset

	}
	else if (n & RST_SRF_bm)
	{
		// Software reset

	}
	// Clear the reset flag
	RST.STATUS=n;

	// Interrupt system initialization
	// Make sure the interrupts are disabled
	asm("cli");
	// Low level interrupt: On
	// Round-robin scheduling for low level interrupt: On
	// Medium level interrupt: Off
	// High level interrupt: Off
	// The interrupt vectors will be placed at the start of the Application FLASH section
	n=(PMIC.CTRL & (~(PMIC_RREN_bm | PMIC_IVSEL_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm))) |
	PMIC_LOLVLEN_bm | PMIC_RREN_bm;
	CCP=CCP_IOREG_gc;
	PMIC.CTRL=n;
	// Set the default priority for round-robin scheduling
	PMIC.INTPRI=0x00;

	// System clocks initialization
	system_clocks_init();

	// GSM initialization
	gsm_init();

	serial_init(&debug, &USARTE0, &PORTE, (uint32_t) BAUD_L, (uint32_t) F_CPU);
	serial_init(&gsm, &USARTC0, &PORTC, (uint32_t) BAUD_L, (uint32_t) F_CPU);
	serial_listen(&gsm);
	serial_listen(&debug);

	// Globally enable interrupts
	asm("sei");

	serial_puts(&debug, "Hello from XMEGA\r");

	while(serial_available(&debug)) {
		serial_putchar(&gsm, serial_getchar(&debug));
	}

	while(serial_available(&gsm)) {
		serial_putchar(&debug, serial_getchar(&gsm));
	}

	// Sync baud rate
	do {
		serial_puts(&gsm, "AT\r");

		if (serial_available(&gsm)) {
			if (sscanf_P((char *) resp_buf, "\r\nOK\r\n")) {
				break;
			}
		} else {
			serial_gets(&gsm, resp_buf);
		}
	} while(1);

	serial_puts(&gsm, "ATI\r");

	while (!serial_available(&gsm));

	serial_gets(&gsm, resp_buf);

	while (1) {}
}

// USART 0 PORTE TX interrupt
ISR(USARTE0_TXC_vect) {
	serial_tx_isr_handler(&debug);
}

// USART 0 PORTE RX interrupt
ISR(USARTE0_RXC_vect) {
	serial_rx_isr_handler(&debug);
}

// USART 0 PORTC TX interrupt
ISR(USARTC0_TXC_vect) {
	serial_tx_isr_handler(&gsm);
}

// USART 0 PORTC RX interrupt
ISR(USARTC0_RXC_vect) {
	serial_rx_isr_handler(&gsm);
}
