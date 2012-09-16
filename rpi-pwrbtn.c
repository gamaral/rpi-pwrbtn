/*
 * Copyright 2012 Guillermo A. Amaral B. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "common/defs.h"

static const uint8_t BTN        = DDB0;
static const uint8_t LED        = DDB5;
static const uint8_t RPI_READY  = DDD3;
static const uint8_t RPI_PWR    = DDD2;
static const uint8_t RPI_PWROFF = DDD4;

static const uint8_t BOOT_TIMEOUT     = 40; /* seconds */
static const uint8_t SHUTDOWN_TIMEOUT = 40; /* seconds */

typedef enum __state_t
{
	unknown_state = 0,
	boot_state,
	idle_state,
	shutdown_state,
	poweroff_state
} state_t;

struct
{
	state_t  state;
	uint16_t timer;
	uint8_t  seconds;
} data;

/****************************************************************************/

static void boot_tick(void);
static void shutdown_tick(void);
static void bork_tick(void);

static void delay(uint16_t msecs);
static void state_change(state_t new_state);

/****************************************************************************/

void
boot_tick(void)
#define BOOT_TICK 200
{
	TOGGLE(PORTB, LED);
	delay(BOOT_TICK);

	if (data.seconds >= BOOT_TIMEOUT)
		state_change(idle_state);
}

void
shutdown_tick(void)
#define SHUTDOWN_TICK 500
{
	TOGGLE(PORTB, LED);
	delay(SHUTDOWN_TICK);

	if (data.seconds >= SHUTDOWN_TIMEOUT)
		state_change(poweroff_state);
}

void
bork_tick(void)
#define BORK_TICK 10
{
	TOGGLE(PORTB, LED);
	DELAY(BORK_TICK);
	TOGGLE(PORTB, LED);
	DELAY(BORK_TICK * 2);
}

void
delay(uint16_t msecs)
#define SECOND 1000
{
	switch(data.state) {
	case boot_state:
	case shutdown_state:
		/*
		 * process timer and second
		 */
		DELAY(msecs);
		data.timer += msecs;
		if (data.timer >= SECOND) {
			++data.seconds;
			data.timer -= SECOND;
		}
		break;

	case idle_state:
	case poweroff_state:
	case unknown_state:
	default: DELAY(msecs);
	}
}

void
state_change(state_t new_state)
{
	data.state = new_state;

	switch(data.state) {
	case boot_state:
		LOW(PORTB, LED);
		LOW(PORTD, RPI_PWR);
		LOW(PORTD, RPI_PWROFF);
		data.timer   = 0;
		data.seconds = 0;
		break;

	case shutdown_state:
		HIGH(PORTD, RPI_PWROFF);
		LOW(PORTB,  LED);
		data.timer   = 0;
		data.seconds = 0;
		break;

	case poweroff_state:
		HIGH(PORTD, RPI_PWR);
		LOW(PORTB,  LED);
		LOW(PORTD,  RPI_PWROFF);
		break;

	case idle_state:
		HIGH(PORTB, LED);
		break;

	default: break;
	}
}

/****************************************************************************/

void
setup(void)
{
	/* turn off interrupts */
	cli();

	/* defaults */
	DDRB  = 0b00000000;
	DDRD  = 0b00000000;
	PORTB = 0b00000000;
	PORTD = 0b00000000;

	/*
	 * Power Indicator LED
	 */
	OUT(DDRB,  LED);
	LOW(PORTB, LED);

	/*
	 * Power BTN Pin
	 */
	IN(DDRB,   BTN);
	LOW(PORTB, BTN);

	/*
	 * RPI Power On Pin
	 */
	OUT(DDRD,   RPI_PWR);
	HIGH(PORTD, RPI_PWR);

	/*
	 * RPI Power Off Pin
	 */
	OUT(DDRD,  RPI_PWROFF);
	LOW(PORTD, RPI_PWROFF);

	/*
	 * RPI Ready Pin
	 */
	IN(DDRD,   RPI_READY);
	LOW(PORTD, RPI_READY);

	/* Handle External Interrupt for BTN */
	PCICR  |= _BV(PCIE0);
	PCMSK0 |= _BV(PCINT0);

	/* Handle External Interrupt for RPI BOOT */
	EICRA |= _BV(ISC10) | _BV(ISC11);
	EIMSK |= _BV(INT1);

	/*
	 * power save settings
	 */
	power_adc_disable();
	power_spi_disable();
	power_twi_disable();
	power_usart0_disable();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	/* initialize data struct */
	data.state = unknown_state;
	data.timer = 0;
	data.seconds = 0;

	/* enable interrupts */
	sei();

	state_change(poweroff_state);
}

void
loop(void)
{
	switch(data.state) {
	case boot_state:
		boot_tick();
		break;

	case shutdown_state:
		shutdown_tick();
		break;

	case idle_state:
	case poweroff_state:
		sleep_mode();
		break;

	case unknown_state:
	default:
		bork_tick();
		break;
	}
}

/*
 * Power button was pressed
 */
ISR(PCINT0_vect)
{
	if (0 == (PINB & _BV(BTN)))
		return;

	switch (data.state) {
	case idle_state:
		state_change(shutdown_state);
		break;

	case poweroff_state:
		state_change(boot_state);
		break;

	case boot_state:
	case shutdown_state:
	case unknown_state:
	default: break;
	}
}

/*
 * RPI has finished booting
 */
ISR(INT1_vect)
{
	switch (data.state) {
	case boot_state:
		/* force boot timeout */
		data.seconds = BOOT_TIMEOUT;
		break;

	case idle_state:
	case poweroff_state:
	case shutdown_state:
	case unknown_state:
	default: break;
	}
}

