/*
 * Copyright 2012-2013 Guillermo A. Amaral B. All rights reserved.
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

#define __DELAY_BACKWARD_COMPATIBLE__

#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include <util/delay.h>

#ifndef REVISION
#  define REVISION 3
#endif

static const uint8_t RPI_USR = DDB0;
static const uint8_t RPI_LED = DDB2;
static const uint8_t RPI_PWR = DDB1;
static const uint8_t RPI_IN  = DDB3;
static const uint8_t RPI_OUT = DDB4;

static const uint8_t BOOT_TIMEOUT     = 30; /* seconds */
static const uint8_t SHUTDOWN_TIMEOUT = 30; /* seconds */

#define HIGH(PORT, BIT) \
	PORT |= _BV(BIT)
#define OUT(PORT, BIT) HIGH(PORT, BIT)
#define ON(PORT, BIT) HIGH(PORT, BIT)

#define LOW(PORT, BIT) \
	PORT &= ~_BV(BIT)
#define IN(PORT, BIT) LOW(PORT, BIT)
#define OFF(PORT, BIT) LOW(PORT, BIT)

#define TOGGLE(PORT, BIT) \
	PORT ^= _BV(BIT)

#define ISSET(PORT, BIT) \
	(_BV(BIT) == (PORT & _BV(BIT)))
#define ISCLR(PORT, BIT) \
	(0 == (PORT & _BV(BIT)))

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
	uint8_t  pinb;
} data;

/****************************************************************************/

static void boot_tick(void);
static void shutdown_tick(void);
static void bork_tick(void);

static void delay(const uint16_t msecs);
static void state_change(state_t new_state);

/****************************************************************************/

void
boot_tick(void)
#define BOOT_TICK 200
{
	TOGGLE(PORTB, RPI_LED);
	delay(BOOT_TICK);

	if (data.seconds >= BOOT_TIMEOUT)
		state_change(idle_state);
}

void
shutdown_tick(void)
#define SHUTDOWN_TICK 500
{
	TOGGLE(PORTB, RPI_LED);
	delay(SHUTDOWN_TICK);

	if (data.seconds >= SHUTDOWN_TIMEOUT)
		state_change(poweroff_state);
}

void
bork_tick(void)
#define BORK_TICK 10
{
	TOGGLE(PORTB, RPI_LED);
	_delay_ms(BORK_TICK);
	TOGGLE(PORTB, RPI_LED);
	_delay_ms(BORK_TICK * 2);
}

void
delay(const uint16_t msecs)
#define SECOND 1000
{
	switch(data.state) {
	case boot_state:
	case shutdown_state:
		/*
		 * process timer and second
		 */
		_delay_ms(msecs);
		data.timer += msecs;
		if (data.timer >= SECOND) {
			++data.seconds;
			data.timer -= SECOND;
		}
		break;

	case idle_state:
	case poweroff_state:
	case unknown_state:
	default: _delay_ms(msecs);
	}
}

void
state_change(state_t new_state)
{
	data.state = new_state;

	switch(data.state) {
	case boot_state:
		HIGH(PORTB, RPI_PWR);
#if REVISION < 3
		LOW(PORTB,  RPI_LED);
#else
		HIGH(PORTB, RPI_LED);
#endif
		LOW(PORTB,  RPI_OUT);
		data.timer   = 0;
		data.seconds = 0;
		break;

	case idle_state:
		HIGH(PORTB, RPI_PWR);
#if REVISION < 3
		HIGH(PORTB, RPI_LED);
#else
		LOW(PORTB,  RPI_LED);
#endif
		LOW(PORTB,  RPI_OUT);
		break;

	case shutdown_state:
		HIGH(PORTB, RPI_PWR);
#if REVISION < 3
		LOW(PORTB,  RPI_LED);
#else
		HIGH(PORTB, RPI_LED);
#endif
		HIGH(PORTB, RPI_OUT);
		data.timer   = 0;
		data.seconds = 0;
		break;

	case poweroff_state:
		LOW(PORTB,  RPI_PWR);
#if REVISION < 3
		LOW(PORTB,  RPI_LED);
#else
		HIGH(PORTB, RPI_LED);
#endif
		LOW(PORTB,  RPI_OUT);
		break;

	default: break;
	}
}

/****************************************************************************/

void
main(void)
{
	/*
	 * Setup
	 */

	/* turn off interrupts */
	cli();

	DDRB  = 0b00000000;
	PORTB = 0b00000000;

	/*
	 * Power Indicator RPI_LED
	 */
	OUT(DDRB,   RPI_LED);
#if REVISION < 3
	LOW(PORTB, RPI_LED);
#else
	HIGH(PORTB, RPI_LED);
#endif

	/*
	 * Power RPI_USR Pin
	 */
	IN(DDRB,    RPI_USR);
	HIGH(PORTB, RPI_USR);

	/*
	 * RPI Power Pin
	 */
	OUT(DDRB,  RPI_PWR);
	LOW(PORTB, RPI_PWR);

	/*
	 * RPI OUT Pin
	 */
	OUT(DDRB,  RPI_OUT);
	LOW(PORTB, RPI_OUT);

	/*
	 * RPI IN Pin
	 */
	IN(DDRB,    RPI_IN);
	HIGH(PORTB, RPI_IN);

	/* Handle External Interrupts */
	GIMSK |= _BV(PCIE);
	PCMSK |= _BV(PCINT0) | _BV(PCINT3);

	/*
	 * power save settings
	 */
	power_adc_disable();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	/* initialize data struct */
	data.state = unknown_state;
	data.timer = 0;
	data.seconds = 0;
	data.pinb = PINB;
	
	state_change(poweroff_state);

	/* enable interrupts */
	sei();

	/*
	 * Main Loop
	 */
	for(;;)
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
 * Interrupt Handler
 */
ISR(PCINT0_vect)
{
	const uint8_t pin_change = PINB ^ data.pinb;
	data.pinb = PINB;

	/*
	 * Raspberry Pi - Status Change
	 */
	if (ISSET(pin_change, RPI_IN)) {
		/*
		 * On
		 */
		if (ISCLR(PINB, RPI_IN)) {
			switch (data.state) {
			case boot_state:
				data.seconds = BOOT_TIMEOUT; // force boot timeout
				break;

			case shutdown_state:
				state_change(idle_state);
				break;

			case idle_state:
			case poweroff_state:
			case unknown_state:
			default: break;
			}
		}

		/*
		 * Off
		 */
		else {
			switch (data.state) {
			case idle_state:
				state_change(shutdown_state);
				break;

			case boot_state:
			case shutdown_state:
			case poweroff_state:
			case unknown_state:
			default: break;
			}
		}
	}

	/*
	 * Power Button
	 */
	if (ISSET(pin_change, RPI_USR)) {
		/*
		 * Released
		 */
		if (ISCLR(PINB, RPI_USR))
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
}

