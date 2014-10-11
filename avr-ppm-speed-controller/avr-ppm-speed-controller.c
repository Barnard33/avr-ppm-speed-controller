/*
 *
 * avr-ppm-speed-controller.c
 *
 * Copyright (C) 2014  Barnard33
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * This code is only intended to be used for educational purposes.
 * It is not production stable and must under no circumstance be used 
 * in any kind of radio controlled machinery, e.g., planes, cars, boats, etc.
 *
 * Created: 03.10.2014 12:50:26
 *
 */ 

#ifndef F_CPU
#define F_CPU 8000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include <util/delay.h>

#define TIMER_PRESCALER 256

/* eeprom I/O */
uint8_t eeprom_full_speed_ahead EEMEM = 0;
uint8_t eeprom_full_stop EEMEM = 0;
uint8_t eeprom_full_speed_astern EEMEM = 0;

/* global variables (normal mode) */
volatile uint8_t ppm_signal_received = 0;
volatile uint8_t full_speed_ahead = 0;
volatile uint8_t full_stop = 0;
volatile uint8_t full_speed_astern = 0;
volatile uint8_t scaling_factor = 0;

/* global variables (for setup procedure) */
volatile uint8_t setup_trigger = 0;
volatile uint8_t setup_mode = 0;
volatile uint8_t setup_pulse_length = 0;

inline void software_reset(void) {
	wdt_enable(8);
	while(1);
}

inline void init_timer0_prescaler_256(void) {
	TCCR0B &= ~((1 << CS00) | (1 << CS01));
	TCCR0B |= (1 << CS02);
}

inline void init_timer1_pwm(void) {
	/* clear OC1A/OC1B on compare match when up-counting
	   set OC1A/OC1B on compare match when down-counting */
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
	
	/* phase correct PWM, 8-bit */
	TCCR1A |= (1 << WGM10);
	
	/* set clock to clkIO/8 */
	TCCR1B |= (1 << CS11);
}

inline void connect_oc1a(void) {
	DDRB |= (1 << PB3);
}

inline void disconnect_oc1a(void) {
	DDRB &= ~(1 << PB3);
	OCR1A = 0;
}

inline uint8_t is_oc1a_connected(void) {
	return DDRB & (1 << PB3);
}

inline void connect_oc1b(void) {
	DDRB |= (1 << PB4);
}

inline void disconnect_oc1b(void) {
	DDRB &= ~(1 << PB4);
	OCR1B = 0;
}

inline uint8_t is_oc1b_connected(void) {
	return DDRB & (1 << PB4);
}

inline uint8_t calculate_ocr1_value(uint8_t signal) {
	int8_t len = (int8_t)signal - (int8_t)full_stop;
	if(len < 0) {
		len = len * (-1);
	}
	return len * scaling_factor;
}

inline void init_interrupts(void) {
	/* activate INT0 on PD2 */
	GIMSK |= (1 << INT0);
	/* generate interrupt on rising edge for INT0 */
	MCUCR |= ((1 << ISC01) | (1 << ISC00));
}

inline void disbale_watchdog(void) {
	uint8_t sreg = SREG;
	cli();
	/* clear WDRF */
	MCUSR &= ~(1 << WDRF);
	/* keep old prescaler setting to prevent unintentional time out */
	WDTCR |= (1 << WDCE) | (1 << WDE);
	/* turn off WDT */
	WDTCR = 0x00;
	SREG = sreg;
}

inline uint8_t is_brownout_reset(void) {
	return MCUSR & (1 << BORF);
}

inline void reset_brownout_reset_flag(void) {
	MCUSR &= ~(1 << BORF);
}

void read_config_from_eeprom(void) {
	full_speed_ahead = eeprom_read_byte(&eeprom_full_speed_ahead);
	full_speed_astern = eeprom_read_byte(&eeprom_full_speed_astern);
	full_stop = eeprom_read_byte(&eeprom_full_stop);
}

void write_config_to_eeprom() {
	eeprom_write_byte(&eeprom_full_speed_ahead, full_speed_ahead);
	eeprom_write_byte(&eeprom_full_speed_astern, full_speed_astern);
	eeprom_write_byte(&eeprom_full_stop, full_stop);
}

void calculate_scaling_factor(void) {
	scaling_factor = (255 / (full_speed_ahead - full_stop));
}

void setup(void) {
	setup_mode = 1;
	
	/* generate interrupt on falling edge for INT1 */
	MCUCR |= (1 << ISC10) | 1 << ISC11;
	/* activate INT1 on PD3 */
	GIMSK |= (1 << INT1);
	
	sei();
	
	PORTD |= (1 << PD5); // LED on
	_delay_ms(250);
	PORTD &= ~(1 << PD5); // LED off
	_delay_ms(250);
	
	while(!setup_trigger) {
		_delay_ms(50);
	}
	
	PORTD |= (1 << PD5);
	
	full_stop = setup_pulse_length;
	_delay_ms(50);
	
	full_speed_ahead = full_stop;
	full_speed_astern = full_stop;
	
	uint8_t current = setup_pulse_length;
	uint8_t has_ahead = 0;
	uint8_t has_astern = 0;
	
	while(1) {
		if(current > full_speed_ahead) {
			full_speed_ahead = current;
			if(current > full_stop + 2) {
				has_ahead = 1;
			}
		} 
		else if(current <  full_speed_astern) {
			full_speed_astern = current;
			if(current < full_stop - 2) {
				has_astern = 1;
			}
		}
		else if(current > full_stop -2 && current < full_stop + 2 && has_astern && has_ahead) {
			break;
		}
		_delay_ms(25);
		current = setup_pulse_length;
	}
	
	cli();
	write_config_to_eeprom();
	
	PORTD &= ~(1 << PD5); // LED off
	_delay_ms(500);
	PORTD |= (1 << PD5); // LED on
	_delay_ms(750);
	PORTD &= ~(1 << PD5); // LED off
	
	software_reset();
}

int main(void)
{
	disbale_watchdog(); // setup requires watchdog to cause a real reset
	read_config_from_eeprom();
	calculate_scaling_factor();
	
	/* in pin init PD2/INT0=ppm in; PD3/INT1=setup button */
	DDRD &= ~((1 << PD2) & (1 << PD3));
	PORTD |= (1 << PD3);
	
	/* init LED out pin */
	DDRD |= (1 << PD5);
	
	init_timer0_prescaler_256();
	init_interrupts();
	
	if(!is_brownout_reset()) {
		if(!(PIND & (1 << PD3))) {
			setup();
			return -1;
		}
	}
	else {
		reset_brownout_reset_flag();
	}
	
	OCR1A = 0;
	OCR1B = 0;
	init_timer1_pwm();
	
	sei();
	
	uint8_t i = 0;
	
	while(1)
	{
		if(!ppm_signal_received) {
			i++;	
		}
		else {
			ppm_signal_received = 0;
			i = 0;
			PORTD &= ~(1 << PD5); // LED off
		}
		if(i > 3) {
			disconnect_oc1a();
			disconnect_oc1b();
			i = 0;
			PORTD |= (1 << PD5); // LED on
		}
		_delay_ms(21);
	}
	
	return 0;
}

/* servo input pin on INT0 */
ISR(INT0_vect) {
	if((MCUCR & (1<<ISC00)) > 0) {
		TCNT0 = 0;
		MCUCR &= ~(1 << ISC00);
	}
	else {
		MCUCR |= (1 << ISC00);
		uint8_t pulse_length = TCNT0;
		
		ppm_signal_received = 1;
		
		if(setup_mode) {
			setup_pulse_length = pulse_length;
			return;
		}
		
		if(pulse_length < full_speed_astern) {
			pulse_length = full_speed_astern;
		}
		if(pulse_length > full_speed_ahead) {
			pulse_length = full_speed_ahead;
		}
		
		if(pulse_length > full_stop + 1) {
			if(!is_oc1a_connected()) {
				disconnect_oc1b();
				connect_oc1a();
			}
			OCR1A = calculate_ocr1_value(pulse_length);
		}
		else if(pulse_length < full_stop - 1) {
			if(!is_oc1b_connected()) {
				disconnect_oc1a();
				connect_oc1b();
			}
			OCR1B = calculate_ocr1_value(pulse_length);
		}
		else {
			disconnect_oc1a();
			disconnect_oc1b();
		}
	}
}

/* setup button */
ISR(INT1_vect) {
	/* deactivate INT1 on PD3 */
	GIMSK &= ~(1 << INT1);
	setup_trigger = 1;
}