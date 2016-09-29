/*
 * A finite state machine implementation in C
 * Copyright © 2016 Christian Rapp
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the organization nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CHRISTIAN RAPP ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CHRISTIAN RAPP BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* make sure to define F_CPU for your mcu */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif /* ifndef F_CPU */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h> /* no timers here, using simple delays */

typedef enum LIGHT_STATES {
    GREEN,
    RED,
    YELLOW,
    TO_GREEN,
    TO_RED,
    TO_YELLOW
} sts_type;
typedef enum AVR_PORT { PB, PD } avrp_type;

/**
 * @brief Read LDR and use the value for the state machine
 *
 * @return 0 -> GREEN, 1 -> RED, 2 -> YELLOW
 */
uint8_t read_sensor(void);
/**
 * @brief Activate LED according to current state
 *
 * @param reg s_type#reg
 * @param led s_type#output_pin
 */
void state_led(avrp_type reg, uint8_t led);

/**
 * @brief Struct representing one state
 */
typedef struct {
    uint8_t output_pin;                 /**< Which pin to use for this state */
    uint8_t reg;                        /**< which pin register */
    void (*output)(avrp_type, uint8_t); /**< Function this state will call to set
                                          the output */
    long counter;                       /**< guaranteed time for this state */
    sts_type next[3];                   /**< array of next states */
} s_type;

/* clang-format off */

/**
 * @brief State transition table
 */
s_type FSM[6] = {
    {PD5, PD, &state_led, 100, {GREEN, TO_RED, TO_YELLOW}},
    {PD6, PD, &state_led, 100, {TO_GREEN, RED, TO_YELLOW}},
    {PD7, PD, &state_led, 100, {TO_GREEN, TO_RED, YELLOW}},
    {PB0, PB, &state_led, 200, {GREEN, GREEN, GREEN}},
    {PB0, PB, &state_led, 300, {RED, RED, RED}},
    {PB0, PB, &state_led, 400, {YELLOW, YELLOW, YELLOW}}
};
/* clang-format on */

/**
 * @brief Current state
 */
s_type current_state;
/**
 * @brief Current counter
 */
long cnt;

int main(void)
{
    /* deactivate all the pins we need */
    PORTB &= ~(1 << PB0);
    PORTD &= ~((1 << PD5) | (1 << PD6) | (1 << PD7));

    /* set our pins to output */
    DDRB |= (1 << DDB0);
    DDRD |= (1 << DDD5) | (1 << DDD6) | (1 << DDD7);

    /* Set reference to avcc */
    ADMUX |= (1 << REFS0);
    /* enable adc, pre scaler 128 */
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    /* get the current state */
    current_state = FSM[0];
    cnt = current_state.counter;

    /* initial state */
    current_state.output(current_state.reg, current_state.output_pin);

    while (1) {
        /* state can have a counter so we have a guaranteed time for each state */
        if (cnt > 0) {
            cnt--;
            _delay_ms(10);
        } else {
            /* read sensor and use the value returned to determine the next state */
            uint8_t sensor = read_sensor();
            if (sensor != 3) {
                current_state = FSM[current_state.next[sensor]];
                cnt = current_state.counter;
                current_state.output(current_state.reg,
                                     current_state.output_pin);
            } else {
                cnt = 100;
            }
        }
    }

    return 0; /* will never reach this */
}

void state_led(avrp_type reg, uint8_t led)
{
    switch (reg) {
    case PB:
        PORTD &= ~((1 << PD5) | (1 << PD6) | (1 << PD7));
        PORTB |= (1 << led);
        break;
    case PD:
        PORTB &= ~(1 << PB0);
        PORTD &= ~((1 << PD5) | (1 << PD6) | (1 << PD7));
        PORTD |= (1 << led);
        break;
    }
}

uint8_t read_sensor(void)
{
    ADCSRA |= (1 << ADSC);
    while ((ADCSRA & (1 << ADSC))) {
    }

    uint16_t adc_value = ADCL;
    adc_value |= ADCH << 8;

    float voltage = (adc_value * 4.8828125) / 1000.0;

    if (voltage < 1.6) {
        return 0;
    }
    if (voltage >= 1.6 && voltage < 3.2) {
        return 1;
    }
    if (voltage >= 3.2) {
        return 2;
    }
    return 3;
}
