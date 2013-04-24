/*
 *  main.c
 *
 *  Copyright (c) 2013 Nathaniel Lewis, Robotics Society at UC Merced
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define THRESHOLD 17

// Pins - 
//        PORTB0 - Enabled Signal (o) 1
//        PORTB1 - Global Enable  (i) 2
//        PORTB2 - Throttle signal(i) 4
//        PORTB4 - MOSFET control (o) 16

// Globals
volatile uint16_t ticks;

// Handle incrementing the timer counter
ISR(TIMER0_COMPA_vect)
{
    // Increment the counter
    ticks++;
}

// Handle radio interrupt
ISR(PCINT0_vect)
{
    // Check if we are enabled
    if(!(PINB & _BV(PORTB1)))
    {
        // Disable the controller immediately after enable line went low
        PORTB &= ~(_BV(PORTB0) | _BV(PORTB4));
    }
}

// Main function
int main ()
{
    // Initalize PortB0,2,4 to outputs
    DDRB = 0x11;
    PORTB = 0x00;
    
    // Initialize varibles
    ticks = 0;
    
    // Prepare the global enable line listener (pin change interrupt)
    PCMSK |= (1 << PORTB1);
    GIMSK |= (1 << PCIE);
    
    // Start the timer with a 1/8 prescaler, so the timer is clocked at 1 MHz.  Use CTC mode
    // so we can generate a reliable slowed microsecond counter
    OCR0A = 100; // Set the overflow interval to 100 us (1 tick = 100 us)
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01);
    TIMSK |= (1 << OCIE0A);
    
    // Start interrupts
    sei();
    
    // Loop forever
    while (1)
    {
        // Wait for the low side of the radio signal, when it goes high, reset the timer and the tick counter
        while(!(PINB & _BV(PORTB2)));
        TCNT0 = 0;
        ticks = 0;
        
        // Wait during the high side, when it drops, measure the ticks and thats the value of the pulse 
        while(PINB & _BV(PORTB2));

        // Check if the radio is providing a go signal
        if((ticks < THRESHOLD) || !(PINB & _BV(PORTB1)))
        {
            // Disable the controller
            PORTB &= ~(_BV(PORTB0) | _BV(PORTB4));

            // Stall for about 500 ms
            ticks = 0;
            while(ticks < 5000);
        } else 
        {
            // Bring the controllers online
            PORTB |= _BV(PORTB0) | _BV(PORTB4);
        } 
    }
    
    return 0;
}