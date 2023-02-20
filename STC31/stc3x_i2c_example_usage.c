/*
 * Copyright (c) 2021, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>  // printf
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "stc3x_i2c.h"
#include <inttypes.h>
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * TO USE CONSOLE OUTPUT (PRINTF) IF NOT PRESENT ON YOUR PLATFORM
 */
//#define printf(...)

void initGPIO()
{
    // Configure GPIO

    // I2C pins
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);


    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    // Clock System Setup
    __bis_SR_register(SCG0);                           // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                         // Set REFO as FLL reference source
    CSCTL0 = 0;                                        // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                            // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;                               // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;                             // DCOCLKDIV = 16MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                           // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));         // FLL locked
}

void initI2C()
{
    UCB0CTLW0 = UCSWRST;                      // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB0CTLW1 |= UCASTP_2;                    // Automatic stop generated
                                              // after UCB0TBCNT is reached
    UCB0BRW = 160;                            // fSCL = SMCLK/160 = ~100kHz
    UCB0I2CSA = 0x29;                         // Slave Address
    UCB0CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB0IE |= UCRXIE | UCNACKIE;
}


// ************************ MAIN ****************************


int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    initClockTo16MHz();
    initGPIO();
    initI2C();

    // turn off LED and turn on power switch
    P2DIR |= 0x01;
    P2OUT = 0x00;

    P3DIR |= 0x04;
    P3OUT = 0x04;

    _delay_cycles(1600000);  // 100ms delay

    uint16_t gas_ticks;
    uint16_t temperature_ticks;

    double gas; // float
    double temperature;

    int16_t error = 0;

    while(1){
        error = stc3x_set_binary_gas(0x0003);
        if (error) {
            P2OUT = 0x01;
            //printf("Error executing stc3x_set_binary_gas(): %i\n", error);
        } else {
            //printf("Set binary gas to 0x0001\n");
        }

        uint16_t relative_humidity_ticks = 32767; // 50% rel. humidity
        error = stc3x_set_relative_humidity(relative_humidity_ticks);
        if (error) {
            P2OUT = 0x01;
        }


        uint16_t absolute_pressure = 980; // 980 mbar
        error = stc3x_set_pressure(absolute_pressure);
        if (error) {
            P2OUT = 0x01;
        }


        // Read Measurement
        error = stc3x_measure_gas_concentration(&gas_ticks, &temperature_ticks);
        if (error) {
            P2OUT = 0x01;
            //printf("Error executing stc3x_measure_gas_concentration(): %i\n", error);
        } else {
            gas = 100 * ((double)gas_ticks - 16384.0) / 32768.0;
            temperature = (double)temperature_ticks / 200.0;
            //printf("Gas: %d - Temperature: %d\n", gas, temperature);
        }

        _delay_cycles(1600000);  // 100ms delay

        // go into LPM until next measurement
        //stc3x_enter_sleep_mode(); // send sensor to sleep
        TA0CCTL0 |= CCIE;
        TA0CCR0 = 32678; // 8s delay
        TA0CTL |= TASSEL__ACLK | MC__UP | ID__8;
        __bis_SR_register(LPM3_bits + GIE);             // Enter LPM0 w/ interrupts
        //P2OUT ^= BIT0;
    }

    return 0;
}


// Timer A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
    //P2OUT ^= BIT0;                                // Toggle LED
    //TA0CCR0 += 32678;                             // Add Offset to TACCR0
    __bic_SR_register_on_exit(LPM3_bits);         // Exit LPM
}
