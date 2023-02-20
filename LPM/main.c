#include <msp430.h>

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    P2SEL0 |= BIT0 + BIT1;                  // P2.0: XOUT; P2.1: XI1

    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;  // MCLK=SMCLK=DCO; ACLK=XT1

    // Port Configuration all un-used pins to output low
    P1OUT = 0x00;
    P2OUT = 0x00;
    P3OUT = 0x00;
    P1DIR = 0x00;
    P2DIR = 0x00;
    P3DIR = 0x00;

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    do
    {
        CSCTL7 &= ~(XT1OFFG | DCOFFG);      // Clear XT1 and DCO fault flag
        SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1 & OFIFG);               // Test oscillator fault flag

    __bis_SR_register(LPM3_bits);           // Enter LPM3
    __no_operation();                       // For debug
}

// Watchdog Timer interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) WDT_ISR (void)
#else
#error Compiler not supported!
#endif
{
    P1OUT ^= BIT0;                          // Toggle P1.0 (LED) every 1s
}

