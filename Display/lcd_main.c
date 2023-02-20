#include "ls013b7dh03.h"
#include <msp430.h>
//#include "oled_font.h"

void lcd_main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    __bis_SR_register(SCG0);    // Disable FLL
    CSCTL3 |= SELREF__REFOCLK;  // Set REFO as FLL reference source
    CSCTL0 = 0;                 // Clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);     // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;        // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;      // set to fDCOCLKDIV = (FLLN + 1)*(fFLLREFCLK/n)
                                //                   = (487  + 1)*(32.768 kHz/1)
                                //                   = 16MHz

    __delay_cycles(3);
    __bic_SR_register(SCG0);                    // Enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));  // FLL locked

    CSCTL5 |= DIVM_0 | DIVS_1;                  // MCLK = DCOCLK = 16MHz,
                                                // SMCLK = MCLK/2 = 8MHz

    UCA1CTLW0 = UCSWRST;                    // **Put eUSCI module in reset**
    UCA1CTLW0 |= UCCKPL | UCMSB | UCSYNC |
                 UCMST | UCSSEL__SMCLK;
    UCA1BRW = 80;                           // BRCLK / UCBRx = UCxCLK
                                            // 8MHz  / 80    = 100kHz
    UCA1CTLW0 &= ~UCSWRST;                  // **Initialize eUSCI module**
    UCA1IE |= UCRXIE;                       // Enable eUSCI0 RX interrupt

    PM5CTL0 &= ~LOCKLPM5;


    // lcd code
	lcd_init();	
	lcd_clear();
			
	lcd_print_char(16,16,'a',16,1);
	
	lcd_print_num(32,32,5,1,16);
	
	lcd_print_string(0,64,"hello,world",16);
		
	lcd_refresh();
}
