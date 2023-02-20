#include <stdio.h>
#include <stdint.h>
#include <msp430.h> 


/**
 * hello.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	while(1){
	    if(0x20){
	        printf("hello");
	    }
	}

}
