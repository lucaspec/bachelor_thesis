#include <stdio.h>
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"
#include "stc3x_i2c.h"
#include <inttypes.h>
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>


//******************************************************************************
// Gauge Definitions and Variables *********************************************
//******************************************************************************

#define SLAVE_ADDR_MAX17260  0x36 // MAX17260
#define MAX_BUFFER_SIZE 20

/* MasterTypeX are example buffers initialized in the master, they will be
 * sent by the master to the slave.
 * SlaveTypeX are example buffers initialized in the slave, they will be
 * sent by the slave to the master.
 * */

// MasterTypes // switched up,  0x0080 => {0x80, 0x00}

uint8_t DesignCap [2] = {0xB0, 0x04}; // 60 mAh
uint8_t IchgTerm [2] = {0x80, 0x00}; // 2.0 mA
uint8_t VEmpty [2] = {0x00, 0x96}; // 3.0 V

uint8_t Write1 [2] = {0x90, 0x00};
uint8_t Write2 [2] = {0x00, 0x00};
uint8_t Write3 [2] = {0x00, 0x80}; // expressions needed for init
uint8_t Reset [2] = {0x0F, 0x00};

// SlaveTypes

uint8_t HibCFG [2] = {0};
uint8_t ModelCFG [2] = {0};
uint8_t FSTAT [2] = {0};
uint8_t Status [2] = {0};
uint8_t StatusPOR [2] = {0};
uint8_t RepSOC [2] = {0};
uint8_t RepCAP [2] = {0};
uint8_t Data [2] = {0};
uint8_t Test [2] = {0};

//******************************************************************************
// I2C FSM and Functions *******************************************************
//******************************************************************************

typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITCH_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;


/* Used to track the state of the software state machine*/
I2C_Mode MasterMode = IDLE_MODE;

uint8_t TransmitRegAddr = 0;
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count);


I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;

}


I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB0I2CSA = dev_addr;
    UCB0IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB0IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB0IE |= UCTXIE;                        // Enable TX interrupt

    UCB0CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;
}



//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

void initGPIO()
{
    // Configure GPIO

    // I2C pins
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);

    // turn off LED
    P2DIR |= 0x01;
    P2OUT = 0x00;

    // turn off power switch for sensor
    P3DIR |= 0x04;
    P3OUT = 0x00;


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



//******************************************************************************
// Gauge Functions *************************************************************
//******************************************************************************


void initializeConfig(void){
    FSTAT[0] = 0;
    while(FSTAT[0] & 0x01){
        _delay_cycles(160000); // 10ms Wait Loop. Do not continue until FSTAT.DNR==0
        I2C_Master_ReadReg(SLAVE_ADDR_MAX17260, 0x3D, 2);
        CopyArray(ReceiveBuffer, FSTAT, 2);
    }


    I2C_Master_ReadReg(SLAVE_ADDR_MAX17260, 0xDB, 2);
    CopyArray(ReceiveBuffer, HibCFG, 2); //Store original HibCFG value

    I2C_Master_WriteReg (SLAVE_ADDR_MAX17260, 0x60 , Write1, 2); // Exit Hibernate Mode step 1
    I2C_Master_WriteReg (SLAVE_ADDR_MAX17260, 0xBA , Write2, 2); // Exit Hibernate Mode step 2
    I2C_Master_WriteReg (SLAVE_ADDR_MAX17260, 0x60 , Write2, 2); // Exit Hibernate Mode step 3

    I2C_Master_WriteReg(SLAVE_ADDR_MAX17260, 0x18, DesignCap, 2); // Design Capacity

    I2C_Master_ReadReg(SLAVE_ADDR_MAX17260, 0x18, 2);
    CopyArray(ReceiveBuffer, Test, 2); //test if Design Capacity was written

    I2C_Master_WriteReg(SLAVE_ADDR_MAX17260, 0x1E, IchgTerm, 2); // Termination Current
    I2C_Master_WriteReg(SLAVE_ADDR_MAX17260, 0x3A, VEmpty, 2); // Empty Voltage

    I2C_Master_WriteReg(SLAVE_ADDR_MAX17260, 0xDB, Write3, 2); // Write ModelCFG, because ChargeVoltage < 4.275V

    // Poll ModelCFG.Refresh(highest bit), proceed to Step 3 when ModelCFG.Refresh==0.
    ModelCFG[1] = 0;
    while (ModelCFG[1] & 0x80){
        _delay_cycles(160000); // 10ms Wait Loop. Do not continue until ModelCFG.Refresh==0
        I2C_Master_ReadReg(SLAVE_ADDR_MAX17260, 0xDB, 2);
        CopyArray(ReceiveBuffer, ModelCFG, 2);
    }
    I2C_Master_WriteReg(SLAVE_ADDR_MAX17260, 0xBA , HibCFG, 2); // Restore Original HibCFG value


    I2C_Master_ReadReg(SLAVE_ADDR_MAX17260, 0x00, 2); //Read Status
    CopyArray(ReceiveBuffer, Status, 2);
    Status[1] &= 0xFF;
    Status[0] &= 0xFD;
    I2C_Master_WriteReg(SLAVE_ADDR_MAX17260, 0x00, Status, 2);
    //WriteAndVerifyRegister (0x00, Status AND 0xFFFD); //Write and Verify

}

// concatenates two uint8 to one uint16, needed for convert
uint16_t concatenate(uint8_t d1, uint8_t d2) {
    uint16_t wd = ((uint16_t)d1 << 8) | d2;
    return wd;
}

// converts 16 bits to integer value which then can be printed
uint16_t convertCAP(uint16_t data) {
    uint16_t res;
    res = 50*data;
    return res;
}

// converts 16 bits to integer value which then can be printed
uint16_t convertSOC(uint16_t data) {
    uint16_t res;
    res = data>>8;
    return res;
}



//******************************************************************************
// Main ************************************************************************
//******************************************************************************

uint16_t resultCAP;
uint16_t resultSOC;
const uint16_t thresholdCAP = 1000;
const uint16_t thresholdCAP_hysteresis = 900;


// Finite state machine
typedef enum ModeEnum{
    CHARGING,
    BOOTING
} Mode;

Mode MainMode = CHARGING; // initial state

int main(void){
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    initClockTo16MHz();
    initGPIO();
    initI2C();
    initializeConfig();

    // crystal setup
    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;          // MCLK=SMCLK=DCO; ACLK=XT1
    P2SEL0 |= BIT0 + BIT1;                             // P2.0: XOUT; P2.1: XI1
    do
    {
        CSCTL7 &= ~(XT1OFFG | DCOFFG);      // Clear XT1 and DCO fault flag
        SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1 & OFIFG);               // Test oscillator fault flag

    _delay_cycles(160000);  // 10ms delay



    while(1){

        // Gauge Measurement
        I2C_Master_ReadReg(SLAVE_ADDR_MAX17260, 0x00, 2);
        CopyArray(ReceiveBuffer, Status, 2);
        StatusPOR[0] = Status[0] & 0x02;

        if (StatusPOR[0] == 0x00){
            // Read Capacity in uAh
            I2C_Master_ReadReg(SLAVE_ADDR_MAX17260, 0x05, 2);
            CopyArray(ReceiveBuffer, RepCAP, 2);
            resultCAP = convertCAP(concatenate(RepCAP[1], RepCAP[0]));

            // Read State of Charge in %
            I2C_Master_ReadReg(SLAVE_ADDR_MAX17260, 0x06, 2);
            CopyArray(ReceiveBuffer, RepSOC, 2);
            resultSOC = convertSOC(concatenate(RepSOC[1], RepSOC[0]));
        }

        switch(MainMode){

            case CHARGING:

                if (resultCAP > thresholdCAP){ // check if threshold capacity has been reached
                    MainMode = BOOTING;
                    // turn on power switch to power application
                    P3OUT |= 0x04;
                    break;
                }

                // every ~16 seconds go out of LPM3 and gauge battery
                TA1CCTL0 |= CCIE;
                TA1CCR0 = 65535;
                TA1CTL |= TASSEL__ACLK | MC__UP | ID__8; // set clock to ACLK/8
                __bis_SR_register(LPM3_bits + GIE);  // Enter LPM3 w/ interrupts
                break;


            case BOOTING:

                if (resultCAP < thresholdCAP_hysteresis){ // check if threshold capacity has been reached
                    MainMode = CHARGING;
                    // turn off power switch
                    P3OUT &= ~0x04;
                    break;
                }

                // every second go out of LPM3 and gauge battery
                TA0CCTL0 |= CCIE;
                TA0CCR0 = 32678;
                TA0CTL |= TASSEL__ACLK | MC__UP;
                __bis_SR_register(LPM3_bits + GIE); // Enter LPM3 w/ interrupts
                break;
        }
    }
}

//******************************************************************************
// Interrupts ******************************************************************
//******************************************************************************

// I2C interrupt service routine for STC31 and gauge
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  //Must read from UCB0RXBUF
  uint8_t rx_val = 0;
  switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
  {
    case USCI_NONE:          break;         // Vector 0: No interrupts
    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG
    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
      break;
    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1
    case USCI_I2C_UCRXIFG0:                 // Vector 22: RXIFG0
        rx_val = UCB0RXBUF;
        if (RXByteCtr)
        {
          ReceiveBuffer[ReceiveIndex++] = rx_val;
          RXByteCtr--;
        }

        if (RXByteCtr == 1)
        {
          UCB0CTLW0 |= UCTXSTP;
        }
        else if (RXByteCtr == 0)
        {
          UCB0IE &= ~UCRXIE;
          MasterMode = IDLE_MODE;
          __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
        }
        break;
    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
        switch (MasterMode)
        {
          case TX_REG_ADDRESS_MODE:
              UCB0TXBUF = TransmitRegAddr;
              if (RXByteCtr)
                  MasterMode = SWITCH_TO_RX_MODE;   // Need to start receiving now
              else
                  MasterMode = TX_DATA_MODE;        // Continue to transmision with the data in Transmit Buffer
              break;

          case SWITCH_TO_RX_MODE:
              UCB0IE |= UCRXIE;              // Enable RX interrupt
              UCB0IE &= ~UCTXIE;             // Disable TX interrupt
              UCB0CTLW0 &= ~UCTR;            // Switch to receiver
              MasterMode = RX_DATA_MODE;    // State state is to receive data
              UCB0CTLW0 |= UCTXSTT;          // Send repeated start
              if (RXByteCtr == 1)
              {
                  //Must send stop since this is the N-1 byte
                  while((UCB0CTLW0 & UCTXSTT));
                  UCB0CTLW0 |= UCTXSTP;      // Send stop condition
              }
              break;

          case TX_DATA_MODE:
              if (TXByteCtr)
              {
                  UCB0TXBUF = TransmitBuffer[TransmitIndex++];
                  TXByteCtr--;
              }
              else
              {
                  //Done with transmission
                  UCB0CTLW0 |= UCTXSTP;     // Send stop condition
                  MasterMode = IDLE_MODE;
                  UCB0IE &= ~UCTXIE;                       // disable TX interrupt
                  __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
              }
              break;

          default:
              __no_operation();
              break;
        }
        break;
    default: break;
  }
}


// Timer0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0 (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0 (void)
#else
#error Compiler not supported!
#endif
{
    //TA0CCR0 += 32678;                             // Add Offset to TACCR0
    __bic_SR_register_on_exit(LPM3_bits);         // Exit LPM
}


// Timer1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1 (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer1 (void)
#else
#error Compiler not supported!
#endif
{
    //P2OUT ^= BIT0;
    //TA1CCR0 += 32678;                             // Add Offset to TACCR0
    __bic_SR_register_on_exit(LPM3_bits);         // Exit LPM
}
