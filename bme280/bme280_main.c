//******************************************************************************
//   MSP430FR243x Demo - eUSCI_B0, I2C Master multiple byte TX/RX
//
//   Description: I2C master communicates to I2C slave sending and receiving
//   3 different messages of different length. I2C master will enter LPM0 mode
//   while waiting for the messages to be sent/receiving using I2C interrupt.
//   ACLK = NA, MCLK = SMCLK = DCO 16MHz.
//
//                                     /|\ /|\
//                   MSP430FR2633      4.7k |
//                 -----------------    |  4.7k
//            /|\ |             P1.3|---+---|-- I2C Clock (UCB0SCL)
//             |  |                 |       |
//             ---|RST          P1.2|-------+-- I2C Data (UCB0SDA)
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//
//   Nima Eskandari and Ryan Meredith
//   Texas Instruments Inc.
//   January 2018
//   Built with CCS V7.3
//******************************************************************************

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>


//******************************************************************************
// Pin Config ******************************************************************
//******************************************************************************

#define LED_OUT     P1OUT
#define LED_DIR     P1DIR
#define LED0_PIN    BIT0
#define LED1_PIN    BIT1

//******************************************************************************
// Example Commands ************************************************************
//******************************************************************************

#define SLAVE_ADDR  0x77 // bme280

/* CMD_TYPE_X_SLAVE are example commands the master sends to the slave.
 * The slave will send example SlaveTypeX buffers in response.
 *
 * CMD_TYPE_X_MASTER are example commands the master sends to the slave.
 * The slave will initialize itself to receive MasterTypeX example buffers.
 * */

#define CMD_TYPE_0_SLAVE      0
#define CMD_TYPE_1_SLAVE      1
#define CMD_TYPE_2_SLAVE      2

#define CMD_TYPE_0_MASTER      3
#define CMD_TYPE_1_MASTER      4
#define CMD_TYPE_2_MASTER      5

#define TYPE_0_LENGTH   1
#define TYPE_1_LENGTH   2
#define TYPE_2_LENGTH   6

#define MAX_BUFFER_SIZE     2

/* MasterTypeX are example buffers initialized in the master, they will be
 * sent by the master to the slave.
 * SlaveTypeX are example buffers initialized in the slave, they will be
 * sent by the slave to the master.
 * */

uint8_t MasterType2 [TYPE_2_LENGTH] = {'F', '4', '1', '9', '2', 'B'};
uint8_t MasterType1 [TYPE_1_LENGTH] = { 8, 9};
uint8_t MasterType0 [TYPE_0_LENGTH] = { 11};


uint8_t SlaveType2 [TYPE_2_LENGTH] = {0};
uint8_t SlaveType1 [TYPE_1_LENGTH] = {0};
uint8_t SlaveType0 [TYPE_0_LENGTH] = {0};





//******************************************************************************
// General I2C State Machine ***************************************************
//******************************************************************************

typedef enum I2C_ModeEnum{
    IDLE_MODE,
    NACK_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    SWITCH_TO_RX_MODE,
    SWITHC_TO_TX_MODE,
    TIMEOUT_MODE
} I2C_Mode;


/* Used to track the state of the software state machine*/
I2C_Mode MasterMode = IDLE_MODE;

/* The Register Address/Command to use*/
uint8_t TransmitRegAddr = 0;

/* ReceiveBuffer: Buffer used to receive data in the ISR
 * RXByteCtr: Number of bytes left to receive
 * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer: Buffer used to transmit data in the ISR
 * TXByteCtr: Number of bytes left to transfer
 * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
 * */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;



/* I2C Write and Read Functions */

/* For slave device with dev_addr, writes the data specified in *reg_data
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 *           Example: CMD_TYPE_0_MASTER
 * *reg_data: The buffer to write
 *           Example: MasterType0
 * count: The length of *reg_data
 *           Example: TYPE_0_LENGTH
 *  */
I2C_Mode I2C_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

/* For slave device with dev_addr, read the data specified in slaves reg_addr.
 * The received data is available in ReceiveBuffer
 *
 * dev_addr: The slave device address.
 *           Example: SLAVE_ADDR
 * reg_addr: The register or command to send to the slave.
 *           Example: CMD_TYPE_0_SLAVE
 * count: The length of data to read
 *           Example: TYPE_0_LENGTH
 *  */
I2C_Mode I2C_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count);
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);


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

    //CopyArray(ReceiveBuffer, i2c_read_reg_2B, 2);
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

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}


//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************


void initGPIO()
{
    // Configure GPIO
    //LED_OUT &= ~(LED0_PIN | LED1_PIN); // P1 setup for LED & reset output
    //LED_DIR |= (LED0_PIN | LED1_PIN);

    // I2C pins
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);

    // UART pins
    //P1SEL1 &= ~(BIT4 | BIT5);                 // USCI_A0 UART operation
    //P1SEL0 |= BIT4 | BIT5;

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
    //while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));         // FLL locked
}

void initI2C()
{
    UCB0CTLW0 = UCSWRST;                      // Enable SW reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK | UCSYNC; // I2C master mode, SMCLK
    UCB0CTLW1 |= UCASTP_2;                  // Automatic stop generated
                                                // after UCB0TBCNT is reached
    UCB0BRW = 160;                            // fSCL = SMCLK/160 = ~100kHz
    UCB0I2CSA = SLAVE_ADDR;                   // Slave Address
    UCB0CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB0IE |= UCRXIE | UCNACKIE;
}



//******************************************************************************
// Main ************************************************************************
// TMP006 temperature sensor
// read out sensor ID and temperature value
//******************************************************************************



// concatenates MSB, LSB, XLSB
uint32_t concatenate(uint8_t d1, uint8_t d2, uint8_t d3) {
    uint32_t lw = ((uint32_t)d1 << 8) | d2;
    uint32_t lwd = (lw << 8) | d3;
    return lwd>>4;
}

// converts temperature
int32_t t_fine;
int32_t convert_temp(uint16_t dig_T1, int16_t dig_T2, int16_t dig_T3, int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((int32_t)dig_T1))) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}


// convert pressure
int32_t convert_press(uint16_t dig_P1, int16_t dig_P2, int16_t dig_P3, int16_t dig_P4, int16_t dig_P5, int16_t dig_P6, int16_t dig_P7, int16_t dig_P8, int16_t dig_P9, int32_t adc_P)
{
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
    var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
    if (var1 == 0) {
        return 0;
    }
    p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t)var1);
    }
    else {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
    return p;
}



uint32_t temp_result, press_result;
uint32_t adc_T, adc_P;
uint8_t temp_MSB[1];
uint8_t temp_LSB[1];
uint8_t temp_XLSB[1];
uint8_t press_MSB[1];
uint8_t press_LSB[1];
uint8_t press_XLSB[1];
uint8_t id;

uint8_t reset [1] = {0xB6};
uint8_t ctrl_meas [1] = {0x47}; // 0b00100111, x1 oversampling, normal mode
uint8_t ctrl_hum [1] = {0x00}; // skip measurement

// compensation values
uint8_t dig_T1_1, dig_T1_2, dig_T2_1, dig_T2_2, dig_T3_1, dig_T3_2;
uint8_t dig_P1_1, dig_P1_2, dig_P2_1, dig_P2_2, dig_P3_1, dig_P3_2, dig_P4_1, dig_P4_2, dig_P5_1, dig_P5_2, dig_P6_1, dig_P6_2, dig_P7_1, dig_P7_2, dig_P8_1, dig_P8_2, dig_P9_1, dig_P9_2;


 int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    initClockTo16MHz();
    initGPIO();
    initI2C();

    // turn on LED and power switch
    P2DIR |= 0x01;
    P2OUT = 0x01;

    P3DIR |= 0x01;
    P3OUT = 0x01;

    _delay_cycles(1600000);  // 100ms delay



    // reset and set mode
    I2C_Master_WriteReg(SLAVE_ADDR, 0xE0, reset, 1);
    I2C_Master_WriteReg(SLAVE_ADDR, 0xF2, ctrl_hum, 1);
    I2C_Master_WriteReg(SLAVE_ADDR, 0xF4, ctrl_meas, 1);



    // read compensation parameters
    // T1, T2, T3
    I2C_Master_ReadReg(SLAVE_ADDR, 0x88, 1);
    CopyArray(ReceiveBuffer, &dig_T1_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x89, 1);
    CopyArray(ReceiveBuffer, &dig_T1_2, 1);
    uint16_t dig_T1 = ((uint16_t)dig_T1_2 << 8) | ((uint16_t)dig_T1_1);

    I2C_Master_ReadReg(SLAVE_ADDR, 0x8A, 1);
    CopyArray(ReceiveBuffer, &dig_T2_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x8B, 1);
    CopyArray(ReceiveBuffer, &dig_T2_2, 1);
    int16_t dig_T2 = ((int16_t)dig_T2_2 << 8) | ((int16_t)dig_T2_1);

    I2C_Master_ReadReg(SLAVE_ADDR, 0x8C, 1);
    CopyArray(ReceiveBuffer, &dig_T3_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x8D, 1);
    CopyArray(ReceiveBuffer, &dig_T3_2, 1);
    int16_t dig_T3 = ((int16_t)dig_T3_2 << 8) | (int16_t)dig_T3_1;

    // P1...P9
    I2C_Master_ReadReg(SLAVE_ADDR, 0x8E, 1);
    CopyArray(ReceiveBuffer, &dig_P1_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x8F, 1);
    CopyArray(ReceiveBuffer, &dig_P1_2, 1);
    uint16_t dig_P1 = ((uint16_t)dig_P1_2 << 8) | ((uint16_t)dig_P1_1);

    I2C_Master_ReadReg(SLAVE_ADDR, 0x90, 1);
    CopyArray(ReceiveBuffer, &dig_P2_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x91, 1);
    CopyArray(ReceiveBuffer, &dig_P2_2, 1);
    int16_t dig_P2 = ((int16_t)dig_P2_2 << 8) | ((int16_t)dig_P2_1);

    I2C_Master_ReadReg(SLAVE_ADDR, 0x92, 1);
    CopyArray(ReceiveBuffer, &dig_P3_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x93, 1);
    CopyArray(ReceiveBuffer, &dig_P3_2, 1);
    int16_t dig_P3 = ((int16_t)dig_P3_2 << 8) | (int16_t)dig_P3_1;

    I2C_Master_ReadReg(SLAVE_ADDR, 0x94, 1);
    CopyArray(ReceiveBuffer, &dig_P4_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x95, 1);
    CopyArray(ReceiveBuffer, &dig_P4_2, 1);
    int16_t dig_P4 = ((int16_t)dig_P4_2 << 8) | ((int16_t)dig_P4_1);

    I2C_Master_ReadReg(SLAVE_ADDR, 0x96, 1);
    CopyArray(ReceiveBuffer, &dig_P5_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x97, 1);
    CopyArray(ReceiveBuffer, &dig_P5_2, 1);
    int16_t dig_P5 = ((int16_t)dig_P5_2 << 8) | ((int16_t)dig_P5_1);

    I2C_Master_ReadReg(SLAVE_ADDR, 0x98, 1);
    CopyArray(ReceiveBuffer, &dig_P6_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x99, 1);
    CopyArray(ReceiveBuffer, &dig_P6_2, 1);
    int16_t dig_P6 = ((int16_t)dig_P6_2 << 8) | (int16_t)dig_P6_1;

    I2C_Master_ReadReg(SLAVE_ADDR, 0x9A, 1);
    CopyArray(ReceiveBuffer, &dig_P7_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x9B, 1);
    CopyArray(ReceiveBuffer, &dig_P7_2, 1);
    int16_t dig_P7 = ((int16_t)dig_P7_2 << 8) | ((int16_t)dig_P7_1);

    I2C_Master_ReadReg(SLAVE_ADDR, 0x9C, 1);
    CopyArray(ReceiveBuffer, &dig_P8_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x9D, 1);
    CopyArray(ReceiveBuffer, &dig_P8_2, 1);
    int16_t dig_P8 = ((int16_t)dig_P8_2 << 8) | ((int16_t)dig_P8_1);

    I2C_Master_ReadReg(SLAVE_ADDR, 0x9E, 1);
    CopyArray(ReceiveBuffer, &dig_P9_1, 1);
    I2C_Master_ReadReg(SLAVE_ADDR, 0x9F, 1);
    CopyArray(ReceiveBuffer, &dig_P9_2, 1);
    int16_t dig_P9 = ((int16_t)dig_P9_2 << 8) | (int16_t)dig_P9_1;



    while(1){

        // read id
        I2C_Master_ReadReg(SLAVE_ADDR, 0xD0, 1);
        CopyArray(ReceiveBuffer, &id, 1);

        P2OUT = 0x01;
        _delay_cycles(8000000);  // 500ms delay

        // read temperature
        I2C_Master_ReadReg(SLAVE_ADDR, 0xFA, 1);
        CopyArray(ReceiveBuffer, temp_MSB, 1);

        I2C_Master_ReadReg(SLAVE_ADDR, 0xFB, 1);
        CopyArray(ReceiveBuffer, temp_LSB, 1);

        I2C_Master_ReadReg(SLAVE_ADDR, 0xFC, 1);
        CopyArray(ReceiveBuffer, temp_XLSB, 1);

        // read pressure
        I2C_Master_ReadReg(SLAVE_ADDR, 0xF7, 1);
        CopyArray(ReceiveBuffer, press_MSB, 1);

        I2C_Master_ReadReg(SLAVE_ADDR, 0xF8, 1);
        CopyArray(ReceiveBuffer, press_LSB, 1);

        I2C_Master_ReadReg(SLAVE_ADDR, 0xF9, 1);
        CopyArray(ReceiveBuffer, press_XLSB, 1);


        adc_T = concatenate(*temp_MSB, *temp_LSB, *temp_XLSB);
        temp_result = convert_temp(dig_T1, dig_T2, dig_T3, adc_T);


        adc_P = concatenate(*press_MSB, *press_LSB, *press_XLSB);
        press_result = convert_press(dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, adc_P);

        P2OUT = 0x00;
        _delay_cycles(8000000);  // 500ms delay

        _no_operation(); // set breakpoint here
    }

    __bis_SR_register(LPM0_bits + GIE);
    return 0;
}


//******************************************************************************
// I2C Interrupt ***************************************************************
//******************************************************************************

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
