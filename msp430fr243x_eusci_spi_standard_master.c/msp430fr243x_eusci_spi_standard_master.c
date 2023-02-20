//******************************************************************************
//   MSP430FR243x Demo - eUSCI_A1, 4-Wire SPI Master multiple-byte RX/TX
//
//   Description: SPI master communicates with SPI slave receiving and sending
//   3 different messages of different length. SPI master will initially wait
//   for a port interrupt from button S1 in LPM0 mode before starting the SPI
//   communication. SPI master will enter LPM0 mode while waiting for the
//   messages to be received/sent using SPI interrupt.
//
//   ACLK = NA, MCLK = DCO = 16MHz, SMCLK = MCLK / 2 = 8MHz
//
//
//                    MSP430FR2433
//                 -----------------
//            /|\ |             P1.1|-> Comms LED2
//             |  |                 |
//             ---|RST          P2.2|-> Slave Reset (GPIO)
//                |                 |
//                |             P2.6|-> Data Out (UCA1SIMO)
//                |                 |
//                |             P2.5|<- Data In (UCA1SOMI)
//    Button S1 ->|P2.3             |
//                |             P2.4|-> Serial Clock Out (UCA1CLK)
//  Button LED1 <-|P1.0             |
//                |             P2.1|-> Slave Chip Select (GPIO)
//
//   James Evans
//   Texas Instruments Inc.
//   May 2021
//   Built with CCS v10.3
//******************************************************************************

#include <msp430.h>
#include <stdint.h>

//******************************************************************************
// Pin Config ******************************************************************
//******************************************************************************

#define SLAVE_CS_OUT            P2OUT
#define SLAVE_CS_DIR            P2DIR
#define SLAVE_CS_PIN            BIT1

#define SLAVE_RST_OUT           P2OUT
#define SLAVE_RST_DIR           P2DIR
#define SLAVE_RST_PIN           BIT2

#define BUTTON_DIR              P2DIR
#define BUTTON_OUT              P2OUT
#define BUTTON_REN              P2REN
#define BUTTON_PIN              BIT3
#define BUTTON_IES              P2IES
#define BUTTON_IE               P2IE
#define BUTTON_IFG              P2IFG
#define BUTTON_VECTOR           PORT2_VECTOR

#define BUTTON_LED_OUT          P1OUT
#define BUTTON_LED_DIR          P1DIR
#define BUTTON_LED_PIN          BIT0

#define COMMS_LED_OUT           P1OUT
#define COMMS_LED_DIR           P1DIR
#define COMMS_LED_PIN           BIT1


//******************************************************************************
// Example Commands ************************************************************
//******************************************************************************

#define DUMMY                   0xFF

/* CMD_TYPE_X_SLAVE are example commands the master sends to the slave.
 * The slave will send example SlaveTypeX buffers in response.
 *
 * CMD_TYPE_X_MASTER are example commands the master sends to the slave.
 * The slave will initialize itself to receive MasterTypeX example buffers.
 * */

#define CMD_TYPE_0_SLAVE        0
#define CMD_TYPE_1_SLAVE        1
#define CMD_TYPE_2_SLAVE        2

#define CMD_TYPE_0_MASTER       3
#define CMD_TYPE_1_MASTER       4
#define CMD_TYPE_2_MASTER       5

#define TYPE_0_LENGTH           1
#define TYPE_1_LENGTH           2
#define TYPE_2_LENGTH           6

#define MAX_BUFFER_SIZE         20

/* MasterTypeX are example buffers initialized in the master, they will be
 * sent by the master to the slave.
 * SlaveTypeX are example buffers initialized in the slave, they will be
 * sent by the slave to the master.
 * */

uint8_t MasterType0 [TYPE_0_LENGTH] = {0x11};
uint8_t MasterType1 [TYPE_1_LENGTH] = {8, 9};
uint8_t MasterType2 [TYPE_2_LENGTH] = {'F', '4', '1', '9', '2', 'B'};

uint8_t SlaveType2 [TYPE_2_LENGTH]  = {0};
uint8_t SlaveType1 [TYPE_1_LENGTH]  = {0};
uint8_t SlaveType0 [TYPE_0_LENGTH]  = {0};


//******************************************************************************
// General SPI State Machine ***************************************************
//******************************************************************************

typedef enum SPI_ModeEnum
{
    IDLE_MODE,
    TX_REG_ADDRESS_MODE,
    RX_REG_ADDRESS_MODE,
    TX_DATA_MODE,
    RX_DATA_MODE,
    TIMEOUT_MODE
} SPI_Mode;

/* Used to track the state of the software state machine*/
SPI_Mode MasterMode = IDLE_MODE;

/* The Register Address/Command to use*/
uint8_t TransmitRegAddr = 0;

/* ReceiveBuffer: Buffer used to receive data in the ISR
 * RXByteCtr: Number of bytes left to receive
 * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer: Buffer used to transmit data in the ISR
 * TXByteCtr: Number of bytes left to transfer
 * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
 * */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE]  = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

/* SPI Write and Read Functions */

/* For slave device, writes the data specified in *reg_data
 *
 * reg_addr: The register or command to send to the slave.
 *           Example: CMD_TYPE_0_MASTER
 * *reg_data: The buffer to write
 *           Example: MasterType0
 * count: The length of *reg_data
 *           Example: TYPE_0_LENGTH
 *  */
SPI_Mode SPI_Master_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

/* For slave device, read the data specified in slaves reg_addr.
 * The received data is available in ReceiveBuffer
 *
 * reg_addr: The register or command to send to the slave.
 *           Example: CMD_TYPE_0_SLAVE
 * count: The length of data to read
 *           Example: TYPE_0_LENGTH
 *  */
SPI_Mode SPI_Master_ReadReg(uint8_t reg_addr, uint8_t count);
void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count);
void SendUCA1Data(uint8_t val);

void SendUCA1Data(uint8_t val)
{
    while (!(UCA1IFG & UCTXIFG));           // USCI_A1 TX buffer ready?
    UCA1TXBUF = val;
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

SPI_Mode SPI_Master_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    // Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);
    SendUCA1Data(TransmitRegAddr);

    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/interrupts enabled

    SLAVE_CS_OUT |= SLAVE_CS_PIN;
    return MasterMode;
}

SPI_Mode SPI_Master_ReadReg(uint8_t reg_addr, uint8_t count)
{
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);
    SendUCA1Data(TransmitRegAddr);

    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/interrupts enabled

    SLAVE_CS_OUT |= SLAVE_CS_PIN;
    return MasterMode;
}


//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

void initSPI()
{
    // Clock polarity select: inactive state is high, MSB first
    // 8-bit data, Synchronous, Master mode, 3-pin SPI, BRCLK source: SMCLK
    UCA1CTLW0 = UCSWRST;                    // **Put eUSCI module in reset**
    UCA1CTLW0 |= UCCKPL | UCMSB | UCSYNC |
                 UCMST | UCSSEL__SMCLK;
    UCA1BRW = 80;                           // BRCLK / UCBRx = UCxCLK
                                            // 8MHz  / 80    = 100kHz
    UCA1CTLW0 &= ~UCSWRST;                  // **Initialize eUSCI module**
    UCA1IE |= UCRXIE;                       // Enable eUSCI0 RX interrupt
}

void initGPIO()
{
    // Configure LEDs and buttons
    COMMS_LED_DIR |= COMMS_LED_PIN;
    COMMS_LED_OUT &= ~COMMS_LED_PIN;

    BUTTON_LED_DIR |= BUTTON_LED_PIN;
    BUTTON_LED_OUT &= ~BUTTON_LED_PIN;

    // Configure eUSCI pins
    P2SEL0 |= BIT4 | BIT5 | BIT6;
    P2SEL1 &= ~(BIT4 | BIT5 | BIT6);

    SLAVE_RST_DIR |= SLAVE_RST_PIN;
    SLAVE_RST_OUT |= SLAVE_RST_PIN;

    SLAVE_CS_DIR |= SLAVE_CS_PIN;
    SLAVE_CS_OUT |= SLAVE_CS_PIN;

    // Button to initiate transfer
    BUTTON_DIR &= ~BUTTON_PIN;              // Button input
    BUTTON_OUT |= BUTTON_PIN;               // Button pull up
    BUTTON_REN |= BUTTON_PIN;               // Button pull up/down resistor enable
    BUTTON_IES |= BUTTON_PIN;               // Button high-to-low edge

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Must be done after clearing LOCKLPM5
    BUTTON_IFG &= ~BUTTON_PIN;              // Button IFG cleared
    BUTTON_IE  |= BUTTON_PIN;               // Button interrupt enabled
}

void initClockTo16MHz()
{
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
}


//******************************************************************************
// Main ************************************************************************
// Send and receive three messages containing the example commands *************
//******************************************************************************

int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Initialize device
    initClockTo16MHz();
    initGPIO();
    initSPI();

    while(1)
    {
        // Reset slave after initializing eUSCI pins
        COMMS_LED_OUT &= ~COMMS_LED_PIN;        // Turn off comms LED
        SLAVE_RST_OUT &= ~SLAVE_RST_PIN;        // Reset slave
        __delay_cycles(100000);                 // Wait
        SLAVE_RST_OUT |= SLAVE_RST_PIN;         // Release slave
        __delay_cycles(100000);                 // Wait for slave to initialize
        COMMS_LED_OUT |= COMMS_LED_PIN;         // Turn on comms LED

        // Enter LPM0 w/interrupts enabled
        __bis_SR_register(LPM0_bits + GIE);

        // CPU wakes up on button press, starts communication
        SPI_Master_ReadReg(CMD_TYPE_2_SLAVE, TYPE_2_LENGTH);
        CopyArray(ReceiveBuffer, SlaveType2, TYPE_2_LENGTH);

        SPI_Master_ReadReg(CMD_TYPE_1_SLAVE, TYPE_1_LENGTH);
        CopyArray(ReceiveBuffer, SlaveType1, TYPE_1_LENGTH);

        SPI_Master_ReadReg(CMD_TYPE_0_SLAVE, TYPE_0_LENGTH);
        CopyArray(ReceiveBuffer, SlaveType0, TYPE_0_LENGTH);

        SPI_Master_WriteReg(CMD_TYPE_2_MASTER, MasterType2, TYPE_2_LENGTH);
        SPI_Master_WriteReg(CMD_TYPE_1_MASTER, MasterType1, TYPE_1_LENGTH);
        SPI_Master_WriteReg(CMD_TYPE_0_MASTER, MasterType0, TYPE_0_LENGTH);

        // Re-enable button to repeat transfer
        BUTTON_IE  |= BUTTON_PIN;               // Button interrupt enabled
        BUTTON_LED_OUT &= ~BUTTON_LED_PIN;      // Turn off button LED
    }
}


//******************************************************************************
// SPI Interrupt ***************************************************************
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    uint8_t uca1_rx_val = 0;
    switch(__even_in_range(UCA1IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        case USCI_SPI_UCRXIFG:
            uca1_rx_val = UCA1RXBUF;
            UCA1IFG &= ~UCRXIFG;
            switch (MasterMode)
            {
                case TX_REG_ADDRESS_MODE:
                    if (RXByteCtr)
                    {
                        // Need to start receiving now
                        MasterMode = RX_DATA_MODE;

                        // Send dummy byte to start
                        __delay_cycles(5000);
                        SendUCA1Data(DUMMY);
                    }
                    else
                    {
                        // Continue to transmission
                        MasterMode = TX_DATA_MODE;

                        // Send first byte
                        SendUCA1Data(TransmitBuffer[TransmitIndex++]);
                        TXByteCtr--;
                    }
                    break;

                case TX_DATA_MODE:
                    if (TXByteCtr)
                    {
                        // Send additional byte
                        SendUCA1Data(TransmitBuffer[TransmitIndex++]);
                        TXByteCtr--;
                    }
                    else
                    {
                        // Done with transmission
                        MasterMode = IDLE_MODE;
                        __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
                    }
                    break;

                case RX_DATA_MODE:
                    if (RXByteCtr)
                    {
                        // Store received byte
                        ReceiveBuffer[ReceiveIndex++] = uca1_rx_val;
                        RXByteCtr--;
                    }
                    if (RXByteCtr == 0)
                    {
                        // Done with reception
                        MasterMode = IDLE_MODE;
                        __bic_SR_register_on_exit(CPUOFF);  // Exit LPM0
                    }
                    else
                    {
                        // Send dummy byte to receive next byte
                        SendUCA1Data(DUMMY);
                    }
                    break;

                default:
                    __no_operation();
                    break;
            }
            __delay_cycles(1000);
            break;
        case USCI_SPI_UCTXIFG:
            break;
        default: break;
    }
}


//******************************************************************************
// PORT Interrupt **************************************************************
// Interrupt occurs on button press and initiates the SPI data transfer ********
//******************************************************************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=BUTTON_VECTOR
__interrupt void Button_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(BUTTON_VECTOR))) Button_ISR (void)
#else
#error Compiler not supported!
#endif
{
    // Check if button was pressed
    if (BUTTON_IFG & BUTTON_PIN)
    {
        // Disable button
        BUTTON_LED_OUT |= BUTTON_LED_PIN;       // Button LED on
        BUTTON_IFG &= ~BUTTON_PIN;              // Button IFG cleared
        BUTTON_IE &= ~BUTTON_PIN;               // Button IFG disabled
        __delay_cycles(1000);

        // Wake up CPU to start communication
        __bic_SR_register_on_exit(LPM0_bits);   // Exit LPM0
    }
}