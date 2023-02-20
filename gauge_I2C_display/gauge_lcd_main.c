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
#include <stdio.h>
#include <stdbool.h>
#include "oled_font.h"
// #include <math.h>

//******************************************************************************
// Pin Config ******************************************************************
//******************************************************************************

#define SLAVE_CS_OUT            P2OUT // rerouted via digital pin
#define SLAVE_CS_DIR            P2DIR
#define SLAVE_CS_PIN            BIT7

//******************************************************************************
// Definitions and Variables ***************************************************
//******************************************************************************

#define SLAVE_ADDR  0x36 // max17260, 0x36
#define MAX_BUFFER_SIZE 128

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
uint8_t EnHib [2] = {0x07, 0x80};

// SlaveTypes

uint8_t HibCFG [2] = {0, 0};
uint8_t ModelCFG [2] = {0, 0};
uint8_t FSTAT [2] = {0, 0};
uint8_t Status [2] = {0, 0};
uint8_t StatusHib [2] = {0, 0};
uint8_t StatusPOR [2] = {0, 0};
uint8_t RepSOC [2] = {0, 0};
uint8_t RepCAP [2] = {0, 0};
uint8_t Vcell [2] = {0, 0};
uint8_t AvgCurrent [2] = {0, 0};
uint8_t Data [2] = {0, 0};
uint8_t Test [2] = {0, 0};


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
// General SPI State Machine ***************************************************
//******************************************************************************

void SendUCA1Data(uint8_t val);
I2C_Mode SPI_Master_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);


void SendUCA1Data(uint8_t val)
{
    while (!(UCA1IFG & UCTXIFG));           // USCI_A1 TX buffer ready?
    UCA1TXBUF = val;
}


I2C_Mode SPI_Master_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    MasterMode = TX_DATA_MODE; // instead of TX_REG_ADDRESS_MODE
    TransmitRegAddr = reg_addr;

    // Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    //SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);
    SendUCA1Data(TransmitBuffer[0]); // send first byte instead of address
    TXByteCtr--;
    TransmitIndex++;

    __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/interrupts enabled

    //SLAVE_CS_OUT |= SLAVE_CS_PIN;
    return MasterMode;
}



//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************


void initGPIO()
{
    // Configure GPIO
    //LED_OUT &= ~(LED0_PIN | LED1_PIN); // P1 setup for LED & reset output
    //LED_DIR |= (LED0_PIN | LED1_PIN);

    // Port Configuration all un-used pins to output low
    P1OUT = 0x00;
    P2OUT = 0x00;
    P3OUT = 0x00;
    P1DIR = 0x00;
    P2DIR = 0x00;
    P3DIR = 0x00;

    // turn on both power switches for level shifter
    P3DIR |= BIT0;
    P3OUT |= BIT0;
    P3DIR |= BIT2;
    P3OUT |= BIT2;
    _delay_cycles(1600);

    // Configure eUSCI pins
    P2SEL0 |= BIT4 | BIT5 | BIT6;
    P2SEL1 &= ~(BIT4 | BIT5 | BIT6);

    // SPI chip select
    SLAVE_CS_DIR |= SLAVE_CS_PIN;
    SLAVE_CS_OUT &= ~SLAVE_CS_PIN;

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

void initSPI()
{
    // Clock polarity select: inactive state is high, MSB first
    // 8-bit data, Synchronous, Master mode, 3-pin SPI, BRCLK source: SMCLK
    UCA1CTLW0 = UCSWRST;                    // **Put eUSCI module in reset**
    UCA1CTLW0 |= /*UCCKPL |*/ UCCKPH | UCMSB | UCSYNC |
                 UCMST | UCSSEL__SMCLK;
    UCA1BRW = 80;                           // BRCLK / UCBRx = UCxCLK
                                            // 8MHz  / 80    = 100kHz
    UCA1CTLW0 &= ~UCSWRST;                  // **Initialize eUSCI module**
    UCA1IE |= UCRXIE;                       // Enable eUSCI0 RX interrupt
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
// Gauge Functions *************************************************************
//******************************************************************************

void initializeConfig(void){
    FSTAT[0] = 0;
    while(FSTAT[0] & 0x01){
        _delay_cycles(160000); // 10ms Wait Loop. Do not continue until FSTAT.DNR==0
        I2C_Master_ReadReg(SLAVE_ADDR, 0x3D, 2);
        CopyArray(ReceiveBuffer, FSTAT, 2);
    }


    I2C_Master_ReadReg(SLAVE_ADDR, 0xDB, 2);
    CopyArray(ReceiveBuffer, HibCFG, 2); //Store original HibCFG value

    I2C_Master_WriteReg (SLAVE_ADDR, 0x60 , Write1, 2); // Exit Hibernate Mode step 1
    I2C_Master_WriteReg (SLAVE_ADDR, 0xBA , Write2, 2); // Exit Hibernate Mode step 2
    I2C_Master_WriteReg (SLAVE_ADDR, 0x60 , Write2, 2); // Exit Hibernate Mode step 3

    I2C_Master_WriteReg(SLAVE_ADDR, 0x18, DesignCap, 2); // Design Capacity

    I2C_Master_ReadReg(SLAVE_ADDR, 0x18, 2);
    CopyArray(ReceiveBuffer, Test, 2); //test if Design Capacity was written

    I2C_Master_WriteReg(SLAVE_ADDR, 0x1E, IchgTerm, 2); // Termination Current
    I2C_Master_WriteReg(SLAVE_ADDR, 0x3A, VEmpty, 2); // Empty Voltage

    I2C_Master_WriteReg(SLAVE_ADDR, 0xDB, Write3, 2); // Write ModelCFG, because ChargeVoltage < 4.275V

    // Poll ModelCFG.Refresh(highest bit), proceed to Step 3 when ModelCFG.Refresh==0.
    ModelCFG[1] = 0;
    while (ModelCFG[1] & 0x80){
        _delay_cycles(160000); // 10ms Wait Loop. Do not continue until ModelCFG.Refresh==0
        I2C_Master_ReadReg(SLAVE_ADDR, 0xDB, 2);
        CopyArray(ReceiveBuffer, ModelCFG, 2);
    }
    I2C_Master_WriteReg(SLAVE_ADDR, 0xBA , HibCFG, 2); // Restore Original HibCFG value

    // enable hibernate mode -> one measurement every 5.625 seconds
    I2C_Master_WriteReg(SLAVE_ADDR, 0xBA , EnHib, 2);


    I2C_Master_ReadReg(SLAVE_ADDR, 0x00, 2); //Read Status
    CopyArray(ReceiveBuffer, Status, 2);
    Status[1] &= 0xFF;
    Status[0] &= 0xFD;
    I2C_Master_WriteReg(SLAVE_ADDR, 0x00, Status, 2);
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

uint16_t convertV(uint16_t data) {
    uint16_t res;
    res = (data>>7)*10;
    return res;
}

uint16_t convertCurrent(uint16_t data) {
    uint16_t res;
    res = data<<4;
    return res;
}


//******************************************************************************
// Display Functions ***********************************************************
//******************************************************************************

#define DUMMY                   0xFF
#define DISPLAY_SIZEX    128
#define DISPLAY_SIZEY    128
#define DISPLAY_NUMBYTES ((DISPLAY_SIZEX*DISPLAY_SIZEY)/8) //8 PIXEL/BYTE
uint8_t display_com, display_com_mask = 0x40;   //needed to generate COM clock on display chip

void display_clear()
{
    //set CS
    SLAVE_CS_OUT |= SLAVE_CS_PIN;
    _delay_cycles(16); //delay_us(1);

    // Send clear command
    uint8_t  NVM_READ_CMD[2] = {0x60 /* 0x02 | display_com*/, 0x00};
    uint16_t NVM_READ_CMD_SIZE = 2;
    SPI_Master_WriteReg(0,NVM_READ_CMD,NVM_READ_CMD_SIZE);


    //unset CS of display
    _delay_cycles(16); //delay_us(1);
    SLAVE_CS_OUT &= ~SLAVE_CS_PIN;
}

void display_init()
{
    //unset CS of display
    SLAVE_CS_OUT &= ~SLAVE_CS_PIN;

    //clear display
    _delay_cycles(160); //delay_us(10);
    display_clear();
    _delay_cycles(160); //delay_us(10);

}

//refreshes LCD memory with new pixel data
void display_update(uint8_t *image_binary)
{
    uint8_t bytes_per_line =  16; //DISPLAY_SIZEX / 8;
    //uint16_t totalbytes = (DISPLAY_SIZEX * DISPLAY_SIZEY) / 8;

    //set CS
    SLAVE_CS_OUT |= SLAVE_CS_PIN;
    _delay_cycles(16); //delay_us(1);

    //transfer write command: 0x01
    uint8_t  NVM_READ_CMD[1] = {0x80 | display_com};
    uint16_t NVM_READ_CMD_SIZE = 1;
    SPI_Master_WriteReg(0,NVM_READ_CMD,NVM_READ_CMD_SIZE);
    display_com = display_com ^ display_com_mask;       //toggle COM signal

    //transfer (pixel) data line-by-line
    uint16_t i = 0;
    for (; i < /*DISPLAY_SIZEY*/128; ++i) {
        uint8_t line[/*bytes_per_line + 2*/18];
        // Send address byte
        uint8_t currentline = (i+1); //((i + 1) / (WIDTH / 8)) + 1;
        //swap bit direction (LSB-> MSB)
        uint8_t spi_Byte_temp = 0;
        int k;
        for(k = 0; k<8; ++k){
          spi_Byte_temp |= (currentline & 0x01) << (7-k);
          currentline = currentline >>1;
        }
        line[0] = spi_Byte_temp;
        // copy over this line
        memcpy(line + 1, image_binary + (i*bytes_per_line), bytes_per_line);
        // Send end of line
        line[bytes_per_line + 1] = 0x00;
        // send it!
        SPI_Master_WriteReg(0,line, bytes_per_line + 2);
    }

    // Send dummy byte for the last line
    NVM_READ_CMD[1] = 0x00;
    NVM_READ_CMD_SIZE = 1;
    SPI_Master_WriteReg(0,NVM_READ_CMD,NVM_READ_CMD_SIZE);

    //unset CS
    _delay_cycles(16); //delay_us(1);
    SLAVE_CS_OUT &= ~SLAVE_CS_PIN;
}



#define OLED_MAX_X (128)
#define OLED_MAX_Y (128)
#define LINE (128)
#define LINE_SIZE (16)
static uint8_t LCD_GRAM[LINE][LINE_SIZE];

void lcd_drawpoint(uint16_t x,uint16_t y,uint8_t bDraw){

 uint16_t pos,bx,tmp;

  if(x>OLED_MAX_X-1||y>OLED_MAX_Y-1)
    return;
  pos=15-y/8;
  bx=y%8;
    tmp=1<<(bx);
  if(bDraw)
     LCD_GRAM[x][pos]|= tmp;
    else
     LCD_GRAM[x][pos]&= ~tmp;

}

void lcd_fillRect(uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint8_t bDraw)
{
  uint16_t wi,hi;
  for(hi=0;hi<h;hi++){
   for(wi=0;wi<w;wi++)
      lcd_drawpoint(x+wi,y+hi,bDraw);
  }
}

void lcd_fillRectByXY(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint8_t bDraw)
{
   uint16_t xStart=0,yStart=0;
   uint16_t w,h;

   if(x0<x1){
     xStart=x0;
     w=x1-x0+1;
   }else{
     xStart=x1;
     w=x0-x1+1;
   }

   if(y0<y1){
     yStart=y0;
     h=y1-y0+1;
   }else{
      yStart=y1;
      h=y0-y1+1;
   }
   lcd_fillRect(xStart,yStart,w,h,bDraw);

}

//uint32_t lcd_getpoint(uint16_t x,uint16_t y)
//{
//}

void lcd_print_char(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode)
{
    uint8_t temp,t,t1;
    uint8_t y0=y;
    uint8_t csize=(size/8+((size%8)?1:0))*(size/2);
    chr=chr-' ';
    for(t=0;t<csize;t++)
    {
        if(size==12)temp=asc2_1206[chr][t];
        else if(size==16)temp=asc2_1608[chr][t];
        else if(size==24)temp=asc2_2412[chr][t];
        else return;
        for(t1=0;t1<8;t1++)
        {
            if(temp&0x80)lcd_drawpoint(x,y,mode);
            else lcd_drawpoint(x,y,!mode);
            temp<<=1;
            y++;
            if((y-y0)==size)
            {
                y=y0;
                x++;
                break;
            }
        }
    }
}

uint32_t mypow(uint8_t m,uint8_t n)
{
    uint32_t result=1;
    while(n--)result*=m;
    return result;
}

void lcd_print_num(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{
    uint8_t t,temp;
    uint8_t enshow=0;
    for(t=0;t<len;t++)
    {
        temp=(num/mypow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                lcd_print_char(x+(size/2)*t,y,' ',size,1);
                continue;
            }else enshow=1;

        }
        lcd_print_char(x+(size/2)*t,y,temp+'0',size,1);
    }
}

void lcd_print_string(uint8_t x,uint8_t y,const uint8_t *p,uint8_t size)
{
    while((*p<='~')&&(*p>=' '))
    {
        if(x>(128-(size/2))){x=0;y+=size;}
        if(y>(128-size)){y=x=0;display_clear();}
        lcd_print_char(x,y,*p,size,1);
        x+=size/2;
        p++;
    }
}


//******************************************************************************
// Main ************************************************************************
//******************************************************************************

uint16_t resultCAP;
uint16_t resultSOC;
uint16_t resultV;
uint16_t resultCurrent;

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    initClockTo16MHz();
    initGPIO();
    initSPI();
    initI2C();

    // crystal setup
    CSCTL4 = SELMS__DCOCLKDIV | SELA__XT1CLK;          // MCLK=SMCLK=DCO; ACLK=XT1
    P2SEL0 |= BIT0 + BIT1;                             // P2.0: XOUT; P2.1: XI1
    do
    {
        CSCTL7 &= ~(XT1OFFG | DCOFFG);      // Clear XT1 and DCO fault flag
        SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1 & OFIFG);               // Test oscillator fault flag


    initializeConfig();
    display_init();

    while(1){

        I2C_Master_ReadReg(SLAVE_ADDR, 0x00, 2);
        CopyArray(ReceiveBuffer, Status, 2);
        StatusPOR[0] = Status[0] & 0x02;

        if (StatusPOR[0] == 0x01){
            initializeConfig();
        }

        I2C_Master_ReadReg(SLAVE_ADDR, 0x00, 2);
        CopyArray(ReceiveBuffer, Status, 2);
        StatusPOR[0] = Status[0] & 0x02;

        if (StatusPOR[0] == 0x00){

            // Read Capacity
            I2C_Master_ReadReg(SLAVE_ADDR, 0x05, 2);
            CopyArray(ReceiveBuffer, RepCAP, 2);
            resultCAP = convertCAP(concatenate(RepCAP[1], RepCAP[0]));

            // Read State of Charge
            I2C_Master_ReadReg(SLAVE_ADDR, 0x06, 2);
            CopyArray(ReceiveBuffer, RepSOC, 2);
            resultSOC = convertSOC(concatenate(RepSOC[1], RepSOC[0]));

            // Read Cell Voltage
            I2C_Master_ReadReg(SLAVE_ADDR, 0x09, 2);
            CopyArray(ReceiveBuffer, Vcell, 2);
            resultV = convertV(concatenate(Vcell[1], Vcell[0]));

            // Read Average Current
            //I2C_Master_ReadReg(SLAVE_ADDR, 0x0B, 2); // average current over last 5 seconds
            //CopyArray(ReceiveBuffer, AvgCurrent, 2);
            //resultCurrent = convertCurrent(concatenate(AvgCurrent[1], AvgCurrent[0]));

            //P2OUT ^= 0x01; // toggle LED

            // update display
            uint8_t SoC_bar = resultSOC*28/100; // compute thickness of bar
            uint32_t SoC_percentage = resultSOC;

            lcd_fillRect(16,16,32,24,1);
            lcd_fillRect(48,24,4,8,1);
            lcd_fillRect(18,18,SoC_bar,20,0);

            lcd_print_num(92,16,SoC_percentage,2,12);
            lcd_print_char(110,16,'%',12,1);
            lcd_print_num(76,28,resultV,4,12);
            lcd_print_string(104,28,"mV",12);


            lcd_print_string(40,48,"Normal Mode",12);
            lcd_fillRect(24,48,12,12,1);
            lcd_print_string(40,64,"Power Saving",12);
            lcd_fillRect(24,64,12,12,1);

            if (resultSOC > 30){
                lcd_fillRect(24,64,12,12,1);
                lcd_fillRect(26,50,8,8,0);
            }
            else{
                lcd_fillRect(24,48,12,12,1);
                lcd_fillRect(26,66,8,8,0);
            }

            display_update(LCD_GRAM);

            // go into LPM0 until next measurement
            I2C_Master_WriteReg(SLAVE_ADDR, 0xBA, EnHib, 2);
            TA0CCTL0 |= CCIE;
            TA0CCR0 = 32678; // 16s delay
            TA0CTL |= TASSEL__ACLK | MC__CONTINUOUS | ID__8; // set clock to ACLK/8
            __bis_SR_register(LPM3_bits + GIE);              // Enter LPM0 w/ interrupts

        }
     }
}

//******************************************************************************
// Interrupts ***************************************************************
//******************************************************************************

// I2C ISR
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
    //P2OUT ^= BIT0;
    TA0CCR0 += 32678;                             // Add Offset to TACCR0
    __bic_SR_register_on_exit(LPM3_bits);
}


// SPI ISR
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
