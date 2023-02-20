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
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "oled_font.h"


//******************************************************************************
// Pin Config ******************************************************************
//******************************************************************************

#define SLAVE_CS_OUT            P2OUT // rerouted via digital pin
#define SLAVE_CS_DIR            P2DIR
#define SLAVE_CS_PIN            BIT7

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

#define MAX_BUFFER_SIZE         128


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


uint8_t ReceiveBuffer[MAX_BUFFER_SIZE]  = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;


SPI_Mode SPI_Master_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count);


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
    UCA1CTLW0 |= /*UCCKPL |*/ UCCKPH | UCMSB | UCSYNC |
                 UCMST | UCSSEL__SMCLK;
    UCA1BRW = 80;                           // BRCLK / UCBRx = UCxCLK
                                            // 8MHz  / 80    = 100kHz
    UCA1CTLW0 &= ~UCSWRST;                  // **Initialize eUSCI module**
    UCA1IE |= UCRXIE;                       // Enable eUSCI0 RX interrupt
}

void initGPIO()
{
    // Configure LEDs and buttons
    //COMMS_LED_DIR |= COMMS_LED_PIN;
    //COMMS_LED_OUT &= ~COMMS_LED_PIN;

    //BUTTON_LED_DIR |= BUTTON_LED_PIN;
    //BUTTON_LED_OUT &= ~BUTTON_LED_PIN;

    // turn on both power switches for level shifter
    P3DIR |= BIT0;
    P3OUT |= BIT0;
    P3DIR |= BIT2;
    P3OUT |= BIT2;
    _delay_cycles(1600);

    // Configure eUSCI pins
    P2SEL0 |= BIT4 | BIT5 | BIT6;
    P2SEL1 &= ~(BIT4 | BIT5 | BIT6);

    //SLAVE_RST_DIR |= SLAVE_RST_PIN;
    //SLAVE_RST_OUT |= SLAVE_RST_PIN;

    SLAVE_CS_DIR |= SLAVE_CS_PIN;
    SLAVE_CS_OUT &= ~SLAVE_CS_PIN;

    // Button to initiate transfer
    //BUTTON_DIR &= ~BUTTON_PIN;              // Button input
    //BUTTON_OUT |= BUTTON_PIN;               // Button pull up
    //BUTTON_REN |= BUTTON_PIN;               // Button pull up/down resistor enable
    //BUTTON_IES |= BUTTON_PIN;               // Button high-to-low edge

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Must be done after clearing LOCKLPM5
    //BUTTON_IFG &= ~BUTTON_PIN;              // Button IFG cleared
    //BUTTON_IE  |= BUTTON_PIN;               // Button interrupt enabled
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


// code py Petar Jokic


uint8_t ascii_font_8x8_lettersNHUSA [5*8] = { //ascii_font_8x8_lettersBPRS
    //source: \\csem.local\data-m\projects\111-SF.2022_MIP19_VISAGE\TechnicalExecution\v1a\Test_demo\creditcard_pjo\LCD_SharpLS013\ascii_5x7_toHorizontal_8x8.py
    0x44,0x44,0x64,0x54,0x4c,0x44,0x44,0x00, //N (eutral)
    0x44,0x44,0x44,0x7c,0x44,0x44,0x44,0x00, //H (appy)
    0x44,0x44,0x44,0x44,0x44,0x44,0x38,0x00, //U (s..rprised)
    0x3c,0x40,0x40,0x38,0x04,0x04,0x78,0x00, //S (ad)
    0x38,0x44,0x44,0x44,0x7c,0x44,0x44,0x00  //A (ngry)
  };
  
#define DISPLAY_SIZEX    128
#define DISPLAY_SIZEY    128
#define DISPLAY_NUMBYTES ((DISPLAY_SIZEX*DISPLAY_SIZEY)/8) //8 PIXEL/BYTE
uint8_t display_image[DISPLAY_NUMBYTES] = {
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA
};
uint8_t display_com, display_com_mask = 0x40;   //needed to generate COM clock on display chip


// *************************************************************************************************************** //
// ** Display: LCD (Sharp LS013) ********************************************************************************* //
// *************************************************************************************************************** //

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

int main(void)
{
    // Stop watchdog timer
     WDTCTL = WDTPW | WDTHOLD;

    // Initialize device
    initClockTo16MHz();
    initGPIO();
    initSPI();

    // turn on LED
    //P2DIR |= BIT0;
    //P2OUT |= BIT0;

    // Display code
    display_init();
    while(1)
    {
          uint8_t SoC_bar = 22;
          uint8_t *SoC_percentage = "72%";
          lcd_fillRect(16,16,32,24,1);
          lcd_fillRect(48,24,4,8,1);
          lcd_fillRect(18,18,SoC_bar,20,0);
          lcd_print_string(80,16,SoC_percentage,24);
          lcd_print_string(32,48,"Power Saving",12);
          //lcd_print_string(32,48,"Normal Mode",12);
          //lcd_print_char(16,16,'a',16,1);

          display_update(LCD_GRAM); // ascii_font_8x8_lettersNHUSA, display_image
          _delay_cycles(1600000); // 100ms delay
          //P2OUT ^= 0x01; // blink LED
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
