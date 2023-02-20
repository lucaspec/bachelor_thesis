// code py Petar Jokic


uint8_t ascii_font_8x8_lettersNHUSA [5*8] = { //ascii_font_8x8_lettersBPRS
	//source: \\csem.local\data-m\projects\111-SF.2022_MIP19_VISAGE\TechnicalExecution\v1a\Test_demo\creditcard_pjo\LCD_SharpLS013\ascii_5x7_toHorizontal_8x8.py
	0x44,0x44,0x64,0x54,0x4c,0x44,0x44,0x00, //N (eutral)
	0x44,0x44,0x44,0x7c,0x44,0x44,0x44,0x00, //H (appy)
	0x44,0x44,0x44,0x44,0x44,0x44,0x38,0x00, //U (s..rprised)
	0x3c,0x40,0x40,0x38,0x04,0x04,0x78,0x00, //S (ad)
	0x38,0x44,0x44,0x44,0x7c,0x44,0x44,0x00  //A (ngry)
  };
  
  
#define DISPLAY_NUMBYTES ((DISPLAY_SIZEX*DISPLAY_SIZEY)/8) //8 PIXEL/BYTE
uint8_t display_image[DISPLAY_NUMBYTES]; 
uint8_t display_com, display_com_mask = 0x40;   //needed to generate COM clock on display chip


// *************************************************************************************************************** //
// ** Display: LCD (Sharp LS013) ********************************************************************************* //
// *************************************************************************************************************** //

void display_clear()
{
	//set CS
	p_ccu_rif->gpo.reg |= (1<<4);
	delay_us(1);
	
	// Send clear command
	uint8_t  NVM_READ_CMD[2] = {0x20 | display_com, 0x00};
	uint16_t NVM_READ_CMD_SIZE = 2;
	spi_master_write(0,NVM_READ_CMD,NVM_READ_CMD_SIZE);
	
	
	//unset CS of display
	delay_us(1);
    p_ccu_rif->gpo.reg &= ~(1<<4);
}	

void display_init()
{
	//unset CS of display
    p_ccu_rif->gpo.reg &= ~(1<<4);
	
	//clear display
	delay_us(10);
	display_clear();
	delay_us(10);
	
}

//refreshes LCD memory with new pixel data
void display_update(uint8_t *image_binary)
{
	uint8_t bytes_per_line = DISPLAY_SIZEX / 8;
	//uint16_t totalbytes = (DISPLAY_SIZEX * DISPLAY_SIZEY) / 8;
	
	//set CS
	p_ccu_rif->gpo.reg |= (1<<4);
	delay_us(1);
	
	//transfer write command: 0x01 
	uint8_t  NVM_READ_CMD[1] = {0x80 | display_com};
	uint16_t NVM_READ_CMD_SIZE = 1;
	spi_master_write(0,NVM_READ_CMD,NVM_READ_CMD_SIZE);
	display_com = display_com ^ display_com_mask; 		//toggle COM signal
	
	//transfer (pixel) data line-by-line
	for (int i = 0; i < DISPLAY_SIZEY; i++) {
		uint8_t line[bytes_per_line + 2];
		// Send address byte
		uint8_t currentline = (i+1); //((i + 1) / (WIDTH / 8)) + 1;
		//swap bit direction (LSB-> MSB)
		uint8_t spi_Byte_temp = 0;
		for(int k = 0; k<8; k++){
		  spi_Byte_temp |= (currentline & 0x01) << (7-k);
		  currentline = currentline >>1;
		}
		line[0] = spi_Byte_temp;
		// copy over this line
		memcpy(line + 1, image_binary + (i*bytes_per_line), bytes_per_line);
		// Send end of line
		line[bytes_per_line + 1] = 0x00;
		// send it!
		spi_master_write(0,line, bytes_per_line + 2);
	}

	// Send dummy byte for the last line  
	NVM_READ_CMD[1] = 0x00;
	NVM_READ_CMD_SIZE = 1;
	spi_master_write(0,NVM_READ_CMD,NVM_READ_CMD_SIZE);
	
	//unset CS
	delay_us(1);
    p_ccu_rif->gpo.reg &= ~(1<<4);
}
