/*
 * Game_consol.c
 *
 * Created: 27/09/2017 9:54:35 PM
 * Author : indunl
 */

#ifndef F_CPU
#define F_CPU 7372800UL
#endif 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <dogm-graphic.h>
#include <avr/iom16.h>

#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))
#define _BV(BIT) (1<<BIT)

#define LOW_VOL_LED PC0
#define BACK_LED PB3
#define PB_INT PD2
#define PB_A1 PD0
#define PB_A2 PD1
#define PB_UP PD3
#define PB_DOWN PD4
#define PB_LEFT PD5
#define PB_RIGHT PD6

#define CS1_RAM PC6
#define CS0_LCD PB2
#define RST_LCD PB1
#define CD PB0

static volatile int LCD_brig_value =1;
uint8_t lcd_current_page = 0;
uint8_t lcd_current_column = 0;





void set_back_led_brightness(void)
{
	if (CHECKBIT(PIND,PB_UP) == 0)
	{
		LCD_brig_value += 5;
	}
	else if (CHECKBIT(PIND,PB_DOWN) == 0)
	{
		LCD_brig_value -= 5;
	}
	OCR0 = LCD_brig_value;
	
}



void SPI_data_sent_to_LCD(uint8_t _SPI_data){
	CLEARBIT(PORTB,CS0_LCD);	//Select SPI LCD
	SPDR = _SPI_data;
	
	while(!(SPSR & _BV(SPIF))){
		
	}
}

void SPI_Config(){
	DDRB |= _BV(CD);				// CD Pin in LCD, Command Data select 
	DDRB |= _BV(PB7) | _BV(PB5);		//CLK and MOSI pin set as output
	DDRB |= _BV(CS0_LCD);	//Set as output
	PORTB|= _BV(CS0_LCD);	//default output high
	
	// SPI configuration
	SPCR |=_BV(SPE)			//Enable SPI
	| _BV(MSTR)		//Set as Master
	| _BV(CPOL)		//Clock high when idle
	| _BV(CPHA);		//Sample in Trailing edge
	//SCK frequency Fosc / 4	
	SPDR = LCD_NO_OP;
}
uint8_t lcd_inc_page(int8_t s) {
  uint8_t p = lcd_current_page;
  p += s;
  p %= LCD_RAM_PAGES;    //all lcd have lcd_ram_pages which is power of two
  lcd_current_page = p;
  return p;
  }

/******************************************************************************
 * Changes the internal cursor by s columns, including wrapping (if selected)
 * s             - number of columns to move
 */ 
uint8_t lcd_inc_column(int16_t s) {
  uint16_t c = lcd_current_column;
  c += s;
#if LCD_WRAP_AROUND == 1
  while (c >= LCD_WIDTH) {
    if (s > 0) lcd_inc_page(1);
    else       lcd_inc_page(-1);
    if (s > 0) c -= LCD_WIDTH;
    else       c += LCD_WIDTH;
    }
#endif
  lcd_current_column = c;
  return c;
  }
  
  
/******************************************************************************
 * Moves the cursor to the given position
 * pages         - page to move to
 * columns       - column to move to
 */ 
void lcd_moveto_xy(uint8_t page, uint8_t column) {
  LCD_GOTO_ADDRESS(page,column);
  lcd_current_column = column; 
  lcd_current_page = page;
  }

/******************************************************************************
 * Moves the cursor relative to the current position
 * pages         - number of pages to move
 * columns       - number of columns to move
 */  
void lcd_move_xy(int8_t pages, int16_t columns) {
  lcd_moveto_xy(lcd_inc_page(pages),lcd_inc_column(columns));
  }


//=============================================================================
//Basic Byte Access to Display
//=============================================================================

/******************************************************************************
 * Writes one data byte
 * data          - the data byte
 */
void lcd_data(uint8_t data) {
  LCD_SELECT();
  LCD_DRAM();
  spi_write(data);
  LCD_UNSELECT();
  lcd_inc_column(1);
  }

/******************************************************************************
 * Writes one command byte
 * cmd           - the command byte
 */
void lcd_command(uint8_t cmd) {
  LCD_SELECT();
  LCD_CMD();
  spi_write(cmd);
  LCD_UNSELECT();
  }


/******************************************************************************
 * This function clears an area of the screen
 * pages         - height of area in pages
 * columns       - width of area in pixels
 * style         - Bit2: sets inverse mode
 * Cursor is moved to start of area after clear
 */
void lcd_clear_area(uint8_t pages, uint8_t columns, uint8_t style) {
  uint8_t i,j,max;
  uint8_t inv = (style & INVERT_BIT)?0xFF:0;
  
  if(pages > (max = LCD_RAM_PAGES - lcd_get_position_page()))   
    pages = max;
  if(columns > (max = LCD_WIDTH - lcd_get_position_column()))   
    columns = max;
  
  for(j=0; j<pages; j++) {
    for(i=0; i<columns; i++) {
      lcd_data(inv);
      }
    lcd_move_xy(1,-columns);
    }
  lcd_move_xy(-pages,0);
  }

/******************************************************************************
 * This function clears an area of the screen starting at the given coordinates
 * pages         - height of area in pages
 * columns       - width of area in pixels
 * style         - style modifier
 * col           - column of upper left corner
 * page          - page of upper left corner
 * Cursor is moved to start of area after clear
 */
void lcd_clear_area_xy(uint8_t pages, uint8_t columns, uint8_t style, uint8_t page, uint8_t col) {
  lcd_moveto_xy(page,col);
  lcd_clear_area(pages,columns,style);
  }



void lcd_init() {
	LCD_SET_PIN_DIRECTIONS();  //set outputs
	//LCD_INIT_SPI();            //Initialize SPI Interface
	LCD_RESET();               //Apply Reset to the Display Controller
	//Load settings

	#if DISPLAY_TYPE == 102
		LCD_SET_FIRST_LINE(0);              //first bit in RAM is on the first line of the LCD
		#if ORIENTATION_UPSIDEDOWN == 0
			LCD_SET_BOTTOM_VIEW();            //6 o'clock mode, normal orientation
			LCD_ORIENTATION_NORMAL();
		#else
			LCD_SET_TOP_VIEW();               //12 o'clock mode, reversed orientation
			LCD_ORIENTATION_UPSIDEDOWN();
		#endif
		LCD_SHOW_ALL_PIXELS_OFF();          //Normal Pixel mode
		LCD_SET_MODE_POSITIVE();            //positive display
		LCD_SET_BIAS_RATIO_1_9();           //bias 1/9
		LCD_SET_POWER_CONTROL(7);           //power control mode: all features on
		LCD_SET_BIAS_VOLTAGE(7);            //set voltage regulator R/R
		LCD_SET_VOLUME_MODE(0x9);           //volume mode set
		LCD_SET_ADV_PROG_CTRL(LCD_TEMPCOMP_HIGH);
	#endif
	lcd_clear_area_xy(LCD_RAM_PAGES,LCD_WIDTH,NORMAL,0,0); //clear display content

	LCD_SWITCH_ON();                    //Switch display on
	return;
}

ISR(INT0_vect)
{

	
	if (CHECKBIT(PIND,PB_A1) == 0)
	{
		CLEARBIT(DDRC,BACK_LED);
		MCUCR |= _BV(SM1) | _BV(SE);
	}
	if (CHECKBIT(PIND,PB_A2) == 0)
	{
		SPI_data_sent_to_LCD(0x52);
	}
	if (CHECKBIT(PIND,PB_LEFT) == 0)
	{
	}
	if (CHECKBIT(PIND,PB_RIGHT)== 0)
	{
	}
	if (CHECKBIT(PIND,PB_UP) == 0)
	{
		set_back_led_brightness();
	}
	if (CHECKBIT(PIND,PB_DOWN) == 0)
	{
		set_back_led_brightness();
		PORTC = 0x00;
	}
	
}

int main(void)

{
	//Input pin configuration
	DDRA &=~_BV(PA0);
	PORTD =0x7F;	//Pull up input buttons  
	//***********************
	//Output pin configuration
					//Back LED 
	DDRC |= _BV(LOW_VOL_LED);		//LED indicator for law Battery voltage 
	
	
	//************************************

	//PWM mode configuration 
	TCCR0 |=_BV(WGM00)
		  |_BV(COM01)
		  |_BV(CS02);
	DDRB |= _BV(BACK_LED);
	//***************************	
	//Interrupts setting up 
	MCUCR |=_BV(ISC01)		//Rising edge interrupt for INT0
			|_BV(ISC00);	//Rising edge interrupt
	GICR  |=_BV(INT0);		//External Interrupt Request Enable
	//SREG  |=_BV(7);
	//******************************************
	
	//********************************************
	SPI_Config();
	set_back_led_brightness();
	lcd_init();
	sei();	  
    while (1) 
    {
	
	//	PORTC = 0x01;
	//	_delay_ms(500);
		PORTC = 0x01;
	//	PORTB = 0x08;
		_delay_ms(10000);
	//	PORTB = 0x00;
	
    }
}


	

