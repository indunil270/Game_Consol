/*
 * Game_consol.c
 *
 * Created: 27/09/2017 9:54:35 PM
 * Author : indunl
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <dogm-graphic.h>
#include <avr/iom16.h>

#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT))

#define LOW_VOL_LED PC0
#define BACK_LED PB3
#define PB_INT PD2
#define PB_A1 PD0
#define PB_A2 PD1
#define PB_UP PD3
#define PB_DOWN PD4
#define PB_LEFT PD5
#define PB_RIGHT PD6

static volatile int LCD_brig_value =1;




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

ISR(INT0_vect)
{

	
	if (CHECKBIT(PIND,PB_A1) == 0)
	{
		CLEARBIT(DDRC,BACK_LED);
		MCUCR |= _BV(SM1) | _BV(SE);
	}
	if (CHECKBIT(PIND,PB_A2) == 0)
	{
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
	//Input pin configeration
	DDRA |=_BV(PA0);
	PORTD =0x7F;	//pulup input buttons  
	//***********************
	DDRC |= _BV(LOW_VOL_LED);
	


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
	SREG  |=_BV(7);
	//******************************************
	
	set_back_led_brightness();
		  
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


	

