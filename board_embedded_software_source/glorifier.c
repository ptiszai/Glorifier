/*******************************************************************************#
#           Glorifier															#
#			https://github.com/ptiszai/Glorifier								#
#           Istvan Tiszai <tiszaii@hotmail.com>                                 #
#                                                                               #
# This program is free software; you can redistribute it and/or modify          #
# it under the terms of the GNU General Public License as published by          #
# the Free Software Foundation; either version 2 of the License, or             #
# (at your option) any later version.                                           #
#                                                                               #
# This program is distributed in the hope that it will be useful,               #
# but WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                 #
# GNU General Public License for more details.                                  #
#                                                                               #
# You should have received a copy of the GNU General Public License             #
# along with this program; if not, write to the Free Software                   #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA     #
#                                                                               #
********************************************************************************/

/*
	AVR Studio 4.18
*/
//*****************************************************************************
//
//            					R E T U R N   S T A C K   3 2
//                   					X T A L  16 MHZ
//
//*****************************************************************************
//                P I N   U S A G E

// PA0 -> COOLER
// PA1 -> MODE
// PA2 -> n/c
// PA3 -> n/c
// PA4 -> n/c
// PA5 -> R1
// PA6 -> R4
// PA7 -> G1

// PB0 -> TOUCH20K
// PB1 -> n/c
// PB2 -> TOUCHIN
// PB3 -> n/c
// PB4 -> n/c
// PB5 -> MOSI
// PB6 -> MISO
// PB7 -> SCK

//
// PC0 -> B2
// PC1 -> G5
// PC2 -> G2
// PC3 -> R5
// PC4 -> B4
// PC5 -> B1
// PC6 -> G4
// PC7 -> R2
//
// PD0 -> scop
// PD1 -> n/c
// PD2 -> B3
// PD3 -> G3
// PD4 -> R3
// PD5 -> B5
// PD6 -> IR input
// PD7 -> n/c
//
//
//*****************************************************************************
//                T I M E R   U S A G E
//
// Timer 0 is not used
// Timer 1 is use by IR remote controller
// Timer 2 is use by Led PWM
//
//*****************************************************************************
#define F_CPU			16000000UL
//*****************************************************************************
//                      I N C L U D E
//*****************************************************************************
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/wdt.h> 
#include <avr/sleep.h> 
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h> 
//#include <avr/macros.h>
//#include <iom32v.h>
//#include <macros.h>


//*****************************************************************************
//                      D E F I N E
//*****************************************************************************

//#define TEST1				  1				

#define TRUE                  1
#define FALSE                 0
#define XTAL                  16000000
#define ushort				  unsigned short
#define uint				  unsigned int
#define ulong				  unsigned long


#define LED1_RED		      (1<<PA5)
#define LED1_GREEN            (1<<PA7)
#define LED1_BLUE	    	  (1<<PC5)
//#define LED1_DDR		      DDRD
//#define LED1_PORT	    	  PORTD
//#define LED1_PIN		      PIND

#define LED2_RED		      (1<<PC7)
#define LED2_GREEN            (1<<PC2)
#define LED2_BLUE	    	  (1<<PC0)
//#define LED2_DDR		      DDRD
//#define LED2_PORT	    	  PORTD
//#define LED2_PIN		      PIND

#define LED3_RED		      (1<<PD4)
#define LED3_GREEN            (1<<PD6)
#define LED3_BLUE	    	  (1<<PD2)
//#define LED3_DDR		      DDRC
//#define LED3_PORT	    	  PORTC
//#define LED3_PIN		      PINC

#define LED4_RED		      (1<<PA6)
#define LED4_GREEN            (1<<PC6)
#define LED4_BLUE	    	  (1<<PC4)
//#define LED4_DDR		      DDRC
//#define LED4_PORT	    	  PORTC
//#define LED4_PIN		      PINC

#define LED5_RED		      (1<<PC3)
#define LED5_GREEN            (1<<PC1)
#define LED5_BLUE	    	  (1<<PD5)
//#define LED5_DDR		      DDRC
//#define LED5_PORT	    	  PORTC
//#define LED5_PIN		      PINC


#define OFF					  0x00
#define RED					  0x01
#define GREEN				  0x02
#define BLUE				  0x03

#define IR_PIN               (1<<PD6)
/*#define IR_DDR                DDRD
#define IR_PIN                PIND
#define IR_PORT               PORTD
#define IR_INPUT              0x08*/

#define COOLER				(1<<PA0)	//ventillátor az A0-án
#define COOLER_H			PORTA |= COOLER;
#define COOLER_L			PORTA &= ~ COOLER;
#define MODE_DDR              DDRA
#define MODE_PIN              PINA
#define MODE_PORT             PORTA
#define MODE_INPUT            0x02

#define THI_DDR               DDRB
#define THI_PIN               PINB
#define THI_PORT              PORTB
#define THI_INPUT             0x04

#define SZKOP		      	(1<<PD0)
#define SZKOP_H				PORTD |= SZKOP;
#define SZKOP_L				PORTD &= ~ SZKOP;


/*#define REMOTE_UP             20
#define REMOTE_DOWN           21
#define REMOTE_RIGHT          22
#define REMOTE_LEFT           23
#define REMOTE_MENU           24
#define REMOTE_STOP           30
#define REMOTE_PLAY           31
#define REMOTE_MUTE           32
#define REMOTE_REW            33
#define REMOTE_FFWD           34
#define REMOTE_PAUSE          35
#define REMOTE_POWER          40
#define REMOTE_NO_KEY		  -1*/

#define _asm	asm			/* old style */

#define WDR() 	asm("wdr")
#define SEI()	asm("sei")
#define CLI()	asm("cli")
#define NOP()	asm("nop")
#define _WDR() 	asm("wdr")
#define _SEI()	asm("sei")
#define _CLI()	asm("cli")
#define _NOP()	asm("nop")

// TAVIRANYITO kodok
#define IR_KEY1		0	
#define IR_KEY2		1
#define IR_KEY3		2
#define IR_KEY4		3
#define IR_KEY5		4
#define IR_KEY6		(char)5
#define IR_KEY7		6
#define IR_KEY8		7
#define IR_KEY9		8
#define IR_KEY0		9
#define IR_PROG_P	16
#define IR_PROG_N 	17
#define IR_VOL_P	18
#define IR_VOL_N 	19
#define IR_POWER 	21
#define IR_AV 		37
#define IR_____ 	29

//#define IR_TIME_OUT 1666 // 1 sec
#define IR_TIME_OUT 1800 // 1 sec 1800_default 

//MENU
#define MENU_PROG_P		101
#define MENU_PROG_N 	102
#define MENU_VOL_P		103
#define MENU_VOL_N 		104
#define MENU_POWER 		105

#define MAX_MENU_NUM		12
#define MIN_MENU_NUM		1
#define DEFAULT_MENU_NUM	7

#define MENU_ITEM_NUM_1		1
#define MENU_ITEM_NUM_2		2
#define MENU_ITEM_NUM_3		3
#define MENU_ITEM_NUM_4		4
#define MENU_ITEM_NUM_5		5
#define MENU_ITEM_NUM_6		6
#define MENU_ITEM_NUM_7		7
#define MENU_ITEM_NUM_8		8
#define MENU_ITEM_NUM_9		9
#define MENU_ITEM_NUM_10	10
#define MENU_ITEM_NUM_11	11
#define MENU_ITEM_NUM_12	12

// Speed
#define SPEED_TEMPO_MIN_1	25
#define SPEED_TEMPO_MAX_1	250

#define SPEED_TEMPO_MIN_2	25
#define SPEED_TEMPO_MAX_2	250

#define SPEED_TEMPO_MIN_3	25
#define SPEED_TEMPO_MAX_3	250

#define SPEED_TEMPO_MIN_4	25
#define SPEED_TEMPO_MAX_4	250

#define SPEED_TEMPO_MIN_5	25
#define SPEED_TEMPO_MAX_5	250

#define SPEED_TEMPO_MIN_6	25
#define SPEED_TEMPO_MAX_6	250

#define SPEED_TEMPO_MIN_7	25
#define SPEED_TEMPO_MAX_7	250

#define SPEED_TEMPO_MIN_8	25
#define SPEED_TEMPO_MAX_8	250

#define SPEED_TEMPO_MIN_9	25
#define SPEED_TEMPO_MAX_9	250

#define SPEED_TEMPO_MIN_10	25
#define SPEED_TEMPO_MAX_10	250

#define SPEED_TEMPO_MIN_11	25
#define SPEED_TEMPO_MAX_11	250

#define SPEED_TEMPO_MIN_12	25
#define SPEED_TEMPO_MAX_12	250

// EEprom
#define MENU_ADDR_EE		1
#define BRNESS_ADDR_EE		MENU_ADDR_EE + 3
#define SPEED_ADDR_EE		BRNESS_ADDR_EE + MAX_MENU_NUM + 5

//LED
//#define LED_TIMER_CNT 	150 //60 usec
//#define LED_TIMER_TICK 	255 

//#define LED_TIMER_CNT 	215 //30 usec
//#define LED_TIMER_TICK 	255


#define LED_TIMER_CNT 	226//226 //226 //20 usec
#define LED_TIMER_TICK 	255 // 255_default

//#define LED_TIMER_CNT 	240 //15 usec, rossz
//#define LED_TIMER_TICK 	255

//*****************************************************************************
//                  P R O T O T Y P E
//*****************************************************************************
int main(void);

ushort GetRand(ushort Mask);
int Delay();

void RGBAllDC();
void ClearAll(char port);
void RandomAll();
void RandomEqualAll();
void RandomSmoothAll();
void RGBAll();
void ColorMixAll();
void ColorMix();
void Sablier();
void Chaser1();
void Chaser2();
void Chaser3();
void Chaser4();
void powerOFF();
void setTimer2_50uSec();
uint8_t  read_eeprom(uint16_t  addr_a);
void write_eeprom(uint8_t data_a, uint16_t  addr_a);

void LedSetup(ushort Led,ushort Color);

void ADInit(void);

void LedInit(void);
void LedStartShow(void);
void LedEngine(void);

void IRInit(void);
void IRLightChange(void);
void IRDecode(void);

//*****************************************************************************
//                    G L O B A L   V A R I A B L E
//*****************************************************************************
uint LatchedIrData = 0;
uint RemoteKey = 1;

ushort Led1Red;
ushort Led1Green;
ushort Led1Blue;

ushort Led2Red;
ushort Led2Green;
ushort Led2Blue;

ushort Led3Red;
ushort Led3Green;
ushort Led3Blue;


int Menu, Menu_prev;
int Brightness[MAX_MENU_NUM+2];		// 5-255
int Speed_tempo[MAX_MENU_NUM+2]; 	// 25-255
int Speed_tempo_min[MAX_MENU_NUM+2];
int Speed_tempo_max[MAX_MENU_NUM+2];
int Speed_tempo_step[MAX_MENU_NUM+2];

char power = 1;

//
// IR, Hama Remote Control: TV, SONY code: 0165
// 
enum {
        Idle,
        Start_bit,
        Capture_bit
};

char IR_Current_state = Idle;
unsigned int IR_counter = 0;
unsigned int IR_input_data, IR_bit_count;
char IR_got_data = 0;
char IR_Command_code0,IR_Command_code1;
int IR_Number_counter, IR_flag;
int mtemp = 0;

//----------------------------------
// ms delays
//----------------------------------
void DelayMs(uint16_t ms)
{
	uint16_t i;

	for (i=0;i<ms;i++)
		_delay_ms(1);

}//void DelayMs(uint8_t ms)

 //----------------------------------
 // us delays
 //----------------------------------
void DelayUs(uint16_t us)
{
	uint16_t i;

	for (i=0;i<us;i++)
		_delay_us(1);

}//void DelayUs(uint8_t ms)

/******************************************************************************
Name:         void LedInit(void)
Description:  This function initialize the Led driving
Input:        none
Output:       none
Misc:
******************************************************************************/
void LedInit(void)
{
 // LED1_DDR |= LED1_RED + LED1_GREEN + LED1_BLUE;
 // LED2_DDR |= LED2_RED + LED2_GREEN + LED2_BLUE;
 // LED3_DDR |= LED3_RED + LED3_GREEN + LED3_BLUE;
 

 	DDRA  |= COOLER + LED1_RED + LED4_RED + LED1_GREEN; 
 	DDRC  |= LED2_BLUE + LED5_GREEN + LED2_GREEN + LED5_RED + LED4_BLUE + LED1_BLUE + LED4_GREEN + LED2_RED;
 	DDRD  |= SZKOP + LED3_BLUE + LED3_GREEN + LED3_RED + LED5_BLUE;

        COOLER_H     //ventillator induláskor be
  //Timer2
 // TCCR2 = (1<<CS20);
 // TCCR2 =  (1<<CS00) | (1<<CS01); //start timer frek/64
  TCCR2 =   (1<<CS01); //start timer frek/8
  TCNT2 = LED_TIMER_CNT; //60 usec
  TIMSK |= (1<<TOIE2);
}

//----------------------------------
// Setting Timer1 600 uSec delays
//----------------------------------
void setTimer1_600uSec()
{
 // init timers
 // TIMER0 initialize - prescale:1024
 // desired value: 10 mSec

	TCCR0 = 0x00; //stop 	
 	TCCR0 =  (1<<CS00) | (1<<CS01); //start timer frek/64
 	TIMSK |= (1<<TOIE0); //timer interrupt sources:over

	TCNT0 = 120;

}

//----------------------------------
// Setting Timer2 50 uSec delays
//----------------------------------
void setTimer2_50uSec()
{
// Timer2
// 50 usec
 /* TCNT2 = 150; //set count
OCR2 = 120;
 TCCR2 = (1<<CS21);
  TIFR |= (1<<TOV2); 

 TIMSK |= (1<<TOIE2);*/
  TCCR2 =   (1<<CS01); //start timer frek/8
  TCNT2 = LED_TIMER_CNT; //60 usec
  TIMSK |= (1<<TOIE2);
}


/******************************************************************************
Name:         void IRInit(void)
Description:  Init device for IR functions
Input:        none
Output:       none
Misc:         Share Timer 1 with Triac functions
			  IR, Hama Remote Control: TV, SONY code: 0165	
******************************************************************************/
void IRInit(void)
{ 
  // interrupt on INT1 pin falling edge (sensor triggered) 
  MCUCR = (1<<ISC11) | (0<<ISC10);

  SFIOR = (1<<PUD);

   // interrupt on INT1 pin rising edge (sensor triggered) 
 // MCUCR = (1<<ISC11) | (1<<ISC10);
  // turn on interrupts! 
  GICR   |= (1<<INT1);

// Timer1
// 600 usec
	setTimer1_600uSec();
	IR_Command_code0 = 0xFF;
	IR_Command_code1 = 0xFF;
	IR_Number_counter = 0;
	IR_got_data = 0;
	IR_flag = 0;
	IR_Current_state = Idle;
}

//-------------------------------------
//Interrupt Service Routine for INT1
//-------------------------------------
ISR(INT1_vect)
{
//int ii;
	CLI();
//	SZKOP_L
	if ((IR_flag == 2) || (IR_flag == 1))
	{
		IR_got_data = 0;
		SEI();
		return;
	}
	//	WDR();
	switch (IR_Current_state)
	{
           case Idle:
		   	
               //  INTCON2.INTEDG0 = 1;        //interrupt on rising edge.
			 //   MCUCR = (1<<ISC11) | (1<<ISC10);
                IR_counter = 0;
				IR_bit_count = 0;
                IR_Current_state = Start_bit;
				setTimer1_600uSec();
                break;
           //found the rising edge, check lenght for 2.4ms
           case Start_bit:
		   //	SZKOP_L
                      //correct signal, move on to next state
                if(IR_counter >= 5) 
				{
                 	IR_counter = 0;
                    IR_bit_count = 0;
                    IR_input_data = 0;
                    IR_Current_state = Capture_bit;
                } 
				else 
				{
                    //fault signal, reset to Idle
                   	IR_Current_state = Idle;
               	}
				setTimer1_600uSec();
                break;
           case Capture_bit:
               //check plus length for 0 or 1
			  
               if (IR_counter == 2)
			   {
               		IR_input_data >>= 1;         // add 0 to received data
                    IR_bit_count++;
               }
			   else 
			   {
                	if(IR_counter == 3)
					{
                      IR_input_data >>= 1;
                    //  IR_input_data |= 0x8000;     //add 1 to received data
					  IR_input_data |= 0x4000;     //add 1 to received data
                      IR_bit_count++;
                    } 
					else 
					{
                            //error occurs, reset to Idle state
                          //  INTCON2.INTEDG0 = 0;    //interrupt on falling edge.
						MCUCR = (1<<ISC11) | (0<<ISC10);
                    	IR_Current_state = Idle;
					//	 SZKOP_L
                    }
                }
                      //compleat 12 bit
			//	if(IR_bit_count >= 12)
                if(IR_bit_count >= 11)
				{
				// SZKOP_L
                   IR_got_data = 1;
                   IR_input_data >>= 4;
                   //INTCON2.INTEDG0 = 0;      //interrupt on falling edge.
				    MCUCR = (1<<ISC11) | (0<<ISC10);
                   IR_Current_state = Idle;
                }
                IR_counter = 0;
				setTimer1_600uSec();
               break;
            default: 
			IR_Current_state = Idle;
       }

	   //SZKOP_H
	   SEI();
}

//-------------------------------------
// Interrupt Service Routine for Timer0
//-------------------------------------
ISR (TIMER0_OVF_vect)
{
//	SZKOP_L
	IR_counter++;
    if(IR_counter > 10) //600-nal
//	if(IR_counter > 16)  //200-nal
	{
        IR_Current_state = Idle;
        // IR_counter = 0;
        //interrupt on falling edge.
		MCUCR = (1<<ISC11) | (0<<ISC10);
    }
	if (IR_Number_counter > 0)
		IR_Number_counter--;
	TCNT0 = 120; //reload counter value
//	SZKOP_H
}

//-------------------------------------
// Interrupt Service Routine for Leds
//-------------------------------------
static ushort Ticker = 0;
ISR (TIMER2_OVF_vect)
{ // 50 uSec
	SZKOP_L
	Ticker++;
//	if (Ticker == 255)  //15 usec
	if (Ticker >= LED_TIMER_TICK) 
		Ticker = 0;

	//--------------------------

	if (Led1Red > Ticker)   
		PORTA |= LED1_RED;
	else 
		PORTA &= ~LED1_RED;
	
	if (Led1Green  > Ticker) 
		PORTA |= LED1_GREEN;
	else 
		PORTA &= ~LED1_GREEN;
	
	if (Led1Blue > Ticker)  
		PORTC |= LED1_BLUE;
	else 
		PORTC &= ~LED1_BLUE;

	//--------------------------

	if (Led2Red > Ticker)   
		PORTC |= LED2_RED;
	else 
		PORTC &= ~LED2_RED;
	
	if (Led2Green > Ticker) 
		PORTC |= LED2_GREEN;
	else 
		PORTC &= ~LED2_GREEN;
	
	if (Led2Blue > Ticker)  
		PORTC |= LED2_BLUE;
	else 
		PORTC &= ~LED2_BLUE;	
	
	//--------------------------
		
	if (Led3Red > Ticker)   
		PORTD |= LED3_RED;
	else 
		PORTD &= ~LED3_RED;
	
	if (Led3Green > Ticker) 
	{
		PORTD |= LED3_GREEN;		
	}
	else 
	{
		PORTD &= ~LED3_GREEN;	
	}
	
	if (Led3Blue > Ticker)  
		PORTD |= LED3_BLUE;
	else 
		PORTD &= ~LED3_BLUE;

	SZKOP_H
	TCNT2 = LED_TIMER_CNT;
}

#ifdef TEST1
//------------------------------------------
void test_impulzus(int num)
{
	int ii = 0;
llll
	SZKOP_H

	switch (num)
	{
			case IR_PROG_P:
				DelayMs(1);
				SZKOP_L
				DelayMs(1);
			break;
			case IR_PROG_N:
				for (ii=0; ii<2; ii++)
				{
					DelayMs(1);
					SZKOP_L
					DelayMs(1);
					SZKOP_H
				}
			break;
			case IR_VOL_N:
				for (ii=0; ii<3; ii++)
				{
					DelayMs(1);
					SZKOP_L
					DelayMs(1);
					SZKOP_H
				};
			break;
			case IR_VOL_P:
				for (ii=0; ii<4; ii++)
				{
					DelayMs(1);
					SZKOP_L
					DelayMs(1);
					SZKOP_H
				}
			break;
			case IR_POWER:
				for (ii=0; ii<5; ii++)
				{
					DelayMs(1);
					SZKOP_L
					DelayMs(1);
					SZKOP_H
				}
			break;
			case IR_AV:
				for (ii=0; ii<6; ii++)
				{
					DelayMs(1);
					SZKOP_L
					DelayMs(1);
					SZKOP_H
				}
			break;
			case IR_____:
				for (ii=0; ii<7; ii++)
				{
					DelayMs(1);
					SZKOP_L
					DelayMs(1);
					SZKOP_H
				}
			break;
			default:
			{
				SZKOP_L
				DelayMs(1*num);
				SZKOP_H
			}
	}
	SZKOP_H
}
#endif

//-------------------------------------
// Read elems of menu from inner EEprom
//-------------------------------------
void Menu_from_ee()
{
	uint8_t* ee_addr = (uint8_t*)MENU_ADDR_EE;
	uint8_t eem = eeprom_read_byte(ee_addr);

	if ((eem <= MAX_MENU_NUM) && (eem >= MIN_MENU_NUM)) 
		Menu = (int)eem;
	else
	{
		Menu = DEFAULT_MENU_NUM;
		write_eeprom(DEFAULT_MENU_NUM, MENU_ADDR_EE);
	}
}

//-------------------------------------
// Read values of brightness from inner EEprom
//-------------------------------------
void Brightness_from_ee(int m)
{
	uint8_t* ee_addr = (uint8_t*)(BRNESS_ADDR_EE+m);
	Brightness[m] = (int)eeprom_read_byte(ee_addr);	
	Brightness[m] = (Brightness[m] < 5)?5:Brightness[m];
}

//-------------------------------------
// Read value of speed from inner EEprom
//-------------------------------------
void Speed_from_ee(int m)
{
	uint8_t* ee_addr = (uint8_t*)(SPEED_ADDR_EE+m);
	Speed_tempo[m] = (int)eeprom_read_byte(ee_addr);	
	Speed_tempo[m] = (Speed_tempo[m] < 25)?25:Speed_tempo[m];
}
/*
//-----------------------------------------------------------
void test_briteness(int wait)
{
	static int count = 0;
	static int wait_count = 0;
	static int flag = 0;

	if (wait_count < wait)
		wait_count++;
    else
	{
		Brightness = count*25;
		if (Brightness == 0)
			Brightness = 5;
		if (flag == 0)
		{
			if (count <  10)
				count++;
			else
			{
				count--;
				flag = 1;
			}
		}
		else
		{
			if ( count == 0 )
			{
				count++;
				flag = 0;
			}
			else
			 count--;
		}	
		wait_count = 0;
	}
}*/

//-------------------------------------
// Values, flags of infra remota contol cleared
//-------------------------------------
void IR_Clear(int cl)
{
	if (cl)
		IR_got_data = 0;
	IR_flag = 0;
	IR_Number_counter = 0;
	IR_Command_code0 = 0xFF;
	IR_Command_code1 = 0xFF;
}

//-------------------------------------
// Key menu value of infra remota contol is reading
//-------------------------------------
int getIR_KeyMenu()
{	
	if(IR_got_data)
	{
		if (IR_flag == 0)
		{ // 1. data
			IR_Command_code0 = IR_input_data & 0x7F;
			IR_flag = 1;	
			IR_got_data = 0;
		//	test_impulzus(IR_Command_code0);
			switch (IR_Command_code0)
			{
				case IR_PROG_P:
					{
					//	Menu = MENU_PROG_P;	
						if (Menu >= MAX_MENU_NUM)
							Menu = MIN_MENU_NUM;
						else
						{						
							if ((Menu < MAX_MENU_NUM) && (Menu >= MIN_MENU_NUM)) 
								Menu++;
							else
								Menu = MAX_MENU_NUM;
						}
						IR_Command_code0 = 0xFF;
						return Menu;
					}
					//	break;
				case IR_PROG_N:
					{
						if (Menu <= MIN_MENU_NUM)
							Menu = MAX_MENU_NUM;
						else
						{
							if ((Menu <= MAX_MENU_NUM) && (Menu > MIN_MENU_NUM)) 
								Menu--;
							else
								Menu = MIN_MENU_NUM;
						}
				//	test_impulzus(IR_PROG_N);
						IR_Command_code0 = 0xFF;
					return Menu;
					}
				case IR_AV:// fényerõ -
					{			
						if ((Menu <= MAX_MENU_NUM) && (Menu >= MIN_MENU_NUM)) 
						{		
					//if  ((Brightness < 255) && (Brightness >= 0))
							if  ((Brightness[Menu] < 30))
								Brightness[Menu] = 5;
							else
								Brightness[Menu]  -= 25;
							write_eeprom((uint8_t)Brightness[Menu], BRNESS_ADDR_EE + Menu);
						}
						WDR();
						IR_Command_code0 = 0xFF;
						return 0;
					}
				case IR_____://	fényerõ +	
					{		
						if ((Menu <= MAX_MENU_NUM) && (Menu >= MIN_MENU_NUM)) 
						{	
				//		if ((Brightness <= 255) && (Brightness >= 10))
							if (Brightness[Menu] < 230)
								Brightness[Menu] += 25;
							else
								Brightness[Menu] = 255;
							write_eeprom((uint8_t)Brightness[Menu], BRNESS_ADDR_EE + Menu);
						}
						WDR();
						IR_Command_code0 = 0xFF;
					return 0;
					}
				case IR_VOL_N:		//speed +
			//	case IR_VOL_P:		//speed -
					{
						if ((Menu <= MAX_MENU_NUM) && (Menu >= MIN_MENU_NUM)) 
						{
							if ((Speed_tempo[Menu] >= Speed_tempo_min[Menu]) && (Speed_tempo[Menu] <= Speed_tempo_max[Menu]))
							{
								if (Speed_tempo[Menu] < Speed_tempo_max[Menu])
									Speed_tempo[Menu] += Speed_tempo_step[Menu];
								if (Speed_tempo[Menu] > Speed_tempo_max[Menu])
									Speed_tempo[Menu] = Speed_tempo_max[Menu];
								write_eeprom((uint8_t)Speed_tempo[Menu], SPEED_ADDR_EE + Menu);	
							}
						}
						WDR();					
						IR_Command_code0 = 0xFF;					
						return 0;
					}
				case IR_VOL_P:		//speed -
			//	case IR_VOL_N:		//speed +
					{	
						if ((Menu <= MAX_MENU_NUM) && (Menu >= MIN_MENU_NUM)) 
						{
							if ((Speed_tempo[Menu] >= Speed_tempo_min[Menu]) && (Speed_tempo[Menu] <= Speed_tempo_max[Menu]))
							{
								if (Speed_tempo[Menu] > Speed_tempo_min[Menu])
									Speed_tempo[Menu] -= Speed_tempo_step[Menu];
								if (Speed_tempo[Menu] < Speed_tempo_min[Menu])
									Speed_tempo[Menu] = Speed_tempo_min[Menu];
							/*	if (Speed_tempo[Menu] > 50)
									Speed_tempo[Menu] -= 25;
								else
									Speed_tempo[Menu] = 25;*/
								write_eeprom((uint8_t)Speed_tempo[Menu], SPEED_ADDR_EE + Menu);
							}	
						}	
						WDR();					
						IR_Command_code0 = 0xFF;
						return 0;					
					}
				case IR_POWER:									
					{ // power off
						if (power == 1)
						{
							if ((Menu > 0) && (Menu < 13))
								write_eeprom((uint8_t)Menu, MENU_ADDR_EE);			
							Menu = 222;
						}
						else
						{
							Menu_from_ee();
								WDR();
						}
						IR_Command_code0 = 0xFF;
									
					//	ClearAll(1);
					//	DelayMs(2000);																							
						return Menu;
					}								
				}											
		}	
		else
		if (IR_flag == 3)
		{							
			if (IR_Number_counter == 0 )
			{	
				IR_flag = 6; // 2 byte van
				IR_Command_code1 = IR_input_data & 0x7F;
				IR_got_data = 0;
				IR_Current_state = Idle;	
			//	test_impulzus(5);
			}					
		}
		return 0;			
	}
	else	
	if (IR_flag == 1)
	{
		IR_Number_counter = 291 ; // 160 ms  at 0.55 ms.
		IR_flag = 2;
		setTimer1_600uSec();		
		return 0;
	}
	else
	if (IR_flag == 2)
	{
		if (IR_Number_counter == 0 )
		{
			if (IR_Command_code0 == 0xFF)
			{
				IR_flag = 0;
			}	
			else
			{
				IR_flag = 3;
				IR_Number_counter = IR_TIME_OUT; // 1 sec
									
			}
			IR_Current_state = Idle;
			IR_bit_count = 0;
			setTimer1_600uSec();
		//	test_impulzus(1);
		}
		return 0;
	}
	else
	if (IR_flag == 3)
	{							
		if (IR_Number_counter == 0 )
		{	
			IR_flag = 5; // it is only 1 byte 
			IR_Current_state = Idle;
			//test_impulzus(1);
		}
		return 0;		
	}
	else
	if ((IR_flag == 5) || (IR_flag == 6))
	{
		mtemp = 0;
			
		if (IR_Command_code0 != 0xFF )
		{
		//	test_impulzus(IR_Command_code0);
				switch (IR_Command_code0)
				{				
					case IR_KEY1:
					case IR_KEY2:
					case IR_KEY3:
					case IR_KEY4:
					case IR_KEY5:
					case IR_KEY6:
					case IR_KEY7:
					case IR_KEY8:
					case IR_KEY9:
					case IR_KEY0:
					{
					//	test_impulzus(IR_Command_code0);
						if (IR_Command_code0 == IR_KEY0)
							mtemp = IR_Command_code0-IR_KEY0;
						else
							mtemp = IR_Command_code0+1;
														
						if (IR_flag != 6)
						{	
							if (mtemp != 0)	
							{					
								Menu = mtemp;
								goto main3;
							}
							else
								goto main1;
						}
						break;
					}
						
				}				
		}
	}

	if (IR_flag == 6)
	{ // it is only 2 bytes		
		if (IR_Command_code1 != 0xFF )
		{
		//	test_impulzus(IR_Command_code0);
			switch (IR_Command_code1)
			{
				case IR_KEY1:
				case IR_KEY2:
				case IR_KEY3:
				case IR_KEY4:
				case IR_KEY5:
				case IR_KEY6:
				case IR_KEY7:
				case IR_KEY8:
				case IR_KEY9:
				case IR_KEY0:
				{
					mtemp *= 10;						
					//	test_impulzus(IR_Command_code0);
					if (IR_Command_code1 == IR_KEY0)
						mtemp += IR_Command_code1-IR_KEY0;
					else
						mtemp += IR_Command_code1+1;	
					Menu = mtemp;
					goto main3;				
				//	break;
				}
				default:
					goto main1;	
			}
		}
	}
main1: 
	/*	IR_flag = 0;
		IR_Number_counter = 0;
		IR_Command_code0 = 0xFF;
		IR_Command_code1 = 0xFF;*/
		IR_Clear(0);
		//		SZKOP_H	
		//	setTimer2_50uSec();
		return 0;

main3:
	//test_impulzus(Menu);
	/*	IR_flag = 0;
		IR_Number_counter = 0;
		IR_Command_code0 = 0xFF;
		IR_Command_code1 = 0xFF;*/
		IR_Clear(0);
		return Menu;

	return 0;
}

//*****************************************************************************
//                            M A I N
//*****************************************************************************
int main(void)
{
	WDR(); 
	WDTCR = 0x0f;     // Enable WatchDog at 2.2 sec

	Speed_tempo_min[1]  = SPEED_TEMPO_MIN_1;
	Speed_tempo_max[1]  = SPEED_TEMPO_MAX_1;
 	Speed_tempo_step[1] = (SPEED_TEMPO_MAX_1 - SPEED_TEMPO_MIN_1)/10;
	Speed_tempo_min[2]  = SPEED_TEMPO_MIN_2;
	Speed_tempo_max[2]  = SPEED_TEMPO_MAX_2;
 	Speed_tempo_step[2] = (SPEED_TEMPO_MAX_2 - SPEED_TEMPO_MIN_2)/10;
	Speed_tempo_min[3]  = SPEED_TEMPO_MIN_3;
	Speed_tempo_max[3]  = SPEED_TEMPO_MAX_3;
 	Speed_tempo_step[3] = (SPEED_TEMPO_MAX_3 - SPEED_TEMPO_MIN_3)/10;
	Speed_tempo_min[4]  = SPEED_TEMPO_MIN_4;
	Speed_tempo_max[4]  = SPEED_TEMPO_MAX_4;
 	Speed_tempo_step[4] = (SPEED_TEMPO_MAX_4 - SPEED_TEMPO_MIN_4)/10;
	Speed_tempo_min[5]  = SPEED_TEMPO_MIN_5;
	Speed_tempo_max[5]  = SPEED_TEMPO_MAX_5;
 	Speed_tempo_step[5] = (SPEED_TEMPO_MAX_5 - SPEED_TEMPO_MIN_5)/10;
	Speed_tempo_min[6]  = SPEED_TEMPO_MIN_6;
	Speed_tempo_max[6]  = SPEED_TEMPO_MAX_6;
 	Speed_tempo_step[6] = (SPEED_TEMPO_MAX_6 - SPEED_TEMPO_MIN_6)/10;
	Speed_tempo_min[7]  = SPEED_TEMPO_MIN_7;
	Speed_tempo_max[7]  = SPEED_TEMPO_MAX_7;
 	Speed_tempo_step[7] = (SPEED_TEMPO_MAX_7 - SPEED_TEMPO_MIN_7)/10;
	Speed_tempo_min[8]  = SPEED_TEMPO_MIN_8;
	Speed_tempo_max[8]  = SPEED_TEMPO_MAX_8;
 	Speed_tempo_step[8] = (SPEED_TEMPO_MAX_8 - SPEED_TEMPO_MIN_8)/10 + 1;
	Speed_tempo_min[9]  = SPEED_TEMPO_MIN_9;
	Speed_tempo_max[9]  = SPEED_TEMPO_MAX_9;
 	Speed_tempo_step[9] = (SPEED_TEMPO_MAX_9 - SPEED_TEMPO_MIN_9)/10;
	Speed_tempo_min[10]  = SPEED_TEMPO_MIN_10;
	Speed_tempo_max[10]  = SPEED_TEMPO_MAX_10;
 	Speed_tempo_step[10] = (SPEED_TEMPO_MAX_10 - SPEED_TEMPO_MIN_10)/10;
	Speed_tempo_min[11]  = SPEED_TEMPO_MIN_11;
	Speed_tempo_max[11]  = SPEED_TEMPO_MAX_11;
 	Speed_tempo_step[11] = (SPEED_TEMPO_MAX_11 - SPEED_TEMPO_MIN_11)/10;
	Speed_tempo_min[12]  = SPEED_TEMPO_MIN_12;
	Speed_tempo_max[12]  = SPEED_TEMPO_MAX_12;
 	Speed_tempo_step[12] = (SPEED_TEMPO_MAX_12 - SPEED_TEMPO_MIN_12)/10;

	power = 1;

	Menu_from_ee();
	Menu_prev = Menu;

	for(int ii=1; ii<=MAX_MENU_NUM; ii++)
	{
		Brightness_from_ee(ii);
		Speed_from_ee(ii);
	}

start:

	CLI();
	DDRD = 0;

	IRInit();
	LedInit();
	ADInit();
	IR_Clear(1);
	SEI();

//	Brightness = 255;
//	Menu = 2;
//	Brightness[Menu] = 25;
//	Speed_tempo[Menu] = 25;

	SZKOP_H
	WDR();
	DelayMs(200);
	WDR();
	DelayMs(200);
	IR_Clear(1);	
	WDR();		

	while(TRUE)
	{
		WDR();	
		DelayMs(1);		
		getIR_KeyMenu();
			
		if ((Menu > 0) && (Menu < 13))
			COOLER_H

		switch (Menu)
		{
			case  1: 				
		 	RandomAll();							
			break;
		 	case  2: 		
			RandomEqualAll();
			break;
		 	case  3:  			
			RandomSmoothAll();
		//	Chaser3();
			break;
		 	case  4:			
			RGBAll();
			break;
		 	case  5: 			 
			ColorMixAll();
			break;
		 	case  6:		 
			ColorMix();
			break;
		 	case  7: 		
			Sablier();
			break;
			case  8:  		
			RGBAllDC();
			break;
			case  9: 		
			Chaser1();
			break;
			case  10: 			
			Chaser2();
			break;
			case  11:
		//	RandomSmoothAll();	
			Chaser3();
			break;
			case  12:		
			Chaser4();
			break;
			case  222:					
			powerOFF();			
			break;
		}
		if ((Menu > 0) && (Menu < 13))
		{
			ClearAll(0);
//			ADInit();
			if (Menu != Menu_prev)
			{
				Menu_prev = Menu;
				write_eeprom((uint8_t)Menu, MENU_ADDR_EE);

			}			
			goto start;
		}
		else
		{
			ClearAll(0);
			if (Menu != 222)
				Menu = Menu_prev;
		//	DelayMs(10);
		}
	//	DelayMs(25);
		WDR();       
	}
	return 1;
}

//-------------------------------------
// Flags of Leds cleared
//-------------------------------------
void ClearAll(char port)
{
	Led1Red =   0;
	Led1Green = 0;
	Led1Blue =  0;
	
	Led2Red = 	0;
	Led2Green = 0;
	Led2Blue =  0;
	
	Led3Red = 	0;
	Led3Green = 0;
	Led3Blue =  0;

	if (port)
	{
		PORTA &= ~LED1_RED;
		PORTA &= ~LED1_GREEN;
		PORTC &= ~LED1_BLUE;
		PORTC &= ~LED2_RED;
		PORTC &= ~LED2_GREEN;
		PORTC &= ~LED2_BLUE;
		PORTD &= ~LED3_RED;
		PORTD &= ~LED3_GREEN;
		PORTD &= ~LED3_BLUE;
		PORTA &= ~LED4_RED;
		PORTC &= ~LED4_GREEN;
		PORTC &= ~LED4_BLUE;
		PORTC &= ~LED5_RED;
		PORTC &= ~LED5_GREEN;
		PORTD &= ~LED5_BLUE;
	}
}

#define DELAY1_TIME 2500 // 2000 uSec
//-------------------------------------
// Delay1 function
//-------------------------------------
int Delay1()
{
	for (int ii =0; ii<Speed_tempo[MENU_ITEM_NUM_1]; ii++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
		//DelayMs(1);
		DelayUs(DELAY1_TIME);
	}
//test_briteness(50);

	return 0;	
}

//-------------------------------------
// Random all function
//-------------------------------------
void RandomAll()
{
//  TEST START
//	Brightness = 5;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;
//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END

	while(TRUE)
	{
		WDR();
		
		Led1Red =   GetRand(Brightness[MENU_ITEM_NUM_1]);
		Led1Green = GetRand(Brightness[MENU_ITEM_NUM_1]);
		Led1Blue =  GetRand(Brightness[MENU_ITEM_NUM_1]);
				
		Led2Red =   Led1Red;
		Led2Green = Led1Green;
		Led2Blue =  Led1Blue;
		
		Led3Red =   Led1Red;
		Led3Green = Led1Green;
		Led3Blue =  Led1Blue;	
			
		if (Delay1()>0)
			return;
	}	
}

#define DELAY2_TIME 80 // 100 uSec
//-------------------------------------
// Delay2 function
//-------------------------------------
int Delay2()
{
	for (int i=0;i<Speed_tempo[MENU_ITEM_NUM_2];i++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
	//	DelayMs(1);
		DelayUs(DELAY2_TIME);
	}
//test_briteness(50);
	return 0;	
}

//-------------------------------------
// Random is all equal  function
//------------------------------------- 
void RandomEqualAll()
{
	ushort First = TRUE;
	int i;

//  TEST START
//	Brightness = 5;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;


//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END
    
	while(TRUE)
	{
		WDR();

		if (First)
		{
			First = FALSE;
			if (Brightness[MENU_ITEM_NUM_2] < 20)
				Brightness[MENU_ITEM_NUM_2] = 20;
			for (i=10;i<Brightness[MENU_ITEM_NUM_2];i++)
			{
				if (Delay2()>0)
					return;
				
				Led1Red = i;
				Led2Red = i;
				Led3Red = i;
			}
		}
		if (Brightness[MENU_ITEM_NUM_2] < 20)
				Brightness[MENU_ITEM_NUM_2] = 20;
		for (i=10;i<Brightness[MENU_ITEM_NUM_2];i++)
		{
			if (Delay2()>0)
				return;

			Led1Green = i;
			Led2Green = i;
			Led3Green = i;
		}
		if (Brightness[MENU_ITEM_NUM_2] < 20)
				Brightness[MENU_ITEM_NUM_2] = 20;
		for (i=Brightness[MENU_ITEM_NUM_2];i!=10;i--)
		{
			if (Delay2()>0)
				return;

			Led1Red = i;
			Led2Red = i;
			Led3Red = i;
		}
		if (Brightness[MENU_ITEM_NUM_2] < 20)
				Brightness[MENU_ITEM_NUM_2] = 20;
		for (i=10;i<Brightness[MENU_ITEM_NUM_2];i++)
		{
			if (Delay2()>0)
				return;

			Led1Blue = i;
			Led2Blue = i;
			Led3Blue = i;
		}
		if (Brightness[MENU_ITEM_NUM_2] < 20)
				Brightness[MENU_ITEM_NUM_2] = 20;
		for (i=Brightness[MENU_ITEM_NUM_2];i!=10;i--)
		{
			if (Delay2()>0)
				return;;

			Led1Green = i;
			Led2Green = i;
			Led3Green = i;
		}
		if (Brightness[MENU_ITEM_NUM_2] < 20)
				Brightness[MENU_ITEM_NUM_2] = 20;
		for (i=10;i<Brightness[MENU_ITEM_NUM_2];i++)
		{
			if (Delay2()>0)
				return;; 

			Led1Red = i;
			Led2Red = i;
			Led3Red = i;
		}
		if (Brightness[MENU_ITEM_NUM_2] < 20)
				Brightness[MENU_ITEM_NUM_2] = 20;
		for (i=Brightness[MENU_ITEM_NUM_2];i!=10;i--)
		{
			if (Delay2()>0)
				return;

			Led1Blue = i;
			Led2Blue = i;
			Led3Blue = i;
		}
	}

}

#define DELAY3_TIME 80 // 500 uSec
//-------------------------------------
// Delay3 function
//-------------------------------------
int Delay3()
{
	for (int i=0;i<Speed_tempo[MENU_ITEM_NUM_3];i++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
	//	DelayMs(1);
		DelayUs(DELAY3_TIME);
	}
//test_briteness(50);
	return 0;	
}

//-------------------------------------
// Random is all smooth function
//------------------------------------- 
void RandomSmoothAll()
{
	ushort RndRed = 0x7f;
	ushort RndGreen = 0x30;
	ushort RndBlue = 0x90;

//  TEST START
//	Brightness = 5;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;
//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END

	while(TRUE)
	{
		WDR();

		if ((Led1Red > RndRed) && Led1Red)
			Led1Red--;
		if ((Led1Red < RndRed) && (Led1Red<Brightness[MENU_ITEM_NUM_3]))
			Led1Red++;
		if (Led1Red == RndRed) 
			RndRed = GetRand(Brightness[MENU_ITEM_NUM_3]);

		if ((Led1Green > RndGreen) && Led1Green)
			Led1Green--;
		if ((Led1Green < RndGreen) && (Led1Green<Brightness[MENU_ITEM_NUM_3]))
			Led1Green++;
		if (Led1Green == RndGreen)
			RndGreen = GetRand(Brightness[MENU_ITEM_NUM_3]);

		if ((Led1Blue > RndBlue)  && Led1Blue)
			Led1Blue--;
		if ((Led1Blue < RndBlue) && (Led1Blue<Brightness[MENU_ITEM_NUM_3]))
			Led1Blue++;
		if (Led1Blue == RndBlue) 
			RndBlue = GetRand(Brightness[MENU_ITEM_NUM_3]);

		Led2Red = 	Led1Red;
		Led2Green = Led1Green;
		Led2Blue =  Led1Blue;
		
		Led3Red = 	Led1Red;
		Led3Green = Led1Green;
		Led3Blue =  Led1Blue;		

		if (Delay3()>0)
			return;
	}
	return;
}

#define DELAY4_TIME 80 // 150 uSec
//-------------------------------------
// Delay4 function
//-------------------------------------
int Delay4()
{
	for (int ii =0; ii<Speed_tempo[MENU_ITEM_NUM_4]; ii++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
		WDR();
		//DelayMs(1);
		DelayUs(DELAY4_TIME);
	}
//test_briteness(50);

	return 0;	
}

//-------------------------------------
// RGBA is all function
//------------------------------------- 
void RGBAll()
{
	short RedAdd = 1;
	short GreenAdd = 1;
	short BlueAdd = 1;


//  TEST START
//	Brightness = 5;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;


//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END

	while(TRUE)
	{
		WDR();
		if (Brightness[MENU_ITEM_NUM_4] <25)
			Brightness[MENU_ITEM_NUM_4] = 25;
	//	if (Speed_tempo[MENU_ITEM_NUM_4] <100)
	//		Speed_tempo[MENU_ITEM_NUM_4] = 50;
		Led1Red += RedAdd;
		if (Led1Red > Brightness[MENU_ITEM_NUM_4]) 
			RedAdd = -1;
		if (Led1Red == 0x00) 
			RedAdd = 0;

		if (RedAdd == 0)
		{
			Led1Green += GreenAdd;
			if (Led1Green > Brightness[MENU_ITEM_NUM_4]) 
				GreenAdd = -1;
			if (Led1Green == 0x00) 
				GreenAdd = 0;
		}

		if (GreenAdd == 0)
		{
			Led1Blue += BlueAdd;
			if (Led1Blue > Brightness[MENU_ITEM_NUM_4]) 
				BlueAdd = -1;
			if (Led1Blue == 0x00) 
				BlueAdd = 0;
		}

		if ((RedAdd == 0) && (GreenAdd == 0) && (BlueAdd == 0))
		{
			RedAdd = 1;
			GreenAdd = 1;
			BlueAdd = 1;
		}
		WDR();
		Led2Red =   Led1Red;
		Led2Green = Led1Green;
		Led2Blue =  Led1Blue;

		Led3Red = 	Led1Red;
		Led3Green = Led1Green;
		Led3Blue =  Led1Blue;		
		
		if (Delay4()>0)
			return;
	}
}


#define DELAY5_TIME 80 // 100 uSec
//-------------------------------------
// Delay5 function
//-------------------------------------
int Delay5()
{
	for (int ii =0; ii<Speed_tempo[MENU_ITEM_NUM_5]; ii++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
		WDR();
		DelayUs(DELAY5_TIME);
	}
//test_briteness(50);

	return 0;	
}

//-------------------------------------
// Color mix is all function
//------------------------------------- 
void ColorMixAll()
{
	short RedAdd = 0;
	short GreenAdd = 0;
	short BlueAdd = 0;
	ushort i = 0;


//  TEST START
//	Brightness = 5;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;


//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END

	while(TRUE)
	{
		WDR();

	//	if (Brightness[MENU_ITEM_NUM_5] < 25)
	//		Brightness[MENU_ITEM_NUM_5] = 25;

		i = GetRand(0x07);

		if ((i & 0x01) == 0x01)
		{
			Led1Red += RedAdd;
			if (Led1Red > Brightness[MENU_ITEM_NUM_5]) 
				RedAdd = -1;
			if (Led1Red == 0x00) 
				RedAdd = 1;
		}

		if ((i & 0x02) == 0x02)
		{
			Led1Green += GreenAdd;
			if (Led1Green  > Brightness[MENU_ITEM_NUM_5]) 
				GreenAdd = -1;
			if (Led1Green == 0x00) 
				GreenAdd = 1;
		}

		if ((i & 0x04) == 0x04)
		{
			Led1Blue += BlueAdd;
			if (Led1Blue > Brightness[MENU_ITEM_NUM_5]) 
				BlueAdd = -1;
			if (Led1Blue == 0x00) 
				BlueAdd = 1;
		}

		if (i == 0x07) 
			i = 1;

		Led2Red =   Led1Red;
		Led2Green = Led1Green;
		Led2Blue =  Led1Blue;
		
		Led3Red = 	Led1Red;
		Led3Green = Led1Green;
		Led3Blue =  Led1Blue;
		
		if (Delay5()>0)
			return;
	}
}

#define DELAY6_TIME 80 // 400 uSec
//-------------------------------------
// Delay6 function
//-------------------------------------
int Delay6()
{
	for (int ii =0; ii<Speed_tempo[MENU_ITEM_NUM_6]; ii++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
		WDR();
		//DelayMs(1);
		DelayUs(DELAY6_TIME);
	}
//test_briteness(50);
	return 0;	
}

//-------------------------------------
// Color mix is function
//------------------------------------- 
void ColorMix()
{
	
//  TEST START
//	Brightness = 25;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;


//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END

	ushort RndRed1 = 0x7f;
	ushort RndGreen1 = 0x30;
	ushort RndBlue1 = 0x90;
	
	ushort RndRed2 = 0x10;
	ushort RndGreen2 = 0x30;
	ushort RndBlue2 = 0xe0;	
	
	ushort RndRed3 = 0xf0;
	ushort RndGreen3 = 0x03;
	ushort RndBlue3 = 0x70;	
/*	
	ushort RndRed4 = 0x55;
	ushort RndGreen4 = 0x45;
	ushort RndBlue4 = 0x10;	
	
	ushort RndRed5 = 0x80;
	ushort RndGreen5 = 0xf3;
	ushort RndBlue5 = 0xaa;	

	VariableDelay = DelayTick;
	VariableDelayMin = DelayMin;
	VariableDelayStep = DelayStep;*/

	while(TRUE)
	{
		WDR();

		if (Led1Red > RndRed1) Led1Red--;
		if (Led1Red < RndRed1) Led1Red++;
		if (Led1Red == RndRed1) RndRed1 = GetRand(Brightness[MENU_ITEM_NUM_6]);

		if (Led1Green > RndGreen1) Led1Green--;
		if (Led1Green < RndGreen1) Led1Green++;
		if (Led1Green == RndGreen1) RndGreen1 = GetRand(Brightness[MENU_ITEM_NUM_6]);

		if (Led1Blue > RndBlue1) Led1Blue--;
		if (Led1Blue < RndBlue1) Led1Blue++;
		if (Led1Blue == RndBlue1) RndBlue1 = GetRand(Brightness[MENU_ITEM_NUM_6]);
		
		//---
		
		if (Led2Red > RndRed2) Led2Red--;
		if (Led2Red < RndRed2) Led2Red++;
		if (Led2Red == RndRed2) RndRed2 = GetRand(Brightness[MENU_ITEM_NUM_6]);

		if (Led2Green > RndGreen2) Led2Green--;
		if (Led2Green < RndGreen2) Led2Green++;
		if (Led2Green == RndGreen2) RndGreen2 = GetRand(Brightness[MENU_ITEM_NUM_6]);

		if (Led2Blue > RndBlue2) Led2Blue--;
		if (Led2Blue < RndBlue2) Led2Blue++;
		if (Led2Blue == RndBlue2) RndBlue2 = GetRand(Brightness[MENU_ITEM_NUM_6]);		

		//---
		
		if (Led3Red > RndRed3) Led3Red--;
		if (Led3Red < RndRed3) Led3Red++;
		if (Led3Red == RndRed3) RndRed3 = GetRand(Brightness[MENU_ITEM_NUM_6]);

		if (Led3Green > RndGreen3) Led3Green--;
		if (Led3Green < RndGreen3) Led3Green++;
		if (Led3Green == RndGreen3) RndGreen3 = GetRand(Brightness[MENU_ITEM_NUM_6]);

		if (Led3Blue > RndBlue3) Led3Blue--;
		if (Led3Blue < RndBlue3) Led3Blue++;
		if (Led3Blue == RndBlue3) RndBlue3 = GetRand(Brightness[MENU_ITEM_NUM_6]);	
		
		//---
		
/*		if (Led4Red > RndRed4) Led4Red--;
		if (Led4Red < RndRed4) Led4Red++;
		if (Led4Red == RndRed4) RndRed4 = GetRand(Brightness[MENU_ITEM_NUM_6]);

		if (Led4Green > RndGreen4) Led4Green--;
		if (Led4Green < RndGreen4) Led4Green++;
		if (Led4Green == RndGreen4) RndGreen4 = GetRand(Brightness[MENU_ITEM_NUM_6]);

		if (Led4Blue > RndBlue4) Led4Blue--;
		if (Led4Blue < RndBlue4) Led4Blue++;
		if (Led4Blue == RndBlue4) RndBlue4 = GetRand(Brightness[MENU_ITEM_NUM_6]);
		
		//---
		
		if (Led5Red > RndRed5) Led5Red--;
		if (Led5Red < RndRed5) Led5Red++;
		if (Led5Red == RndRed5) RndRed5 = GetRand(Brightness[MENU_ITEM_NUM_6]);

		if (Led5Green > RndGreen5) Led5Green--;
		if (Led5Green < RndGreen5) Led5Green++;
		if (Led5Green == RndGreen5) RndGreen5 = GetRand(Brightness[MENU_ITEM_NUM_6]);

		if (Led5Blue > RndBlue5) Led5Blue--;
		if (Led5Blue < RndBlue5) Led5Blue++;
		if (Led5Blue == RndBlue5) RndBlue5 = GetRand(Brightness[MENU_ITEM_NUM_6]);	*/				

		if (Delay6()>0)
			return;
	}
}

#define DELAY7_TIME_COUNT 8 // counter number
//-------------------------------------
// Delay7 function
//-------------------------------------
int Delay7()
{
	for (int ii =0; ii<DELAY7_TIME_COUNT; ii++)
	{
	WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
	WDR();
		DelayMs(1);
	}
//test_briteness(50);

	return 0;	
}
//0.5 - 5 sec

//-------------------------------------
// Helper of Sablier is function
//------------------------------------- 
int Sablier_helper(ushort* LEDcurrent, ushort* LEDprev)
{
		ushort i,e,b;
		b = 0;
		for (i=0;i<Speed_tempo[MENU_ITEM_NUM_7];i++)
		{
			if (Delay7()>0)
				return 1;

			if (Speed_tempo[Menu] <= Brightness[MENU_ITEM_NUM_7])
			{
				for (e=0;e<(Brightness[MENU_ITEM_NUM_7]/Speed_tempo[MENU_ITEM_NUM_7]);e++)
				{
					if (b <= Brightness[MENU_ITEM_NUM_7] )
					{
						*LEDcurrent = b;
						b++;
					}
					if (LEDprev)
					{			
						if (*LEDprev)			
							(*LEDprev)--;	
					}				
				}
			}
			else
			{
				if (!(i%(Speed_tempo[Menu]/Brightness[MENU_ITEM_NUM_7])))
				{
					if (b <= Brightness[MENU_ITEM_NUM_7] )
					{
						*LEDcurrent = b;
						b++;
					}
					if (LEDprev)
					{			
						if (*LEDprev)			
							(*LEDprev)--;	
					}						
				}
			}							
		}
		return 0;
}

//-------------------------------------
// Sablier is function
//------------------------------------- 
void Sablier()
{
	ushort First = TRUE;

//  TEST START
//	Brightness = 25;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;


//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END

	while(TRUE)
	{
		WDR();
		
		if (First)
		{ //
			Sablier_helper(&Led1Red, NULL);
			First = FALSE;
		}
	//	Led2Red = 0;
		if (Sablier_helper(&Led2Red, &Led1Red)>0)
			return;
		Led1Red = 0;
		if (Sablier_helper(&Led3Red, &Led2Red)>0)
			return;
		Led2Red = 0;
		if (Sablier_helper(&Led1Green, &Led3Red)>0)
			return;
		Led3Red = 0;
		if (Sablier_helper(&Led2Green, &Led1Green)>0)
			return;
		Led1Green = 0;
		if (Sablier_helper(&Led3Green, &Led2Green)>0)	
			return;	
		Led2Green = 0;
		if (Sablier_helper(&Led1Blue, &Led3Green)>0)	
			return;
		Led3Green = 0;
		if (Sablier_helper(&Led2Blue, &Led1Blue)>0)	
			return;
		Led1Blue = 0;
		if (Sablier_helper(&Led3Blue, &Led2Blue)>0)
			return;
		Led2Blue = 0;
		if (Sablier_helper(&Led1Red, &Led3Blue)>0)
			return;
		Led3Blue = 0;
	}
}

#define DELAY8_TIME 1700 // 750 uSec
//-------------------------------------
// Delay8 function
//-------------------------------------
int Delay8()
{
	for (int ii =0; ii<Speed_tempo[MENU_ITEM_NUM_8]; ii++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
	//	WDR();
		//DelayMs(1);
		DelayUs(DELAY8_TIME);
	}
	return 0;	
}

//-------------------------------------
// RGB All DC is function
//------------------------------------- 
void RGBAllDC()
{
//  TEST START
//	Brightness = 25;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;

//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END
	
	while(TRUE)
	{
		WDR();
		
		if (Delay8()>0)
			return;
		
		LedSetup(1,RED);
		LedSetup(2,RED);
		LedSetup(3,RED);
		
		if (Delay8()>0)
			return;
		
		LedSetup(1,GREEN);
		LedSetup(2,GREEN);
		LedSetup(3,GREEN);
		
		if (Delay8()>0)
			return;
		
		LedSetup(1,BLUE);
		LedSetup(2,BLUE);
		LedSetup(3,BLUE);				
	//	if (Delay8()>0)
	//		return;	
	}
}

#define DELAY9_TIME 1700 // 750 uSec
//-------------------------------------
// Delay9 function
//-------------------------------------
int Delay9()
{
	for (int ii =0; ii<Speed_tempo[MENU_ITEM_NUM_9]; ii++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
				WDR();
		//DelayMs(1);
		DelayUs(DELAY9_TIME);
	}
//test_briteness(50);

	return 0;	
}

//-------------------------------------
// Chaser1 is function
//------------------------------------- 
void Chaser1()
{

//  TEST START
//	Brightness = 25;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;
//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END

	while(TRUE)
	{
		WDR();
		
		LedSetup(1,GREEN);
		LedSetup(2,RED);
		LedSetup(3,RED);

		if (Delay9()>0)
			return;

		LedSetup(1,GREEN);
		LedSetup(2,GREEN);
		LedSetup(3,RED);

		if (Delay9()>0)
			return;

		LedSetup(1,GREEN);
		LedSetup(2,GREEN);
		LedSetup(3,GREEN);

		if (Delay9()>0)
			return;

		LedSetup(1,BLUE);
		LedSetup(2,GREEN);
		LedSetup(3,GREEN);

		if (Delay9()>0)
			return;

		LedSetup(1,BLUE);
		LedSetup(2,BLUE);
		LedSetup(3,GREEN);

		if (Delay9()>0)
			return;
	
		LedSetup(1,BLUE);
		LedSetup(2,BLUE);
		LedSetup(3,BLUE);

		if (Delay9()>0)
			return;

		LedSetup(1,RED);
		LedSetup(2,BLUE);
		LedSetup(3,BLUE);

		if (Delay9()>0)
			return;

		LedSetup(1,RED);
		LedSetup(2,RED);
		LedSetup(3,BLUE);

		if (Delay9()>0)
			return;

		LedSetup(1,RED);
		LedSetup(2,RED);
		LedSetup(3,RED);

		if (Delay9()>0)
			return;		
	}
	return;
}

#define DELAY10_TIME 1700 // 750 uSec
//-------------------------------------
// Delay10 function
//-------------------------------------
int Delay10()
{
	for (int ii =0; ii<Speed_tempo[MENU_ITEM_NUM_10]; ii++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
		//DelayMs(1);
		DelayUs(DELAY10_TIME);
	}
//test_briteness(50);

	return 0;	
}

//-------------------------------------
// Chaser2 is function
//------------------------------------- 
void Chaser2()
{
	int i;

//  TEST START
//	Brightness = 25;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;


//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END

	while(TRUE)
	{
		WDR();
		
		for (i=0;i<5;i++)
		{
			LedSetup(1,GREEN);
			LedSetup(2,RED);
			LedSetup(3,RED);

			if (Delay10()>0)
				return;

			LedSetup(1,RED);
			LedSetup(2,GREEN);
			LedSetup(3,RED);

			if (Delay10()>0)
				return;

			LedSetup(1,RED);
			LedSetup(2,RED);
			LedSetup(3,GREEN);

			if (Delay10()>0)
				return;
		}

		for (i=0;i<5;i++)
		{
			LedSetup(1,BLUE);
			LedSetup(2,GREEN);
			LedSetup(3,GREEN);
		
			if (Delay10()>0)
				return;

			LedSetup(1,GREEN);
			LedSetup(2,BLUE);
			LedSetup(3,GREEN);

			if (Delay10()>0)
				return;

			LedSetup(1,GREEN);
			LedSetup(2,GREEN);
			LedSetup(3,BLUE);

			if (Delay10()>0)
				return;
		}

		for (i=0;i<5;i++)
		{
			LedSetup(1,RED);
			LedSetup(2,BLUE);
			LedSetup(3,BLUE);

			if (Delay10()>0)
				return;

			LedSetup(1,BLUE);
			LedSetup(2,RED);
			LedSetup(3,BLUE);

			if (Delay10()>0)
				return;

			LedSetup(1,BLUE);
			LedSetup(2,BLUE);
			LedSetup(3,RED);

			if (Delay10()>0)
				return;
		}
	}
	return;
}

#define DELAY11_TIME 1700 // 750 uSec
//-------------------------------------
// Delay11 function
//-------------------------------------
int Delay11()
{
	for (int ii =0; ii<Speed_tempo[MENU_ITEM_NUM_11]; ii++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
		//DelayMs(1);
		DelayUs(DELAY11_TIME);
	}
//test_briteness(50);

	return 0;	
}

//-------------------------------------
// Chaser3 is function
//------------------------------------- 
void Chaser3()
{
	int i;

//  TEST START
//	Brightness = 25;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;
//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END

	while(TRUE)
	{
		WDR();
		
		for (i=0;i<5;i++)
		{
			LedSetup(3,OFF);
			LedSetup(1,GREEN);

			if (Delay11()>0)
				return;

			LedSetup(1,OFF);
			LedSetup(2,GREEN);

			if (Delay11()>0)
				return;
            
			LedSetup(2,OFF);
			LedSetup(3,GREEN);

			if (Delay11()>0)
				return;
		}

		for (i=0;i<5;i++)
		{
			LedSetup(3,OFF);
			LedSetup(1,BLUE);

			if (Delay11()>0)
				return;

			LedSetup(1,OFF);
			LedSetup(2,BLUE);

			if (Delay11()>0)
				return;

			LedSetup(2,OFF);
			LedSetup(3,BLUE);

			if (Delay11()>0)
				return;
		}

		for (i=0;i<5;i++)
		{
			LedSetup(3,OFF);
			LedSetup(1,RED);

			if (Delay11()>0)
				return;

			LedSetup(1,OFF);
			LedSetup(2,RED);

			if (Delay11()>0)
				return;

			LedSetup(2,OFF);
			LedSetup(3,RED);

			if (Delay11()>0)
				return;
		}
	}
	return;
}

#define DELAY12_TIME 1700 // 750 uSec
//-------------------------------------
// Delay12 function
//-------------------------------------
int Delay12()
{
	for (int ii =0; ii<Speed_tempo[MENU_ITEM_NUM_12]; ii++)
	{
		WDR();
		if (getIR_KeyMenu() > 0)
			return 1;
		//DelayMs(1);
		DelayUs(DELAY12_TIME);
	}
	return 0;
//test_briteness(50);
}

//-------------------------------------
// Chaser4 is function
//------------------------------------- 
void Chaser4()
{
	int i;

//  TEST START
//	Brightness = 25;
//	Brightness = 50;
//	Brightness = 150;
//	Brightness = 255;
//	Speed_tempo = 25;
//	Speed_tempo = 50;
//	Speed_tempo = 150;
//	Speed_tempo = 255;
//  TEST END

	while(TRUE)
	{
		WDR();
		
		for (i=0;i<5;i++)
		{
			LedSetup(1,RED);
			LedSetup(2,OFF);
			LedSetup(3,OFF);

			if (Delay12()>0)
				return;

			LedSetup(1,OFF);
			LedSetup(2,RED);
			LedSetup(3,OFF);

			if (Delay12()>0)
				return;

			LedSetup(1,OFF);
			LedSetup(2,OFF);
			LedSetup(3,RED);

			if (Delay12()>0)
				return;
			
			LedSetup(1,OFF);
			LedSetup(2,RED);
			LedSetup(3,OFF);

			if (Delay12()>0)
				return;
		}

		for (i=0;i<5;i++)
		{
			LedSetup(1,GREEN);
			LedSetup(2,OFF);
			LedSetup(3,OFF);

			if (Delay12()>0)
				return;

			LedSetup(1,OFF);
			LedSetup(2,GREEN);
			LedSetup(3,OFF);

			if (Delay12()>0)
				return;;

			LedSetup(1,OFF);
			LedSetup(2,OFF);
			LedSetup(3,GREEN);

			if (Delay12()>0)
				return;
			
			LedSetup(1,OFF);
			LedSetup(2,GREEN);
			LedSetup(3,OFF);

			if (Delay12()>0)
				return;
		}

		for (i=0;i<5;i++)
		{
			LedSetup(1,BLUE);
			LedSetup(2,OFF);
			LedSetup(3,OFF);

			if (Delay12()>0)
				return;

			LedSetup(1,OFF);
			LedSetup(2,BLUE);
			LedSetup(3,OFF);

			if (Delay12()>0)
				return;

			LedSetup(1,OFF);
			LedSetup(2,OFF);
			LedSetup(3,BLUE);
			
			if (Delay12()>0)
				return;
			
			LedSetup(1,OFF);
			LedSetup(2,BLUE);
			LedSetup(3,OFF);

			if (Delay12()>0)
				return;
		}
	}
	return;
}

//-------------------------------------
// Power OFF is function
//------------------------------------- 
void powerOFF()
{
int res,ii;
	ClearAll(1);
//	Led1Red = 1;
	for (ii = 0; ii<500;ii++)
	{
		DelayMs(1);
		WDR();
	}
	power = 0;
	while(TRUE)
	{
		WDR();	
		ClearAll(1);
		COOLER_L
		
		res = getIR_KeyMenu();
		if (res > 0)
		{
		//	if (res == 15)
		//	{				
				WDR();
		//		Led1Red = 1;
				for (ii = 0; ii<200;ii++)
				{			
					DelayMs(1);
					WDR();
				}
				if (res == 222)
					IR_Clear(1);
		//	}
			power = 1;
			return;			
		}	
		DelayMs(1);
	}
}

//-------------------------------------
// One led setting is function
//------------------------------------- 
void LedSetup(ushort Led,ushort Color)
{
	WDR();
	if ((Led == 1) && (Color == OFF))   
	{
		Led1Red = 0; 
		Led1Green = 0; 
		Led1Blue = 0;
	}
	if ((Led == 1) && (Color == RED))   
	{
		Led1Red = Brightness[Menu]; 
		Led1Green = 0; 
		Led1Blue = 0;
	}
	if ((Led == 1) && (Color == GREEN)) 
	{
		Led1Red = 0; 
		Led1Green = Brightness[Menu]; 
		Led1Blue = 0;
	}
	if ((Led == 1) && (Color == BLUE))  
	{
		Led1Red = 0; 
		Led1Green = 0; 
		Led1Blue = Brightness[Menu];
	}
	
	if ((Led == 2) && (Color == OFF))   
	{
		Led2Red = 0; 
		Led2Green = 0; 
		Led2Blue = 0;
	}
	if ((Led == 2) && (Color == RED))   
	{
		Led2Red = Brightness[Menu]; 
		Led2Green = 0; 
		Led2Blue = 0;
	}
	if ((Led == 2) && (Color == GREEN)) 
	{
		Led2Red = 0; 
		Led2Green = Brightness[Menu]; 
		Led2Blue = 0;
	}
	if ((Led == 2) && (Color == BLUE))  
	{
		Led2Red = 0; 
		Led2Green = 0; 
		Led2Blue = Brightness[Menu];
	}
	
	if ((Led == 3) && (Color == OFF))   
	{
		Led3Red = 0; 
		Led3Green = 0; 
		Led3Blue = 0;
	}
	if ((Led == 3) && (Color == RED))   
	{
		Led3Red = Brightness[Menu]; 
		Led3Green = 0; 
		Led3Blue = 0;
	}
	if ((Led == 3) && (Color == GREEN)) 
	{
		Led3Red = 0; 
		Led3Green = Brightness[Menu]; 
		Led3Blue = 0;
	}
	if ((Led == 3) && (Color == BLUE))  
	{
		Led3Red = 0; 
		Led3Green = 0; 
		Led3Blue = Brightness[Menu];
	}		
}

//-------------------------------------
// get value, before random number process is runned ... function
//------------------------------------- 
//#define NEW_RANDOM 1

#ifdef NEW_RANDOM
ushort GetRand(ushort Mask)
{kkk
//	srand(ADC);
//	ushort n = (ushort)((double)rand() / ((double)rmax + 1) * Mask) ;
	ushort n = (ushort)(rand()) % Mask ;
	return n;

}
#else
ushort GetRand(ushort Mask)
{
	static ushort x;
	uint y;

	Mask = (Mask == 0) ? 1 : Mask;
    
	x = x + (ADC * ADC) + (ADC + (x<<3));
    y = ((x & 0xff) * Mask) / 255;
    return (ushort)y;
}
#endif

//-------------------------------------
// AD is init function
//------------------------------------- 
void ADInit(void)
{
	ADMUX = (1<<REFS1) + (1<<REFS0) + 0x0b;
	ADCSRA = (1<<ADEN) + (1<<ADATE) + (1<<ADPS1) + (1<<ADPS0);
 	ADCSRA |= (1<<ADSC);
}

//-------------------------------------
// Saving one data to inner Epprom
//-------------------------------------
void write_eeprom(uint8_t data_a, uint16_t  addr_a)
{
    uint8_t* 	addr = (uint8_t*)addr_a;
    eeprom_write_byte((uint8_t*)addr, data_a);     
	DelayMs(10);
}



