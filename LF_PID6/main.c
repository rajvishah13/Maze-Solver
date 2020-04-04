/*
 * LF_PID5.c
 *
 * Created: 20-07-2018 17:37:53
 * Author : Vivek Adajania
 */ 
#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/delay.h>
#include "usart.h"
#include <avr/eeprom.h>

double desired_position=3.5 ,kp=125,kd=0,ki=0,integral=0, set_speed=350,saturation_value=350, flag=0,current_position=0 , n=0 ,sum_left=0, sum_right=0,sum_left_prev=0,sum_right_prev=0;
uint8_t EEMEM nd[20];
uint8_t *p=&nd[0];

void lsa_check()
{
	for(int i=0;i<8;i++)
	{
		if(bit_is_set(PIND,i))
		USART_TransmitNumber(i,0);
		_delay_ms(100);
	}
}
void pwm_init()//16 bit timer atmega128
{
	TCCR1A|=(1<<WGM11 | 1<<COM1A1 | 1<<COM1B1);
	TCCR1B|=(1<<WGM13 | 1<<WGM12 | 1<<CS10);
	DDRB|=(1<<PINB5 | 1<<PINB6); //motor pwm
	ICR1=1000;
}
void directions(int x)
{
	switch(x)
	{
		case -1 :
		PORTE|=(1<<PINE7);
		PORTB&=~(1<<PINB0);
		break;						//hard right
		
		case 0 :
		PORTB|=(1<<PINB0);
		PORTE|=(1<<PINE7);			//forward
		break;
		
		case 1 :
		PORTE&=~(1<<PINE7);
		PORTB|=(1<<PINB0);		//hard left
		break;
	}
}
void algo(int x)
{
	switch(x)
	{
		case 0:
		directions(0);
		break;
		
		case 1:
		directions(1);
		OCR1A=500;						//left
		OCR1B=500;
		while(!(bit_is_set(PIND,3)||bit_is_set(PIND,4)));
		break;
		
		case -1:
		directions(-1);
		OCR1A=500;						//right
		OCR1B=500;
		while(!(bit_is_set(PIND,3)||bit_is_set(PIND,4)));
		break;
		
		case 3:
		directions(1);					//left
		OCR1B=500;
		OCR1A=500;
		_delay_ms(500);
		while(!(bit_is_set(PIND,3)||bit_is_set(PIND,4)));
		break;
	}
	flag=0;
	directions(0);
}
void ir_reading()
{
	current_position=0 , n=0, sum_right=0,sum_left=0;
	
	for(int i=0;i<8;i++)							// line sensor weightage from 0 to 7 (left to right)
	{
		if(bit_is_set(PIND,i))						// n is number of IR ON
		{
			current_position=current_position+i;
			if(i<2)
			sum_left=sum_left+i;
			else if(i>5)
			sum_right=sum_right+i;
			
			n++;
		}
	}
		
			if(n!=0)
			current_position=current_position/n;
			else
			{
				algo(1);
			}
}
 double error(double current_position)
{
	double current_error; 
	
	current_error=desired_position-current_position;
	return current_error;
}
double pid(double current_error)
{
	
	double correction , /*previous_error=0,*/propotional/*,derivative*/;
	
	propotional=kp*current_error;
	
	//integral=integral+ki*current_error;
	
	//derivative=kd*(current_error-previous_error);
	
	//previous_error=current_error;
	
	correction=propotional/*+integral+derivative*/;
	
	if(correction>saturation_value)
		correction=saturation_value;
	else if(correction<-saturation_value)
		correction=-saturation_value;
	//else;
	
	
	return correction;
}


void me_init()
{
	DDRD=0x00;
	DDRB|=(1<<PINB0);
	DDRE|=(1<<PINE7);
	directions(0);
	
}
void  calc_pid()
{
	int correction=(int)pid(error(current_position));
	
	
	OCR1B=set_speed+correction;
	OCR1A=set_speed-correction;
}

int main(void)
{
   double error_left=0,error_right=0;
   pwm_init();     
   me_init();										//direction1-left(E7) direction2-right(B0)
	
   while(1)
   {
			ir_reading();
			if(n>=4)
			{
				flag=1;
				sum_left_prev=sum_left;
				sum_right_prev=sum_right;
			}
			while(flag==1)
			{	
				error_left=sum_left-sum_left_prev;
				error_right=sum_right-sum_right_prev;
				
				if(error_left<0 && error_right<0)
				{
					algo(3);//+ intersection
					eeprom_write_byte(p,1);
					p++;
					continue;
				}
				else if(error_left==0 && error_right<0)
				{
					if(n!=0)
					{
						algo(0);// |- intersection
						eeprom_write_byte(p,2);
						p++;	
					}
					else
					{
						algo(-1);// hard right
						eeprom_write_byte(p,3);
						p++;
					}
					continue;
				}
				else if(error_left<0 && error_right==0)
				{
					if(n!=0)
					{
						algo(3);// -| intersection
						eeprom_write_byte(p,4);
						p++;
					}
					else
					{
						algo(1);//hard left
						eeprom_write_byte(p,5);
						p++;
					}
						continue;
				}
				ir_reading();
			}
			calc_pid();
			
			
	}
		 						
}




