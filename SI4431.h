/*
Name:		EZRadioPRO library for Arduino
Version:	1.0
Created:	24.02.2012
Updated:	24.02.2012
Programmer:	Erezeev A.
Production:	JT5.RU
Source:		https://github.com/jt5/EZRadioPRO

This library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this library.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>
#define F_CPU     16000000UL     /**< Системная частота, МГц */
#include <util/delay.h>

#define LED_ENABLED

#define NIRQ       (PINB & (1<<PB1))

#define NSS_HI     PORTB |= (1<<PB2);
#define NSS_LO     PORTB &= ~(1<<PB2);

#define NSS_DDR_SET DDRB |= (1<<PB2);
#define NSS_DDR_CLR DDRB &= ~(1<<PB2);


#ifdef LED_ENABLED
	#define TX_LED_SET     PORTD |= (1<<PD2);
	#define TX_LED_CLR     PORTD &= ~(1<<PD2);
	#define TX_LED_DDR_SET DDRD |= (1<<PD2);
	#define TX_LED_DDR_CLR DDRD &= ~(1<<PD2);
	#define RX_LED_SET     PORTD |=	(1<<PD3);
	#define RX_LED_CLR     PORTD &= ~(1<<PD3);
	#define RX_LED_DDR_SET DDRD |= (1<<PD3);
	#define RX_LED_DDR_CLR DDRD &= ~(1<<PD3);
#endif

#define DDR_SPI DDRB
#define PORT_SPI PORTB
#define PORT_LED PORTD
#define DDR_LED DDRD
#define PIN_SPI  PINB

#define DD_MOSI PB3
#define DD_SCK PB5
#define DD_SS PB2

typedef unsigned char u8;
typedef signed char s8;
typedef unsigned int u16;
typedef signed int s16;



class SI4431Class{
public:
	static void begin();
	static void end();	
	static void WriteRegister(u8 reg, u8 value);
	static u8	ReadRegister (u8 reg);
	static void TXData(u8* DataSrc, u8 len);
	static void RXData(u8* DataDst, u8 len);
	static u8 RXPacketLen(void);
	inline static void nIRQWait(void);
	inline static u8 IRQstate(void);
	static void Init(u8 TXPower);
	static void RXEnable(void);	
	static void RXDisable(void);	
	static void FIFOReset(void);	
	static void RXIRQEnable(void);
	static void ReadStatus(u8* Var1, u8* Var2);
};

extern SI4431Class SI4431;

void SI4431Class::nIRQWait(void)
{	
	while(NIRQ != 0);
}

u8 SI4431Class::IRQstate(void)
{	
	if (NIRQ) return 1;
	else return 0;
}