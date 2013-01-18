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

#include "SI4431.h"

SI4431Class SI4431;

/****************************************************
Низкоуровневые функции для работы с модулем SI4431
*****************************************************/

#define EZ_WR(reg, val)       SI4431Class::WriteRegister (reg, val);
#define EZ_RR(reg)            SI4431Class::ReadRegister (reg);
#define EZ_SWRST()            EZ_WR (0x07, 0x80);
#define EZ_RSTATUS(ST1, ST2)  {ST1 = EZ_RR(0x03); ST2 = EZ_RR(0x04);}
#define EZ_WAIT(IRQBIT)       {while(IRQBIT != 0);}

void SI4431Class::begin()
{
	#ifdef LED_ENABLED
	// Конфигурация на вывод портов светодиодов TX и RX
    TX_LED_DDR_SET
    RX_LED_DDR_SET
	#endif
	// Конфигурация на вывод ножки выбора радиомодуля
    NSS_DDR_SET
	// Подтяжка для ножки nIRQ
    PORTB |= (1<<PB1);
	// Инициализация SPI интерфейса
	DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK)|(1 << DD_SS);
	SPCR = (1<<SPE)|(1<<MSTR);
	SPSR = (1<<SPI2X); // 8 Мгц
}


void SI4431Class::WriteRegister(u8 reg, u8 value)
{
    // Выбор радиомодуля установкой сигнала nSEL в низкий уровень
    NSS_LO
    // Пишем адрес регистра в регистр данных SPI МК
    // (важно установить старшый бит!)     
    SPDR  =   (reg|0x80);
    while(!(SPSR & (1<<SPIF))); //ждём окончания передачи
    // Пишем новое значение в регистр радиомодуля 
    SPDR = value;
    while(!(SPSR & (1<<SPIF))); //ждём окончания передачи    
    // Освобождаем  радиомодуль установкой сигнала nSEL в высокий уровень
    NSS_HI   
}

u8 SI4431Class::ReadRegister (u8 reg)
{   
    u8 value;
    // Выбор радиомодуля установкой сигнала nSEL в низкий уровень
    NSS_LO
    // Пишем адрес регистра в регистр данных SPI МК
    SPDR  =   (reg);
    while(!(SPSR & (1<<SPIF))); //ждём окончания передачи   
    SPDR = 0xFF; // Отправляем пустой байт
    while(!(SPSR & (1<<SPIF))); // //ждём окончания передачи    
    // Освобождаем  радиомодуль установкой сигнала nSEL в высокий уровень
    NSS_HI
    // Возвращаем принятое по SPI значение требуемого регистра радиомодуля
    value = SPDR;
    return value;               
}

void SI4431Class::Init(u8 TXPower)
{
	u8 ItStatus1, ItStatus2;
	  //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ
      EZ_RSTATUS(ItStatus1, ItStatus2)
      //Программный сброс
      EZ_SWRST()     //запись значения 0x80 в регистр Operating & Function Control1
      // Ждем сигнал прерывания POR (сброс при подаче питания) от радиомодуля
      EZ_WAIT (NIRQ)  	        
      //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
      EZ_RSTATUS(ItStatus1, ItStatus2)  
      //put_messageP(PSTR("\r\nRadio is ready"));   
      // Ждем сигнал прерывания "готовность" от радиомодуля        
      EZ_WAIT (NIRQ)                  
      //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
      EZ_RSTATUS(ItStatus1, ItStatus2)  
      //print_P(PSTR("\r\nRadio is ready"));     		      
      /*Установка параметров согласно расчету в xls листе от производителя Si4431*/
      //Установка несущей частоты 868.7 MHz
      EZ_WR(0x75, 0x73)
      EZ_WR(0x76, 0x6C)
      EZ_WR(0x77, 0xC0)
      //Скорость передачи данных (9.6kbps)
      EZ_WR(0x6E, 0x4E)
      EZ_WR(0x6F, 0xA5)
      EZ_WR(0x70, 0x2C)
      //Установим величину девиации частоты для передачи (+-45kHz)
      EZ_WR(0x72, 0x48)
      /*Установка параметров модема согласно xls листу(9.6 kbps, deviation: 45 kHz, channel filter BW: 102.2 kHz)*/
      EZ_WR(0x1C, 0x1E)
      EZ_WR(0x20, 0xD0)
      EZ_WR(0x21, 0x00)
      EZ_WR(0x22, 0x9D)
      EZ_WR(0x23, 0x49)
      EZ_WR(0x24, 0x00)
      EZ_WR(0x25, 0x24)
      EZ_WR(0x1D, 0x40)
      EZ_WR(0x1E, 0x0A)
      EZ_WR(0x2A, 0x20)						
      //Длина преамбулы 5 байт
      EZ_WR(0x34, 0x0A)
      //Порог определения преамбулы 20 бит
      EZ_WR(0x35, 0x2A)	
      //Запрет заголовочных байтов (header bytes), установка переменной длины пакетов, установка длины синхрослова в 2 байта
      EZ_WR(0x33, 0x02)	
      //Шаблон синхрослова 0x2DD4
      EZ_WR(0x36, 0x2D)
      EZ_WR(0x37, 0xD4)
      //Разрешение обработчиков пакетов TX и RX и расчет КС CRC-16 (IBM)
      EZ_WR(0x30, 0x8D)
      //Запрет фильтров заголовочных байт при приеме
      EZ_WR(0x32, 0x00)
      //Установка режима FIFO и GFSK модуляции
      EZ_WR(0x71, 0x63)
      /* конфигурация ножек GPIO*/
      EZ_WR(0x0C, 0x12)
      EZ_WR(0x0D, 0x15)	
      //Конфигурация АРУ
      EZ_WR(0x69, 0x60)
	  //Установка мощности передатчика
	  EZ_WR(0x6D, (TXPower | 0x18) & 0x1F);
      //конфигурация конденсаторов
      EZ_WR(0x09, 0xD7)
      /*Разрешение приёмного тракта*/
      EZ_WR(0x07, 0x05)
      //Разрешим 2 прерывания:
      // a) первое показывает прием правильного пакета:  'ipkval'
      // б) второе показывает прием пакета с неправильной КС CRC:  'icrcerror' 
      EZ_WR(0x05, 0x03)
      EZ_WR(0x06, 0x00)
      //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
      EZ_RSTATUS(ItStatus1, ItStatus2);	
}

void SI4431Class::RXEnable(void)
{
        /*разрешаем тракт приёма*/
        EZ_WR(0x07, 0x05)
}		

void SI4431Class::RXDisable(void)
{
        /*Заперещаем тракт приёма*/
        EZ_WR(0x07, 0x01)
}		

void SI4431Class::FIFOReset(void)
{       //сброс FIFO
        EZ_WR(0x08, 0x02)
        EZ_WR(0x08, 0x00)	
}
/************************************************************************
* Функция для передачи блока данных в эфир                             
* len - длина передаваемого блока
* DataSrc - указатель на начало передаваемых данных в SRAM
************************************************************************/
void SI4431Class::TXData(u8* DataSrc, u8 len)
{
	u8 Status1, Status2;
	//put_messageP(PSTR("\r\nTX dump:"));												
    //запрет канала приема (но XTAL работает для обеспечения быстрого переключения в TX!)
	EZ_WR(0x07, 0x01)
    //включим светодиод отображающий режим передачи пакета
	#ifdef LED_ENABLED
	TX_LED_SET						
	#endif
    //задаем длину пакета 
	EZ_WR(0x3E, len)
    // заполним буфер FIFO полезными данными                        
    for (u8 i=0; i< len; i++)
    {
	  u8 c = *DataSrc++;
      EZ_WR(0x7F, c)		
	  //i2hex(c, I2A_BUFFER,2);
	  //put_message(I2A_BUFFER);				  
	}                                         
	//Запрет всех прерываний кроме прерывания по окончании передачи
	//Которое будет использовано для информирования МК об успешной отправке пакета
	EZ_WR(0x05, 0x04)
	EZ_WR(0x06, 0x00)
	//читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
	EZ_RSTATUS(Status1, Status2)
	/*разрешение передатчика*/
	//Радиомодуль формирует пакет и автоматически его отправляет
	EZ_WR(0x07, 0x09)
	/*ожидание прерывания об отправленном пакете*/
	//МК ждет прерывание 'ipksent'
	EZ_WAIT(NIRQ)
	//читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
	EZ_RSTATUS(Status1, Status2)						
	// ждем чтобы светодиод был виден   
	_delay_ms(100);
	// выключаем светодиод
	#ifdef LED_ENABLED
	TX_LED_CLR
	#endif
	//put_messageP(PSTR("\r\nTX end. Sent bytes: "));		
	//i2hex(len, I2A_BUFFER,2);
	//put_message(I2A_BUFFER);				  																
}

void SI4431Class::RXIRQEnable(void)
{
                              //Разрешим 2 прерывания:
                              // a) первое показывает прием правильного пакета:  'ipkval'
                              // б) второе показывает прием пакета с неправильной КС CRC:  'icrcerror' 
				  EZ_WR(0x05, 0x03)
			      EZ_WR(0x06, 0x00)
}				  

void SI4431Class::ReadStatus(u8* Var1, u8* Var2)
{
 *Var1 = EZ_RR(0x03); *Var2 = EZ_RR(0x04);
 }				  

void SI4431Class::RXData(u8* DataDst, u8 len)
{
                    // Забираем данные из RX FIFO
					//put_messageP(PSTR("\r\nRX dump:"));	
					while(len--)
					{
						//Читаем FIFO через спец. регистр 0x7F
						u8 c =  EZ_RR(0x7F)
						*DataDst++ = c;						
//						i2hex(c, I2A_BUFFER,2);
						//put_message(I2A_BUFFER);							
                        //putchar_ (c);
					}
}	

u8 SI4431Class::RXPacketLen(void)
{	
	 //Читаем размер полезных данных
	 u8 len = EZ_RR(0x4B)   //read the Received Packet Length register
	//     i2hex(len, I2A_BUFFER, 2);
	//put_message(I2A_BUFFER);
	 return len;
}


