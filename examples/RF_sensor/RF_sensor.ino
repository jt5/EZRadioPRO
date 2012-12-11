/*
Пример передачи данных от датчика головному устройству с режимом сна микроконтроллера и радиопередатчика.
Отправляются данные SensorData[0], SensorData[1], SensorData[2], SensorData[3].
Используется плата «Колибри».
*/

#include <SI4431.h>
#include "SimRF.h"
#include <avr/sleep.h>
#include <avr/wdt.h> 

#define DEBUG_MODE

// Локальный адрес радиомодуля (0 для мастера)
#define SLAVE_ADDR (1)
// Коды функций протокола

// Функция 1: передача измерений мастеру
#define RF_FUNC01 (1)
// Функция 2: прием команд от мастера
#define RF_FUNC02 (2)

// Буфер передаваемых данных
unsigned char RFTX_buffer[32];
// Буфер принимаемых данных
unsigned char RFRX_buffer[32];

unsigned char putch(unsigned char send_char)
{
  while (!(UCSR0A & 0x20));
  UDR0 = (unsigned char) send_char;
  while (!(UCSR0A & 0x20));
//  delay(8);
  return(send_char);
}


/*
 Печать в консоль строки из FLASH ПЗУ
*/
void print_P (const char* s) 
{
        char c;
        while ((c = pgm_read_byte(s++)) != 0)
         //Serial.print(c);
         putch(c);
}

/*
 Вычисление XOR для массива данных
 u8* Src - указатель на начало данных в ОЗУ
 u8 len - длина обрабатываемого массива
*/
u8 get_xor(u8* Src, u8 len)
{
	u8 xoracc = 0;
	while (len--)
	{
		xoracc ^= *Src++;   		
	}
	return xoracc;
}

/*
 Передача функции 01 с параметрами измерений
 u8 addr - Адрес назначения
 u8* Data - Указатель на начало массива данных
 u8 Len - Длина массива данных
*/
void RFTX_FUNC01(u8 addr, u8* Data, u8 Len)
{           
  RFTX_buffer[0] = addr;       //Адрес назначения
  RFTX_buffer[1] = SLAVE_ADDR; //Адрес отправителя  
  RFTX_buffer[2] = 0x01;	//Код функции 01

  //Укладываем массив данных в пакет для передачи
  for (u8 i = 0 ; i < Len; i++)
  {
    RFTX_buffer[3+i] = *Data++;
  } 
  // получем XOR код передаваемых данных
  RFTX_buffer[3 + Len] = get_xor ((u8*)RFTX_buffer, Len + 3);
  
  SI4431.TXData((u8*) RFTX_buffer, 3 + Len + 1); //Отправим пакет по заданному адресу по радиоканалу
}


/** Обработка принятых пакетов
*  ВХОД:
*		addr - ожидаемый адрес удаленного устройства
*		len - длина принятого пакета
*		pData - указатель на начало принятого пакета в SRAM
* ВЫХОД:
*		(0..127) код принятой функции
*		-1		 ошибка 	
*/

s8 RFRX_PROCESS(u8 addr, u8 len, u8* pData)
{  
  s8 funccode;
  // Слишком короткие пакеты не рассматриваем  
  if (len < 3) return -1;
  if ((RFRX_buffer[0] == SLAVE_ADDR)&&(RFRX_buffer[1] == addr))
  { //Если пакет предназначен для нас и принят от заданного адреса отправителя    
    if (get_xor((u8*)RFRX_buffer, len - 1) == RFRX_buffer[len-1])
    {
      funccode = RFRX_buffer[2];
      pData = (u8*) &RFRX_buffer[3];
      return  funccode;                  
    }
    else return -1;    
  }
  else return -1;
}


/*
  Обработчик ответа от мастера для функции 02
  u8* PayloadData - начало полезных данных
  u8 PayloadLen   - длина полезных данных
*/

void RFRX_RESP_FUNC02(u8* PayloadData, u8 PayloadLen)
{
  // Пока просто распечатаем принятые регистры от мастера в консоли
  #ifdef DEBUG_MODE 
  for (u8 i = 0; i< PayloadLen; i++)
  {
    Serial.print(*PayloadData++, HEX);
  }   
  #endif
}



//Формирование и отправка подтверждения приёма пакета
void RFTX_ACK_FUNC02(u8 addr, u8* Data, u8 Len)
{
  RFTX_buffer[0] = 0x00; //Адрес мастера
  RFTX_buffer[1] = SLAVE_ADDR; //Наш адрес 
  RFTX_buffer[2] = RF_FUNC02;
  
  //Укладываем массив данных в пакет для передачи
  for (u8 i = 0 ; i < Len; i++)
  {
    RFTX_buffer[3+i] = *Data++;
  } 
  // получем XOR код передаваемых данных
  RFTX_buffer[3 + Len] = get_xor ((u8*)RFTX_buffer, Len+3);        
  SI4431.TXData((u8*) RFTX_buffer, 3 + Len + 1); //Отправим пакет по заданному адресу по радиоканалу
}


u16 SensorData [4]; // Массив с 4мя 16 битными словами датчика
/*
SensorData [0] - Температура
SensorData [1] - Влажность
SensorData [2] - Допустим АЦП
SensorData [3] - Доп. информация
*/



volatile unsigned char WDT_wake;
volatile unsigned char SLEEP_TIME = 15;
//volatile unsigned char WDT_counter;  // увеличим счетчик прерываний

/*
Обработчик прерываний по срабатыванию охранного таймера
*/
ISR(WDT_vect) {  
  static unsigned char WDT_counter;  // увеличим счетчик прерываний
  if (WDT_counter++ == SLEEP_TIME) 
  {
    WDT_counter = 0;
    WDT_wake = 1;
  }
  //Serial.print(WDT_counter, HEX);   
  asm ("WDR");
}


void setup_WDT(void) 
{
 asm ("CLI");
 asm ("WDR");
// WDT_counter = 0; 
 MCUSR &= ~(1<<WDRF); //Сброс флага срабатывания
 WDTCSR = (1 << WDCE)|(1<<WDE);
 WDTCSR = (1 << WDP2)|(1 << WDP1)|(1 << WDIE)|(1 << WDIF);  // Настройка сторожевого таймера на период 1 сек. Перевод в режим работы генерирования прерываний
 asm ("SEI"); 
}

void off_WDT(void)
{
  asm ("CLI");
  asm ("WDR");
//  WDT_counter = 0;
  MCUSR &= ~(1<<WDRF); //Сброс флага срабатывания
  WDTCSR = (1 << WDCE)|(1<<WDE)|(1 << WDIF); 
  /* Turn off WDT */
  WDTCSR = 0x00;
  asm ("SEI"); 
}

void sys_sleep(void)
{
  asm ("CLI");    
 // asm ("WDR");      
  ADCSRA &= ~(1 << ADEN);  // Выключим АЦП  
  SMCR |= (1<<SE); // Конфигурируем режим power-down
  //MCUCR |= (1<<BODSE) | (1<<BODS); // Отключаем BOD на время сна
  asm ("SEI");    
  asm ("SLEEP");
}

void sys_wake(void)
{
  // тут можно включить АЦП
}

void RF_sleep (void)
{
    u8  ItStatus1, ItStatus2;
    //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
    SI4431.ReadStatus(&ItStatus1, &ItStatus2);
    SI4431.WriteRegister(0x08, 0x03); //Сброс FIFO
    SI4431.WriteRegister(0x08, 0x00); 
    SI4431.WriteRegister(0x07, 0x00); // Усыпляем модуль
}

void setup()
{
  Serial.begin(38400);  
 #ifdef DEBUG_MODE
   print_P(PSTR("\r\nWait 1 seconds!"));    
   #endif
  delay (1000);
  u8  ItStatus1, ItStatus2;
  u8 length;
  u8 temp8;   	
  #ifdef DEBUG_MODE
    print_P (PSTR("\r\nHello!"));
  #endif
  delay (100);
  SI4431.begin();  
  #ifdef DEBUG_MODE  
  // Распечатаем регистры радиомодуля, для проверки SPI связи
  for (u8 reg = 0; reg <=0x7f; reg++)
  {     
      print_P(PSTR("\r\nReg:"));
      u8 c = SI4431.ReadRegister (reg);
      Serial.print(reg, HEX);
      print_P(PSTR(", "));
      Serial.print(c, HEX);
  }  	
  #endif
  SI4431.Init(7);  
  #ifdef DEBUG_MODE
    print_P(PSTR("\r\nRadio initialisation is OK"));    
  #endif
  delay (1000);
  setup_WDT();
  SMCR = (1<<SE)|(1<<SM1); // Конфигурируем режим power-down
  sys_sleep();
}


void loop()
{
  u8  ItStatus1, ItStatus2;
  unsigned long Timer;
  if (WDT_wake)
  {
  Timer = 0;
  WDT_wake = 0;
  // Просыпаемся раз в N секунд
  // Отключаем сторожевой таймер
  off_WDT();    
  #ifdef DEBUG_MODE
    print_P(PSTR("\r\n WAKE UP!"));  
  #endif 
  // 1) Сначала получим данные, например от АЦП или от SHT11x   
    SensorData[0] = 31.11*100;
    SensorData[1] = 55.15*100;
    SensorData[2] = 3.27*100;
    SensorData[3] = 45;  
  // 2) Потом, сформируем в ОЗУ пакет для передачи в эфир  
  // 3) Потом, выводим радиомодули из спящего режима 
  // 4) Отправляем данные мастеру по адресу 0 (ответа ждать не будем...)  
  for (u8 TXcnt = 0; TXcnt< 4; TXcnt++)
  {
  #ifdef DEBUG_MODE
    print_P(PSTR("\r\n TX to MASTER #"));   
  #endif  
    Serial.print(TXcnt + 1, DEC);
  RFTX_FUNC01(0x00, (u8*) SensorData, 8); // Отправим по адресу 0  8 байт информации с датчика
  // 4) Повторяем отправку данных мастеру по адресу 0 через короткое время для надежности
  }
  // 5) Переводим радиомодуль в режим RX
    //после передачи установим биты разрешения прерываний для режима приёма
    //Разрешим 2 прерывания:
    // a) первое показывает прием правильного пакета:  'ipkval'
    // б) второе показывает прием пакета с неправильной КС CRC:  'icrcerror' 
  SI4431.RXIRQEnable();
 //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
  SI4431.ReadStatus(&ItStatus1, &ItStatus2);
  //сброс FIFO передачи
  SI4431.FIFOReset();  
 /*Снова разрешаем тракт приёма*/
  SI4431.RXEnable();   
  
  // 6) Ждем команды от мастера в течение 1 секунд
    #ifdef DEBUG_MODE
    print_P(PSTR("\r\n Wait Resp"));    
  #endif  
  Timer = 0;
  #define RESP_TIMEOUT (65535 * 32)
  while (Timer < RESP_TIMEOUT)
  {
    //asm ("WDR");
    if(SI4431.IRQstate() == 0 )
    { //принят пакет либо возникла ошибка CRC
      //запрет тракта приёма
      SI4431.RXDisable();
      //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
      SI4431.ReadStatus(&ItStatus1, &ItStatus2);
      // Было прерывание по ошибке CRC
      if( (ItStatus1 & 0x01) == 0x01 )
      {
        #ifdef DEBUG_MODE
          print_P(PSTR("\r\n RX CRC Error!"));
        #endif  
        //сброс FIFO передачи
        SI4431.FIFOReset();
        //помигаем всеми светодиодами для индикации ошибки
        break; // Выход из цикла ожидания ответа  
      }
      else if( (ItStatus1 & 0x02) == 0x02 )
      {
        // Было прерывание по приёму нормального пакета
        #ifdef DEBUG_MODE
          print_P(PSTR("\r\n RX packet Len = "));
        #endif  
        //Читаем размер полезных данных        
        u8 len = SI4431.RXPacketLen();
        #ifdef DEBUG_MODE
          Serial.print(len, HEX);        
        #endif  
        //убеждаемся, что число принятых байт поместится в буфер МК
        if(len <= sizeof(RFRX_buffer))
        {
          SI4431.RXData((u8 *)RFRX_buffer, len); //Читаем принятые данные
          s8 funccode = RFRX_PROCESS (0x00, len, (u8*)RFRX_buffer);
          switch (funccode)
          {
             case RF_FUNC02:
               {
                 // Получили ответ от мастера нам!
                 #ifdef DEBUG_MODE
                   print_P(PSTR("\r\n MASTER RESPONSE!"));
                 #endif  
                 u8* PayloadData = (u8*) &RFRX_buffer[3]; //Начало полезных данных
                 u8 PayloadLen =  len - 4; //Длина полезных данных
                 RFRX_RESP_FUNC02(PayloadData, PayloadLen); //Обработаем ответ
// 7) Если команда пришла, то выполняем ее и отправляем подтверждение выполнения 
                 //Формирование и отправка подтверждения приёма пакета мастеру
                  for (u8 TXcnt = 0; TXcnt< 4; TXcnt++)
                  {
                 #ifdef DEBUG_MODE
                   print_P(PSTR("\r\n ACK MASTER RESPONSE!"));
                 #endif                   
                 RFTX_ACK_FUNC02(0x00, PayloadData, PayloadLen);
                  }
               }
               break;
             default:
               #ifdef DEBUG_MODE
                 print_P(PSTR("\r\nInvalid RX packet!"));
               #endif  
               break;
          }
        }        
      }

      break; // Выход из цикла ожидания ответа      
    }
    else
    {
      Timer++;
    }
  }    
  #ifdef DEBUG_MODE
    if (Timer == RESP_TIMEOUT)   print_P(PSTR("\r\n Response Timeout!"));  
    print_P(PSTR("\r\n SLEEP..."));  
  #endif 

  // 8) Выводим радиомодуль в режим сна
  RF_sleep();
  setup_WDT();  
  //digitalWrite(13, LOW);  
  }
  // 9) Выводим атмегу в режим сна, запуская таймер на N секунд для пробуждения
  // 10) Как проснемся, идем на п. 1.    
  sys_sleep();
}