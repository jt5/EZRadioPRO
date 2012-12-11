/*
Пример передачи данных от "Мастера" -> "Слейву". Используется плата «Колибри».
На плате "Мастера" к пин4 подсоедините тактовую кнопку. Не забудьте поставить в этой части цепи подтягивающий к GND резистор на 10kOM.
Плату слейва соедините с другой платой Ардуино (без микроконтроллера), проводками соедините пин0 с пин0, пин1 с пин1. Соедините Ардуино с ПК и запустите терминал (9600 бод). Через терминал будет видно данные присылаемые мастером.
Подайте на обе платы питание 5 или 3.3V (соедините соответствующие контакты). Выставьте джампер PWR в режим 5 или 3.3V соответственно.
На мастере нажмите на тактовую кнопку. После нажатия проверьте терминал соединенный со слейвом, в него должны придти передаваемые данные.
*/

#include <SI4431.h>
#include "SimRF.h"
#define PB         (PIND & (1<<PD4))

// Локальный адрес радиомодуля (0 для мастера)
#define LOCAL_ADDR (0)
// Коды функций протокола
// Функция 0: пинг ведомого модуля
#define RF_FUNC00 (0)
// Функция 1: передача 4-х координат сервоприводу
#define RF_FUNC01 (1)

// Буфер передаваемых данных
char RFTX_buffer[32];
// Буфер принимаемых данных
char RFRX_buffer[32];

t_cmd_servo cmd_servo;

void print_P (const char* s) 
{
        char c;
        while ((c = pgm_read_byte(s++)) != 0)
         Serial.print(c);
}

u8 get_xor(u8* Src, u8 len)
{
	u8 xoracc = 0;
	while (len--)
	{
		xoracc ^= *Src++;   		
	}
	return xoracc;
}

// Передача функции 01 с параметрами сервоприводов
void RFTX_FUNC01(u8 addr, t_cmd_servo* pcmd_servo)
{  
  t_cmd_servo* pcmd_servo_local;    
  pcmd_servo_local = (t_cmd_servo*)(&RFTX_buffer[3]);
  
  RFTX_buffer[0] = addr;       //Адрес назначения
  RFTX_buffer[1] = LOCAL_ADDR; //Адрес отправителя  
  RFTX_buffer[2] = RF_FUNC01;	//Код функции 01
  
  pcmd_servo_local->POS1 = pcmd_servo->POS1;
  pcmd_servo_local->POS2 = pcmd_servo->POS2;
  pcmd_servo_local->POS3 = pcmd_servo->POS3;
  pcmd_servo_local->POS4 = pcmd_servo->POS4;
 
  // полученный ХЭШ код передаваемых данных
  RFTX_buffer[11] = get_xor ((u8*)RFTX_buffer, 11);
  SI4431.TXData((u8*) RFTX_buffer, 12); //Отправим пакет по заданному адресу   
}

/** Обработка принятых пакетов
*  ВХОД:
*		addr - ожидаемый адрес ведомого устройства
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
  if ((RFRX_buffer[0] == LOCAL_ADDR)&&(RFRX_buffer[1] == addr))
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



// Обработчик ответа для функции 01
void RFRX_RESP_FUNC01(t_cmd_servo* pcmd_servo)
{
  s16 POS1, POS2, POS3, POS4;
  POS1 = pcmd_servo->POS1;
  POS2 = pcmd_servo->POS2;
  POS3 = pcmd_servo->POS3;
  POS4 = pcmd_servo->POS4;
  print_P(PSTR("\r\nServo function response OK!"));
  print_P(PSTR("\r\nPOS1:"));
  Serial.print(POS1, HEX);
  Serial.print(POS2, HEX);
  Serial.print(POS3, HEX);
  Serial.print(POS4, HEX);   
}

void setup()
{
  u8  ItStatus1, ItStatus2;
  // Подтяжка для кнопки
  PORTD |= (1<<PD4);
  Serial.begin(9600);
  u8 length;
  u8 temp8;   	
   print_P (PSTR("\r\nHello!"));
  delay (100);
  SI4431.begin();     
 	
  SI4431.Init(7);  // инициализация радиомодуля  с мощностью передачи 7
  
  // Распечатаем регистры радиомодуля, для проверки SPI связи
  for (u8 reg = 0; reg <=0x7f; reg++)
  {     
      print_P(PSTR("\r\nReg:"));
      u8 c = SI4431.ReadRegister (reg);
      Serial.print(reg, HEX);
      print_P(PSTR(", "));
      Serial.print(c, HEX);
  } 
  
  print_P(PSTR("\r\nRadio initialisation is OK"));     		            
 
  cmd_servo.POS1 = 360;
  cmd_servo.POS2 = 1;
  cmd_servo.POS3 = 60;
  cmd_servo.POS4 = 18;
}

void loop()
{
      u8  ItStatus1, ItStatus2;
      u8 len;
     // Опрос входа МК: нажата кнопка или нет        
     if(PB == 0)
      {                        
        //Ждем отпускания кнопки
        while(PB == 0);
        print_P(PSTR("\r\n Transmitting packet"));				
        RFTX_FUNC01(0x01, &cmd_servo); // Отправим команду для сервопривода с адресом 0x01
        //после передачи установим биты разрешения прерываний для режима приёма
        //Разрешим 2 прерывания:
        // a) первое показывает прием правильного пакета:  'ipkval'
        // б) второе показывает прием пакета с неправильной КС CRC:  'icrcerror' 
        SI4431.RXIRQEnable();
        //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
        SI4431.ReadStatus(&ItStatus1, &ItStatus2);

        /*Снова разрешаем тракт приёма*/
        SI4431.RXEnable();
    }
    // ждём событие прерывания
    // если оно наступило, значит принят пакет либо возникла ошибка CRC
    if(SI4431.IRQstate() == 0 )
    {
      //запрет тракта приёма
      SI4431.RXDisable();
      //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
      SI4431.ReadStatus(&ItStatus1, &ItStatus2);
      // Было прерывание по ошибке CRC
      if( (ItStatus1 & 0x01) == 0x01 )
      {
        print_P(PSTR("\r\n Received CRC Error!"));
        //сброс FIFO передачи
        SI4431.FIFOReset();
        //помигаем всеми светодиодами для индикации ошибки
        TX_LED_SET
        RX_LED_SET
        delay(500);
        TX_LED_CLR
        RX_LED_CLR
      }
      // Было прерывание по приёму нормального пакета
      if( (ItStatus1 & 0x02) == 0x02 )
      {
        print_P(PSTR("\r\n Received good packet! Len = "));
        //Читаем размер полезных данных        
        len = SI4431.RXPacketLen();
        Serial.print(len, HEX);        
        //убеждаемся, что число принятых байт поместится в буфер МК
        if(len <= sizeof(RFRX_buffer))
        {
          SI4431.RXData((u8 *)RFRX_buffer, len); //Читаем принятые данные
          u8* pData = (u8*) RFRX_buffer;
          //проверим, является ли это ответом от сервопривода с aдресом 0x01
          s8 funccode = RFRX_PROCESS (0x01, len, pData); 
          if (funccode > 0)
          {
            switch (funccode)
            {
              case RF_FUNC01:
              // Здесь будет обработчик ответа команды 01
                RFRX_RESP_FUNC01((t_cmd_servo*) pData);
                break;
            }
            RX_LED_SET
            delay(100);
            RX_LED_CLR
          }
         else
         {
            print_P(PSTR("\r\nInvalid function code!"));
         }                                    	                                        
       }
      }      
    }
}