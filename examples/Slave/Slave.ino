/*
Пример передачи данных от "Мастера" -> "Слейву". Используется плата «Колибри».
На плате "Мастера" к пин4 подсоедините тактовую кнопку. Не забудьте поставить в этой части цепи подтягивающий к GND резистор на 10kOM.
Плату слейва соедините с другой платой Ардуино (без микроконтроллера), проводками соедините пин0 с пин0, пин1 с пин1. Соедините Ардуино с ПК и запустите терминал (9600 бод). Через терминал будет видно данные присылаемые мастером.
Подайте на обе платы питание 5 или 3.3V (соедините соответствующие контакты). Выставьте джампер PWR в режим 5 или 3.3V соответственно.
На мастере нажмите на тактовую кнопку. После нажатия проверьте терминал соединенный со слейвом, в него должны придти передаваемые данные.
*/

#include <SI4431.h>
#include "SimRF.h"

/*
#define EZ_WR(reg, val)       SI4431.WriteRegister (reg, val);
#define EZ_RR(reg)            SI4431.ReadRegister (reg);
#define EZ_SWRST()            EZ_WR (0x07, 0x80);
#define EZ_RSTATUS(ST1, ST2)  {ST1 = EZ_RR(0x03); ST2 = EZ_RR(0x04);}
#define EZ_WAIT(IRQBIT)       {SI4431.nIRQWait();}
*/

// Локальный адрес радиомодуля (0 для мастера)
#define SLAVE_ADDR (1)
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


//Формирование и отправка подтверждения пакета
void RFTX_ACK_FUNC01(u8 addr, t_cmd_servo* pcmd_servo)
{
  s16 POS1, POS2, POS3, POS4;
  t_cmd_servo* pcmd_servo_local;    
  
  POS1 = pcmd_servo->POS1;
  POS2 = pcmd_servo->POS2;
  POS3 = pcmd_servo->POS3;
  POS4 = pcmd_servo->POS4;
  print_P(PSTR("\r\nServo function command received!"));
  print_P(PSTR("\r\nPOS1-4:"));
  Serial.println(POS1, HEX);
  Serial.println(POS2, HEX);
  Serial.println(POS3, HEX);
  Serial.println(POS4, HEX);
       
  pcmd_servo_local = (t_cmd_servo*)(&RFTX_buffer[3]);
  
  RFTX_buffer[0] = 0x00; //Адрес мастера
  RFTX_buffer[1] = SLAVE_ADDR; //Наш адрес 
  RFTX_buffer[2] = RF_FUNC01;
  
  pcmd_servo_local->POS1 = pcmd_servo->POS1;
  pcmd_servo_local->POS2 = pcmd_servo->POS2;
  pcmd_servo_local->POS3 = pcmd_servo->POS3;
  pcmd_servo_local->POS4 = pcmd_servo->POS4;
        
  RFTX_buffer[11] = get_xor((u8*)RFTX_buffer, 11);
  SI4431.TXData((u8*) RFTX_buffer, 12); //Отправим пакет по заданному адресу   
}

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
}


void loop()
{
    u8  ItStatus1, ItStatus2;
    u16 length;
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
		length = SI4431.ReadRegister (0x4B);   //read the Received Packet Length register
                Serial.print(length, HEX);
                //убеждаемся, что число принятых байт поместится в буфер МК
		if(length <= sizeof(RFRX_buffer))
		{
		   SI4431.RXData((u8*)RFRX_buffer, length);
                   u8* pData = (u8*) RFRX_buffer;
                   // Проверим, адресовано ли это нам?
				   s8 funccode = RFRX_PROCESS (0x00, length, pData);
                   if (funccode > 0)
                   {
		      RX_LED_SET
                      switch (funccode)
                      {
                          case RF_FUNC01:
                              // Приняли команду 01
    			      // Сформируем и отправим ответ-подтверждение
			      pData = (u8*) &RFRX_buffer[3]; 
                              RFTX_ACK_FUNC01(0x00, (t_cmd_servo*) pData);
                              //после передачи установим биты разрешения прерываний для режима приёма
                              //Разрешим 2 прерывания:
                              // a) первое показывает прием правильного пакета:  'ipkval'
                              // б) второе показывает прием пакета с неправильной КС CRC:  'icrcerror' 
                       	      SI4431.RXIRQEnable();
                              //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
                              SI4431.ReadStatus(&ItStatus1, &ItStatus2);
			      /*Снова разрешаем тракт приёма*/
                              SI4431.RXEnable();
                              break;
                        }
			delay(50);
  		   	RX_LED_CLR
                    }
                    else
                    {
                      print_P(PSTR("\r\nInvalid function code!"));
                    }                                    	                                        
	        }
       	      }
  	      //сброс RX FIFO
            SI4431.FIFOReset();
	      //Разрешение тракта приёма
            SI4431.RXEnable();
	} 
   }
