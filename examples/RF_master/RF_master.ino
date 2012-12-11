/*
Пример приема данных на головном устройстве от датчика.
Подключите Мастера к Ардуино (без микроконтроллера), проводками соедините пин0 с пин0, пин1 с пин1. Соедините Ардуино с ПК и запустите терминал (38400 бод). Через терминал будет видно данные присылаемые от Слейва (сенсора). Можно подключать множество слейв-устройств.
Используется плата «Колибри».
*/

#include <SI4431.h>
#include "SimRF.h"

// Локальный адрес радиомодуля (0 для мастера)
#define LOCAL_ADDR (0)
// Коды функций протокола

// Функция 1: приём данных с датчика
#define RF_FUNC01 (1)
// Функция 2: передача данных датчику
#define RF_FUNC02 (2)

// Буфер передаваемых данных
unsigned char RFTX_buffer[32];
// Буфер принимаемых данных
unsigned char RFRX_buffer[32];

/*
Структура с 4-мя 16-битными регистрами
удаленных датчиков
*/
typedef struct
{
  u16 Register0; 
  u16 Register1;
  u16 Register2;
  u16 Register3;
} tRemoteSensor;

tRemoteSensor RemoteSensorStatus[2]; // Массив регистров состояния удаленных датчиков 1 и 2
tRemoteSensor RemoteSensorCmd[2];    // Массив регистров для отправки на удаленные датчики 1 и 2

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

// Передача функции 02 с параметрами для датчиков из массива RemoteSensorCmd[];
void RFTX_FUNC02(u8 addr)
{    
  RemoteSensorCmd[addr -1].Register0 = RemoteSensorStatus[addr -1].Register0;
  RemoteSensorCmd[addr -1].Register1 = RemoteSensorStatus[addr -1].Register1;
  RemoteSensorCmd[addr -1].Register2 = RemoteSensorStatus[addr -1].Register2;
  RemoteSensorCmd[addr -1].Register3 = RemoteSensorStatus[addr -1].Register3;
    
  RFTX_buffer[0] = addr;       //Адрес назначения
  RFTX_buffer[1] = LOCAL_ADDR; //Адрес отправителя  
  RFTX_buffer[2] = RF_FUNC02;	//Код функции 02
  u16* p16 = (u16*) &RFTX_buffer[3];
  *p16++ = RemoteSensorCmd[addr -1].Register0;
  *p16++ = RemoteSensorCmd[addr -1].Register1;
  *p16++ = RemoteSensorCmd[addr -1].Register2;
  *p16++ = RemoteSensorCmd[addr -1].Register3;  
  // полученный ХЭШ код передаваемых данных
  RFTX_buffer[11] = get_xor ((u8*)RFTX_buffer, 11);
  //Отправим пакет по заданному адресу
  print_P(PSTR("\r\nData to TX: "));  
  for (u8 i = 0; i< 12; i++)
    {
      Serial.print((u8)RFTX_buffer[i], HEX);
      print_P(PSTR(","));
     }     
  SI4431.TXData((u8*) RFTX_buffer, 12); 
}



void RFRX_FUNC01(u8 SlaveAddr, u16* Payload)
{
/*   RemoteSensorStatus [SlaveAddr - 1].Register0 = (u16) RFRX_buffer[4] << 8 + (u16) RFRX_buffer[3];
     RemoteSensorStatus [SlaveAddr - 1].Register1 = (u16) RFRX_buffer[6] << 8 + (u16) RFRX_buffer[5];
     RemoteSensorStatus [SlaveAddr - 1].Register2 = (u16) RFRX_buffer[8] << 8 + (u16) RFRX_buffer[7];
     RemoteSensorStatus [SlaveAddr - 1].Register3 = (u16) RFRX_buffer[10] << 8 + (u16) RFRX_buffer[9];
     */
     RemoteSensorStatus [SlaveAddr - 1].Register0 = *Payload++;
     RemoteSensorStatus [SlaveAddr - 1].Register1 = *Payload++;
     RemoteSensorStatus [SlaveAddr - 1].Register2 = *Payload++;
     RemoteSensorStatus [SlaveAddr - 1].Register3 = *Payload++;
}
            
/** Пример обработки принятых пакетов
*  ВХОД:
*		addr - ожидаемый адрес ведомого устройства
*		len - длина принятого пакета
*		pData - указатель на начало принятого пакета в SRAM
* ВЫХОД:
*		(0..127) код принятой функции
*		-1		 ошибка 	
*/

void RFRX_MASTER_PROCESS(u8 len)
{  
  s8 funccode;
  u8 SlaveAddr;
  // Слишком короткие пакеты не рассматриваем  
  if (len < 3) return;         
       print_P(PSTR("\r\n Dump:"));
        for (u8 i = 0; i< len; i++)
      {
        Serial.print(RFRX_buffer[i], HEX);
              print_P(PSTR(","));
       } 
  SlaveAddr = RFRX_buffer[1]; // Получили адрес слейва  
  if ((RFRX_buffer[0] == LOCAL_ADDR)&&(SlaveAddr != LOCAL_ADDR))  
  { //Если пакет предназначен для MASTER и принят от SLAVE (адрес >= 1)
    // Проверим XOR    
//    print_P(PSTR("\r\n Correct Addr!"));
    if (get_xor((u8*)RFRX_buffer, len - 1) == (u8) RFRX_buffer[len-1])
    {                   
      funccode = RFRX_buffer[2];       //получили код функции
      switch (funccode)
      {
        case RF_FUNC01:
        {
          // Пришли данные с датчика
           print_P(PSTR("\r\nData from sensor: "));
           Serial.print(SlaveAddr, HEX);


		   tRemoteSensor* pSensorData = (tRemoteSensor*)&RFRX_buffer[3];   // Получили указатель на структуру данных с датчика
          // Разбираем данные по переменным
          s16 Temperature_x100 = (s16) (pSensorData-> Register0);         
          u16 Humidity_x100 =    pSensorData-> Register1;         
          u16 Voltage_x100 =     pSensorData-> Register2;
          u16 Data =             pSensorData-> Register3;

          Serial.print("\r\n");
          if (Temperature_x100 < 0)
          {
            Serial.print("-"); //Печатаем "-";
            Temperature_x100 = -Temperature_x100; // преобразем в положительное
          }
          u16 Temp = Temperature_x100 / 100; //Целая часть
          Serial.print(Temp);
          Serial.print(".");
          Serial.print(Temperature_x100 - Temp * 100);
          Serial.print("C\r\n");
          
          Temp = Humidity_x100 / 100;
          Serial.print(Temp);
          Serial.print(".");
          Serial.print(Humidity_x100 - Temp * 100);
          Serial.print("%\r\n");
          
          Temp = Voltage_x100 / 100;
          Serial.print(Temp);
          Serial.print(".");
          Serial.print(Voltage_x100 - Temp * 100);
          Serial.print("V\r\n");
          
          Serial.print(Data);
          Serial.print("\r\n");

          // Скопируем данные в виртуальные регистры датчика для датчиков 01 и 02
          if (SlaveAddr <= 0x02)
          {
            RFRX_FUNC01(SlaveAddr, (u16*) &RFRX_buffer[3]);
          }   
          // Теперь ждем 500 мс
          delay (500);
          // И отправляем посылку с командой 02
          RFTX_FUNC02(SlaveAddr);                    
        }
          break;
        case RF_FUNC02:
          {
            // Пришло подтверждение команды с датчика  
            // Пока ничего делать не будем - просто напишем в консоли, что комнда дошла
            print_P(PSTR("\r\nACK from sensor:"));
            Serial.print(SlaveAddr, HEX);
          }                  
          break;      
        default:
          break;    
      }            
    }  else  print_P(PSTR("\r\n XOR error!"));
  }
  else  print_P(PSTR("\r\n ADDR error!"));
}



// Обработчик ответа для функции 01
void RFRX_RESP_FUNC02(t_cmd_servo* pcmd_servo)
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
  Serial.begin(38400);
  u8 length;
  u8 temp8;   	
  print_P (PSTR("\r\nHello!"));
  delay (1000);
  SI4431.begin();     
  // Распечатаем регистры радиомодуля, для проверки SPI связи
  for (u8 reg = 0; reg <=0x7f; reg++)
  {     
      print_P(PSTR("\r\nReg:"));
      u8 c = SI4431.ReadRegister (reg);
      Serial.print(reg, HEX);
      print_P(PSTR(", "));
      Serial.print(c, HEX);
  }  	
  SI4431.Init(7);  
  print_P(PSTR("\r\nRadio initialisation is OK"));     		             
}



void loop()
{
    u8  ItStatus1, ItStatus2;
    u8 len;
    //Разрешим 2 прерывания:
    // a) первое показывает прием правильного пакета:  'ipkval'
    // б) второе показывает прием пакета с неправильной КС CRC:  'icrcerror' 
    SI4431.RXIRQEnable();
    //читаем регистры состояния прерываний для очистки флагов прерываний и освобождения вывода NIRQ        
     SI4431.ReadStatus(&ItStatus1, &ItStatus2);
    /*Снова разрешаем тракт приёма*/
     SI4431.RXEnable();
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
          RFRX_MASTER_PROCESS (len);                    	                                        
       }
      }
   	      //сброс RX FIFO
            SI4431.FIFOReset();
	      //Разрешение тракта приёма
            SI4431.RXEnable();            
    }
}