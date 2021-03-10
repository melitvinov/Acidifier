/***********************************************************************************
* Наименование модуля	: MODBUS Protocol
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 25.05.2018
************************************************************************************/

//--- INCLUDES -------------------
#include "FreeRTOS.h"
#include "task.h"

#include "modbus.h"

#include "leds.h"
#include "modbus_regs.h"

// --- DEFINES -------------------

// --- TYPES ---------------------

//--- CONSTANTS ------------------

//--- FORWARD --------------------

//--- GLOBAL VARIABLES -----------
//EMBusState MbusState;
TPacketBuffer Mbus_RecvMsg;
TPacketBuffer Mbus_Answer;
TIM_TimeBaseInitTypeDef timer;
bool IsPacketReady;

extern uint16_t g_Status;

//--- IRQ ------------------------
// Таймер
void TIM4_IRQHandler()
{
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	On_Timer();
}

//--- EXTERN ---------------------
extern uint8_t GetDeviceAddress(void);
extern void Rs485_Init(void);
extern void RS485_SendBuf( TPacketBuffer * buffer );

//--- FUNCTIONS ------------------

/*******************************************************
Функция		: Обрабатывает принятый байт
Параметр 1	: байт данных
Возвр. знач.: нет
********************************************************/
void On_Usart_ReceiveChar( char chr )
{
	//if( MbusState == STATE_RECEIVE )
	if( !IsPacketReady )
	{
		//restartWaitTimer();
		
		TIM_Cmd(TIM4, DISABLE);
		TIM_SetCounter(TIM4, 0);
		TIM_Cmd(TIM4, ENABLE);
		
		//addCharToInputPacket( chr );
		
		if( Mbus_RecvMsg.msgLen >= MBUS_MAX_MSGLEN )
		{
			// Ошибка приема! Переполнение приемного буфера
			// Очищаем буфер
			resetInputPacket();
		}

		Mbus_RecvMsg.msgData[Mbus_RecvMsg.msgLen++] = chr;
	}
}

/*******************************************************
Функция		: Инициализация таймера
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void timerInit()
{
	//Включаем тактирование таймера TIM4
    //Таймер 4 у нас висит на шине APB1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//А тут настройка таймера
    //Заполняем поля структуры дефолтными значениями
    TIM_TimeBaseStructInit(&timer);
    //Выставляем предделитель
    timer.TIM_Prescaler = TIMER_PRESCALER;
    //Тут значение, досчитав до которого таймер сгенерирует прерывание
    //Кстати это значение мы будем менять в самом прерывании
    timer.TIM_Period = MBUS_PAKETS_TIMEOUT;
    //Инициализируем TIM4 нашими значениями
    TIM_TimeBaseInit(TIM4, &timer);	
	//Настраиваем таймер для генерации прерывания по обновлению (переполнению)
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    //Разрешаем соответствующее прерывание
    NVIC_EnableIRQ(TIM4_IRQn);
}

/*******************************************************
Функция		: Останавливает прием по modbus
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void On_Timer()
{
	// останавливаем таймер 
	stopTimer();

	IsPacketReady = true;
}

/*******************************************************
Функция		: Стартует процедуру обработки сообщений modbus
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void startReceiveMessages(void)
{
	// останавливаем таймер
	stopTimer();
	
	// очищаем буфер входного пакета
	resetInputPacket();

	// сбрасываем флаг приема пакета
	IsPacketReady = false;
}

/*******************************************************
Функция		: Подготавливает пакет modbus из входных данных
			  Сохраняет результат в глоб. переменной Mbus_Answer
Параметр 1	: номер устройства
Параметр 2	: код функции (плюс признак ошибки если есть 0x80)
Параметр 3	: данные пакета
Параметр 4	: длина данных
Возвр. знач.: нет
********************************************************/
void makeModbusAnswer( uint8_t slaveId, uint8_t funcId, uint8_t * data, uint8_t datalen )
{
	// проверка входных параметров
	if( datalen > 0 && !data )
		return;
	
	// проверка длины данных
	if( datalen > MBUS_MAX_MSGLEN - sizeof(TMB_Header) - sizeof(uint16_t)/*CRC*/ )
		return;
	
	// чистим буфер ответа
	memset( &Mbus_Answer, 0, sizeof(TPacketBuffer) );

	// заполняем заголовок
	PMB_Header pHeader = (PMB_Header)  &(Mbus_Answer.msgData[0]);
	pHeader->slaveId = slaveId;
	pHeader->funcId = funcId;
	
	// заполняем поле данных
	if( datalen )
	for(int i=0; i<datalen; i++)
	{
		memcpy( &(pHeader->firstdataByte), data, datalen );
	}
	
	// получаем текущую длину пакета
	Mbus_Answer.msgLen = sizeof(TMB_Header) - 1 + datalen;
	
	// считаем контрольную сумму
	uint16_t crc = calcCRC( Mbus_Answer.msgData, Mbus_Answer.msgLen );
	
	// дописываем CRC в конец пакета
	*((uint16_t*) &(Mbus_Answer.msgData[Mbus_Answer.msgLen])) = crc;
	
	// увеличиваем длину пакета на размер CRC
	Mbus_Answer.msgLen += sizeof(uint16_t);
	
	// Готово! Сформированный пакет лежит в Mbus_Answer, можно отпрвлять.
}

/*******************************************************
Поток		: Функциональность modbus
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void MBUS_Thread( void *pvParameters )
{
	// Инициализация modbus
	
	// Таймер
	timerInit();
	stopTimer();
	
	// Инициализация RS-485, разрешение прерываний от USART
	Rs485_Init();
	
	g_Status = 0;
	
	// Начинаем прием пакетов
	startReceiveMessages();
	// бесконечный цикл работы modbus
	for(;;)
	{
		Led_Off(LED_SYS);
		
		vTaskDelay(1);

		if( IsPacketReady )
		{
			Led_On(LED_SYS);
			
			TPacketBuffer * answer;
			answer = Parser( &Mbus_RecvMsg );
			
			IsPacketReady = false;
			
			if( answer )
			{
				RS485_SendBuf( answer );
			}
			
			startReceiveMessages();
		}
	}
}


/*******************************************************
Функция		: Пересчитывает значение CRC для следующего байта
Параметр 1	: предыдущее значение CRC
Параметр 2	: следующий байт
Возвр. знач.: CRC
********************************************************/
uint16_t CRC16(uint16_t crc, uint16_t data)
{
	const unsigned int Poly16=0xA001;
	unsigned int LSB, i;
	crc = ((crc^data) | 0xFF00) & (crc | 0x00FF);
	for (i=0; i<8; i++) {
		LSB=(crc & 0x0001);
		crc=crc/2;
		if (LSB)
		crc=crc^Poly16;
	}
	return (crc);
}

/*******************************************************
Функция		: Вычисляет CRC пакета modbus
Параметр 1	: указатель на массив байтов пакета
Параметр 2	: длина принятого пакета в байтах (без CRC)
Возвр. знач.: CRC
********************************************************/
uint16_t calcCRC( uint8_t * data, uint8_t len )
{
	unsigned int crc = 0xFFFF;
	
	for (uint8_t i = 0; i < len; i++) 
	{
		crc = CRC16 (crc, data[i] );
	}
	
	return crc;
}
 
/*******************************************************
Функция		: Парсит принятый Modbus пакет, возвращает указатель на ответ
Параметр 1	: указатель на массив байтов принятого пакета
Параметр 2	: длина принятого пакета в байтах
Возвр. знач.: указатель на ответ
********************************************************/
bool isCrcOk( TPacketBuffer * pMsg )
{
	uint16_t crcInPacket = * ((uint16_t*) &(pMsg->msgData[pMsg->msgLen - 2]));
	uint16_t crcCalculated = calcCRC( (uint8_t*) pMsg->msgData, pMsg->msgLen - 2 );
	
	return ( crcInPacket == crcCalculated );
}

/*******************************************************
Функция		: Подготавливает ответ с описанием ошибки
Параметр 1	: команда из запроса
Параметр 2	: номер ошибки
Возвр. знач.: возвращает флаг - ответ подготовлен
********************************************************/
bool makeErrorAnswer( uint8_t cmd, uint8_t err )
{
	TErrAnswer errAnswer;
	
	errAnswer.devAddr = GetDeviceAddress();
	errAnswer.errCmd = (cmd & 0x3F) | 0x80;
	errAnswer.errCode = err;
	errAnswer.crc = calcCRC( (uint8_t *) &errAnswer, sizeof(TErrAnswer) - sizeof(uint16_t) );
	
	memset( &Mbus_Answer, 0, sizeof(Mbus_Answer) );
	memcpy( &(Mbus_Answer.msgData[0]), &errAnswer, sizeof(TErrAnswer) );
	Mbus_Answer.msgLen = sizeof(TErrAnswer);
	
	return true;
}

/*******************************************************
Функция		: Читает указанныe регистр(регистры)
Параметр 1	: указатель на массив байтов запроса
Параметр 2	: длина запроса в байтах
Возвр. знач.: ответ подготовлен 
********************************************************/
bool Read_HoldingRegs( uint8_t * data, uint8_t datalen )
{
	if( !data || datalen != 4 )
		return false;
	
	// распаковываем запрос
	TReqReadHoldRegsHeader * pReadReq = (TReqReadHoldRegsHeader *) data;
	// меняем порядок байт
	pReadReq->countRegs = SWAP_W(pReadReq->countRegs);
	pReadReq->startAddr = SWAP_W(pReadReq->startAddr);
	
	if( pReadReq->countRegs > MBUS_MAX_REQREGS )
	{
		// ответ не поместится в буфер
		// отвечаем ошибкой входных параметров запроса
		makeErrorAnswer( READ_HOLDING_REGS, err_Value );
		return true;
	}
	
	// буфер для временного хранения запрошенных регистров
	uint8_t buffer[MBUS_MAX_REQREGS*2 + 1];
	uint16_t * pRegs = ( uint16_t* ) &(buffer[1]);
	uint8_t indxReg = 0;
	
	// читаем запрошенные регистры в буфер
	for( uint16_t regAdr = pReadReq->startAddr; regAdr < pReadReq->startAddr + pReadReq->countRegs; regAdr++, indxReg++ )
	{
		
		if( !REG_Read( regAdr, &(pRegs[indxReg]) ) )
		{
			// если не удается прочитать запрошенный регистр 
			// возвращаем ошибку адреса регистра
			makeErrorAnswer( READ_HOLDING_REGS, err_DataAddr );
			return true;
		}
		// меняем порядок байт в полученном значении
		pRegs[indxReg] = SWAP_W(pRegs[indxReg]);
	}

	// все регистры прочитаны, осталось добавить кол-во байт
	buffer[0] = pReadReq->countRegs * 2;
	
	// формируем пакет modbus
	
	makeModbusAnswer( GetDeviceAddress(), READ_HOLDING_REGS, buffer, pReadReq->countRegs * 2 + 1 );
	
	return true;
}

/*******************************************************
Функция		: Записывает указанныe регистр(регистры)
Параметр 1	: указатель на массив байтов запроса
Параметр 2	: длина запроса в байтах
Возвр. знач.: ответ подготовлен 
********************************************************/
bool Write_MultipleRegs( uint8_t * data, uint8_t datalen )
{
	if( !data || datalen < sizeof(TReqWriteRegsHeader) + sizeof(uint16_t) )
		return false;
	
	// распаковываем запрос
	TReqWriteRegsHeader * pWriteReq = (TReqWriteRegsHeader *) data;
	// меняем порядок байт
	pWriteReq->countRegs = SWAP_W(pWriteReq->countRegs);
	pWriteReq->startAddr = SWAP_W(pWriteReq->startAddr);
	
	if(	( pWriteReq->countRegs > MBUS_MAX_REQREGS ) || 
		( pWriteReq->countRegs != pWriteReq->bytesCount / 2 ) )
	{
		// если ответ не поместится в буфер или в запросе неправильно указано кол-во байт данных
		// отвечаем ошибкой входных параметров запроса
		makeErrorAnswer( WRITE_MULTIPLE_REGS, err_Value );
		return true;
	}

	// проверяем запрошенные регистры на наличие и возможность записи
	for( int i=0; i<pWriteReq->countRegs; i++ )
	{

		if( !REG_isWriteEnable( pWriteReq->startAddr + i ) )
		{
			// если любой из регистров отсутствует или запись в него запрещена 
			// то возвращаем ошибку параметров запроса
			makeErrorAnswer( WRITE_MULTIPLE_REGS, err_Value );
			return true;
		}
	}
	
	// все регистры в наличии и запись в них разрешена, пишем
	uint16_t * pValues = (uint16_t*) ( &(pWriteReq->bytesCount) + 1);
	for( int i=0; i<pWriteReq->countRegs; i++ )
	{
		REG_Write( pWriteReq->startAddr + i, SWAP_W( pValues[i] ) );
	}
	
	// формируем пакет modbus
	uint16_t answer[2];
	answer[0] = SWAP_W( pWriteReq->startAddr );
	answer[1] = SWAP_W( pWriteReq->countRegs );
	
	makeModbusAnswer( GetDeviceAddress(), WRITE_MULTIPLE_REGS, (uint8_t*) answer, sizeof(answer) );
	
	return true;
}

/*******************************************************
Функция		: Парсит принятый Modbus пакет, возвращает указатель на ответ
Параметр 1	: указатель на массив байтов принятого пакета
Параметр 2	: длина принятого пакета в байтах
Возвр. знач.: указатель на ответ
********************************************************/
TPacketBuffer * Parser( TPacketBuffer * pMsg )
{
	// Проверка входных данных
	if( !pMsg )
		return 0;
	
	// Проверка CRC пакета запроса
	if( !isCrcOk(pMsg) )
		return 0;
	
	// Проверка совпадения адреса в запросе с адресом устройства
	PMB_Header pHeader = (PMB_Header) pMsg->msgData;
	if( pHeader->slaveId != GetDeviceAddress() )
		return 0;

	bool isNeedSendAnswer = false;
	switch( pHeader->funcId )
	{
		case READ_HOLDING_REGS:
			isNeedSendAnswer = Read_HoldingRegs( &(pHeader->firstdataByte), pMsg->msgLen - PACK_HEADER_CRC_LEN );
			break;
		case WRITE_MULTIPLE_REGS:
			isNeedSendAnswer = Write_MultipleRegs( &(pHeader->firstdataByte), pMsg->msgLen - PACK_HEADER_CRC_LEN );
			break;
		default:
			isNeedSendAnswer = makeErrorAnswer( pHeader->funcId, err_FuncId );
			break;
	}
	
	return (isNeedSendAnswer ? &Mbus_Answer : 0);
}

/*******************************************************
Функция		: Останавливает таймер 
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void stopTimer()
{
    TIM_Cmd(TIM4, DISABLE);
	TIM_SetCounter(TIM4,0);
}

/*******************************************************
Функция		: Запускает таймер на минимальное время
				паузы между пакетами modbus 
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void startTimer()
{
    //Запускаем таймер 
	TIM_SetCounter(TIM4,0);
    TIM_Cmd(TIM4, ENABLE);
}

void restartWaitTimer(void)
{
	TIM_Cmd(TIM4, DISABLE);
	TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

/*******************************************************
Функция		: Очищает буфер последнего пакета modbus
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void resetInputPacket()
{
	//memset( Mbus_RecvMsg.msgData, 0, MBUS_MAX_MSGLEN );
	Mbus_RecvMsg.msgLen = 0;
}

/*******************************************************
Функция		: Добавляет принятый символ в буфер последнего пакета modbus
Параметр 1	: Принятый символ
Возвр. знач.: нет
********************************************************/
void addCharToInputPacket( char chr )
{

	if( Mbus_RecvMsg.msgLen >= MBUS_MAX_MSGLEN )
	{
		// Ошибка приема! Переполнение приемного буфера
		// Очищаем буфер
		resetInputPacket();
	}

	Mbus_RecvMsg.msgData[Mbus_RecvMsg.msgLen++] = chr;
	
}

