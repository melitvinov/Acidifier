/***********************************************************************************
* Заголовочный файл для	: 
*
************************************************************************************/

#ifndef __MODBUS_H
#define __MODBUS_H

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

// --- DEFINES -------------------

#define PACK_HEADER_CRC_LEN			4							// длина заголовка пакета + длина поля CRC
#define MIN_MB_PACK_LEN				(PACK_HEADER_CRC_LEN+1)		// минимальная длина пакета протокола modbus
#define MBUS_MAX_MSGLEN				250							// максимальная длина запроса в байтах на шине modbus
#define MBUS_MAX_REQREGS			100							// ограничение длины буфера - максимально возможное кол-во регистров в ответе

#define MBUS_PAKETS_TIMEOUT			3							// время тайм аута между пакетами modbus (мс.)
#define TIMER_PRESCALER				8000

// --- TYPES ---------------------

#pragma pack(push,1)

// структура данных в запросе на чтение регистров
typedef struct __req_read_holding_regs
{
	uint16_t startAddr;
	uint16_t countRegs;
} TReqReadHoldRegsHeader;

// структура данных в запросе на запись регистров
typedef struct __req_write_regs
{
	uint16_t startAddr;
	uint16_t countRegs;
	uint8_t bytesCount;
} TReqWriteRegsHeader;

// буфер для хранения пакета modbus
typedef struct __mb_packet_buffer
{
	uint8_t msgLen;
	uint8_t msgData[MBUS_MAX_MSGLEN];
} TPacketBuffer;

// состояние приема по modbus
typedef enum __modbus_states
{
	STATE_STOP = 0,		// прием остановлен
	STATE_RECEIVE		// прием пакета
} EMBusState;

// стандартный заголовок пакета
typedef struct __mb_header
{
	uint8_t  	slaveId;
	uint8_t		funcId;
	uint8_t		firstdataByte;
} TMB_Header, *PMB_Header;

// структура пакета сообщения об ошибке
typedef struct __mb_err_answer
{
	uint8_t		devAddr;
	uint8_t		errCmd;
	uint8_t		errCode;
	uint16_t	crc;
} TErrAnswer;

// номера ошибок modbus
typedef enum __cmd_exec_err
{
	err_FuncId		= 1,	// Принятый код функции не может быть обработан
	err_DataAddr	= 2,	// Адрес данных, указанный в запросе, недоступен
	err_Value		= 3,	// Значение, содержащееся в поле данных запроса, является недопустимой величиной
	err_Error		= 4,	// Невосстанавливаемая ошибка имела место, пока ведомое устройство пыталось выполнить затребованное действие
	err_Wait		= 5,	// Ведомое устройство приняло запрос и обрабатывает его, но это требует много времени. Этот ответ предохраняет ведущее устройство от генерации ошибки тайм-аута
	err_Busy		= 6,	// Ведомое устройство занято обработкой команды. Ведущее устройство должно повторить сообщение позже, когда ведомое освободится
	err_ExecFunc	= 7,	// Ведомое устройство не может выполнить программную функцию, заданную в запросе. Этот код возвращается для неуспешного программного запроса, использующего функции с номерами 13 или 14. Ведущее устройство должно запросить диагностическую информацию или информацию об ошибках от ведомого.
	err_Mem			= 8		// Ведомое устройство при чтении расширенной памяти обнаружило ошибку паритета. Ведущее устройство может повторить запрос, но обычно в таких случаях требуется ремонт.	
} EFuncResult;

// обрабатываемые функции в запросах
typedef enum __valid_commands
{
	READ_HOLDING_REGS 	= 0x03,		// Read Holding Registers
	WRITE_MULTIPLE_REGS	= 0x10		// Preset Multiple Registers
} EValidCmd;


#pragma pack(pop)

//--- FUNCTIONS ------------------

void MBUS_Thread( void *pvParameters );
TPacketBuffer * Parser( TPacketBuffer * pMsg );

void timerInit(void);
void startTimer(void);
void stopTimer(void);
void restartWaitTimer(void);

void startReceiveMessages(void);
void resetInputPacket(void);
void addCharToInputPacket( char chr );
uint16_t calcCRC( uint8_t * data, uint8_t len );
bool isCrcOk( TPacketBuffer * pMsg );

void On_Usart_ReceiveChar( char chr );
void On_Timer( void );

bool makeErrorAnswer( uint8_t cmd, uint8_t err );
void makeModbusAnswer( uint8_t slaveId, uint8_t funcId, uint8_t * data, uint8_t datalen );

bool Write_MultipleRegs( uint8_t * data, uint8_t datalen );
bool Read_HoldingRegs( uint8_t * data, uint8_t datalen );
#endif
