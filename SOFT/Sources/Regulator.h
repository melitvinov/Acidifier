/***********************************************************************************
* Заголовочный файл для	: Управление регулятором PH
*
************************************************************************************/

#ifndef __REGULATOR_H_
#define __REGULATOR_H_

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

#include "FreeRTOS.h"
#include "task.h"

// --- DEFINES -------------------
#define REL_ON	(0) // логический уровень для включения реле
#define REL_OFF	(1)	// логический уровень для отключения сегмента


// --- TYPES ---------------------

//-- Индексы реле
enum 
{
	REL_PLUS = 0,
	REL_MINUS,
};

typedef struct _relay_desc
{
	GPIO_TypeDef *GPIOx;
	uint16_t pin;
} TRelDesc;

//--- FUNCTIONS ------------------
void Reg_Init(void);
void Reg_RelayOn(uint8_t indx);
void Reg_RelayOff(uint8_t indx);
void Reg_RelayAllOff(void);

void Thread_Regulator( void *pvParameters );

int Reg_ReadCoefficient( uint16_t idx );
bool Reg_WriteCoefficient( uint16_t idx, uint16_t val );

int Reg_Read_MAX_OUT_OF_WATER_SEC( uint16_t idx );
int Reg_Read_MAX_TIME_ERROR_PH_SEC( uint16_t idx );
bool Reg_Write_MAX_OUT_OF_WATER_SEC( uint16_t idx, uint16_t val );
bool Reg_Write_MAX_TIME_ERROR_PH_SEC( uint16_t idx, uint16_t val );

#endif
