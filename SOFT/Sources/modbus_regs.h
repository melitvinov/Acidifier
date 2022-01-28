/***********************************************************************************
* Заголовочный файл для	: 
*
************************************************************************************/

#ifndef __MODBUS_REGS_H
#define __MODBUS_REGS_H

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

// --- DEFINES -------------------
#define START_REG_NUM 			0 			
#define START_REG_VALUES		1000						// номер первого регистра значений
#define START_REG_TAR_POINT		2000						// номер первого регистра тарировочных точек
#define START_REG_COEFFICIENT	3000						// номер первого регистра коэффициентов для расчета PidValue
#define START_REG_TIME_DEFINE	4000						// номер первого регистра таймаутов
#define START_REG_MONITORING	5000						// номер первого регистра мониторинга текущих параметров
#define START_REG_OPTVALUES		6000						// номер первого регистра таблицы оптимальных значений регулятора

// --- TYPES ---------------------

#pragma pack(push,1)

typedef struct __reg_entry
{
    uint16_t addr;											// Относительный адрес регистра (смещение относительно нулевого адреса)
    uint16_t idx;											// индекс входа (0-9)
	int (*read)(uint16_t idx);								// адрес функции чтения значения регистра
    bool (*write)(uint16_t idx, uint16_t val);				// адрес функции записи значения регистра
} TRegEntry;

#pragma pack(pop)

//--- FUNCTIONS ------------------

bool REG_isWriteEnable( uint16_t regAddr );
bool REG_Write( uint16_t RegAddr, uint16_t value );
bool REG_Read( uint16_t RegAddr, uint16_t * pValue );

#endif
