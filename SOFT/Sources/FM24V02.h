/*****************************************************************************************
Заголовочный файл для FM24V02 

*****************************************************************************************/

#ifndef __FM24V02
#define __FM24V02

#include "GPIO_STM32F10x.h"
//#include "I2C_STM32F10x.h"
#include "stm32f10x.h"

#include "Board.h"
//#include "types.h"

#include "FreeRTOS.h"
#include "semphr.h"

// адрес микросхемы памяти на плате
#define MEMORY_ADDRESS	0xA0		

void FM24_Init( void );
bool FM24_WriteBytes( uint16_t addr, const uint8_t *data, uint16_t num );
bool FM24_WriteWords( uint16_t addr, const uint16_t *data, uint16_t num );
bool FM24_ReadBytes( uint16_t addr, uint8_t *data, uint16_t num );
bool FM24_ReadWords( uint16_t addr, uint16_t *data, uint16_t num );

#endif
