/***********************************************************************************
* Заголовочный файл для	: Модуль аналоговых входов
*
************************************************************************************/

#ifndef __ANA_INPUTS_H
#define __ANA_INPUTS_H

//--- INCLUDES -------------------

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "math.h"

#include "Board.h"
#include "Leds.h"
#include "FM24V02.h"

// --- DEFINES -------------------

#define AVERAGE_COUNT	20

// --- TYPES ---------------------
typedef struct __pindesc
{
	GPIO_TypeDef *GPIOx;
	uint32_t num;
} TPinDesc;

typedef struct __tar_point
{
	uint16_t adc_value;
	uint16_t tar_value;
} TTarPoint;

typedef struct __tar_table
{
	TTarPoint point1;
	TTarPoint point2;
} TTarTable;

//--- FUNCTIONS ------------------
void AInp_Thread( void *pvParameters );
int AInp_ReadAdcValue( uint16_t idx );
int AInp_ReadPhValue( uint16_t idx );
int AInp_ReadAdcTar1( uint16_t idx );
int AInp_ReadPhTar1( uint16_t idx );
int AInp_ReadAdcTar2( uint16_t idx );
int AInp_ReadPhTar2( uint16_t idx );
bool AInp_WriteAdcTar1( uint16_t idx, uint16_t val );
bool AInp_WritePhTar1( uint16_t idx, uint16_t val );
bool AInp_WriteAdcTar2( uint16_t idx, uint16_t val );
bool AInp_WritePhTar2( uint16_t idx, uint16_t val );

int AInp_ReadSystemPh( uint16_t idx );

float AInp_GetFloatSensorPh( uint8_t idx );
float AInp_GetSystemPh( void );
#endif
