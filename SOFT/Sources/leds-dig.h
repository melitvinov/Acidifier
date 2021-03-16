/***********************************************************************************
* ������������ ���� ���	: ������ ��������� �����������
*
************************************************************************************/

#ifndef __LEDS_DIG_H_
#define __LEDS_DIG_H_

//--- INCLUDES -------------------
#include "FreeRTOS.h"
#include "task.h"

#include "GPIO_STM32F10x.h"
#include "stm32f10x.h"
#include "Board.h"

// --- DEFINES -------------------
typedef enum _lcdside
{
	SideLEFT,
	SideRIGHT
} ELcdSide;

// --- TYPES ---------------------
typedef struct _led_dig
{
	uint8_t	value;
	bool isOn;
	bool isBlinking;
	bool isPoint;
} TLedDig;

//--- FUNCTIONS ------------------
void Thread_Leds_Dig( void *pvParameters );

void LcdDig_init(void);
void LcdDig_SetDigit( uint8_t idx, TLedDig * value );
void LcdDig_PrintPH( float valuePH, ELcdSide side, bool isBlink );
void LcdDig_DispBlinkOff( void );
void LcdDig_DispBlinkOn( void );
void LcdDig_DispOff( void );

#endif
