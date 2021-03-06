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
	SideLEFT 	= 0x01,
	SideRIGHT 	= 0x02
} ELcdSide;

// --- TYPES ---------------------
typedef struct _led_dig
{
	uint8_t	value;
	bool isOn;
	bool isBlinking;
	bool isPoint;
} TLedDig;

enum LINE_SIMBOL {
	line_top = 11,
	line_right_top,
	line_right_bot,
	line_bot,
	line_left_bot,
	line_left_top,
};

//--- FUNCTIONS ------------------
void Thread_Leds_Dig( void *pvParameters );

void LcdDig_init(void);
void LcdDig_SetDigit( uint8_t idx, TLedDig * value );
void LcdDig_PrintPH( float valuePH, ELcdSide side, bool isBlink );
void LcdDig_PrintUInt( uint8_t uiValue, ELcdSide side, bool isBlink );
void LcdDig_DispBlinkOff( uint8_t side );
void LcdDig_DispBlinkOn( uint8_t side );
void LcdDig_DispOff( void );
void LcdDig_refresh( bool current_blinking_on );
#endif
