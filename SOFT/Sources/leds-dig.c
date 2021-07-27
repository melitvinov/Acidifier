/***********************************************************************************
* Наименование модуля	: Управление четырьмя семисегментными индикаторами
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 09.03.2021
************************************************************************************/

//--- INCLUDES -------------------
#include "leds-dig.h"
#include "leds.h"
#include "math.h"

//--- CONSTANTS ------------------

uint8_t DIGIT[] = {
	0x3F,	// '0'
	0x06,	// '1'
	0x5B,	// '2'
	0x4F,	// '3'
	0x66,	// '4'
	0x6D,	// '5'
	0x7D,	// '6'
	0x07,	// '7'
	0x7F,	// '8'
	0x6F,	// '9'
	0x40,	// '-'
	
	// линии 
	0x01,	//	line_top = 11,
	0x02,	//	line_right_top,
	0x04,	//	line_right_bot,
	0x08,	//	line_bot,
	0x10,	//	line_left_bot
	0x20	//	line_left_top,
};

const int CLOCK_DELAY_TIME = 20;
const int LATCH_DELAY_TIME = 20;

//--- GLOBAL VARIABLES -----------

TLedDig g_LedDig[4];	// текущее состояние четырех цифр дисплея

extern bool g_isNoWater;
//extern bool g_isErrRegulator;
extern bool g_isErrTimeoutSetupPh;
extern bool g_isErrSensors;

extern void switchALARM( uint8_t on );

//--- FUNCTIONS ------------------
void delay_cycles( int num )
{
	while( num >= 0 )
	{
		num--;
	}
}

/*******************************************************
Функция		: Установка отображаемой цифры
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void LcdDig_SetDigit( uint8_t idx, TLedDig * value )
{
	if( idx < 4 && value )
	{
		memcpy( &(g_LedDig[idx]), value, sizeof( TLedDig ) );
	}
}
	
/*******************************************************
Функция		: Инициализация железа
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void LcdDig_init(void)
{
	// Перевод выводов для светодиода на выход
	GPIO_PinConfigure( PORT_LED_CLOCK, PIN_LED_CLOCK, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
	GPIO_PinConfigure( PORT_LED_LATCH, PIN_LED_LATCH, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
	GPIO_PinConfigure( PORT_LED_SERIAL, PIN_LED_SERIAL, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );

	GPIO_PinWrite( PORT_LED_CLOCK, PIN_LED_CLOCK, 0 );
	GPIO_PinWrite( PORT_LED_LATCH, PIN_LED_LATCH, 0 );
	GPIO_PinWrite( PORT_LED_SERIAL, PIN_LED_SERIAL, 0 );
}

void lcd_clock( void )
{
	delay_cycles(CLOCK_DELAY_TIME);
	GPIO_PinWrite( PORT_LED_CLOCK, PIN_LED_CLOCK, 1 );
	delay_cycles(CLOCK_DELAY_TIME);
	GPIO_PinWrite( PORT_LED_CLOCK, PIN_LED_CLOCK, 0 );
	delay_cycles(CLOCK_DELAY_TIME);
}

void lcd_latch( void )
{
	delay_cycles( LATCH_DELAY_TIME );
	GPIO_PinWrite( PORT_LED_LATCH, PIN_LED_LATCH, 1 );
	delay_cycles(LATCH_DELAY_TIME);
	GPIO_PinWrite( PORT_LED_LATCH, PIN_LED_LATCH, 0 );
	delay_cycles(LATCH_DELAY_TIME);
}

/*******************************************************
Функция		: Обновляет дисплей в соответствии с установленными значениями
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void LcdDig_refresh( bool current_blinking_on )
{
	uint8_t dig;
	uint32_t shreg;
	
	vPortEnterCritical();
	
	if( !g_LedDig[3].isOn || (g_LedDig[3].isBlinking && !current_blinking_on) )
		dig = 0;
	else
	{
		dig = DIGIT[g_LedDig[3].value];
		if( g_LedDig[3].isPoint ) dig |= 0x80;
	}
	shreg = dig << 24;
	
	if( !g_LedDig[2].isOn || (g_LedDig[2].isBlinking && !current_blinking_on) )
		dig = 0;
	else
	{
		dig = DIGIT[g_LedDig[2].value];
		if( g_LedDig[2].isPoint ) dig |= 0x80;
	}
	shreg |= dig << 16;

	if( !g_LedDig[1].isOn || (g_LedDig[1].isBlinking && !current_blinking_on) )
		dig = 0;
	else
	{
		dig = DIGIT[g_LedDig[1].value];
		if( g_LedDig[1].isPoint ) dig |= 0x80;
	}
	shreg |= dig << 8;

	if( !g_LedDig[0].isOn || (g_LedDig[0].isBlinking && !current_blinking_on) )
		dig = 0;
	else
	{
		dig = DIGIT[g_LedDig[0].value];
		if( g_LedDig[0].isPoint ) dig |= 0x80;
	}
	shreg |= dig;
	
	// загружаем в сдвиговые регистры полученное значение
	for( int i=31; i>=0; i-- )
	{
		GPIO_PinWrite( PORT_LED_SERIAL, PIN_LED_SERIAL, GETBIT( shreg, i ) );
		//vTaskDelay(1);
		lcd_clock();
		//lcd_latch();
	}
	GPIO_PinWrite( PORT_LED_SERIAL, PIN_LED_SERIAL, 0 );
	delay_cycles(100);
	lcd_latch();
	
	vPortExitCritical();
	
}

void LcdDig_DispOff( void )
{
	for( int i=0; i<4; i++ )
		g_LedDig[0].isOn = false;
	LcdDig_refresh(false);
}

void LcdDig_ShowBegin( void )
{
	LcdDig_DispOff();
	vTaskDelay(500);
	
	TLedDig tdig;
	tdig.isPoint = true;
	tdig.isBlinking = false;
	tdig.isOn = true;

	tdig.value = 10;
	tdig.isPoint = false;
	for( int i=0; i<4; i++ )
	{
		LcdDig_SetDigit( i, &tdig );
	}
	LcdDig_refresh(true);
	vTaskDelay(500);

	for( int dig = 9; dig >= 0; dig-- )
	{
		tdig.value = dig;
		tdig.isPoint = true;
		for( int i=0; i<4; i++ )
		{
			LcdDig_SetDigit( i, &tdig );
		}
		LcdDig_refresh(true);
		vTaskDelay(250);
	}
	tdig.value = 10;
	tdig.isPoint = false;
	for( int i=0; i<4; i++ )
	{
		LcdDig_SetDigit( i, &tdig );
	}
	LcdDig_refresh(true);
	vTaskDelay(500);
}

/*******************************************************
Поток		: Обновляет индикатор
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_Leds_Dig( void *pvParameters )
{
	bool current_blinking_on = true;
	
	LcdDig_init();
	LcdDig_ShowBegin();
	
	const TickType_t CYCLETIME_MS = 500;			// интервал между циклами регулирования
	TickType_t xLastWakeTime;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, CYCLETIME_MS );

		LcdDig_refresh( current_blinking_on );
		
		// управляем реле тревоги
		//switchALARM( g_isErrRegulator || g_isErrSensors || g_isErrTimeoutSetupPh );

		Led_OnOff( LED_ERR_SENSORS, g_isErrSensors && current_blinking_on );
		Led_OnOff( LED_ERR_SETUP_PH_TIMEOUT, g_isErrTimeoutSetupPh && current_blinking_on );
		Led_OnOff( LED_NO_WATER, g_isNoWater && current_blinking_on );
		
		current_blinking_on = !current_blinking_on;
	}
}

void LcdDig_PrintUInt( uint8_t uiValue, ELcdSide side, bool isBlink )
{
	TLedDig ledDig;
	int startIndex = side == SideLEFT ? 0 : 2;
	
	if( uiValue > 99 )
		uiValue = 99;
		
	int beforePoint = uiValue / 10;
	int afterPoint = uiValue - (beforePoint*10);
	
	ledDig.isBlinking = isBlink;
	ledDig.isOn = true;
	ledDig.isPoint = false;

	ledDig.value = beforePoint;
	LcdDig_SetDigit( startIndex, &ledDig );
	
	ledDig.value = afterPoint;
	LcdDig_SetDigit( startIndex+1, &ledDig );
}

void LcdDig_PrintPH( float valuePH, ELcdSide side, bool isBlink )
{
	TLedDig ledDig;
	int startIndex = side == SideLEFT ? 0 : 2;
	float disp_valuePH;
	
	if( valuePH < 0 )
	{
		ledDig.isBlinking = false;
		ledDig.isOn = true;
		ledDig.isPoint = false;
		ledDig.value = 10;
		LcdDig_SetDigit( startIndex, &ledDig );
		LcdDig_SetDigit( startIndex+1, &ledDig );
	}
	else
	{
		disp_valuePH = roundf( valuePH * 10.0 );
		if( disp_valuePH > 99 )
			disp_valuePH = 99;
		int beforePoint = disp_valuePH / 10;
		
		int afterPoint = disp_valuePH - (beforePoint * 10);
		
		ledDig.isBlinking = isBlink;
		ledDig.isOn = true;
		ledDig.isPoint = true;
		ledDig.value = beforePoint;
		LcdDig_SetDigit( startIndex, &ledDig );
		
		ledDig.isPoint = false;
		ledDig.value = afterPoint;
		LcdDig_SetDigit( startIndex+1, &ledDig );
	}
}

void LcdDig_DispBlinkOff( uint8_t side )
{
	if( side & SideLEFT )
		for( int i=0; i<2; i++ )
		{
			g_LedDig[i].isBlinking = false;
		}
	if( side & SideRIGHT )
		for( int i=2; i<4; i++ )
		{
			g_LedDig[i].isBlinking = false;
		}
}

void LcdDig_DispBlinkOn( uint8_t side )
{
	if( side & SideLEFT )
		for( int i=0; i<2; i++ )
		{
			g_LedDig[i].isBlinking = true;
		}
	if( side & SideRIGHT )
		for( int i=2; i<4; i++ )
		{
			g_LedDig[i].isBlinking = true;
		}
}
