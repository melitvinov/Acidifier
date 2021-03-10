/***********************************************************************************
* Наименование модуля	: Управление четырьмя семисегментными индикаторами
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 09.03.2021
************************************************************************************/

//--- INCLUDES -------------------
#include "leds-dig.h"

//--- CONSTANTS ------------------
uint8_t DIGIT[] = {
	0x36,	// '0'
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
};

//--- GLOBAL VARIABLES -----------

TLedDig g_LedDig[4];	// текущее состояние четырех цифр дисплея


//--- FUNCTIONS ------------------
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
	GPIO_PinWrite( PORT_LED_CLOCK, PIN_LED_CLOCK, 1 );
	GPIO_PinWrite( PORT_LED_CLOCK, PIN_LED_CLOCK, 0 );
}

void lcd_latch( void )
{
	GPIO_PinWrite( PORT_LED_LATCH, PIN_LED_LATCH, 1 );
	GPIO_PinWrite( PORT_LED_LATCH, PIN_LED_LATCH, 0 );
}

/*******************************************************
Функция		: Обновляет дисплей в соответствии с 
				установленными значениями
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void LcdDig_refresh( bool current_blinking_on )
{
	uint32_t shift_reg_value = 0;
	uint32_t dig_value;
	
	// подготавливаем значение для сдвиговых регистров
	for( int i=0; i<4; i++ )
	{
		dig_value = 0;
		
		if( ( g_LedDig[i].isOn && !g_LedDig[i].isBlinking ) || ( g_LedDig[i].isOn && g_LedDig[i].isBlinking && current_blinking_on ) )
		{
			// отображаем содержимое этой цифры
			dig_value = DIGIT[g_LedDig[i].value];
		}
		else
		{
			// гасим эту цифру
			dig_value = 0;
		}
		
		shift_reg_value |= dig_value << (i*8);
	}
	
	// загружаем в сдвиговые регистры полученное значение
	for( int n=0; n<32; n++ )
	{
		GPIO_PinWrite( PORT_LED_SERIAL, PIN_LED_SERIAL, GETBIT( shift_reg_value, n ) );
		lcd_clock();
	}

	lcd_latch();
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
	
	TLedDig dig;
	dig.value = 8;
	dig.isPoint = true;
	dig.isBlinking = false;
	dig.isOn = true;
	
	for( int i=0; i<4; i++ )
		LcdDig_SetDigit( i, &dig );
	LcdDig_refresh(true);
	vTaskDelay( 500 );
	
	dig.value = 11;
	dig.isPoint = false;
	for( int i=0; i<4; i++ )
		LcdDig_SetDigit( i, &dig );
	LcdDig_refresh(true);
	vTaskDelay( 500 );
	
	for(;;)
	{
		LcdDig_refresh( current_blinking_on );
		
		current_blinking_on = !current_blinking_on;
		vTaskDelay(500);
	}
}
