/***********************************************************************************
* Наименование модуля	: Управление системными светодиодами
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 23.05.2018
************************************************************************************/

//--- INCLUDES -------------------
#include "leds.h"

//--- CONSTANTS ------------------
const TLedDesc LedArray[] = {
	{PORT_LED_WORK_OK, PIN_LED_WORK_OK},
	{PORT_LED_NO_WATER, PIN_LED_NO_WATER},
	{PORT_LED_ERR_SETUP_PH_TIMEOUT, PIN_LED_ERR_SETUP_PH_TIMEOUT},
	{PORT_LED_ERR_SENSORS, PIN_LED_ERR_SENSORS},
	{PORT_LED_TAR_P1, PIN_LED_TAR_P1},
	{PORT_LED_TAR_P2, PIN_LED_TAR_P2},
	{PORT_LED_VALVE, PIN_LED_VALVE},
};

const int LEDS_COUNT = sizeof( LedArray ) / sizeof( TLedDesc );

//--- GLOBAL VARIABLES -----------

//--- FUNCTIONS ------------------

/*******************************************************
Функция		: Инициализация железа
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Leds_init(void)
{
	for( int i=0; i<LEDS_COUNT; i++ )
	{
		// Перевод вывода для светодиода на выход с открытым коллектором
		GPIO_PinConfigure( LedArray[i].GPIOx, LedArray[i].pin, GPIO_OUT_PUSH_PULL, GPIO_MODE_OUT2MHZ );
		
		// Отключение светодиода
		GPIO_PinWrite( LedArray[i].GPIOx, LedArray[i].pin, TO_OFF );
	}

	// системный светодиод
	GPIO_PinConfigure( PORT_LED_SYS, PIN_LED_SYS, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
	
	// Отключение светодиода
	LedSYS(0);
}

/*******************************************************
Функция		: включение/выключение светодиода
Параметр 1	: индекс светодиода
Возвр. знач.: нет
********************************************************/
void Led_OnOff(uint8_t indx, uint8_t state)
{
	if( indx >= LEDS_COUNT )
		return;
	
	GPIO_PinWrite( LedArray[indx].GPIOx, LedArray[indx].pin, state );
}

void LedSYS( uint8_t state )
{
	bool bstate = state;
	
	GPIO_PinWrite( PORT_LED_SYS, PIN_LED_SYS, !bstate );
}

/*******************************************************
Функция		: включение светодиода
Параметр 1	: номер светодиода
Возвр. знач.: нет
********************************************************/
/*
void Led_On(uint8_t indx)
{
	if( indx >= LEDS_COUNT )
		return;
	
	GPIO_PinWrite( LedArray[indx].GPIOx, LedArray[indx].pin, TO_ON );
}
*/
/*******************************************************
Функция		: отключение светодиода
Параметр 1	: номер светодиода
Возвр. знач.: нет
********************************************************/
/*
void Led_Off(uint8_t indx)
{
	if( indx >= LEDS_COUNT )
		return;
	
	GPIO_PinWrite( LedArray[indx].GPIOx, LedArray[indx].pin, TO_OFF );
}
*/

/*******************************************************
Функция		: Возвращает состояние светодиода
Параметр 1	: номер светодиода
Возвр. знач.: состояние светодиода
********************************************************/
bool Led_IsOn(uint8_t indx)
{
	if( indx >= LEDS_COUNT )
		return false;
	
    uint32_t pinState = GPIO_PinRead( LedArray[indx].GPIOx, LedArray[indx].pin );
	
	bool isOn = ( pinState == TO_ON );

    return isOn;
}

/*******************************************************
Функция		: переключение светодиода
Параметр 1	: номер светодиода
Возвр. знач.: нет
********************************************************/
void Led_Switch(uint8_t indx)
{
    Led_OnOff(indx, !Led_IsOn(indx) );
}

/*******************************************************
Функция		: включение всех светодиодов
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Leds_OnAll(void)
{
	for( int i=0; i<LEDS_COUNT; i++ )
	{
		GPIO_PinWrite( LedArray[i].GPIOx, LedArray[i].pin, TO_ON );
	}
}

/*******************************************************
Функция		: отключение всех светодиодов
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Leds_OffAll(void)
{
	for( int i=0; i<LEDS_COUNT; i++ )
	{
		GPIO_PinWrite( LedArray[i].GPIOx, LedArray[i].pin, TO_OFF );
	}
}
