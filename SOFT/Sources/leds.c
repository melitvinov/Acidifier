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
	{PORT_LED_OK, PIN_LED_OK},
	{PORT_LED_ERR1, PIN_LED_ERR1},
	{PORT_LED_ERR2, PIN_LED_ERR2},
	{PORT_LED_ERR3, PIN_LED_ERR3},
	{PORT_LED_VIEW, PIN_LED_VIEW},
	{PORT_LED_TAR_P1, PIN_LED_TAR_P1},
	{PORT_LED_TAR_P2, PIN_LED_TAR_P2}
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
		GPIO_PinConfigure( LedArray[i].GPIOx, LedArray[i].pin, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
		
		// Отключение светодиода
		GPIO_PinWrite( LedArray[i].GPIOx, LedArray[i].pin, TO_OFF );
	}
}

/*******************************************************
Функция		: включение светодиода
Параметр 1	: номер светодиода
Возвр. знач.: нет
********************************************************/
void Led_On(uint8_t indx)
{
	if( indx >= LEDS_COUNT )
		return;
	
	GPIO_PinWrite( LedArray[indx].GPIOx, LedArray[indx].pin, TO_ON );
}

/*******************************************************
Функция		: отключение светодиода
Параметр 1	: номер светодиода
Возвр. знач.: нет
********************************************************/
void Led_Off(uint8_t indx)
{
	if( indx >= LEDS_COUNT )
		return;
	
	GPIO_PinWrite( LedArray[indx].GPIOx, LedArray[indx].pin, TO_OFF );
}

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
    if( Led_IsOn(indx) ) 
        Led_Off(indx);
    else
        Led_On(indx);
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
