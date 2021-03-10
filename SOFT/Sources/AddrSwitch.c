/***********************************************************************************
* Наименование модуля	: Переключатель адреса для шины RS-485
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 23.05.2018
************************************************************************************/

//--- INCLUDES -------------------
#include "AddrSwitch.h"

//--- GLOBAL VARIABLES -----------

//--- FUNCTIONS ------------------

/*******************************************************
Функция		: Инициализация железа
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void ADRSW_init(void)
{
	// Перевод выводов для переключателя адреса на вход с подтяжкой к земле
	GPIO_PinConfigure( PORT_ADDR_1, PIN_ADDR_1, GPIO_IN_PULL_DOWN, GPIO_MODE_INPUT );
	GPIO_PinConfigure( PORT_ADDR_2, PIN_ADDR_2, GPIO_IN_PULL_DOWN, GPIO_MODE_INPUT );
	GPIO_PinConfigure( PORT_ADDR_4, PIN_ADDR_4, GPIO_IN_PULL_DOWN, GPIO_MODE_INPUT );
	GPIO_PinConfigure( PORT_ADDR_8, PIN_ADDR_8, GPIO_IN_PULL_DOWN, GPIO_MODE_INPUT );
}

/*******************************************************
Функция		: Получение адреса устройства на шине RS-485
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
uint8_t ADRSW_GetAdr(void)
{
	uint8_t addr = 0;

	addr |= GPIO_PinRead( PORT_ADDR_1, PIN_ADDR_1 );
	addr |= GPIO_PinRead( PORT_ADDR_2, PIN_ADDR_2 ) << 1;
	addr |= GPIO_PinRead( PORT_ADDR_4, PIN_ADDR_4 ) << 2;
	addr |= GPIO_PinRead( PORT_ADDR_8, PIN_ADDR_8 ) << 3;
	
	return addr & 0x0F;
}

