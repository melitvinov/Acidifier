/***********************************************************************************
* Наименование модуля	: Управление регулятором PH
*-----------------------------------------------------------------------------------
* Версия				: 1.0
* Автор					: Тимофеев
* Дата создания			: 10.03.2021
************************************************************************************/

//--- INCLUDES -------------------
#include "Regulator.h"

//--- CONSTANTS ------------------
const TRelDesc Relay[] = {
	{PORT_REL_PLUS, PIN_REL_PLUS},
	{PORT_REL_MINUS, PIN_REL_MINUS},
};

//--- GLOBAL VARIABLES -----------

//--- FUNCTIONS ------------------
extern int ReadWorkMode( uint16_t idx );
extern bool IsWaterOk( void );

/*******************************************************
Функция		: Инициализация железа
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Reg_Init(void)
{
	for( int i=0; i<2; i++ )
	{
		// Перевод вывода для светодиода на выход с открытым коллектором
		GPIO_PinConfigure( Relay[i].GPIOx, Relay[i].pin, GPIO_OUT_OPENDRAIN, GPIO_MODE_OUT2MHZ );
		
		// Отключение светодиода
		GPIO_PinWrite( Relay[i].GPIOx, Relay[i].pin, REL_OFF );
	}
}

void regulator_cycle( void )
{
	vTaskDelay(10);
}

/*******************************************************
Поток		: Рабочий поток устройства
Параметр 1	: не используется
Возвр. знач.: бесконечный цикл
********************************************************/
void Thread_Regulator( void *pvParameters )
{
	Reg_Init();
	
	for(;;)
	{
		vTaskDelay(50);
		
		if( ReadWorkMode(0) == Mode_RegulatorPh )
		{
			if( IsWaterOk() )
			{
				regulator_cycle();
			}
		}
		else
		{
			Reg_RelayAllOff();
		}
	}
}


/*******************************************************
Функция		: Включает реле по его индексу
Параметр 1	: индекс реле
Возвр. знач.: нет
********************************************************/
void Reg_RelayOn(uint8_t indx)
{
	if( indx > 1 )
		return;
	
	GPIO_PinWrite( Relay[indx].GPIOx, Relay[indx].pin, REL_ON );
}

/*******************************************************
Функция		: Отключает реле по его индексу
Параметр 1	: индекс реле
Возвр. знач.: нет
********************************************************/
void Reg_RelayOff(uint8_t indx)
{
	if( indx > 1 )
		return;
	
	GPIO_PinWrite( Relay[indx].GPIOx, Relay[indx].pin, REL_OFF );
}

/*******************************************************
Функция		: Отключает оба реле
Параметр 1	: нет
Возвр. знач.: нет
********************************************************/
void Reg_RelayAllOff(void)
{
	GPIO_PinWrite( Relay[0].GPIOx, Relay[0].pin, REL_OFF );
	GPIO_PinWrite( Relay[1].GPIOx, Relay[1].pin, REL_OFF );
}

